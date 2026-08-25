# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for the Webots ``shell+tools`` bridge -- none of them need Webots.

The load-bearing test is **completeness** (plan 2.2's rule, made mechanical):
the public Supervisor/Robot API of the local controller package is enumerated
by AST -- no import of ``controller``, which would need the simulator's DLL --
and every public method or property must be either wrapped by the bridge or
listed with a justification in ``adapters/webots/EXCLUSIONS.md``. A bridge
faithful in every included verb but curated by omission is the abuse this
test exists to catch.

The rest: tool names must be upstream's own ``wb_*`` function-index names
(checked against the ``wb.wb_*`` references the package itself makes),
schemas must be valid and 1:1 with the controller-side argument table (they
are generated from the same table, and that table is exercised here), the
manifest hash must be deterministic, and the JSON-over-TCP protocol must
round-trip -- including every structured error surface -- against a fake
Supervisor.
"""

from __future__ import annotations

import ast
import json
import re
import socket
import sys
import threading
import time
from pathlib import Path

import pytest

BENCHMARKS = Path(__file__).resolve().parents[3]
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

from agentbench.adapters.webots import bridge_controller as bc   # noqa: E402
from agentbench.common.paths import REPO                         # noqa: E402
from agentbench.runner.isolation import Sandbox                  # noqa: E402
from agentbench.runner.tools import webots_bridge as wb          # noqa: E402

# The Python controller API MOVED on 2026-08-08 (commit 14d303c7: "the Python
# controller API now LIVES in omnisim -- controller becomes the shim"). The old
# path still exists as a re-export shim with no `wb.wb_*` references in it, so
# pointing here was not an import error -- referenced_wb_names() just came back
# EMPTY and every name assertion below failed with "is not a wb_* function the
# controller package references". Point at the implementation, not the shim.
CONTROLLER_PKG = REPO / "lib" / "controller" / "python" / "omnisim"
EXCLUSIONS_MD = (Path(bc.__file__).resolve().parent / "EXCLUSIONS.md")

# The scope of the completeness rule: the classes that carry upstream's
# Supervisor + Robot (device-generic) function reference in the local
# controller package.
SCOPE = {
    "robot.py": ("Robot",),
    "supervisor.py": ("Supervisor",),
    "node.py": ("Node", "ContactPoint"),
    "field.py": ("Field",),
    "proto.py": ("Proto",),
    "device.py": ("Device",),
}


# ---------------------------------------------------------------------------
# AST enumeration of the public API (no simulator, no DLL)
# ---------------------------------------------------------------------------

def _class_defs(tree, name):
    """Every ClassDef with this name; node.py forward-declares ``Node``."""
    return [n for n in ast.walk(tree)
            if isinstance(n, ast.ClassDef) and n.name == name]


def _public_functions(class_def):
    out = set()
    for stmt in class_def.body:
        if isinstance(stmt, (ast.FunctionDef, ast.AsyncFunctionDef)):
            if not stmt.name.startswith("_"):
                out.add(stmt.name)
    return out


def enumerate_public_api():
    """{"Class.method", ...} for every public method/property in scope,
    including module-level monkeypatches (``Field.getSFNode = getSFNode``
    in supervisor.py)."""
    api = set()
    class_names = {c for names in SCOPE.values() for c in names}
    trees = {}
    for filename in SCOPE:
        path = CONTROLLER_PKG / filename
        trees[filename] = ast.parse(path.read_text(encoding="utf-8"),
                                    filename=str(path))
    for filename, classes in SCOPE.items():
        for cls in classes:
            defs = _class_defs(trees[filename], cls)
            assert defs, "class %s not found in %s" % (cls, filename)
            # The real definition is the one with the most methods (skips
            # the empty forward declaration of Node).
            best = max(defs, key=lambda d: len(_public_functions(d)))
            for method in _public_functions(best):
                api.add("%s.%s" % (cls, method))
    # Module-level ``Class.attr = <module function>`` monkeypatches.
    for filename, tree in trees.items():
        module_functions = {n.name for n in tree.body
                            if isinstance(n, ast.FunctionDef)}
        for stmt in tree.body:
            if not isinstance(stmt, ast.Assign):
                continue
            for target in stmt.targets:
                if (isinstance(target, ast.Attribute)
                        and isinstance(target.value, ast.Name)
                        and target.value.id in class_names
                        and not target.attr.startswith("_")
                        and isinstance(stmt.value, ast.Name)
                        and stmt.value.id in module_functions):
                    api.add("%s.%s" % (target.value.id, target.attr))
    return api


def referenced_wb_names():
    """Every ``wb.wb_*`` C function the controller package references --
    i.e. upstream's function-index names, as inherited by this fork."""
    names = set()
    for path in CONTROLLER_PKG.glob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"),
                         filename=str(path))
        for node in ast.walk(tree):
            if (isinstance(node, ast.Attribute)
                    and isinstance(node.value, ast.Name)
                    and node.value.id == "wb"
                    and node.attr.startswith("wb_")):
                names.add(node.attr)
    return names


def excluded_tokens():
    """`Class.method` tokens from EXCLUSIONS.md, restricted to the scoped
    classes so incidental backticked text cannot count as a justification."""
    text = EXCLUSIONS_MD.read_text(encoding="utf-8")
    class_names = {c for names in SCOPE.values() for c in names}
    out = set()
    for match in re.finditer(r"`([A-Za-z]+)\.([A-Za-z_][A-Za-z0-9_]*)`",
                             text):
        if match.group(1) in class_names:
            out.add("%s.%s" % (match.group(1), match.group(2)))
    return out


# ---------------------------------------------------------------------------
# THE COMPLETENESS RULE (plan 2.2)
# ---------------------------------------------------------------------------

def test_every_public_method_is_wrapped_or_justified():
    api = enumerate_public_api()
    wrapped = set(wb.WRAPPED_API.values())
    excluded = excluded_tokens()
    unaccounted = sorted(api - wrapped - excluded)
    assert not unaccounted, (
        "public controller-API members neither wrapped by the bridge nor "
        "justified in EXCLUSIONS.md (curation-by-omission is exactly what "
        "the completeness rule forbids): %s" % ", ".join(unaccounted))


def test_no_phantom_wrappings():
    """Everything the bridge claims to wrap must actually exist."""
    api = enumerate_public_api()
    phantoms = sorted(set(wb.WRAPPED_API.values()) - api)
    assert not phantoms, ("bridge wraps methods the controller package does "
                          "not have: %s" % ", ".join(phantoms))


def test_nothing_is_both_wrapped_and_excluded():
    both = sorted(set(wb.WRAPPED_API.values()) & excluded_tokens())
    assert not both, ("wrapped functions also listed as exclusions "
                      "(pick one): %s" % ", ".join(both))


def test_tool_names_are_upstream_function_index_names():
    """Every tool is named by upstream's own wb_* function name -- and that
    name is one the controller package genuinely binds, so a paraphrased or
    invented name cannot slip in."""
    real = referenced_wb_names()
    for name in bc.FUNCTIONS:
        assert re.fullmatch(r"wb_[a-z0-9_]+", name), name
        assert name in real, (
            "%r is not a wb_* function the controller package references; "
            "tool names must come from upstream's function index, not be "
            "paraphrased" % name)


def test_no_invented_composites():
    """The bridge adds nothing beyond the shell set and the wb_* functions:
    no scene-tree aggregate, no batch verb upstream does not publish."""
    ts = _build_tool_set()
    shell_names = {"run_shell", "read_file", "write_file", "list_dir"}
    for spec in ts.specs:
        assert spec.name in shell_names or spec.name in bc.FUNCTIONS, (
            "unexpected tool %r: the bridge must be faithful "
            "function-for-function" % spec.name)
    assert len(ts.specs) == len(shell_names) + len(bc.FUNCTIONS)


def test_contact_points_and_import_from_string_are_wrapped():
    """The two verbs plan 2.2 records as the first draft's omissions."""
    assert "wb_supervisor_node_get_contact_points" in bc.FUNCTIONS
    assert "wb_supervisor_field_import_mf_node_from_string" in bc.FUNCTIONS
    assert "wb_supervisor_field_import_sf_node_from_string" in bc.FUNCTIONS


def test_exclusion_rows_carry_a_reason():
    """Every exclusion token sits in a table row with non-empty reason text."""
    text = EXCLUSIONS_MD.read_text(encoding="utf-8")
    for line in text.splitlines():
        if not line.startswith("| `"):
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        assert len(cells) >= 2 and len(cells[1]) >= 4, (
            "exclusion row without a justification: %r" % line)


# ---------------------------------------------------------------------------
# Tool specs and manifest
# ---------------------------------------------------------------------------

def _sandbox(tmp_path, name="run"):
    return Sandbox.create(tmp_path / name, tmp_path / name / "scratch",
                          ports=False)


def _build_tool_set(tmp_path=None):
    import tempfile
    if tmp_path is None:
        tmp_path = Path(tempfile.mkdtemp(prefix="agentbench_wbtest_"))
    return wb.build(_sandbox(tmp_path))


def test_every_tool_spec_is_schema_valid(tmp_path):
    ts = wb.build(_sandbox(tmp_path))
    names = [s.name for s in ts.specs]
    assert len(names) == len(set(names)), "duplicate tool names"
    for spec in ts.specs:
        assert spec.description.strip(), spec.name
        schema = spec.input_schema
        assert schema.get("type") == "object", spec.name
        props = schema.get("properties", {})
        assert isinstance(props, dict), spec.name
        for pname, pschema in props.items():
            assert pschema.get("type") in ("string", "integer", "number",
                                           "boolean", "array"), (spec.name,
                                                                 pname)
            assert pschema.get("description", "").strip(), (spec.name, pname)
        for req in schema.get("required", []):
            assert req in props, (spec.name, req)
        assert callable(spec.handler), spec.name


def test_schemas_agree_with_the_controller_argument_table(tmp_path):
    """Runner-side schemas and controller-side dispatch come from one table;
    assert the generated schema really mirrors it, arg for arg."""
    ts = wb.build(_sandbox(tmp_path))
    kind_to_type = {"int": "integer", "float": "number", "str": "string",
                    "bool": "boolean", "node": "integer", "field": "integer",
                    "proto": "integer", "device": "integer",
                    "vec2": "array", "vec3": "array", "vec4": "array",
                    "vec6": "array", "color": "array"}
    for name, fn in bc.FUNCTIONS.items():
        spec = ts.get(name)
        assert spec is not None, "tool missing for %s" % name
        props = spec.input_schema["properties"]
        assert set(props) == {a[0] for a in fn.args}, name
        assert set(spec.input_schema.get("required", [])) == {
            a[0] for a in fn.args if a[2]}, name
        for arg_name, kind, _req in fn.args:
            assert props[arg_name]["type"] == kind_to_type[kind], (name,
                                                                   arg_name)


def test_manifest_hash_is_deterministic(tmp_path):
    a = wb.build(_sandbox(tmp_path, "a"))
    b = wb.build(_sandbox(tmp_path, "b"))
    assert a.tools_sha256 == b.tools_sha256
    assert a.definitions() == b.definitions()
    # And the definitions are exactly the generated ones -- what ships is
    # what the table produces, nothing hand-tuned on the side.
    generated = {d["name"]: d for d in wb.tool_definitions()}
    for spec in a.specs:
        if spec.name in generated:
            assert spec.definition() == generated[spec.name]


def test_manifest_hash_does_not_depend_on_the_run_port(tmp_path):
    """The per-run bridge endpoint must stay out of every hashed field, or
    tools_sha256 stops being stable across runs."""
    sandbox_a = _sandbox(tmp_path, "a")
    sandbox_a.webots_bridge_port = 54321
    sandbox_b = _sandbox(tmp_path, "b")
    sandbox_b.webots_bridge_port = 12345
    a, b = wb.build(sandbox_a), wb.build(sandbox_b)
    assert a.tools_sha256 == b.tools_sha256
    dump = json.dumps(a.to_json())
    assert "54321" not in dump, "the run's port leaked into the manifest"


# ---------------------------------------------------------------------------
# Dispatch: the structured error surface, against a fake Supervisor
# ---------------------------------------------------------------------------

class ContactPoint:
    def __init__(self, point, node_id, depth):
        self.point = point
        self.node_id = node_id
        self.depth = depth


class Field:
    def __init__(self):
        self.imports = []

    def getSFFloat(self):
        return 9.5

    def importMFNodeFromString(self, position, node_string):
        self.imports.append((position, node_string))


class Node:
    def __init__(self, def_name="BOX"):
        self._def = def_name
        self._field = Field()

    def getDef(self):
        return self._def

    def getPosition(self):
        return [1.0, 2.0, 3.0]

    def getField(self, name):
        return self._field if name == "translation" else None

    def getContactPoints(self, include_descendants=False):
        return [ContactPoint([0.5, 0.0, 0.01], 42, 0.001)]

    def addForce(self, force, relative):
        self.last_force = (force, relative)


class FakeSupervisor:
    def __init__(self):
        self.node = Node()
        self.quit_status = None

    def getSupervisor(self):
        return True

    def getTime(self):
        return 1.5

    def step(self, time_step=None):
        return 0

    def getFromDef(self, def_name):
        return self.node if def_name == "BOX" else None

    def getRoot(self):
        return self.node

    def simulationQuit(self, status):
        self.quit_status = status

    def worldLoad(self, filename):
        raise RuntimeError("boom: no world here")


@pytest.fixture()
def bridge():
    return bc.Bridge(FakeSupervisor())


def rpc(bridge, fn, **args):
    return bridge.dispatch({"fn": fn, "args": args})


def test_dispatch_happy_path_and_handle_interning(bridge):
    got = rpc(bridge, "wb_supervisor_node_get_from_def", **{"def": "BOX"})
    assert got == {"ok": True, "result": {"node": 1}}
    # Same underlying object -> same handle (upstream refs are stable).
    again = rpc(bridge, "wb_supervisor_node_get_root")
    assert again["result"] == {"node": 1}
    pos = rpc(bridge, "wb_supervisor_node_get_position", node=1)
    assert pos == {"ok": True, "result": [1.0, 2.0, 3.0]}
    missing = rpc(bridge, "wb_supervisor_node_get_from_def",
                  **{"def": "NOPE"})
    assert missing == {"ok": True, "result": None}


def test_dispatch_contact_points_serialize_inline(bridge):
    rpc(bridge, "wb_supervisor_node_get_from_def", **{"def": "BOX"})
    got = rpc(bridge, "wb_supervisor_node_get_contact_points", node=1,
              include_descendants=True)
    assert got["ok"]
    assert got["result"] == [{"point": [0.5, 0.0, 0.01], "node_id": 42,
                              "depth": 0.001}]


def test_dispatch_field_chain_and_import_from_string(bridge):
    rpc(bridge, "wb_supervisor_node_get_from_def", **{"def": "BOX"})
    fld = rpc(bridge, "wb_supervisor_node_get_field", node=1,
              name="translation")
    handle = fld["result"]["field"]
    assert rpc(bridge, "wb_supervisor_field_get_sf_float",
               field=handle)["result"] == 9.5
    got = rpc(bridge, "wb_supervisor_field_import_mf_node_from_string",
              field=handle, position=-1, node_string="Solid { }")
    assert got["ok"]
    assert bridge.robot.node._field.imports == [(-1, "Solid { }")]


def test_dispatch_vector_argument(bridge):
    rpc(bridge, "wb_supervisor_node_get_from_def", **{"def": "BOX"})
    got = rpc(bridge, "wb_supervisor_node_add_force", node=1,
              force=[0.0, 0.0, 9.81], relative=False)
    assert got["ok"]
    assert bridge.robot.node.last_force == ([0.0, 0.0, 9.81], False)
    bad = rpc(bridge, "wb_supervisor_node_add_force", node=1,
              force=[1.0, 2.0], relative=False)
    assert bad["error"]["code"] == "BAD_ARGUMENT"


def test_dispatch_error_surfaces(bridge):
    assert rpc(bridge, "wb_no_such_thing")["error"]["code"] == \
        "UNKNOWN_FUNCTION"
    assert bridge.dispatch("not a dict")["error"]["code"] == "BAD_REQUEST"
    assert rpc(bridge, "wb_supervisor_node_get_position")["error"]["code"] \
        == "MISSING_ARGUMENT"
    assert rpc(bridge, "wb_supervisor_node_get_position",
               node="one")["error"]["code"] == "BAD_ARGUMENT"
    assert rpc(bridge, "wb_supervisor_node_get_position",
               node=99)["error"]["code"] == "UNKNOWN_HANDLE"
    # A field handle where a node handle is required.
    rpc(bridge, "wb_supervisor_node_get_from_def", **{"def": "BOX"})
    fld = rpc(bridge, "wb_supervisor_node_get_field", node=1,
              name="translation")["result"]["field"]
    assert rpc(bridge, "wb_supervisor_node_get_position",
               node=fld)["error"]["code"] == "BAD_HANDLE_TYPE"


def test_dispatch_survives_a_raising_call(bridge):
    got = rpc(bridge, "wb_supervisor_world_load", filename="x.wbt")
    assert not got["ok"] and got["error"]["code"] == "EXCEPTION"
    assert "boom" in got["error"]["message"]
    # The bridge keeps serving afterwards.
    assert rpc(bridge, "wb_robot_get_time")["result"] == 1.5


def test_dispatch_optional_args_fill_left_to_right(bridge):
    # step with the optional arg omitted...
    assert rpc(bridge, "wb_robot_step")["ok"]
    # ...and a later optional without the earlier one is a structured error.
    got = rpc(bridge, "wb_supervisor_set_label", id=1, label="x", x=0.1,
              y=0.1, size=0.1, color=0xFF0000, font="Arial")
    assert got["error"]["code"] == "MISSING_ARGUMENT"
    assert "transparency" in got["error"]["message"]


def test_quit_is_acknowledged_then_flagged(bridge):
    got = rpc(bridge, "wb_supervisor_simulation_quit", status=0)
    assert got["ok"] and bridge.quit_requested
    assert bridge.robot.quit_status == 0


# ---------------------------------------------------------------------------
# The wire: a live (fake-backed) server, the runner handlers against it
# ---------------------------------------------------------------------------

def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture()
def live_server():
    bridge = bc.Bridge(FakeSupervisor())
    port = _free_port()
    thread = threading.Thread(target=bc.serve,
                              args=(bridge, "127.0.0.1", port), daemon=True)
    thread.start()
    for _ in range(100):
        try:
            socket.create_connection(("127.0.0.1", port),
                                     timeout=0.2).close()
            break
        except OSError:
            time.sleep(0.02)
    yield bridge, port, thread
    if not bridge.quit_requested:
        try:
            wb.call_bridge("127.0.0.1", port,
                           "wb_supervisor_simulation_quit", {"status": 0},
                           5.0)
        except OSError:
            pass
    thread.join(timeout=5.0)


def test_wire_round_trip(live_server):
    _bridge, port, _thread = live_server
    got = wb.call_bridge("127.0.0.1", port, "wb_robot_get_time", {}, 5.0)
    assert got == {"ok": True, "result": 1.5}
    err = wb.call_bridge("127.0.0.1", port, "wb_nope", {}, 5.0)
    assert err["error"]["code"] == "UNKNOWN_FUNCTION"


def test_handler_happy_and_error_paths(tmp_path, live_server):
    _bridge, port, _thread = live_server
    sandbox = _sandbox(tmp_path)
    sandbox.webots_bridge_port = port
    ts = wb.build(sandbox)
    out = ts.get("wb_robot_get_time").handler({}, 10.0)
    assert not out.is_error and out.text == "1.5"
    assert out.data == {"ok": True, "result": 1.5}
    bad = ts.get("wb_supervisor_node_get_position").handler({"node": 99},
                                                            10.0)
    assert bad.is_error and "UNKNOWN_HANDLE" in bad.text


def test_handler_reports_unreachable_bridge_plainly(tmp_path):
    sandbox = _sandbox(tmp_path)
    sandbox.webots_bridge_port = _free_port()   # nothing listening there
    ts = wb.build(sandbox)
    out = ts.get("wb_robot_get_time").handler({}, 2.0)
    assert out.is_error
    assert "cannot reach the Webots bridge controller" in out.text


def test_quit_over_the_wire_stops_the_server(live_server):
    bridge, port, thread = live_server
    got = wb.call_bridge("127.0.0.1", port,
                         "wb_supervisor_simulation_quit", {"status": 0}, 5.0)
    assert got["ok"]
    thread.join(timeout=5.0)
    assert not thread.is_alive()
    assert bridge.quit_requested
