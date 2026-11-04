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

"""Engine-free tests for OmniSim Manufacturing's shared core and its command."""
import importlib.util
from pathlib import Path
import re
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
SECTOR = ROOT / "projects/manufacturing"
sys.path.insert(0, str(SECTOR / "core"))

import drives  # noqa: E402
import worldkit  # noqa: E402


def _cli():
    spec = importlib.util.spec_from_file_location("manufacturing_cli", SECTOR / "cli.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_effective_inertia_explains_both_machines():
    # The LongMill (CNC): 12.5 kg gantry, 20 kN drives, 2 ms substeps -> raised to 20 kg.
    assert drives.accel_feedforward(12.5, 20000, 0.002) == pytest.approx(1.0e-4)   # its hand-tuned value
    # The SV08 (printer): a 0.65 kg toolhead at 1 ms substeps moves like 10 kg.
    assert drives.accel_feedforward(0.65, 20000, 0.001) == pytest.approx(5.0e-5)
    # Heavier than the bound, the mass itself counts.
    assert drives.accel_feedforward(40.0, 20000, 0.001) == pytest.approx(40.0 / 200000.0)


def test_fed_forward_drives_reproduce_the_controllers_arithmetic():
    """The formula grbl_twin and fdm_twin each carried before core/drives.py."""
    lead, ff, dt = 0.00095, (1e-4, 1e-4, 1e-4), 0.008
    d = drives.FedForwardDrives(3, lead, ff)
    prev, cur, nxt, after = [0.0, 1.0, 5.0], [0.4, 1.0, 5.0], [1.0, 1.0, 5.0], [1.8, 1.0, 5.0]
    measured = [0.39, 0.998, 5.0]
    out = d.targets(prev, cur, nxt, after, measured, dt)
    for i in range(3):
        v = (nxt[i] - cur[i]) / dt
        a = ((after[i] - nxt[i]) - (cur[i] - prev[i])) / (2 * dt * dt)
        trim = 0.0
        if abs(v) < 0.05 and abs(cur[i] - prev[i]) < 0.05 * dt:
            trim = 4.0 * (cur[i] - measured[i]) * dt
        assert out[i] == pytest.approx(0.5 * (cur[i] + nxt[i]) + (0.05 - lead) * v + ff[i] * a + trim)
    assert d.trim[0] == 0.0 and d.trim[1] == pytest.approx(4.0 * 0.002 * dt)   # only the axis at rest trims


def test_the_trim_is_clamped_and_holds_while_moving():
    d = drives.FedForwardDrives(1, 0.0, (0.0,))
    for _ in range(100000):
        d.targets([0.0], [0.0], [0.0], [0.0], [-50.0], 0.01)           # a large static sag
    assert d.trim[0] == pytest.approx(drives.TRIM_LIMIT_MM)
    held = d.trim[0]
    d.targets([0.0], [1.0], [2.0], [3.0], [0.0], 0.01)                  # moving: no integration
    assert d.trim[0] == held


def test_worldkit_numbers_and_camera():
    assert worldkit.num(0.0) == "0" and worldkit.num(0.12345678) == "0.1234568" and worldkit.num(2.0) == "2"
    assert worldkit.look_at((1, 0, 0), (0, 0, 0)).endswith("3.1415927")   # looking back along -X: half a turn
    assert worldkit.box((0, 0, 0), (1, 2, 3), ".5 .5 .5").count("Box { size 1 2 3 }") == 1


def test_every_machine_folder_has_a_config_and_a_licenced_cad_pack():
    machines = {m["machine"]: m for m in _cli()._machines()}
    assert {"longmill_mk2_30x30", "example_3018", "sovol_sv08"} <= set(machines)
    assert machines["sovol_sv08"]["process"] == "printing" and machines["longmill_mk2_30x30"]["process"] == "cnc"
    assert machines["longmill_mk2_30x30"]["lasers"] == ["sienci_laserbeam_7w.json"]
    for m in machines.values():
        assert m["configs"], m
        if m["cad"]:
            cad = ROOT / m["cad"]
            assert (cad / "PROVENANCE.md").exists() and m["cad_licence"], m
    assert "GPL-3.0" in machines["sovol_sv08"]["cad_licence"] and (ROOT / machines["sovol_sv08"]["cad"] / "LICENSE").exists()


def test_every_manufacturing_world_names_files_that_exist():
    worlds = sorted(w for w in (ROOT / "projects/samples/demos/worlds/manufacturing").glob("*.omniworld")
                    if not w.name.startswith("."))   # the harness's hidden working copies
    assert len(worlds) >= 9
    for w in worlds:
        text = w.read_text(encoding="utf-8")
        for ref in re.findall(r'"(projects/[^"]+\.(?:grbl|json|gcode))"', text):
            assert (ROOT / ref).exists(), f"{w.name}: {ref}"
        for url in set(re.findall(r'url "([^"]+)"', text)):
            assert (w.parent / url).resolve().exists(), f"{w.name}: {url}"


def test_the_command_routes_to_a_process_tool():
    out = subprocess.run([sys.executable, "-m", "omnisim", "manufacturing", "printing", "printer", "show",
                          str(SECTOR / "machines/sovol_sv08/sovol_sv08.json")],
                         cwd=ROOT, capture_output=True, text=True, timeout=120)
    assert out.returncode == 0 and '"kinematics": "corexy"' in out.stdout
    bad = subprocess.run([sys.executable, str(SECTOR / "cli.py"), "printing", "nope"], capture_output=True, text=True)
    assert bad.returncode == 2 and "slice" in bad.stderr
