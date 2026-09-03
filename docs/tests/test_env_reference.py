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
"""docs/reference/environment-variables.md is GENERATED and must not drift.

Two gates:

1. Regenerating the page with scripts/dev/gen_env_reference.py must reproduce the
   committed page byte-for-byte (line endings normalised, so an autocrlf checkout
   still passes). Any new, removed or re-parsed ``OMNISIM_*`` read in the tree --
   or a moved line number -- goes red until the page is regenerated.
2. Every variable on the page carries a non-empty Description, or is named in
   NO_DESCRIPTION_TODO below. That list is the debt: a variable whose read site
   gains a comment must be removed from it, so the count only ever shrinks.
"""
import difflib
import importlib.util
import os
import re
import unittest

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, os.pardir))
GENERATOR = os.path.join(REPO_ROOT, "scripts", "dev", "gen_env_reference.py")
PAGE = os.path.join(REPO_ROOT, "docs", "reference", "environment-variables.md")
REGENERATE = "python scripts/dev/gen_env_reference.py"

# TODO: read sites with no comment to harvest. Add a comment above the read (or a
# trailing one) and remove the name here. The count is reported by
# test_every_variable_is_described_or_allowlisted.
NO_DESCRIPTION_TODO = frozenset({
    "OMNISIM_AGENT_BUILD_VOICE_RUNTIME",
    "OMNISIM_ALLOWED_ORIGINS",
    "OMNISIM_BATCH_N",
    "OMNISIM_BH2_PROBE_OUT",
    "OMNISIM_BUILD_JOBS",
    "OMNISIM_CAMERA_LIGHTDEPTH",
    "OMNISIM_CAPTURE_BACKEND",
    "OMNISIM_CAPTURE_DISABLE_CAMERA",
    "OMNISIM_CAPTURE_SUPERVISOR_HOST",
    "OMNISIM_CAPTURE_SUPERVISOR_PORT",
    "OMNISIM_CAPTURE_SUPERVISOR_PORT_OVERRIDE",
    "OMNISIM_CLOTH_CONTACT_MARGIN",
    "OMNISIM_CLOTH_FRICTION_EPSILON",
    "OMNISIM_CLOTH_FULL_SURFACE_CONTACT",
    "OMNISIM_CLOTH_JOINT_ANGULAR_KE",
    "OMNISIM_CLOTH_JOINT_LINEAR_KE",
    "OMNISIM_CLOTH_PROXY_ITERATIONS",
    "OMNISIM_CLOTH_SELF_CONTACT_MARGIN",
    "OMNISIM_CLOTH_TELEMETRY_EVERY",
    "OMNISIM_CLOTH_TELEMETRY_FULL",
    "OMNISIM_CLOTH_VBD_ITERATIONS",
    "OMNISIM_CONTROLLER_URL",
    "OMNISIM_DAMAGE_FORCE_DETACH_T",
    "OMNISIM_DECODED_TEXTURE_CACHE_MB",
    "OMNISIM_DEPLOY_DBG",
    "OMNISIM_DEPLOY_LOOP",
    "OMNISIM_DISTRO_EXE",
    "OMNISIM_DONE_GRACE",
    "OMNISIM_DONE_LOG",
    "OMNISIM_EXTRA_MAKE_ARGS",
    "OMNISIM_G1PRIM_FOOT",
    "OMNISIM_HARNESS_DAMAGE_ROBOT",
    "OMNISIM_HARNESS_ENGINE_MODE",
    "OMNISIM_HARNESS_SUPERVISOR_HOST",
    "OMNISIM_HARNESS_SUPERVISOR_PORT",
    "OMNISIM_HARNESS_URL",
    "OMNISIM_HIL_STEP_MS",
    "OMNISIM_INTERCEPT_CAMERA_PROFILE",
    "OMNISIM_INTERCEPT_CAPTURE_ACCELERATION",
    "OMNISIM_INTERCEPT_CAPTURE_DIR",
    "OMNISIM_INTERCEPT_CAPTURE_END_S",
    "OMNISIM_INTERCEPT_CAPTURE_FPS",
    "OMNISIM_INTERCEPT_CAPTURE_START_S",
    "OMNISIM_INTERCEPT_FILM_AUTOQUIT",
    "OMNISIM_INTERCEPT_MODE",
    "OMNISIM_INTERCEPT_OBSERVER_OUT",
    "OMNISIM_INTERCEPT_RESULT",
    "OMNISIM_INTERCEPT_X",
    "OMNISIM_INVOKED_AS",
    "OMNISIM_IPC_HANDSHAKE_TIMEOUT_MS",
    "OMNISIM_KIN_PROBE_OUT",
    "OMNISIM_KIN_SCENARIO",
    "OMNISIM_LIB_PATH",
    "OMNISIM_LIGHTSENSOR_PROBE_OUT",
    "OMNISIM_MAX_REQUEST_BYTES",
    "OMNISIM_MAZE_AUTO_QUIT",
    "OMNISIM_MAZE_CAMERA_MODE",
    "OMNISIM_MAZE_DIRECT_CAMERA",
    "OMNISIM_MAZE_FINISH_HOLD_S",
    "OMNISIM_MAZE_FRAME_ACCELERATION",
    "OMNISIM_MAZE_FRAME_DIR",
    "OMNISIM_MAZE_FRAME_IDLE_ACCELERATION",
    "OMNISIM_MAZE_FRAME_START_S",
    "OMNISIM_MAZE_MAX_HOPS",
    "OMNISIM_MAZE_MODE",
    "OMNISIM_MAZE_MOVIE",
    "OMNISIM_MAZE_MOVIE_ACCELERATION",
    "OMNISIM_MAZE_RESULT",
    "OMNISIM_MAZE_START_DELAY_S",
    "OMNISIM_MAZE_STORY_WIDE_END_S",
    "OMNISIM_MPM_MAX_ACTIVE_CELLS",
    "OMNISIM_NEWTON_ASYNC_PRELOAD",
    "OMNISIM_NEWTON_BRIDGE_CONTACT_PROPERTIES",
    "OMNISIM_NEWTON_CONE",
    "OMNISIM_NEWTON_CONTACT_FRICTION_PROBE_OUT",
    "OMNISIM_NEWTON_LS_ITERS",
    "OMNISIM_NEWTON_MIDPHASE",
    "OMNISIM_NEWTON_NO_EFFORT_LIMIT",
    "OMNISIM_NEWTON_NO_SUBSTEP_BATCH",
    "OMNISIM_NEWTON_PRELOAD_PROFILE",
    "OMNISIM_NEWTON_SOFT_KD",
    "OMNISIM_NEWTON_TARGET_KD",
    "OMNISIM_NEWTON_VEL_TRACE",
    "OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE",
    "OMNISIM_OLLAMA",
    "OMNISIM_ORACLE_OUT",
    "OMNISIM_ORACLE_STEPS",
    "OMNISIM_PARTICLE_POOL_HOST",
    "OMNISIM_PARTICLE_POOL_PORT",
    "OMNISIM_PARTICLE_POOL_SIZE",
    "OMNISIM_PROBE_SOAK_ITERS",
    "OMNISIM_PROBE_SOAK_RES",
    "OMNISIM_PROBE_TRAJ_MS",
    "OMNISIM_PROBE_WGPU",
    "OMNISIM_RADAR_PROBE_OUT",
    "OMNISIM_RAYCAST_PROBE_OUT",
    "OMNISIM_RECEIVER_PROBE_OUT",
    "OMNISIM_RELOAD_PROFILE",
    "OMNISIM_RENDER_BACKEND",
    "OMNISIM_REQUIRE_NEWTON_BUNDLE",
    "OMNISIM_ROBOT_NAME",
    "OMNISIM_ROBOT_SPEC",
    "OMNISIM_ROLL_BINARY",
    "OMNISIM_ROLL_OMEGA",
    "OMNISIM_ROS2_TIMEOUT",
    "OMNISIM_RUNAWAY_OUT",
    "OMNISIM_SKY_BENCH_KEEP",
    "OMNISIM_SKY_RENDER_OUTPUT",
    "OMNISIM_SKY_RENDER_SETTLE",
    "OMNISIM_STRUCTURED_LOG",
    "OMNISIM_TEST_RESULT_FILE",
    "OMNISIM_TOUCH_PROBE_OUT",
    "OMNISIM_TRIANGLE_MESH_CACHE_SIZE",
    "OMNISIM_URDF_DEBUG",
    "OMNISIM_USAGE_SNAPSHOT",
    "OMNISIM_VX",
    "OMNISIM_VY",
    "OMNISIM_WELD_PROBE_OUT",
    "OMNISIM_WGPU_MUSCLE",
    "OMNISIM_WGPU_NO_SHADOW",
    "OMNISIM_WGPU_PROBE_DIR",
    "OMNISIM_WGPU_SENSOR_DRAW_CACHE",
    "OMNISIM_WGPU_SHADOW_DEBUG",
    "OMNISIM_WGPU_SSR_STRENGTH",
    "OMNISIM_WGPU_SYNTH_EVERY",
    "OMNISIM_WGPU_VOL_DENSITY",
    "OMNISIM_WITH_NEWTON",
    "OMNISIM_WZ",
})

ROW_RE = re.compile(r"^\| `(OMNISIM_[A-Z0-9_]+)`")
SUMMARY_RE = re.compile(r"^\*\*Summary:\*\* (\d+) variables, (\d+) presence-gated, (\d+) mixed, (\d+) documented nowhere")


def load_generator():
    spec = importlib.util.spec_from_file_location("gen_env_reference", GENERATOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def read_page():
    with open(PAGE, "rb") as f:
        return f.read().decode("utf-8").replace("\r\n", "\n")


def parse_rows(text):
    """Return {name: (kind, read_at, description)} from every table row on the page."""
    rows = {}
    for line in text.split("\n"):
        m = ROW_RE.match(line)
        if not m:
            continue
        cells = [c.strip() for c in re.split(r"(?<!\\)\|", line)]
        # ['', variable, kind, read_at, description, '']
        assert len(cells) == 6, "unexpected table row shape: %r" % line
        rows[m.group(1)] = (cells[2], cells[3], cells[4])
    return rows


class TestEnvReference(unittest.TestCase):
    """The generated environment-variable reference is in sync with the source tree."""

    def test_page_exists_and_is_marked_generated(self):
        self.assertTrue(os.path.isfile(PAGE), msg="%s is missing; run: %s" % (PAGE, REGENERATE))
        self.assertIn("GENERATED FILE", read_page().split("\n", 4)[2])

    def test_page_matches_a_fresh_generation(self):
        """Byte-identical (modulo CRLF) to what the generator produces from the tree right now."""
        expected = load_generator().generate()
        actual = read_page()
        if expected != actual:
            diff = list(difflib.unified_diff(actual.split("\n"), expected.split("\n"),
                                             "committed environment-variables.md", "regenerated", lineterm=""))
            head = "\n".join(diff[:60])
            if len(diff) > 60:
                head += "\n... (%d more diff lines)" % (len(diff) - 60)
            self.fail("docs/reference/environment-variables.md has drifted from the source tree.\n"
                      "Regenerate it: %s\n%s" % (REGENERATE, head))

    def test_summary_counts_match_the_tables(self):
        text = read_page()
        rows = parse_rows(text)
        summary = next((SUMMARY_RE.match(ln) for ln in text.split("\n") if SUMMARY_RE.match(ln)), None)
        self.assertIsNotNone(summary, msg="summary line missing from the page")
        total, presence, mixed, _undocumented = (int(g) for g in summary.groups())
        self.assertEqual(total, len(rows))
        self.assertEqual(presence, sum(1 for kind, _r, _d in rows.values() if kind == "presence"))
        self.assertEqual(mixed, sum(1 for kind, _r, _d in rows.values() if kind.startswith("mixed")))

    def test_every_variable_is_described_or_allowlisted(self):
        rows = parse_rows(read_page())
        self.assertGreater(len(rows), 0)
        undescribed = sorted(n for n, (_k, _r, desc) in rows.items() if not desc)
        missing = [n for n in undescribed if n not in NO_DESCRIPTION_TODO]
        self.assertEqual(
            missing, [],
            msg="%d variable(s) have no Description and are not in NO_DESCRIPTION_TODO -- add a comment "
                "above the read site (preferred) or list them: %s" % (len(missing), ", ".join(missing)))
        stale = sorted(n for n in NO_DESCRIPTION_TODO if n not in undescribed)
        self.assertEqual(
            stale, [],
            msg="%d NO_DESCRIPTION_TODO entries are now described (or no longer read); remove them: %s"
                % (len(stale), ", ".join(stale)))
        print("TODO: %d of %d environment variables still have no harvested description"
              % (len(undescribed), len(rows)))


if __name__ == "__main__":
    unittest.main()
