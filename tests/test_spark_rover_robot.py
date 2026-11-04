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

"""SparkRover Mini as a simulated robot (spark_rover/robot.py, its drive-test world and the
spark_rover_drive controller), checked against the design. Engine-free: it reads the
committed world and the controller's model, never launches the simulator.

The drive test itself (speed, spin, arm wave) runs on the engine:
    python -m omnisim run-headless projects/samples/demos/worlds/manufacturing/spark_rover_drive_test.omniworld --duration 60
"""
import importlib.util
import math
import re
import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
PRODUCT = ROOT / "projects/manufacturing/products/spark_rover"
WORLD = ROOT / "projects/samples/demos/worlds/manufacturing/spark_rover_drive_test.omniworld"
CONTROLLER = ROOT / "projects/samples/demos/controllers/spark_rover_drive/spark_rover_drive.py"
sys.path.insert(0, str(PRODUCT))

import chassis  # noqa: E402
import skeleton as sk  # noqa: E402


def _ours(name):
    """spark_rover's own module (desk_robot has a robot.py, an electrical.py and an assemble.py too)."""
    key = f"_spark_rover_test_{name}"
    if key not in sys.modules:
        spec = importlib.util.spec_from_file_location(key, PRODUCT / f"{name}.py")
        mod = importlib.util.module_from_spec(spec)
        sys.modules[key] = mod
        spec.loader.exec_module(mod)
    return sys.modules[key]


rb = _ours("robot")


def _controller():
    """Import the controller without the controller library (it only resolves in a run)."""
    stub = types.ModuleType("omnisim")
    stub.Supervisor = type("Supervisor", (), {})
    saved = sys.modules.get("omnisim")
    sys.modules["omnisim"] = stub
    try:
        spec = importlib.util.spec_from_file_location("_spark_rover_drive_probe", CONTROLLER)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    finally:
        if saved is not None:
            sys.modules["omnisim"] = saved
        else:
            sys.modules.pop("omnisim", None)


@pytest.fixture(scope="module")
def world():
    return WORLD.read_text(encoding="utf-8")


def _args(text):
    m = re.search(r'controllerArgs \[([^\]]*)\]', text)
    return dict(a.strip('"').lstrip("-").split("=", 1) for a in m.group(1).split() if "=" in a)


def test_the_world_uses_the_canonical_lighting_and_extension(world):
    assert WORLD.suffix == ".omniworld"
    for node in ("OmniSimSky {", "DEF SUN OmniSimSun {", "DEF SUN_MARKER OmniSimSunMarker {"):
        assert node in world
    assert 'newtonSolver "mujoco"' in world


def test_the_controller_gets_the_design_s_motor_and_pack(world):
    gm = chassis.BOUGHT["gearmotor"]
    a = _args(world)
    assert float(a["pack-v"]) == _ours("electrical").PACK["v_nom"]
    assert float(a["rated-v"]) == gm["rated_v"]
    assert float(a["no-load-rpm"]) == gm["no_load_rpm"]
    assert float(a["stall-kgcm"]) == gm["stall_kgcm"]
    c = _controller()
    assert (c.NO_LOAD_A, c.STALL_A_RATED) == (gm["no_load_a"], gm["stall_a"])


def test_the_motor_model_brakes_above_no_load_speed():
    c = _controller()
    m = c.Motor(7.2, 12.0, 300.0, 11.0)
    assert m.w0 == pytest.approx(300 * 0.6 * 2 * math.pi / 60)
    assert m.stall == pytest.approx(11.0 * 0.6 * 0.0980665)
    assert m.available_torque(1, 0.0) == pytest.approx(m.stall)
    assert m.available_torque(1, 0.5 * m.w0) == pytest.approx(0.5 * m.stall)
    assert m.available_torque(-1, -1.5 * m.w0) == pytest.approx(0.5 * m.stall)   # a brake, never a free wheel
    assert m.available_torque(0, 3 * m.w0) == pytest.approx(m.stall)
    assert m.current(0.0) == pytest.approx(m.stall_a)
    assert m.current(m.w0) == pytest.approx(m.i0)


def test_the_current_model_is_chassis_operating():
    """robot.current_at at the speed chassis.operating predicts gives chassis.operating's current."""
    gm = chassis.BOUGHT["gearmotor"]
    for v in (6.0, 7.2, 8.4):
        op = chassis.operating(v, 1.2622)
        w = op["rpm"] * 2 * math.pi / 60
        assert rb.current_at(gm, v, w) == pytest.approx(op["a"], abs=2e-3)


def test_the_arm_profile_is_the_design_s_wave():
    c = _controller()
    import arms
    T, s = c.trapezoid(math.radians(arms.WAVE["deg"]))
    assert T == pytest.approx(arms.WAVE["s"], rel=1e-9)
    assert s(T) == pytest.approx(math.radians(arms.WAVE["deg"]))
    assert s(T / 2) == pytest.approx(math.radians(arms.WAVE["deg"]) / 2)
    T2, s2 = c.trapezoid(math.radians(10.0))          # a short move never reaches the peak speed
    assert T2 < T and s2(T2) == pytest.approx(math.radians(10.0))


def test_the_joints_are_the_design_s(world):
    import arms
    for t in rb.WHEEL_TAGS:
        assert f'name "wheel_{t}_motor"' in world and f'name "wheel_{t}_sensor"' in world
    for j in arms.joints():
        lo, hi = j["limits_rad"]
        pat = (rf'name "{j["name"]}" minPosition {rb._f(lo)} maxPosition {rb._f(hi)} '
               rf'maxTorque {rb._f(j["max_torque_nm"])} maxVelocity {rb._f(j["max_velocity_rad_s"])}')
        assert re.search(re.escape(pat).replace(r"\ ", " "), world), j["name"]
        assert j["max_torque_nm"] == arms.SERVO["stall_nm"][5.0]         # the e-manual's 0.93 N m at 5.0 V


def test_the_wheels_are_cylinders_and_the_body_clears_the_floor(world):
    w = chassis.BOUGHT["wheel"]
    assert world.count(f"Cylinder {{ radius {rb._f(w['od'] / 2000)} height {rb._f(w['width'] / 1000)}") == 4
    low = min(lo[2] for lo, _ in rb.base_collision_boxes())
    assert low == sk.GROUND_CLEARANCE > 0.0        # the root collider is 15 mm above the wheels' contact


def test_the_world_carries_the_design_s_mass(world):
    masses = [float(m) for m in re.findall(r"Physics \{ density -1 mass ([0-9.e-]+)", world)]
    assert len(masses) == 1 + 4 + 4                 # base, four wheels, four arm links
    total = sum(masses) * 1000.0
    lo, hi = sk.MASS_TARGET_G
    assert lo <= total <= hi
    assert total == pytest.approx(2397.8, abs=0.5)  # integrate.mass(), 2026-11-04 (the battery at its volume)
    wheels = masses[:4]                             # the file's order: four wheels, the arms, the base last
    assert max(wheels) == pytest.approx(min(wheels)) and masses[-1] == max(masses)


def test_every_mesh_the_world_names_exists(world):
    for url in re.findall(r'url "([^"]+)"', world):
        assert (WORLD.parent / url).resolve().exists(), url


def test_the_assembly_path_ends_on_the_seat():
    assemble = _ours("assemble")
    u = assemble.Unit("m", ["x"], [((0.0, 0.0, -1.0), 40.0), ((0.0, -1.0, 0.0), 17.0)])
    pts = assemble.path_points(u)
    assert pts[0][0].tolist() == [0.0, -17.0, -40.0] and pts[0][1] == pytest.approx(57.0)
    assert pts[-1][0].tolist() == [0.0, 0.0, 0.0] and pts[-1][1] == 0.0
    togo = [p[1] for p in pts]
    assert all(a >= b for a, b in zip(togo, togo[1:]))


# ---- the face's devices: U9's camera and U8's screen --------------------------------------------

def _node(world, kind):
    m = re.search(kind + r' \{ name "[^"]+" (.*?) \}(?: \] \})?\n', world)
    assert m, kind
    return m.group(0)


def _vec(text, field):
    return [float(v) for v in re.search(field + r" ([-0-9.e]+) ([-0-9.e]+) ([-0-9.e]+)", text).groups()]


def test_the_head_camera_is_the_camera_module_3(world):
    el = _ours("electrical")
    face = rb.face_specs(el)
    cam = _node(world, "Camera")
    assert 'name "camera"' in cam
    hfov, vfov = el.CAMERA_MOD["fov_deg"]
    assert (hfov, vfov) == (66.0, 41.0)                      # the CM3 product brief, standard lens
    assert float(re.search(r"fieldOfView ([0-9.]+)", cam).group(1)) == pytest.approx(math.radians(hfov), abs=1e-6)
    w, h = (int(re.search(f"{k} ([0-9]+)", cam).group(1)) for k in ("width", "height"))
    assert (w, h) == (640, 360)
    sw, sh = rb.CAMERA_SENSOR_PX
    assert w * sh == h * sw                                  # the IMX708's 16:9
    v = math.degrees(2 * math.atan(math.tan(math.radians(hfov) / 2) * h / w))
    assert abs(v - vfov) < 1.0                               # 40.1 deg against the brief's 41
    # at U9's lens, on the skeleton's camera axis, looking +x (the frame the firmware delivers upright)
    lens = _vec(cam, "translation")
    assert lens == pytest.approx([c * 1e-3 for c in face["camera"]["lens_mm"]], abs=1e-6)
    assert face["camera"]["lens_mm"][1:] == pytest.approx(list(sk.CAMERA["lens_at"][1:]), abs=1e-6)
    assert face["camera"]["axis"] == pytest.approx(list(sk.CAMERA["axis"]))
    assert face["camera"]["board_v"] == pytest.approx([0.0, 0.0, -1.0])     # mounted FPC edge down
    assert "rotation" not in cam
    assert face["camera"]["lens_mm"][0] < sk.UPPER_BODY.x1                  # behind the face cover
    near = float(re.search(r"near ([0-9.]+)", cam).group(1))
    assert sk.UPPER_BODY.x1 - face["camera"]["lens_mm"][0] < near * 1000 < 100.0   # past the cover, short of focus


def test_the_screen_is_the_5_inch_panel_in_the_face_window(world):
    el = _ours("electrical")
    shell = _ours("shell")
    face = rb.face_specs(el)
    disp = _node(world, "Display")
    assert 'name "display"' in disp
    assert re.search(r"width 800 height 480 ", disp)
    aw, ah = el.DISPLAY["active_mm"]
    assert (aw / 800, ah / 480) == pytest.approx((0.135, 0.135))           # square pixels: Waveshare's area
    pts = [[float(v) for v in p.split()] for p in re.search(r"point \[ ([^\]]+) \]", disp).group(1).split(",")]
    ys, zs = [p[1] for p in pts], [p[2] for p in pts]
    assert (max(ys) - min(ys), max(zs) - min(zs)) == pytest.approx((aw * 1e-3, ah * 1e-3), abs=1e-9)
    c = _vec(disp, "translation")
    fw = sk.FACE_WINDOW
    assert c[1:] == pytest.approx([fw["centre"][1] * 1e-3, fw["centre"][2] * 1e-3], abs=1e-6)
    assert aw <= fw["size"][0] and ah <= fw["size"][1]
    module_front = el.placements()["U8"].origin[0] + el.DISPLAY["outline_mm"][2]
    cover_inside = sk.UPPER_BODY.x1 - shell.Shell().cover_t
    assert module_front < c[0] * 1000 < cover_inside                       # on the panel, behind the cover
    assert "filtering 0" in disp and "baseColorMap ImageTexture" in disp


def test_the_controller_runs_the_face_at_a_modest_rate():
    c = _controller()
    assert 50 <= c.CAMERA_MS <= 200 and c.FACE_MS >= 50
    assert c.mood_of("straight") == "driving" and c.mood_of("spin") == "spinning"
    assert c.mood_of("move_3") == "waving" and c.mood_of("wave_7") == "waving"
    assert 640 // c.INSET <= 200                                            # the inset fits the screen's corner


def test_the_camera_target_is_visual_only(world):
    t = re.search(r"DEF TARGET Solid \{.*\n", world).group(0)
    assert "boundingObject" not in t and "physics" not in t
