"""Capability-based assessment inventory. Asset presence never means tested support."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
import runpy

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "projects/samples/demos/controllers/omnilink_mobile_bridge/_mobile_configs.py"


@dataclass(frozen=True)
class MobileProfile:
    robot: str
    model: Path
    spawn_height_m: float
    config: dict
    wheel_names: tuple[str, ...]


def profiles():
    module = runpy.run_path(str(CONFIG))
    result = {}
    for name, cfg in module["MOBILE_CONFIGS"].items():
        if cfg.get("kinematic") or cfg["layout"] == "kinematic":
            continue
        world = ROOT / f"projects/samples/demos/worlds/chat/omnilink_{name}.omniworld"
        text = world.read_text(encoding="utf-8")
        stanza = text[text.rfind("URDFRobot {"):]
        model = (world.parent / re.search(r'url\s+"([^"]+)"', stanza)[1]).resolve()
        height = float(re.search(r'translation\s+\S+\s+\S+\s+(\S+)', stanza)[1])
        motors = module["WHEEL_MOTORS"][cfg["layout"]]
        result[name] = MobileProfile(name, model, height, cfg,
                                    tuple(motors["left"] + motors["right"]))
    return result


def get_profile(robot):
    items = profiles()
    if robot not in items:
        raise ValueError(f"UNSUPPORTED: {robot} has no physical-wheel braking adapter; use assess robots for coverage")
    return items[robot]


def inventory():
    supported = {p.model: name for name, p in profiles().items()}
    rows = []
    for path in sorted((ROOT / "projects/robots").rglob("*.urdf")):
        name = supported.get(path.resolve())
        rows.append({"model": path.relative_to(ROOT).as_posix(), "robot": name,
                     "supported_tasks": ["acquire_and_brake"] if name else [],
                     "state": "adapter_available" if name else "asset_only",
                     "reason": "Live qualification is per case, see measured campaign" if name else
                               "No assessment controller/task adapter for this model variant; not a robustness verdict"})
    return rows
