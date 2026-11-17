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

"""Fixed-price mobility task definitions; a profile is not a qualified release."""
from __future__ import annotations

from dataclasses import dataclass, replace
from .registry import ROOT, get_profile
from .schema import BrakingSpec

VERSION = "1.1.0"
SLOPES = (0., 5., 10., 20.)
FRICTIONS = (.1, .2, .4, .8)
# Payloads are experimental rigid fixtures, never vendor-rated capacities.
DEFINITIONS = {
    "husky": ("Husky", "Clearpath Robotics", (0.,10.,20.)),
    "jackal": ("Jackal", "Clearpath Robotics", (0.,2.,5.)),
    "rosbot": ("ROSbot", "Husarion", (0.,.5,1.)),
    "rosbot_xl": ("ROSbot XL", "Husarion", (0.,1.,2.)),
    "tb3_burger": ("TurtleBot3 Burger", "ROBOTIS", (0.,.1,.25)),
    "tb3_waffle": ("TurtleBot3 Waffle", "ROBOTIS", (0.,.25,.5)),
    "tb3_waffle_pi": ("TurtleBot3 Waffle Pi", "ROBOTIS", (0.,.25,.5)),
}


@dataclass(frozen=True)
class PackageProfile:
    robot: str
    label: str
    manufacturer: str
    payloads: tuple[float, ...]
    speed: float
    height: float
    stop_distance: float
    model: str
    notices: tuple[str, ...]

    @property
    def sku(self):
        return self.robot.upper().replace("_", "-") + "-BRAKING-100"

    @property
    def slug(self):
        return self.robot.replace("_", "-") + "-braking"

    def spec(self, **changes):
        return replace(BrakingSpec(schema_version=2,robot=self.robot,
                       target_speed_m_s=self.speed,payload_height_m=self.height,
                       max_stop_distance_m=self.stop_distance),**changes)

    def to_dict(self):
        return {"schema_version":1,"robot":self.robot,"label":self.label,
                "manufacturer":self.manufacturer,"sku":self.sku,"version":VERSION,
                "price_usd":100,"task":"acquire_and_brake","model":self.model,
                "model_notices":list(self.notices),"slopes_deg":list(SLOPES),
                "contact_mu":list(FRICTIONS),"payloads_kg":list(self.payloads),
                "base_spec":self.spec().to_dict(),"grid_episodes":48,
                "refinement_episodes":12,"confirmation_episodes":4,"total_episodes":64,
                "speed_rule":"70% of registered wheel command envelope, capped at 0.6 m/s",
                "distance_rule":"target_speed^2 / (2 * 0.72 m/s^2), capped at 0.25 m; experimental",
                "qualification":"Definition only; requires valid campaign and extracted-delivery acceptance",
                "controller":"OmniSim reference wheel driver; not manufacturer production firmware"}


def get_package_profile(robot):
    if robot not in DEFINITIONS:
        raise ValueError(f"UNSUPPORTED package robot: {robot}; only registered wheeled task profiles are available")
    p=get_profile(robot)
    label,manufacturer,payloads=DEFINITIONS[robot]
    speed=min(.6,.7*p.config["wheel_radius_m"]*p.config["max_wheel_speed_radps"])
    height=.35 if robot == "husky" else max(.12,p.config["wheel_radius_m"]*2.3)
    model_root=p.model.parent.parent
    notices=[q.relative_to(ROOT).as_posix() for q in model_root.glob("*")
             if q.is_file() and any(token in q.name.upper() for token in ("LICENSE","NOTICE","PROVENANCE"))]
    parent_notice=model_root.parent/"PROVENANCE.md"
    if parent_notice.is_file(): notices.append(parent_notice.relative_to(ROOT).as_posix())
    if not any("LICENSE" in n.upper() for n in notices):
        raise ValueError(f"Missing upstream model license for {robot}")
    return PackageProfile(robot,label,manufacturer,payloads,speed,height,
                          min(.25,speed*speed/(2*.72)),p.model.relative_to(ROOT).as_posix(),tuple(sorted(set(notices))))


def qualify(robot, directory, workers=2):
    from .campaign import sweep
    from .runner import save
    profile=get_package_profile(robot)
    state=sweep(directory,[robot],list(SLOPES),list(FRICTIONS),list(profile.payloads),
                12,2,workers,64,template=profile.spec())
    state["package_profile"]=profile.to_dict()
    save(directory/"campaign.json",state)
    return state


def render_document(text, profile):
    fmt=lambda values:"/".join(f"{v:g}" for v in values)
    values={"LABEL":profile.label,"ROBOT":profile.robot,"SKU":profile.sku,"VERSION":VERSION,
            "MANUFACTURER":profile.manufacturer,"SPEED":f"{profile.speed:.6g}",
            "DISTANCE":f"{profile.stop_distance:.6g}","HEIGHT":f"{profile.height:.6g}",
            "PAYLOADS":fmt(profile.payloads),"SLOPES":fmt(SLOPES),"FRICTIONS":fmt(FRICTIONS),
            "MODEL":profile.model,"NOTICES":"\n".join(f"- {n}" for n in profile.notices)}
    for key,value in values.items(): text=text.replace("{{"+key+"}}",value)
    if "{{" in text: raise ValueError("Unresolved customer document placeholder")
    return text
