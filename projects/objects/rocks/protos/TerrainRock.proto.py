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

"""TerrainRock — smooth-collision rock for rough-terrain robot demos.

Authored via :mod:`omnisim.protogen`. Run
``python -m omnisim proto build projects/objects/rocks`` to regenerate
:file:`TerrainRock.proto`. This file is the source of truth.

Why a separate PROTO instead of editing Rock.proto:
* Rock.proto uses a 21-vertex IndexedFaceSet as both visual and bounding,
  which is fine for static scenery but produces sharp-edged contacts
  that catch robot wheels and produce yaw drift on every climb.
* TerrainRock keeps a low-poly visual (sphere subdivision 2-3) but uses
  a smooth sphere as boundingObject, so wheels roll over the rock
  predictably. The visual is intentionally simple — for terrain demos
  the rock is something to drive over, not something to admire.
"""

from omnisim.protogen import emit


emit(
    name="TerrainRock",
    license="Apache License 2.0",
    license_url="https://www.apache.org/licenses/LICENSE-2.0",
    documentation_url=(
        "https://github.com/omnilink-tech/omnisim/blob/main/"
        "projects/objects/rocks/protos/TerrainRock.proto.py"
    ),
    keywords=["exterior/obstacle", "terrain"],
    description=(
        "Smooth-collision rock for rough-terrain robot demos. Sphere "
        "boundingObject so wheels can roll over cleanly; the visual is "
        "a slightly less detailed sphere with a configurable color."
    ),
    fields=[
        ("SFVec3f", "translation", [0, 0, 0], "Is `Solid.translation`."),
        ("SFRotation", "rotation", [0, 0, 1, 0], "Is `Solid.rotation`."),
        ("SFString", "name", "terrain_rock", "Is `Solid.name`."),
        ("SFFloat", "radius", 0.15,
         "Sphere radius (visual + bounding). Default ~15 cm — climbable for a Husky."),
        ("SFColor", "color", [0.45, 0.4, 0.35],
         "Base color of the rock surface (PBR baseColor)."),
        ("SFFloat", "density", 2500.0,
         "Physics density in kg/m^3 (granite ~2500)."),
        ("SFBool", "locked", True,
         "Is `Solid.locked`. Default TRUE — these are terrain features, "
         "not props; we don't want a wheeled robot punting them around "
         "like billiard balls when the goal is to climb over them. "
         "Override to FALSE only if you specifically want loose rocks."),
    ],
    body="""
        Solid {
          translation IS translation
          rotation IS rotation
          name IS name
          model "terrain_rock"
          children [
            Shape {
              appearance PBRAppearance {
                baseColor IS color
                roughness 1
                metalness 0
              }
              geometry Sphere {
                radius IS radius
                subdivision 3
              }
            }
          ]
          boundingObject Sphere {
            radius IS radius
            subdivision 2
          }
          physics Physics {
            density IS density
          }
          locked IS locked
        }
    """,
)
