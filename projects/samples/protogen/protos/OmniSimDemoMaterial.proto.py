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

"""Python-authored PROTO demo (OmniSim protogen).

Run ``python -m omnisim proto build projects/samples/protogen`` to
regenerate :file:`OmniSimDemoMaterial.proto` from this source. This file
is the source of truth; the ``.proto`` is a derived artifact.

Compared with hand-writing the ``.proto``:
* The IDE flags type errors in defaults immediately.
* The body is normal indented text (no comment-embedded JS gymnastics
  for static cases).
"""

from omnisim.protogen import emit


emit(
    name="OmniSimDemoMaterial",
    license="Apache License 2.0",
    license_url="https://www.apache.org/licenses/LICENSE-2.0",
    documentation_url=(
        "https://github.com/omnilink-tech/omnisim/blob/main/"
        "projects/samples/protogen/protos/OmniSimDemoMaterial.proto.py"
    ),
    keywords=["appearance/mineral"],
    description=(
        "OmniSim protogen demo: a PBR material parameterized by base "
        "color, roughness, and an optional texture transform."
    ),
    fields=[
        ("SFColor", "baseColor", [0.8, 0.8, 0.8],
         "Base color of the material."),
        ("SFFloat", "roughness", 0.5,
         "PBR roughness in [0, 1]."),
        ("SFFloat", "metalness", 0.0,
         "PBR metalness in [0, 1]."),
        ("SFNode", "textureTransform", None,
         "Optional 2D texture transform."),
        ("SFFloat", "IBLStrength", 1.0,
         "Strength of ambient lighting from the Background node."),
    ],
    body="""
        PBRAppearance {
          baseColor IS baseColor
          roughness IS roughness
          metalness IS metalness
          textureTransform IS textureTransform
          IBLStrength IS IBLStrength
        }
    """,
)
