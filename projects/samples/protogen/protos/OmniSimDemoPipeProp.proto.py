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

"""Python-authored prop with an inline boundingObject (OmniSim protogen).

The collision geometry is computed at *author* time by
:func:`omnisim.protogen.bounding.pipe`, so the resulting ``.proto`` file
ships a literal ``Group { children [ Box ... ] }`` block. No
``PipeBoundingObject`` EXTERNPROTO indirection, no JavaScript template
re-evaluated at world load.

Regenerate with ``python -m omnisim proto build``.
"""

from omnisim.protogen import emit
from omnisim.protogen.bounding import pipe


_BOUNDING = pipe(height=1.0, radius=0.25, thickness=0.02, subdivision=24)


emit(
    name="OmniSimDemoPipeProp",
    license="Apache License 2.0",
    license_url="https://www.apache.org/licenses/LICENSE-2.0",
    documentation_url=(
        "https://github.com/omnilink-tech/omnisim/blob/main/"
        "projects/samples/protogen/protos/OmniSimDemoPipeProp.proto.py"
    ),
    keywords=["object/primitive"],
    description=(
        "Hollow pipe prop with an author-time boundingObject (no JS "
        "template, no EXTERNPROTO trampoline). Useful as a reference "
        "for the protogen.bounding helpers."
    ),
    fields=[
        ("SFVec3f", "translation", [0, 0, 0], "Pose translation."),
        ("SFRotation", "rotation", [0, 0, 1, 0], "Pose rotation."),
        ("SFString", "name", "pipe prop", "Solid name."),
    ],
    body=f"""
        Solid {{
          translation IS translation
          rotation IS rotation
          name IS name
          children [
            Shape {{
              appearance Appearance {{ material Material {{ }} }}
              geometry Cylinder {{ height 1 radius 0.25 }}
            }}
          ]
          boundingObject {_BOUNDING}
        }}
    """,
)
