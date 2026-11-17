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

"""OmniSim PROTO generation API.

Author PROTOs in Python instead of VRML-with-embedded-JavaScript. A
``Foo.proto.py`` source declares the header, fields, and body via the
:func:`emit` builder. The transpiler renders a deterministic
``Foo.proto`` next to it.

Why bother:

* Field declarations are normal Python literals — your IDE flags type
  mistakes immediately instead of at world-load time.
* The body can be assembled with Python control flow (loops, helpers,
  shared snippets) rather than VRML's procedural-template comment syntax.
* For procedural shapes, the :mod:`omnisim.protogen.bounding` helpers
  emit collision geometry at author time, avoiding the upstream "PROTO
  built from a Group of Boxes" trampoline pattern.

Example::

    from omnisim.protogen import emit

    emit(
        name="MyMaterial",
        license="Apache License 2.0",
        license_url="https://www.apache.org/licenses/LICENSE-2.0",
        keywords=["appearance/mineral"],
        description="A simple material with an overridable color.",
        fields=[
            ("SFColor", "colorOverride", [1, 1, 1],
             "Defines the color to be multiplied with the texture color."),
            ("SFNode", "textureTransform", None,
             "Defines an optional 2d texture transform."),
        ],
        body=\"\"\"
            PBRAppearance {
              baseColor IS colorOverride
              metalness 0
              textureTransform IS textureTransform
            }
        \"\"\",
    )
"""

from __future__ import annotations

from .api import emit, FieldSpec, ProtoSpec, render

__all__ = ["emit", "FieldSpec", "ProtoSpec", "render"]
