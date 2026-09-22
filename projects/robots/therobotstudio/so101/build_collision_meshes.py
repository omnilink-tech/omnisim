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

"""Rebuild jaw collision slices from the attributed upstream STL meshes.

Requires numpy, scipy and trimesh. A plane splits source triangle edges, then
each band is convex-hulled. This preserves the tapered finger rather than
filling the entire jaw's concavity with a single hull. These are approximations,
not a material or contact calibration. Visual meshes are never changed.
Run from any working directory; existing URDF replacements are left intact.
"""
from pathlib import Path
import json
import numpy as np
import trimesh

root=Path(__file__).resolve().parent
out=root/'assets'/'collision';out.mkdir(exist_ok=True)
report={}
for name,axis,cuts in [('moving_jaw_so101_v1',1,[-.06,-.025]),('wrist_roll_follower_so101_v1',2,[.04,.065,.085])]:
    source=trimesh.load_mesh(root/'assets'/f'{name}.stl'); edges=[-1,*cuts,1];parts=[]
    for i,(lo,hi) in enumerate(zip(edges,edges[1:])):
        v=source.vertices; ends=v[source.edges_unique]
        points=[v[(v[:,axis]>=lo)&(v[:,axis]<=hi)]]
        for cut in [lo,hi]:
            mask=(ends[:,0,axis]<cut)&(ends[:,1,axis]>cut)|(ends[:,1,axis]<cut)&(ends[:,0,axis]>cut)
            e=ends[mask]
            t=(cut-e[:,0,axis])/(e[:,1,axis]-e[:,0,axis])
            points.append(e[:,0]+t[:,None]*(e[:,1]-e[:,0]))
        hull=trimesh.convex.convex_hull(np.concatenate(points))
        file=f'{name}_{i:02d}.obj'
        (out/file).write_text(hull.export(file_type='obj').rstrip() + '\n', encoding='utf-8')
        parts.append({'path':f'assets/collision/{file}','volume':hull.volume,'vertices':len(hull.vertices),'bounds':hull.bounds.tolist()})
    report[name]={'axis':axis,'cuts_m':cuts,'source_volume':source.volume,'whole_hull_volume':source.convex_hull.volume,'parts':parts}
for filename in ['so101.urdf','so101_camera.urdf']:
    path=root/filename; original=path.read_text()
    # Preserve the upstream text; replace only the two relevant collision blocks.
    import re
    def convert(match):
        block=match.group(0)
        for name,info in report.items():
            old=f'assets/{name}.stl'
            if old in block:
                return '<!-- Convex slices retain the jaw face; see PROVENANCE.md. -->\n    '+'\n    '.join(block.replace(old,p['path']) for p in info['parts'])
        return block
    path.write_text(re.sub(r'<collision>.*?</collision>',convert,original,flags=re.S),encoding='utf-8')
(out/'manifest.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
