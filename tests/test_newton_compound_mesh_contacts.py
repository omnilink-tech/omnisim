# Copyright 2026 OmniLink. SPDX-License-Identifier: Apache-2.0
"""Mixed mesh/primitive compounds must keep their load-bearing mesh hull.

The old compound walker silently skipped meshes whenever any primitive sibling
registered. Foundry then fell through the floor after its support wheels broke.
The second body also checks an authored translation/rotation, and both are rebuilt.
"""
import pytest
from tests.test_newton_rebuild_physics import _binary, _run

pytestmark = [pytest.mark.engine, pytest.mark.skipif(_binary() is None, reason='engine not built')]

MESH = '''IndexedFaceSet {
 coord Coordinate { point [ -.3 -.2 -.1, .3 -.2 -.1, .3 .2 -.1, -.3 .2 -.1,
                            -.3 -.2 .1, .3 -.2 .1, .3 .2 .1, -.3 .2 .1 ] }
 coordIndex [ 0 3 2 1 -1 4 5 6 7 -1 0 1 5 4 -1 1 2 6 5 -1 2 3 7 6 -1 3 0 4 7 -1 ]
}'''

WORLD = '''#OMNISIM R2025a utf8
WorldInfo { basicTimeStep 8 coordinateSystem "ENU" newtonSolver "mujoco"
 newtonStatics TRUE newtonCompoundColliders TRUE newtonSubsteps 8
 newtonContactKe 100000 newtonContactKd 1000 }
Viewpoint { position -4 0 2 }
Solid { name "floor" translation 0 0 -.05 newtonFriction 0 boundingObject Box { size 8 8 .1 } }
DEF FLAT Solid { name "flat" translation -1 0 1
 boundingObject Group { children [ @MESH@
   Pose { translation 0 0 .4 children [ Box { size .1 .1 .1 } ] } ] }
 physics Physics { density -1 mass 2 centerOfMass [0 0 0] inertiaMatrix [.2 .2 .2 0 0 0] }
}
DEF POSED Solid { name "posed" translation 1 0 1
 boundingObject Group { children [
   Pose { translation 0 0 -.2 rotation 0 1 0 1.570796326795 children [ @MESH@ ] }
   Pose { translation 0 0 .6 children [ Box { size .1 .1 .1 } ] } ] }
 physics Physics { density -1 mass 2 centerOfMass [0 0 0] inertiaMatrix [.2 .2 .2 0 0 0] }
}
DEF LOW Solid { name "low" translation 0 -2 .3 newtonFriction .1
 boundingObject @MESH@ physics Physics { density -1 mass 2 inertiaMatrix [.2 .2 .2 0 0 0] } }
DEF HIGH Solid { name "high" translation 0 2 .3 newtonFriction .9
 boundingObject @MESH@ physics Physics { density -1 mass 2 inertiaMatrix [.2 .2 .2 0 0 0] } }
Robot { supervisor TRUE controller "compoundmesh" }
'''.replace('@MESH@',MESH)

CONTROLLER = '''import os
from omnisim import Supervisor
s = Supervisor()
dt = int(s.getBasicTimeStep())
out = open(os.environ['REBUILD_PROBE_OUT'],'w',buffering=1)
for rebuilt in (False,True):
    if rebuilt:
        s.simulationRebuildPhysics()
    for _ in range(300):
        if s.step(dt) == -1:
            break
    for name in ('FLAT','POSED'):
        out.write('%s %.9f\\n' % (name.lower()+('_rebuilt' if rebuilt else ''),s.getFromDef(name).getPosition()[2]))
for name in ('LOW','HIGH'):
    s.getFromDef(name).setVelocity([1,0,0,0,0,0])
for _ in range(50):
    s.step(dt)
for name in ('LOW','HIGH'):
    out.write('%s_speed %.9f\\n' % (name.lower(),s.getFromDef(name).getVelocity()[0]))
out.write('done\\n')
out.close()
s.simulationQuit(0)
'''


@pytest.fixture(scope='module')
def compound_result(tmp_path_factory):
    return _run(tmp_path_factory.mktemp('compound_mesh'),'compound_mesh',WORLD,'compoundmesh',CONTROLLER,60)


def test_mesh_hull_survives_primitive_sibling_and_rebuild(compound_result):
    values = compound_result
    assert 'Skipped node' not in values['_log']
    assert 'ERROR:' not in values['_log']
    for suffix in ('','_rebuilt'):
        assert values['flat'+suffix] == pytest.approx(.1,abs=.003),values['_raw']
        assert values['posed'+suffix] == pytest.approx(.5,abs=.003),values['_raw']


def test_mesh_friction_changes_actual_sliding(compound_result):
    # Same shape, mass, floor and initial speed; only authored friction differs.
    assert compound_result['low_speed'] > .4,compound_result['_raw']
    assert abs(compound_result['high_speed']) < .05,compound_result['_raw']
