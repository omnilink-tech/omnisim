# Copyright 2026 OmniLink
# SPDX-License-Identifier: Apache-2.0
"""Engine regression: a small flat-ended cylinder must rest on its surface.

The optional multiple-contact setting must reach the solver, and setting it
to zero must restore the default. Run after staging the Newton runtime module.
"""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'projects/robots/trossen/aloha'))
from build_spring_scene import battery_collider

BINARY = ROOT / 'msys64/mingw64/bin/omnisim-bin.exe'
pytestmark = [pytest.mark.engine, pytest.mark.skipif(not BINARY.is_file(), reason='Windows engine required')]


def run_probe(directory, setting):
    worlds = directory / 'worlds'
    controllers = directory / 'controllers' / 'flat_probe'
    worlds.mkdir(parents=True)
    controllers.mkdir(parents=True)
    output = directory / 'result.json'
    log = directory / 'engine.log'
    world = worlds / 'flat.omniworld'
    world.write_text('''#OMNISIM R2025a utf8
WorldInfo { basicTimeStep 2 newtonSolver "mujoco" newtonCompoundColliders TRUE
 newtonGroundMu 3 newtonCone "elliptic" newtonImpratio 10 newtonCondim 6 newtonNoslipIterations 5 }
Solid { translation 0 0 -.015 boundingObject Box { size 1.5 1 .05 } }
DEF BATTERY Solid { translation .1 .1 .06 rotation 0 1 0 1.57079632679
 boundingObject ''' + battery_collider() + '''
 physics Physics { density -1 mass .023 centerOfMass [ 0 0 0 ]
 inertiaMatrix [ .000004867 .000004867 .00000053176 0 0 0 ] } }
Robot { supervisor TRUE controller "flat_probe" }
''')
    (controllers / 'flat_probe.py').write_text('''import json, os
from omnisim import Supervisor
s=Supervisor(); b=s.getFromDef('BATTERY'); rows=[]
for k in range(1200):
    if s.step(2)==-1:raise RuntimeError('Incomplete probe')
    if k>=1100:rows.append(b.getPosition()[2])
with open(os.environ['FLAT_PROBE_OUTPUT'],'w') as f:json.dump(rows,f)
s.simulationQuit(0)
''')
    env = dict(os.environ, OMNISIM_HOME=str(ROOT), OMNISIM_LOG_PATH=str(log),
               OMNISIM_NO_WINDOW='1', FLAT_PROBE_OUTPUT=str(output),
               WARP_CACHE_PATH=str(directory.parent/'warp-cache'))
    env['PATH']=str(Path(sys.executable).parent)+os.pathsep+env.get('PATH','')
    for key in ('OMNISIM_NEWTON_MULTICCD','OMNISIM_NEWTON_CONTACT_KE','OMNISIM_NEWTON_CONTACT_KD'):
        env.pop(key, None)
    if setting is not None:env['OMNISIM_NEWTON_MULTICCD']=setting
    subprocess.run([str(BINARY),'--batch','--mode=fast','--no-rendering',str(world)],
                   env=env, cwd=ROOT, capture_output=True, timeout=60, check=True)
    assert output.is_file(), log.read_text(errors='replace')[-2500:]
    sidecar=json.loads(Path(str(log)+'.newton.json').read_text())
    assert sidecar['finalised'] and not sidecar['degraded']
    return json.loads(output.read_text())


def test_flat_mesh_contact_and_explicit_disable(tmp_path):
    default=run_probe(tmp_path/'default',None)
    disabled=run_probe(tmp_path/'disabled','0')
    enabled=run_probe(tmp_path/'enabled','1')
    assert default == disabled
    assert max(abs(z-.0168) for z in enabled)<.0005
    assert min(abs(z-.0168) for z in disabled)>.005
