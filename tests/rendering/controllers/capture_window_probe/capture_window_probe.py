"""Exercise actual Supervisor image/movie capture after a host-side barrier."""
import json
from pathlib import Path
import sys
import time
from omnisim import Supervisor

out=Path(sys.argv[1]);kind=sys.argv[2];count=int(sys.argv[3])
robot=Supervisor();step=int(robot.getBasicTimeStep())
for _ in range(32):robot.step(step)
(out/'ready.json').write_text(json.dumps({'sim_time':robot.getTime()}))
deadline=time.monotonic()+30
while not (out/'go').exists():
    if time.monotonic()>deadline:raise RuntimeError('Host observation barrier timed out')
    time.sleep(.01)
translation=robot.getFromDef('MARKER').getField('translation')
if kind=='movie':
    robot.movieStartRecording(str(out/'movie.mp4'),640,360,0,75,1,False)
    for i in range(125):
        translation.setSFVec3f([2.,.35 if (i//15)%2 else -.35,.2]);robot.step(step)
    robot.movieStopRecording()
    deadline=time.monotonic()+45
    while not robot.movieIsReady() and time.monotonic()<deadline:robot.step(step)
    result={'movie_ready':robot.movieIsReady(),'movie_failed':robot.movieFailed()}
else:
    for i in range(count):
        translation.setSFVec3f([2.,.35 if i%2 else -.35,.2]);robot.step(step)
        robot.exportImage(str(out/f'frame-{i:02d}.png'),100)
        for _ in range(6):robot.step(step)
    result={'frames':count}
(out/'done.json').write_text(json.dumps(result))
for _ in range(12):robot.step(step)
robot.simulationQuit(0)
