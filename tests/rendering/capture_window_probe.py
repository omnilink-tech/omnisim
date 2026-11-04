"""Read-only Win32 window-state telemetry around owned simulator capture.

This never activates, restores, hides or minimises any window. The regression
must save real changing frames while keeping --minimize in effect. Baseline
and candidate runs use the same fixture. Do not run against an unrelated PID.
"""
import argparse
import ctypes
from ctypes import wintypes
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from omnisim.paths import linux_runtime_env,resolve_omnisim_binary
from omnisim.assessment.runner import _reap_owned

def observe_window(pid):
    user32=ctypes.windll.user32
    callback=ctypes.WINFUNCTYPE(wintypes.BOOL,wintypes.HWND,wintypes.LPARAM)
    user32.GetForegroundWindow.restype=wintypes.HWND
    user32.IsIconic.argtypes=[wintypes.HWND]
    user32.IsWindowVisible.argtypes=[wintypes.HWND]
    user32.GetWindowThreadProcessId.argtypes=[wintypes.HWND,ctypes.POINTER(wintypes.DWORD)]
    user32.GetWindowTextLengthW.argtypes=[wintypes.HWND]
    windows=[]
    @callback
    def visit(hwnd,param):
        found=wintypes.DWORD();user32.GetWindowThreadProcessId(hwnd,ctypes.byref(found))
        if found.value==pid and user32.IsWindowVisible(hwnd) and user32.GetWindowTextLengthW(hwnd)>0:
            windows.append({'minimized':bool(user32.IsIconic(hwnd)),'foreground':user32.GetForegroundWindow()==hwnd})
        return True
    user32.EnumWindows(visit,0)
    return windows

def run(out,binary=None,kind='images',no_rendering=False,count=6):
    import psutil
    from PIL import Image,ImageChops,ImageStat
    out=Path(out).resolve();out.mkdir(parents=True,exist_ok=False)
    binary=Path(binary or resolve_omnisim_binary()).resolve()
    world=ROOT/'tests/rendering/worlds'/('.capture_window_'+uuid.uuid4().hex+'.omniworld')
    source='''#OMNISIM R2025a utf8
WorldInfo { basicTimeStep 16 newtonSolver "mujoco" }
Viewpoint { position 0 0 1 orientation 0 1 0 .3 fieldOfView .8 }
Background { skyColor [ .05 .08 .12 ] }
DirectionalLight { direction 1 -.2 -1 intensity 2 }
Robot { supervisor TRUE controller "capture_window_probe" controllerArgs [ OUT KIND COUNT ] }
DEF MARKER Solid { translation 2 -.35 .2 children [ Shape { appearance PBRAppearance { baseColor .15 .65 .9 roughness .8 metalness 0 } geometry Box { size .4 .4 .4 } } ] }
Solid { translation 2.5 0 -.1 children [ Shape { appearance PBRAppearance { baseColor .9 .65 .1 roughness .8 metalness 0 } geometry Box { size .5 1.5 .1 } } ] }
'''.replace('OUT',json.dumps(out.as_posix())).replace('KIND',json.dumps(kind)).replace('COUNT',json.dumps(str(count)))
    world.write_text(source,encoding='utf-8');(out/'world.omniworld').write_text(source,encoding='utf-8')
    env=linux_runtime_env(ROOT);env.pop('OMNISIM_NO_WINDOW',None);env.pop('OMNI_KEY',None)
    env.update(OMNISIM_HOME=str(ROOT),OMNISIM_LOG_PATH=str(out/'engine.log'),OMNISIM_WGPU_TAA='0')
    if os.name=='nt':
        runtime=ROOT/'msys64/mingw64/bin/newton-runtime'
        env['PATH']=os.pathsep.join([str(runtime),str(runtime.parent),env.get('PATH','')])
    args=[str(binary),str(world),'--batch','--mode=realtime','--minimize','--stdout','--stderr']
    if no_rendering:args.append('--no-rendering')
    samples=[];children={};proc=None;error=None
    log=(out/'stdout.log').open('w',encoding='utf-8')
    try:
        proc=subprocess.Popen(args,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        deadline=time.monotonic()+90
        while not (out/'ready.json').exists():
            if proc.poll() is not None:raise RuntimeError('Engine exited before barrier')
            if time.monotonic()>deadline:raise TimeoutError('Readiness barrier')
            time.sleep(.02)
        for child in psutil.Process(proc.pid).children(recursive=True):children[child.pid]=child
        initial=observe_window(proc.pid)
        if len(initial)!=1 or not initial[0]['minimized']:raise RuntimeError('Expected one minimized simulator window at capture start: '+str(initial))
        started=time.monotonic();(out/'go').touch()
        while not (out/'done.json').exists():
            windows=observe_window(proc.pid)
            samples.append({'wall_s':time.monotonic()-started,'windows':windows})
            if proc.poll() is not None:raise RuntimeError('Engine exited during capture')
            if time.monotonic()-started>90:raise TimeoutError('Capture deadline')
            time.sleep(.002)
        proc.wait(timeout=20)
    except Exception as exc:error=type(exc).__name__+': '+str(exc)
    finally:
        cleanup=_reap_owned(proc,children) if proc else []
        log.close();world.unlink(missing_ok=True)
    (out/'window-samples.json').write_text(json.dumps(samples),encoding='utf-8')
    transitions=0;previous=True
    for sample in samples:
        if len(sample['windows'])!=1:continue
        minimized=sample['windows'][0]['minimized']
        if previous and not minimized:transitions+=1
        previous=minimized
    frames=sorted(out.glob('frame-*.png'));images=[Image.open(p).convert('RGB') for p in frames]
    changes=[sum(ImageStat.Stat(ImageChops.difference(a,b)).mean)/3 for a,b in zip(images,images[1:])]
    movie_check=None
    if kind=='movie' and (out/'movie.mp4').exists():
        try:
            metadata=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_streams','-of','json',str(out/'movie.mp4')]))
            streams=metadata['streams'];video=next(s for s in streams if s['codec_type']=='video')
            # Decode the ENTIRE movie, then measure sampled pixel changes. A
            # valid container or a recording-ready flag cannot prove footage.
            decoded=subprocess.run(['ffmpeg','-v','error','-i',str(out/'movie.mp4'),'-vf','scale=160:90','-f','rawvideo','-pix_fmt','rgb24','pipe:1'],capture_output=True,check=True)
            raw=decoded.stdout;size=160*90*3;frames_rgb=[Image.frombytes('RGB',(160,90),raw[i:i+size]) for i in range(0,len(raw)-size+1,size)]
            delta=max((sum(ImageStat.Stat(ImageChops.difference(a,b)).mean)/3 for a,b in zip(frames_rgb,frames_rgb[1:])),default=0)
            movie_check={'width':video['width'],'height':video['height'],'decoded_frames':len(frames_rgb),'audio_streams':sum(s['codec_type']=='audio' for s in streams),'max_frame_change_mean':delta,'decode_stderr':decoded.stderr.decode(errors='replace')}
            movie_check['passed']=video['width']==640 and video['height']==360 and len(frames_rgb)>20 and delta>.1 and not movie_check['audio_streams'] and not movie_check['decode_stderr']
        except (OSError,ValueError,StopIteration,subprocess.SubprocessError) as exc:movie_check={'passed':False,'error':str(exc)}
    result={'binary':str(binary),'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),'kind':kind,'no_rendering':no_rendering,
      'error':error,'cleanup_errors':cleanup,'samples':len(samples),'restores_during_capture':transitions,
      'foreground_samples':sum(any(w['foreground'] for w in s['windows']) for s in samples),
      'nonminimized_samples':sum(any(not w['minimized'] for w in s['windows']) for s in samples),
      'frame_count':len(frames),'frame_dimensions':[list(im.size) for im in images],'min_frame_change_mean':min(changes,default=0),
      'done':json.loads((out/'done.json').read_text()) if (out/'done.json').exists() else None,'movie_validation':movie_check,
      'newton':json.loads((out/'engine.log.newton.json').read_text()) if (out/'engine.log.newton.json').exists() else None}
    result['passed']=not error and not cleanup and not result['nonminimized_samples'] and not result['foreground_samples'] and (kind=='movie' and result['done']['movie_ready'] and not result['done']['movie_failed'] and movie_check and movie_check['passed'] or kind=='images' and len(images)==count and result['min_frame_change_mean']>.1)
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result),flush=True);return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--binary');p.add_argument('--kind',choices=['images','movie'],default='images');p.add_argument('--no-rendering',action='store_true');p.add_argument('--count',type=int,default=6);a=p.parse_args()
    r=run(a.out,a.binary,a.kind,a.no_rendering,a.count);raise SystemExit(0 if r['passed'] else 1)
