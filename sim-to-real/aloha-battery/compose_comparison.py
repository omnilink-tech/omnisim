# Copyright 2026 OmniLink
# SPDX-License-Identifier: Apache-2.0
"""Encode a native capture and an honestly labelled real/simulation comparison."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--real-view',choices=['high','low'],default='high')
    native = parser.add_mutually_exclusive_group(required=True)
    native.add_argument('--simulation', type=Path, help='Encode captured frames to this new file')
    native.add_argument('--native-video', type=Path, help='Reuse an existing saved native video')
    args = parser.parse_args()
    result = json.loads((args.run/'result.json').read_text())
    cfg = json.loads((args.run/'config.json').read_text())
    count=len(cfg['targets']);duration=count/cfg['fps'];spring=bool(cfg.get('spring_compartment'))
    if not result['complete'] or result['frames'] != count or cfg['fps'] != 50:
        raise ValueError('A complete run at 50 Hz is required')
    if args.output.exists() or (args.simulation and args.simulation.exists()):
        raise FileExistsError('Choose new output paths; saved videos are never overwritten')
    frames = [args.run/'frames'/f'sim_{k:05d}.jpg' for k in range(count)]
    if args.simulation and not all(p.exists() for p in frames):
        raise ValueError(f'All {count} captured simulation frames are required')
    ffmpeg = shutil.which('ffmpeg') or ('C:/ffmpeg/bin/ffmpeg.exe' if Path('C:/ffmpeg/bin/ffmpeg.exe').exists() else None)
    if ffmpeg is None:
        raise FileNotFoundError('FFmpeg must be installed')
    for path in (args.output,args.simulation):
        if path:path.parent.mkdir(parents=True,exist_ok=True)
    if args.simulation:
        subprocess.run([ffmpeg,'-nostdin','-n','-loglevel','error','-framerate','50','-i',
                    str(args.run/'frames/sim_%05d.jpg'),'-vf','pad=ceil(iw/2)*2:ceil(ih/2)*2',
                    '-frames:v',str(count),'-an','-c:v','libx264','-crf','18','-pix_fmt','yuv420p',
                        '-movflags','+faststart',str(args.simulation)],check=True)
    simulation=args.simulation or args.native_video
    if not simulation.is_file():raise FileNotFoundError(simulation)
    font="fontfile='C\\:/Windows/Fonts/arial.ttf'" if Path('C:/Windows/Fonts/arial.ttf').exists() else 'font=Arial'
    def label(text,x,y,size=26,color='white'):
        return f"drawtext={font}:text='{text}':x={x}:y={y}:fontsize={size}:fontcolor={color}"
    authored=cfg['motion_mode'] in ('authored','authored simulation')
    mode='Mechanism reconstruction' if spring else ('Authored simulation' if authored else 'Recorded actions with estimated mapping')
    if cfg.get('view') == 'insertion':
        mode += '  |  Slot close-up'
    real=f'[0:v]setpts=PTS-STARTPTS,tpad=stop_mode=clone:stop_duration={max(0,duration-12)},scale=960:720,setsar=1,pad=960:850:0:100:color=0x111820,'
    real+=label('REAL ALOHA',28,22,34,'0xF2BC71')+','+label('Physical recording  |  Episode 0  |  Original speed',28,68,23)
    if duration>12:
        real+=','+label('REFERENCE ENDED AT 12 s - FRAME HELD',28,817,23,'0xF2BC71')+":enable='gte(t,12)'"
    real+='[real];'
    sim='[1:v]setpts=PTS-STARTPTS,crop=trunc(ih*4/3/2)*2:trunc(ih/2)*2,scale=960:720,setsar=1,pad=960:850:0:100:color=0x111820,'
    sim+=label('OMNISIM',28,22,34,'0x77D6C4')+','+label(mode+'  |  Original speed',28,68,23)+'[sim];'
    status='Placement prototype. Spring-loaded insertion is not modeled.' if result['task_success'] else 'Prototype. The simulated grasp or placement still fails.'
    if spring:
        status='Motor-driven grasp, spring compression, release and fingertip seating.' if result['task_success'] else 'Development run. Mechanical insertion checks still fail.'
    graph=real+sim+'[real][sim]hstack=2,pad=1920:1080:0:0:color=0x111820,'
    graph+=label('ALOHA / BATTERY INSERTION',28,872,36)+','+label(status,28,927,29)+','
    limits='Estimated geometry and spring forces. Mechanical contacts only; no hardware validation.' if spring else 'Rigid slots; no springs or battery terminals. No hardware transfer validation.'
    graph+=label(limits,28,978,25)+','
    graph+=label('Real recording | lerobot/aloha_static_battery | ALOHA / Tony Zhao and collaborators',28,1025,22,'0xADBAC7')
    if spring:
        rows=[json.loads(line) for line in (args.run/'trace.jsonl').read_text().splitlines()]
        if len(rows)!=count:raise ValueError('Complete telemetry is required for the spring overlay')
        def stamp(frame):
            cs=round(frame*100/50);return f'{cs//360000}:{cs//6000%60:02}:{cs//100%60:02}.{cs%100:02}'
        phases={'engage_spring':'ENGAGE TERMINAL','seat':'LOWER TILTED BATTERY',
                'press':'FINGERTIP PRESS','press_approach':'POSITION EMPTY GRIPPER',
                'press_withdraw':'RELEASE PRESSURE','inspect':'FINAL CONTACT CHECK'}
        ass='[Script Info]\nScriptType: v4.00+\nPlayResX: 1920\nPlayResY: 1080\n\n[V4+ Styles]\n'
        ass+='Format: Name, Fontname, Fontsize, PrimaryColour, OutlineColour, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding\n'
        ass+='Style: Default,Arial,23,&H00FFFFFF,&H00111820,1,1,0,7,0,0,0,1\n\n[Events]\n'
        ass+='Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text\n'
        for k,row in enumerate(rows):
            phase=phases.get(row['stage'],row['stage'].replace('_',' ').upper())
            caption=f"{phase}  |  Spring compression {max(0,row['spring_compression_m'])*1000:.1f} mm"
            ass+=f'Dialogue: 0,{stamp(k)},{stamp(k+1)},Default,,0,0,0,,{{\\pos(988,818)}}{caption}\n'
        subtitle=args.output.with_suffix('.ass');subtitle.write_text(ass,encoding='utf-8')
        escaped=subtitle.resolve().as_posix().replace(':',r'\:')
        graph+=f",subtitles=filename='{escaped}'"
    graph+='[out]'
    subprocess.run([ffmpeg,'-nostdin','-n','-loglevel','error','-i',str(ROOT/f'source/episode-000-{args.real_view}.mp4'),
                    '-i',str(simulation),'-filter_complex',graph,'-map','[out]','-frames:v',str(count),'-an',
                    '-c:v','libx264','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(args.output)],check=True)
    print(args.output.resolve())


if __name__=='__main__':main()
