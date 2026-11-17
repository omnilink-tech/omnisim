from pathlib import Path
import json,hashlib,sys,re
from PIL import Image
repo=Path(__file__).resolve().parents[2];sys.path.insert(0,str(repo))
from scripts.dev.render_ab import summarize_gpu_profile,diff
out=repo/'.local-runs/motion-pipeline'
quality=json.loads((out/'quality.json').read_text())
controls=json.loads((out/'final-controls/results.json').read_text())
assert len(controls)==3 and all(r['baseline_image_check']['pixels_over_threshold']==0 for r in controls)
checks=(repo/'.local-runs/beauty-realism/motion-release-checks.stdout').read_text()+(repo/'.local-runs/beauty-realism/motion-last-checks.stdout').read_text()
assert '8 passed' in checks and '16 passed' in checks and '2 passed' in checks
unit=re.search(r'(\d+) passed, (\d+) skipped, (\d+) deselected',checks)
assert unit
binary=repo/'msys64/mingw64/bin/omnisim-bin-motion-final.exe'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
moving=out/'release-moving'
profiles={str(arm):summarize_gpu_profile((moving/str(arm)/'render.gpu').read_text(),warmup_frames=0) for arm in (0,1)}
sensor=[diff(moving/'0'/f'{frame:02}-sensor.png',moving/'1'/f'{frame:02}-sensor.png',0) for frame in range(12)]
assert all(r['pixels_over_threshold']==0 for r in sensor)
for arm,label in (('0','before'),('1','after')):
    with Image.open(moving/arm/'03.png') as im:
        im.convert('RGB').crop((720,230,1220,680)).save(out/f'{label}.png')
static=json.loads((out/'static/results.json').read_text())
evidence={
 'date':'2026-09-12',
 'machine':{'id':'9722d23d12a3','cpu':'AMD64 Family 25 Model 80 Stepping 0, AuthenticAMD','logical_cores':16,'gpu':'NVIDIA GeForce RTX 3060 Laptop GPU','driver':'596.36','os':'Windows 11'},
 'baseline_binary_sha256':'5f9d17723ed2e35fae5a9de78bea3b2a4477c200c0682837f9d60afa0b92b309',
 'candidate_binary_sha256':sha(binary),
 'tested_behavior_binary_sha256':'7a9d6ce11a561911ea92c88577a06ef5611d6b3b0e7c42455af50fe2fd1c85f0',
 'final_rebuild_note':'The final rebuild adds only the environment-variable description comment required by the generated reference check; runtime source behavior is unchanged.',
 'quality':quality,
 'validation':{'gpu_shader_tests':13,'cpp_history_test':'PASS','engine_and_renderer_tests':8,'unit_tests':int(unit[1]),'unit_skipped':int(unit[2]),'unit_deselected':int(unit[3]),'docs_tests':16,'license_tests':2,'sensor_comparison_frames':12,'sensor_changed_pixels':0},
 'taa_off_controls':[{'scene':r['scene'],'image_comparison':r['baseline_image_check'],'candidate_binary_sha256':r['result']['binary_sha256']} for r in controls],
 'final_moving_scene_gpu_profiles':profiles,
 'static_diagnostics':{'scope':'Earlier motion candidate before the invariant-position vertex variant. Capped at 5 FPS. Sequential runs, GPU clocks/load not locked; do not infer an FPS change or a speedup from these spans.',
  'candidate_binary_sha256':'d6161663fe7ca0cd94210ef66a441b9830d12970b46f0d3e0c3255735366b59e',
  'scenes':[{'scene':r['scene'],'image_difference':r['difference'],'profiles':{arm:r['arms'][arm]['gpu_profile'] for arm in ('0','1')}} for r in static]},
 'live_cloth':{'source_world':'projects/samples/demos/worlds/rendering/camera_cloth_wgpu_smoke.omniworld','viewport':'Matched to the proven sensor pose: position -1.6 0 .85, orientation 0 0 1 0, fieldOfView .9','sensor_steps':[8,64],'verified_by':'tests/test_object_motion_scene.py::ObjectMotionScene::test_live_cloth_uses_deformation_motion'},
 'limitations':['Viewport snapshots are not synchronized to the same jitter phase; use the controlled shader fixture for numerical history-alignment comparisons.','Glass and particle/Track instances without persistent correspondence reject history. Animated textures and changing screen-space lighting use color rejection.','Motion targets consume 32 additional bytes per viewport pixel once needed; TAA-off and sensor images do not allocate them.','GPU timestamps exclude presentation, physics and readback and are not FPS.'],
 'source_hashes':{str(p.relative_to(repo)).replace('\\','/'):sha(p) for p in [repo/'src/omnisim/render/OmMotionHistory.hpp',repo/'src/omnisim/render/OmMotionVectors.hpp',repo/'src/omnisim/render/OmWgpuShaders.cpp',repo/'src/omnisim/render/OmWgpuRenderTarget.cpp']}}
path=repo/'docs/benchmarks/data/object-motion-2026-09-12.json'
path.write_text(json.dumps(evidence,indent=2)+'\n')
metric=quality['result']
report=f'''# Moving detail: before and after

OmniSim now follows moving objects and bending surfaces when it blends frames for anti-aliasing. This applies across the rendering pipeline, including robots and imported CAD shapes, cloth, soft bodies, and muscles.

| Before: camera movement only | After: object movement too |
|---|---|
| ![Before](O:/omnisim/.local-runs/motion-pipeline/before.png) | ![After](O:/omnisim/.local-runs/motion-pipeline/after.png) |

These are unscaled crops of real viewport captures at the same scripted pose. Capture timing can differ in the anti-aliasing cycle, so this is a visual comparison rather than a numerical quality benchmark.

In the controlled moving-pattern GPU test, the average error against correctly aligned frame history fell from **{metric['camera_only']:.1f} to {metric['object_motion']:.1f}** on the 0–255 color scale. This measures that specific test, not every scene.

- The house, warehouse, and city match the previous rendering exactly with temporal anti-aliasing disabled.
- All 12 robot-camera comparisons match exactly. Live cloth motion is also verified.
- {int(unit[1]):,} unit tests, 13 GPU shader tests, 8 renderer/engine checks, the C++ history checks, and the documentation/license checks pass.

Static opaque surfaces skip the extra drawing pass. Moving surfaces add work, and the motion buffer uses about 63 MiB at 1080p. GPU pass timings are recorded in the [evidence](O:/omnisim/docs/benchmarks/data/object-motion-2026-09-12.json); these capped runs do not establish an FPS improvement.

The feature is enabled by default alongside temporal anti-aliasing. `OMNISIM_WGPU_MOTION_VECTORS=0` restores the previous camera-only tracking. Glass, animated textures, and changing reflections remain areas for further work.
'''
(out/'before-after.md').write_text(report,encoding='utf-8')
print(path,flush=True)
