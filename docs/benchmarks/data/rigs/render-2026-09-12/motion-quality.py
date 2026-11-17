from pathlib import Path
import sys,json,importlib.util
repo=Path(__file__).resolve().parents[2];sys.path.insert(0,str(repo))
source=repo/'tests/rendering/test_taa_validation_gpu.py'
spec=importlib.util.spec_from_file_location('temporal_gpu',source);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
case=module.TemporalValidationGPU;case.setUpClass();test=case()
reference,_,_=test.resolve(color_delta=.02)
before,_,_=test.resolve(history_shift=1,color_delta=.02)
after,_,_=test.resolve(history_shift=1,color_delta=.02,motion_shift=1/16)
import numpy as np
roi=np.s_[2:14,2:14,:3]
result={arm:float(np.abs(im[roi].astype('int16')-reference[roi]).mean()) for arm,im in [('camera_only',before),('object_motion',after)]}
assert result['object_motion']==0 and result['camera_only']>30
out=repo/'.local-runs/motion-pipeline';out.mkdir(exist_ok=True)
(out/'quality.json').write_text(json.dumps({'metric':'mean absolute 8-bit channel error against correctly aligned history','fixture':'16x16 checker, one-pixel horizontal translation, 0.02 stable history offset, central 12x12 ROI','result':result},indent=2))
print(result,flush=True)
