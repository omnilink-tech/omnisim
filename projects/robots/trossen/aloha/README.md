# ALOHA battery insertion in OmniSim

Two ViperX 300S arms, a public real recording, and a motor-driven simulation of
the same battery task. The authored trajectory and the recorded-action replay
are separate experiments. This is not validated transfer to hardware.
The spring variant now includes a moving negative terminal, a physical spring
force, a positive contact, a retaining lip and a separate fingertip seating
press. The original rigid-slot placement prototype remains archived separately.
See [the mechanism, assumptions and reproduction commands](MECHANISM.md).

[Watch the comparison and read the results](../../../../sim-to-real/aloha-battery/README.md).
Model, scene, pads and gripper-mapping assumptions are listed in
[PROVENANCE.md](PROVENANCE.md).

## Run

For the spring reconstruction, use [the dedicated instructions](MECHANISM.md#run-the-experiment).
It requires both `OMNISIM_NEWTON_MULTICCD=1` and
`OMNISIM_NEWTON_FINGER_KE=2000` on the harness process. The commands below
reproduce the earlier placement prototype and recorded-action diagnostic.

Use a dedicated harness because the replay command loads a new world. Start
it with the authored experiment's finger stiffness setting; PowerShell:

```powershell
$env:OMNISIM_NEWTON_FINGER_KE = '2000'
.\omnisim.bat harness --port 6889 --supervisor-port 6890
```

Then, from another terminal at the repository root:

```sh
python projects/robots/trossen/aloha/replay.py --mode authored --harness http://127.0.0.1:6889 --out-dir outputs/aloha-authored --capture
python projects/robots/trossen/aloha/replay.py --mode authored --view insertion --harness http://127.0.0.1:6889 --out-dir outputs/aloha-insertion-angle --capture
python projects/robots/trossen/aloha/replay.py --mode authored --harness http://127.0.0.1:6889 --out-dir outputs/aloha-open-gripper --no-grip
python projects/robots/trossen/aloha/replay.py --mode recorded-actions --harness http://127.0.0.1:6889 --out-dir outputs/aloha-recorded --capture
```

Run these sequentially. Each output directory must be empty. The launcher
exits 0 for task success or an expected negative control, 2 for an unsuccessful
task, and raises an error for incomplete execution. The recorded replay is
expected to fail. `--mode measured-arms` is available for diagnosis.
`--view insertion` selects a closer oblique camera that shows the battery slots;
the default `--view overhead` retains the original comparison angle. Neither
camera option changes the motor targets or physics settings.

On Windows without a system Python, the saved-data launcher also runs with
`msys64\mingw64\bin\newton-runtime\python.exe` in place of `python`.
Only Windows has a prebuilt OmniSim package; other platforms require a build.

Opening [the world](worlds/aloha_battery.omniworld) directly runs the raw
recorded-action diagnostic with the world's initial estimates. Use the CLI
above for the authored comparison and its declared 10 mm table offset.

## Rebuild and check

The generated URDFs, meshes and target files are saved, so normal execution
needs no third-party Python packages in the controller. To rebuild, install
[requirements-dev.txt](requirements-dev.txt) in a separate development environment:

```sh
python projects/robots/trossen/aloha/build_model.py
python projects/robots/trossen/aloha/build_scene.py
python projects/robots/trossen/aloha/prepare_episode.py
python projects/robots/trossen/aloha/author_motion.py
python -m omnisim validate-urdf projects/robots/trossen/aloha/aloha_left.urdf
python -m omnisim validate-urdf projects/robots/trossen/aloha/aloha_right.urdf
python -m unittest discover -s projects/robots/trossen/aloha/tests -v
python projects/robots/trossen/aloha/verify_model.py
```

The last command additionally needs MuJoCo, available in the OmniSim runtime's
`site-packages`. It compares against upstream MJCF rather than a second copy
of the URDF converter. The original source assets and licenses remain in `source/`.
