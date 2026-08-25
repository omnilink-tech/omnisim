## General Bugs

### Collision Detection

Although collision detection generally works well , `Cylinder-Cylinder`, `Cylinder-Capsule`, `IndexedFaceSet-IndexedFaceSet` and `IndexedFaceSet-Cylinder` collision detection may occasionaly yield wrong contact points.
Sometimes the contact points may be slightly off the shape, therefore causing unrealistic reaction forces to be applied to the objects.
Other times there are too few contact points, therefore causing vibration or instabilities.

### Intel Graphics Cards

OmniSim should run on any fairly recent computer equipped with a NVIDIA or AMD graphics card and up-to-date graphics drivers.
OmniSim is not guaranteed to work with Intel graphics cards: it may crash or exhibit display bugs.
Upgrading to the latest versions of the Intel graphics driver may help resolve such problems (without any guarantee).
Graphics drivers from Intel may be obtained from the [Intel download center website](http://downloadcenter.intel.com).
Linux graphics drivers from Intel may be obtained from the [Intel Linux Graphics website](http://intellinuxgraphics.org).

### Orientation Dependent Friction

The physics backend approximates the true friction cone with a linearized (pyramidal) version, which can introduce orientation-specific artifacts: an object may slip more easily in one direction than another even with symmetric friction coefficients.
On the current Newton/MuJoCo backend this is MuJoCo's stock pyramidal cone, and it is **measurable**: OmniBench scene T2 recorded **181 mm** of pseudo-slip below the true static-friction transition angle. Setting `WorldInfo.newtonCone "elliptic"` together with `newtonImpratio 10` took the same scene to **0.6 mm**.
The elliptic cone is not the global default on purpose — it changes contact physics for every world and would break the train==deploy bit-exactness of shipped policies — so declare it per world when cone accuracy matters. Raising `newtonGroundMu` also helps in the common "it slides when it should grip" case.

### Remote Desktop

OmniSim is not guaranteed to work through a remote desktop application.
This is because OmniSim has strong ties with the local graphics card for on-screen and off-screen OpenGL rendering.
Unfortunately, several remote desktop applications do not support this very well.

### Virtualization

Because it highly relies on OpenGL, OmniSim may not work properly in virtualized environments (such as VMWare or VirtualBox) which often lack good OpenGL support.
Hence, OmniSim may exhibit some display bugs, run very slowly or crash in such environments.

In the case of VirtualBox, it is known that enabling 3D acceleration in `Settings > Display > Acceleration > Enable 3D Acceleration` will cause OmniSim to crash (due to the old version of OpenGL provided by VirtualBox).
Therefore, please disable the 3D acceleration in your VirtualBox to use OmniSim.
