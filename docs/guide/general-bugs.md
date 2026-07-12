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

Although the friction model of ODE is very accurate, the true friction cone is approximated by a linearized version which can introduce some orientation specific artifacts.
It is for example possible that an object slips more easily on another object in some direction than in some other, even if the friction coefficients are set to be symmetric.
However, in most of the cases it is possible to get rid of these effects by tuning correctly the friction parameters.

### Remote Desktop

OmniSim is not guaranteed to work through a remote desktop application.
This is because OmniSim has strong ties with the local graphics card for on-screen and off-screen OpenGL rendering.
Unfortunately, several remote desktop applications do not support this very well.

### Virtualization

Because it highly relies on OpenGL, OmniSim may not work properly in virtualized environments (such as VMWare or VirtualBox) which often lack good OpenGL support.
Hence, OmniSim may exhibit some display bugs, run very slowly or crash in such environments.

In the case of VirtualBox, it is known that enabling 3D acceleration in `Settings > Display > Acceleration > Enable 3D Acceleration` will cause OmniSim to crash (due to the old version of OpenGL provided by VirtualBox).
Therefore, please disable the 3D acceleration in your VirtualBox to use OmniSim.
