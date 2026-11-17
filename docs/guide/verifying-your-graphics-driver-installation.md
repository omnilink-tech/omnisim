## Verifying your Graphics Driver Installation

### What OmniSim actually needs

OmniSim renders through **wgpu-native**, which targets Vulkan on Linux and
D3D12 on Windows. Since WREN was deleted on 2026-08-23 (commit `976b9449d`) it
is the only renderer, so the requirement is **a GPU and driver supporting
Vulkan 1.2 or D3D12** — not an OpenGL version. There is no OpenGL path left to
fall back to.

The failure mode is worth knowing before you debug the wrong thing: if
wgpu-native cannot initialise, OmniSim logs one line and keeps running with
**no renderer at all**. Physics and controllers work; nothing draws. A black or
absent 3D view with a healthy-looking simulation is this, not a world bug.

### Supported Graphics Cards

Recent NVIDIA and AMD adapters with up-to-date vendor drivers are the tested
configuration. Such drivers are often bundled with the operating system
(Windows, Linux), but in some cases it may be necessary to fetch one from the
card manufacturer's website.

### Unsupported Graphics Cards

OmniSim may nevertheless work with other graphics adapters, in particular Intel
integrated graphics — modern Intel parts generally satisfy Vulkan 1.2, but they
are not tested here and carry no guarantee.
Graphics drivers from Intel may be obtained from the [Intel download center website](http://downloadcenter.intel.com).
If graphical bugs persist, reducing render quality from the OmniSim
[preferences](preferences.md) may help, at some cost to visual quality.

### Upgrading your Graphics Driver

On Linux and Windows, you should make sure that the latest graphics driver is
installed. (macOS is not supported: there is no package, no verified build, and
Newton physics is unverified — use Windows or Ubuntu 24.04.)
Note that OmniSim can run far slower, or not render at all, without a driver
that exposes Vulkan 1.2 or D3D12.
Updating your driver may also solve various problems, i.e., odd graphics rendering or OmniSim crashes.

#### Upgrading the GPU Driver on Linux

On Linux, the direct check is Vulkan, since that is what wgpu-native uses:

```sh
$ vulkaninfo --summary        # from the vulkan-tools package
```

It must report a real GPU under `deviceName`. If it reports `llvmpipe`, or
fails outright, you are on a software rasteriser or have no Vulkan ICD, and
OmniSim will come up with no renderer.

`glxinfo` remains a quick way to confirm that *some* hardware driver is loaded
at all, even though OmniSim no longer uses OpenGL:

```sh
$ glxinfo | grep OpenGL
```

If the output contains the string "NVIDIA", "AMD", or "Intel", this indicates that a hardware driver is currently installed:

```sh
$ glxinfo | grep OpenGL
OpenGL vendor string: NVIDIA Corporation
OpenGL renderer string: GeForce 8500 GT/PCI/SSE2
OpenGL version string: 3.0.0 NVIDIA 180.44
...
```

If you read "Mesa", "Software Rasterizer" or "GDI Generic", this indicates that the hardware driver is currently not installed and that your computer is currently using a slow software emulation of OpenGL:

```sh
$ glxinfo | grep OpenGL
OpenGL vendor string: Mesa project: www.mesa3d.org
OpenGL renderer string: Mesa GLX Indirect
OpenGL version string: 1.4 (1.5 Mesa 6.5.2)
...
```

In this case you should definitely install the hardware driver.

On Ubuntu the driver can usually be installed automatically from the `Additional Drivers` tab of the `Software & Update` window.
Otherwise you can find out what graphics hardware is installed on your computer by using this command:

```sh
$ lspci | grep VGA
01:00.0 VGA compatible controller: nVidia Corporation GeForce 8500 GT (rev a1)
```

Then you can normally download the appropriate driver from the graphics hardware manufacturer's website: [http://www.nvidia.com](http://www.nvidia.com) for an NVIDIA card or [http://www.amd.com](http://www.amd.com) for a AMD graphics card.
Please follow the manufacturer's instructions for the installation.

#### Upgrading the GPU Driver on Windows

1. Right-click on `My Computer`.
2. Select `Properties`.
3. Click on the `Device Manager` tab.
4. Click on the plus sign to the left of `Display adapters`.
The name of the driver appears.
Make a note of it.
5. Go to the website of your card manufacturer: [http://www.nvidia.com](http://www.nvidia.com) for an NVIDIA card or [http://www.amd.com](http://www.amd.com) for a AMD graphics card.
6. Download the driver corresponding to your graphics card.
7. Follow the instructions from the manufacturer to install the driver.

### Hardware Acceleration Tips

#### Linux: Disable Desktop Effects

Depending on the graphics hardware, there may be a huge performance drop of the rendering system (up to 10x) when *compiz* desktop effects are on.
Also these visual effects may cause some display bug where the main window of OmniSim is not properly refreshed.
Hence, on Ubuntu (or other Linux) we recommend to deactivate the desktop effects.
You can easily disable them using some tools like *Compiz Config Settings Manager* or *Unity Tweak Tool*.
