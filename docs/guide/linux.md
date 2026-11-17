## Linux

### "ssh -X"

Do not expect the 3D view to work over an `ssh -X` (X tunneling) connection.
OmniSim renders through wgpu-native, which needs Vulkan on Linux, and Vulkan has
no equivalent of GLX indirect rendering — there is nothing for X to tunnel. The
failure is quiet: wgpu-native fails to initialise, OmniSim logs one line, and you
get physics and controllers with **no renderer at all**.

For remote work use a headless run instead (`python3 -m omnisim run-headless
<world>`, under Xvfb), the harness's `POST /world/screenshot`, or the
`--stream` web viewer — see [Web Streaming](web-streaming.md).
