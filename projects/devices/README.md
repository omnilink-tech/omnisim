# OmniSim device catalog (xacro)

Reusable URDF/xacro macros for sensors and actuators that mount onto a
robot via a fixed joint. Each device exposes a `xacro:macro` that:

- Creates a child link with visual + collision + inertial blocks at the
  vendor's published geometry.
- Attaches it to the parent link via a fixed joint at the caller's
  `<origin>`.
- Declares the matching `<gazebo><sensor>` block so the OmniSim URDF
  importer materialises the sensor device with vendor-calibrated
  parameters (FoV, resolution, range, layers, etc.).

The catalog replaces the legacy `projects/devices/*/protos/*.proto`
files. Those PROTOs are gone — assemblies that referenced them through
`EXTERNPROTO` paths no longer resolve and need to be re-authored in
xacro form.

## Mounting a device on a robot

Two pieces have to line up:

1. Author the robot itself in xacro and `<xacro:include>` the device
   macro file. Then call the macro inside the robot's `<robot>` block,
   parented to the link you want the sensor attached to.

   ```xml
   <robot name="my_bot" xmlns:xacro="http://ros.org/wiki/xacro">
     <link name="base_link"> ... </link>
     <xacro:include filename="$(find omnisim_devices)/velodyne/urdf/velodyne_vlp16.urdf.xacro"/>
     <xacro:velodyne_vlp16 parent_link="base_link" prefix="velo">
       <origin xyz="0 0 0.30" rpy="0 0 0"/>
     </xacro:velodyne_vlp16>
   </robot>
   ```

2. Render the xacro to a flat `.urdf` before loading in OmniSim
   (`URDFRobot { url ... }` only reads `.urdf`, not `.urdf.xacro`):

   ```bash
   xacro my_bot.urdf.xacro > my_bot.urdf
   ```

3. The robot's bridge controller (or any controller you point at it)
   needs `OMNISIM_URDF_USE_SENSORS=1` in the environment, otherwise the
   importer drops `<gazebo><sensor>` blocks and the bridge's
   `getDevice("velo")` returns `None`. The Mavic 2 Pro and Husky
   bridges both rely on this flag.

The device name a controller sees is whatever `prefix` you passed to
the macro — `getDevice("velo")` in the example above.

## Available devices

Grouped by category. Each xacro lives at
`projects/devices/<vendor>/urdf/<device>.urdf.xacro`.

### Lidars
- **velodyne**: `velodyne_vlp16`, `velodyne_puck`, `velodyne_hdl32e`, `velodyne_hdl64e`
- **slamtec**: `rplidar_a2`
- **hokuyo**: `hokuyo_urg04lx`, `hokuyo_urg04lx_ug01`, `hokuyo_utm30lx`
- **sick**: `sick_lms291`, `sick_ld_mrs`, `sick_s300`
- **robotis**: `lds01`

### Cameras + depth
- **microsoft**: `kinect`
- **orbbec**: `astra`
- **multisense**: `multisense_s21`

### Radars
- **delphi**: `delphi_esr`
- **ibeo**: `ibeo_lux`
- **smartmicro**: `umrr_0a29`, `umrr_0a30`, `umrr_0a31`

### IMU + boards
- **tdk**: `mpu_9250`
- **nvidia**: `jetson_nano`

### Proximity
- **sharp**: `gp2d120`, `gp2y0a02yk0f`, `gp2y0a41sk0f`, `gp2y0a710k0f`

### Grippers + actuators
- **robotiq**: `robotiq_2f85`, `robotiq_2f140`, `robotiq_3f`, `robotiq_epick`
- **generic**: `servo`

## Calibration parameters

Vendor calibration (FoV, resolution, range, etc.) is baked into each
xacro from the upstream Webots PROTO. The xacro form is the source of
truth from this point forward — when a vendor updates a spec, update
the xacro and any rendered URDFs that include it.

## Limitations

- The OmniSim URDF importer needs `OMNISIM_URDF_USE_SENSORS=1` to
  materialise the `<gazebo><sensor>` blocks at all. Without it, only
  the visual/collision/inertial links materialise; the sensor side is
  dropped silently. The importer emits a warning when this happens.
- Multi-layer lidars (Velodyne, Sick LD-MRS) carry their
  `numberOfLayers` + `verticalFieldOfView` via the `<vertical>` scan
  block. The importer started honouring `<vertical>` alongside the
  catalog migration; older OmniSim builds will materialise these as
  single-layer.
