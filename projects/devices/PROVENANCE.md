# `projects/devices/` — provenance

**Original work of this repository**, Apache-2.0 (© OmniLink), like the rest of
the tree. This file exists because `scripts/release/publish_snapshot.sh`
publishes a squashed single commit, so provenance has to live beside the files.

## Contents

30 xacro device macros across 15 vendor directories — lidars (Velodyne, SICK,
Hokuyo, SlamTec, ibeo), radars (Delphi, smartmicro), IR rangers (Sharp), depth
cameras (Microsoft Kinect, Carnegie Robotics MultiSense), an IMU (TDK), a
compute module (NVIDIA Jetson Nano), grippers (Robotiq), a scanner (ROBOTIS) and
a generic servo.

## Geometry

**Every one is parametric: boxes, cylinders and spheres declared inline, driven
by xacro properties. There is not a single mesh, texture or CAD file in this
directory tree** — verified with

    grep -l 'mesh' projects/devices/*/urdf/*.xacro     # returns nothing
                                                        # (the two velodyne hits
                                                        # are xacro:include, not
                                                        # geometry)

so nothing here is imported from, tessellated from, or derived from any
manufacturer's product model. The models capture published mounting dimensions
and field-of-view figures, which are facts rather than expression.

⚠️ One exception existed and was **removed on 2026-08-22**: `orbbec/` referenced
`astra.dae`, a 9.8 MB CAD tessellation of an Orbbec Astra sensor that was sitting
inside the ROBOTIS `turtlebot3_description` package. ROBOTIS' Apache-2.0 grant
covers what ROBOTIS authored and cannot convey rights in Orbbec's CAD, and no
world, controller or PROTO loaded it. Both the mesh and the xacro are gone.

## Names

The vendor and product names in the directory and file names — "Velodyne",
"SICK", "Hokuyo", "Robotiq", "Kinect", "Jetson Nano" and the rest — are the
trademarks of their respective owners and are used **nominatively**: to identify,
factually, which sensor each macro models. This project is not affiliated with,
sponsored by or endorsed by any of them, and neither Apache-2.0 (§6) nor any
other licence here grants rights in those marks. See `TRADEMARKS.md`.

This is the same posture as `projects/robots/dji/mavic/`, which likewise ships
zero mesh data and models the aircraft from primitives.
