## Mesh

```
Mesh {
  field MFString url  [ ]
  field SFBool ccw TRUE
  field SFString name ""
  field SFInt32 materialIndex -1
  field SFVec3f scale 1 1 1
}
```

### Description

The [Mesh](#mesh) node represents a 3D shape imported from an external file created by a 3D modeling software.
The [Mesh](#mesh) node can be used either as a graphical or as a collision detection primitive (in a boundingObject).
Currently, the following formats are supported:
  - [Collada](https://en.wikipedia.org/wiki/COLLADA) files (.dae).
  - [STL](https://en.wikipedia.org/wiki/STL_(file_format)) files (.stl).
  - [Wavefront](https://wiki.fileformat.com/3d/obj) files (.obj).

If the file contains more than one mesh, the meshes will be merged into a single one.

### Field Summary

The `url` field defines the 3D file.
If the `url` value starts with `http://` or `https://`, OmniSim will get the file from the web.
Otherwise, the file should be specified with a relative path.
The same search algorithm as for [ImageTexture](imagetexture.md) is used (cf. [this section](imagetexture.md#search-rule-of-the-texture-path)).
Absolute paths work as well, but they are not recommended because they are not portable across systems.

The `ccw` field indicates whether the vertices of the faces are ordered in a counter-clockwise direction.

The `name` field defines which sub-mesh is included.
If the `name` value is an empty string then all sub-meshes are included.
Note that the `name` field is applied only for Collada files.

The `materialIndex` field applies only to Collada files.
It defines a material for which all the geometries associated with this material will be included.
If `materialIndex` is strictly negative then all the geometries of the Collada file are included.

The `scale` field multiplies the decoded vertices per axis (normals follow, and a mirroring scale with an odd number of negative components keeps the faces pointing outwards).
It is the way to scale a mesh used as a collision primitive: a `boundingObject` cannot contain a [Transform](transform.md), so a `Transform.scale` around a collision mesh is refused.
The URDF importer writes it for a `<collision><mesh scale="...">` (before 2026-10-03 that scale was dropped and the collider loaded at unit scale -- a mesh authored in millimetres collided 1000 times too large).
A zero component is read as 1, with a warning.
