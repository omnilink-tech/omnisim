## Generalities

OmniSim world files use the ".omniworld" file name extension; legacy ".wbt" files are read forever but never written (a ".wbt" opened in OmniSim saves as ".omniworld").
The first line of an ".omniworld" file uses this header (a legacy ".wbt" carries `#VRML_SIM R2025a utf8` instead):

```
#OMNISIM R2025a utf8
```

The version *R2025a* specifies that the file can be open with *OmniSim 2025a*.
Although the header specifies *utf8*, at the moment only ascii is supported.

The comments placed just below the header store the window configuration associated with this world.

One (and only one) instance of each of the `WorldInfo, Viewpoint` and `Background` nodes must be present in every world file.
For example:

```
#OMNISIM R2025a utf8

WorldInfo {
  info [
    "Description"
    "Author: first name last name <e-mail>"
    "Date: DD MMM YYYY"
  ]
}
Viewpoint {
  orientation 1 0 0 -0.8
  position 0.25 0.708035 0.894691
}
Background {
  skyColor [
    0.4 0.7 1
  ]
}
PointLight {
  ambientIntensity 0.54
  intensity 0.5
  location 0 1 0
}
```
