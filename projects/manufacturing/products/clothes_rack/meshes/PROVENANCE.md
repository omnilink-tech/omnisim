# Clothes rack meshes: provenance

- `connector_half.stl`: one half of the rack's aluminium connector, **as milled by the
  OmniSim CNC twin**. `../assembly.py` writes it (`design.write_half_stl`) from the
  height map the LongMill twin carved (`OMNISIM_MILLING_STOCK_DUMP`), so it is the
  simulated part, not the drawing. OmniLink's own work, Apache-2.0 like the rest of
  this directory.
