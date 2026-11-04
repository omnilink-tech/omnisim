# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Turn a CNC machine's CAD into a mesh pack the twin world can move.

A pack is the machine's geometry cut into motion groups (the fixed frame plus
one body per moving axis), every mesh expressed in ONE frame: the CAD
assembly's own, converted to metres and Z-up, then shifted so the machine
stands on z = 0, centred in x/y. Because a CNC's axes only translate, a body's
meshes never need re-posing: at the CAD snapshot's carriage positions every
body frame coincides with the pack frame, and `world.py` offsets each body by
its axis direction times (power-up position - CAD position).

The manifest (JSON) names the source and how to cut it:

    {
      "name": "longmill_mk2_30x30",
      "source": {"url": "...", "licence": "...", "attribution": "..."},
      "units": "mm", "up": "z", "tolerance_mm": 0.3,
      "parts": "machine.step",                       # one assembly ...
      "groups": {"z": ["(?i)z.?axis|spindle|router"],  # ... cut by regex on each part's
                 "carriage": ["(?i)x.?carriage"],    # assembly path; first match wins,
                 "gantry": ["(?i)gantry|y.?plate"]}, # unmatched parts -> "frame"
      "exclude": ["(?i)wood screw"],                 # optional: parts to drop
      "rod": ["(?i)lead screw"],                     # optional: parts replaced by their bounding cylinder
      #  or "groups": {"gantry": {"files": ["gantry.step"]}, ...}  # one file per group
      "bodies": [{"group": "gantry",   "axis": "y", "parent": "frame"},
                 {"group": "carriage", "axis": "x", "parent": "gantry"},
                 {"group": "z",        "axis": "z", "parent": "carriage"}],
      "axis_dirs": {"x": [1, 0, 0], "y": [0, 1, 0], "z": [0, 0, 1]},  # +MPos, CAD frame after `up`
      "cad_pose_s": [x, y, z],        # each carriage's rail position (mm from its min end) in the CAD
      "tool_tip": [x, y, z],          # optional, CAD coordinates (units) at that pose, or
                                      # "lowest": the lowest point of "tool_group" (default "z")
      "tool_part": "(?i)router",      # optional: of the parts matching it, the one reaching
                                      # lowest is the CAD's own bit (and the collet it sits in).
                                      # It is left out of the meshes, so the twin can draw the
                                      # tool actually loaded; its profile gives `collet` in the pack.
      "masses_kg": {"gantry": 3.0, "carriage": 1.0, "z": 1.5}
    }

    python cad_pack.py manifest.json --out projects/manufacturing/machines/<name>/cad

writes `meshes/<group>_<n>.stl` (one per CAD colour per group) and `pack.json`.
The pack keeps the manifest's source and licence; ship a PROVENANCE.md with it.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path
from typing import Dict, List

UNIT_SCALE = {"mm": 0.001, "cm": 0.01, "m": 1.0, "in": 0.0254}
UP_ROTATIONS = {"z": (0.0, [0, 0, 1]), "-z": (math.pi, [1, 0, 0]), "y": (math.pi / 2, [1, 0, 0]),
                "-y": (-math.pi / 2, [1, 0, 0]), "x": (-math.pi / 2, [0, 1, 0]), "-x": (math.pi / 2, [0, 1, 0])}
FACE_BUDGET = 400_000      # per pack; above this the world gets slow to load and render
SHADOW_FACE_LIMIT = 21_000 # the renderer drops shadows (and warns) for a mesh over 21845 triangles
MESH_SUFFIXES = {".stl", ".obj", ".ply", ".3mf", ".glb", ".gltf"}


class PackError(ValueError):
    pass


def _load_placed(path: Path, tol_mm: float, scratch: Path, mesh_scale: float, angular_rad: float = 0.5):
    """Every placed solid of a CAD/mesh file: [(node name, trimesh.Trimesh in metres)].

    cascadio writes glTF, which is metres whatever the STEP's units; plain mesh
    files carry no units, so `mesh_scale` (the manifest's `units`) applies to them.
    """
    import trimesh

    if path.suffix.lower() in (".step", ".stp"):
        import cascadio
        glb = scratch / f"{path.stem}_{tol_mm:g}_{angular_rad:g}.glb"
        # cascadio tessellates in the file's units; the tolerance is in those units too.
        cascadio.step_to_glb(str(path), str(glb), float(tol_mm), float(angular_rad))
        loaded = trimesh.load(str(glb))
        mesh_scale = 1.0
    elif path.suffix.lower() in MESH_SUFFIXES:
        loaded = trimesh.load(str(path))
    else:
        raise PackError(f"unsupported CAD/mesh format: {path.name}")
    if not isinstance(loaded, trimesh.Scene):
        loaded.apply_scale(mesh_scale)
        return [(path.stem, loaded)]
    out = []
    graph = loaded.graph
    parents = graph.transforms.parents

    def assembly_path(node):
        names = []
        while node != graph.base_frame:
            names.append(node)
            node = parents.get(node, graph.base_frame)
        return " / ".join(reversed(names))

    # Apply the scene graph: placements live there, and an instanced part (one solid
    # placed twice) must come out twice (see scripts/dev/step_to_urdf.py). Each part
    # is named by its full assembly path, so a group can be cut by subassembly.
    for node in graph.nodes_geometry:
        transform, geom_name = graph[node]
        geom = loaded.geometry.get(geom_name)
        if geom is None:
            continue
        m = geom.copy()
        m.apply_transform(transform)
        m.apply_scale(mesh_scale)
        out.append((assembly_path(node), m))
    return out


def _rgba(mesh) -> List[float]:
    mat = getattr(getattr(mesh, "visual", None), "material", None)
    col = getattr(mat, "baseColorFactor", None) if mat is not None else None
    if col is None:
        face_colors = getattr(getattr(mesh, "visual", None), "face_colors", None)
        col = face_colors[0] if face_colors is not None and len(face_colors) else [180, 180, 180, 255]
    col = [float(c) for c in col]
    if max(col) > 1.0:
        col = [c / 255.0 for c in col]
    return [round(c, 3) for c in (col + [1.0])[:4]]


def _chunks(meshes):
    """Concatenate parts into meshes of at most SHADOW_FACE_LIMIT faces (a bigger part is cut by faces)."""
    import trimesh

    pieces = []
    for m in meshes:
        if len(m.faces) <= SHADOW_FACE_LIMIT:
            pieces.append(m)
            continue
        for start in range(0, len(m.faces), SHADOW_FACE_LIMIT):
            idx = list(range(start, min(start + SHADOW_FACE_LIMIT, len(m.faces))))
            pieces.append(m.submesh([idx], append=True))
    batch, count = [], 0
    for p in pieces:
        if batch and count + len(p.faces) > SHADOW_FACE_LIMIT:
            yield trimesh.util.concatenate(batch)
            batch, count = [], 0
        batch.append(p)
        count += len(p.faces)
    if batch:
        yield trimesh.util.concatenate(batch)


def build_pack(manifest: dict, base: Path, out: Path, scratch: Path) -> dict:
    import numpy as np
    import trimesh

    for key in ("name", "bodies", "axis_dirs", "cad_pose_s", "groups", "source"):
        if key not in manifest:
            raise PackError(f"manifest is missing '{key}'")
    scale = UNIT_SCALE.get(manifest.get("units", "mm"))
    if scale is None:
        raise PackError(f"units must be one of {sorted(UNIT_SCALE)}")
    angle, axis = UP_ROTATIONS[manifest.get("up", "z")]
    R = trimesh.transformations.rotation_matrix(angle, axis)
    # STEP tessellation tolerance is in the CAD's own units (mm for nearly all CNC CAD).
    tol = float(manifest.get("tolerance_mm", 0.3))
    angular = float(manifest.get("angular_rad", 0.5))

    group_names = ["frame"] + [b["group"] for b in manifest["bodies"]]
    placed: Dict[str, List] = {g: [] for g in group_names}
    groups = manifest["groups"]
    if "parts" in manifest:
        patterns = {g: [re.compile(p) for p in pats] for g, pats in groups.items()}
        unknown = set(patterns) - set(group_names)
        if unknown:
            raise PackError(f"groups without a body: {sorted(unknown)}")
        exclude = [re.compile(p) for p in manifest.get("exclude", [])]
        rods = [re.compile(p) for p in manifest.get("rod", [])]
        tool_part = re.compile(manifest["tool_part"]) if manifest.get("tool_part") else None
        tool_candidates = []
        # Optionally some groups at a coarser tessellation ("coarse": {"tolerance_mm",
        # "angular_rad", "groups"}): a printer's static frame need not carry the detail of
        # the toolhead the camera sits next to. The CAD is then tessellated twice.
        coarse = manifest.get("coarse") or {}
        coarse_groups = set(coarse.get("groups", []))
        sources = [(_load_placed(base / manifest["parts"], tol, scratch, scale, angular), False)]
        if coarse_groups:
            sources.append((_load_placed(base / manifest["parts"], float(coarse["tolerance_mm"]), scratch, scale,
                                         float(coarse.get("angular_rad", angular))), True))
        for name, m, is_coarse in ((n, m, c) for parts, c in sources for n, m in parts):
            if any(p.search(name) for p in exclude):
                continue
            # First group whose pattern matches wins, in manifest order: list the
            # innermost (most specific) group first.
            hit = next((g for g, pats in patterns.items() if any(p.search(name) for p in pats)), "frame")
            if (hit in coarse_groups) != is_coarse:
                continue
            if tool_part is not None and tool_part.search(name):
                tool_candidates.append((name, m))
            if any(p.search(name) for p in rods):
                # A threaded screw tessellates into ~6x10^4 triangles and its convex hull is
                # no lighter (every crest is on the hull); its bounding cylinder is 64.
                colour = _rgba(m)
                cyl = trimesh.bounds.minimum_cylinder(m)
                m = trimesh.creation.cylinder(radius=cyl["radius"], height=cyl["height"],
                                              transform=cyl["transform"], sections=16)
                m.visual = trimesh.visual.ColorVisuals(m, face_colors=[int(c * 255) for c in colour])
            placed[hit].append((name, m))
    else:
        for g, spec in groups.items():
            if g not in placed:
                raise PackError(f"group '{g}' has no body")
            for f in spec["files"]:
                placed[g].extend(_load_placed(base / f, tol, scratch, scale))
    bit = None
    if "parts" in manifest and manifest.get("tool_part"):
        if not tool_candidates:
            raise PackError("tool_part matched no parts")
        # Part names carry the exporter's random suffixes: the bit is found by where it is.
        bit = min(tool_candidates, key=lambda nm: nm[1].bounds[0][2])
        for g in placed:
            placed[g] = [(n, m) for n, m in placed[g] if m is not bit[1]]
    for g in group_names[1:]:
        if not placed[g]:
            raise PackError(f"group '{g}' matched no parts")

    # Pack frame: metres, Z-up, machine standing on z=0 and centred in x/y.
    for g in placed:
        for _, m in placed[g]:
            m.apply_transform(R)
    if bit is not None:
        bit[1].apply_transform(R)
    every = [m for g in placed for _, m in placed[g]]
    lo = np.min([m.bounds[0] for m in every], axis=0)
    hi = np.max([m.bounds[1] for m in every], axis=0)
    shift = np.array([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])

    meshes_dir = out / "meshes"
    meshes_dir.mkdir(parents=True, exist_ok=True)
    pack_groups, total_faces = {}, 0
    for g, items in placed.items():
        buckets: Dict[tuple, list] = {}
        for _, m in items:
            m.apply_translation(shift)
            buckets.setdefault(tuple(_rgba(m)), []).append(m)
        entries = []
        bmin, bmax = np.full(3, np.inf), np.full(3, -np.inf)
        n = 0
        for rgba, ms in sorted(buckets.items()):
            for mesh in _chunks(ms):
                fn = f"{g}_{n}.stl"
                n += 1
                mesh.export(str(meshes_dir / fn))
                total_faces += len(mesh.faces)
                bmin, bmax = np.minimum(bmin, mesh.bounds[0]), np.maximum(bmax, mesh.bounds[1])
                entries.append({"file": f"meshes/{fn}", "rgba": list(rgba), "faces": int(len(mesh.faces))})
        pack_groups[g] = {"meshes": entries, "parts": len(items),
                          "bbox": [[round(float(v), 5) for v in bmin], [round(float(v), 5) for v in bmax]]}

    dirs = {}
    for a, d in manifest["axis_dirs"].items():
        v = (R[:3, :3] @ np.array(d, dtype=float))
        n = float(np.linalg.norm(v))
        if n == 0:
            raise PackError(f"axis_dirs.{a} is zero")
        dirs[a] = [round(float(c) / n, 6) for c in v]
    tool_tip, collet = None, None
    if bit is not None:
        bit[1].apply_translation(shift)
    if manifest.get("tool_tip") == "lowest":
        # The cutter is the lowest thing on the tool body: its tip is the centre of the
        # vertices within 0.3 mm of that body's lowest point (a flat end is a ring of them).
        group = manifest.get("tool_group", "z")
        verts = bit[1].vertices if bit is not None else np.vstack([m.vertices for _, m in placed[group]])
        low = verts[verts[:, 2] <= verts[:, 2].min() + 0.0003]
        tool_tip = [round(float(low[:, 0].mean()), 6), round(float(low[:, 1].mean()), 6),
                    round(float(verts[:, 2].min()), 6)]
    elif "tool_tip" in manifest:
        tip = np.array(list(manifest["tool_tip"]) + [1.0])
        tip[:3] *= scale
        tool_tip = (R @ tip)[:3] + shift
        tool_tip = [round(float(v), 6) for v in tool_tip]
    if bit is not None and tool_tip is not None:
        # The collet the bit sits in, from the part's profile around the tool axis: the
        # nose is where the shank (the bit's own radius) ends, the body its widest radius.
        v = bit[1].vertices
        r = np.hypot(v[:, 0] - tool_tip[0], v[:, 1] - tool_tip[1])
        shank = float(r[v[:, 2] < tool_tip[2] + 0.005].max())
        nose = float(v[r <= shank + 5e-5, 2].max())
        wide = float(r.max())
        body = float(v[r >= wide - 5e-5, 2].min())
        collet = {"nose_z_m": round(nose, 6), "nose_radius_m": round(shank, 6), "body_z_m": round(body, 6),
                  "radius_m": round(wide, 6), "top_z_m": round(float(v[:, 2].max()), 6),
                  "rgba": list(_rgba(bit[1]))}
    pack = {
        "name": manifest["name"], "source": manifest["source"],
        "groups": pack_groups, "bodies": manifest["bodies"], "axis_dirs": dirs,
        "cad_pose_s": [float(v) for v in manifest["cad_pose_s"]], "tool_tip": tool_tip,
        # The collet the twin draws its tools from (only with "tool_part").
        "collet": collet,
        # Spoilboard top, pack frame (the manifest gives it in mm, already Z-up and grounded).
        "bed_top_m": (float(manifest["bed_top_mm"]) / 1000.0) if "bed_top_mm" in manifest else None,
        "masses_kg": manifest.get("masses_kg", {}),
        "extent_m": [round(float(v), 5) for v in (hi - lo)],
        "faces": total_faces,
    }
    if total_faces > FACE_BUDGET:
        pack["warning"] = (f"{total_faces} faces is over the {FACE_BUDGET} budget: raise tolerance_mm "
                           f"or drop hidden parts (fasteners) from the CAD")
    (out / "pack.json").write_text(json.dumps(pack, indent=1), encoding="utf-8", newline="\n")
    return pack


def main(argv=None) -> int:
    import tempfile

    p = argparse.ArgumentParser(description="Cut CNC CAD into a twin mesh pack.")
    p.add_argument("manifest")
    p.add_argument("--out", required=True)
    p.add_argument("--cad-dir", help="where the CAD files are (default: next to the manifest); "
                                     "CAD under a share-alike or large licence need not live in the repo")
    args = p.parse_args(argv)
    manifest_path = Path(args.manifest).resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        pack = build_pack(manifest, Path(args.cad_dir) if args.cad_dir else manifest_path.parent, out, Path(tmp))
    summary = {g: (v["parts"], sum(e["faces"] for e in v["meshes"])) for g, v in pack["groups"].items()}
    print(f"wrote {out / 'pack.json'}: extent {pack['extent_m']} m, (parts, faces) per group {summary}")
    if "warning" in pack:
        print("WARNING:", pack["warning"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
