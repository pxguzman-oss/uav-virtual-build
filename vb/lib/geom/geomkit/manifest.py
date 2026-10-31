"""Manifest = the hand-off format between stages.

out/
  manifest.json      list of parts (metadata + relative STL path), plan, frame info
  parts/<name>.stl   one mesh per part, mm, structural frame
  oml.stl            watertight outer mould line (CFD)
Both the parametric builder and the STL importer write this; mass properties,
fit check and rendering only read it.
"""
from __future__ import annotations

import json
import os

import numpy as np
import trimesh

from .parametric import Part

FRAME = "structural: origin wing-root LE on centreline, +x aft, +y right, +z up, mm"


def _clean(o):
    if isinstance(o, dict):
        return {k: _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer, np.bool_)):
        return o.item()
    return o


def write(out_dir: str, parts: list[Part], plan: dict, oml: trimesh.Trimesh | None = None, extra: dict | None = None):
    pdir = os.path.join(out_dir, "parts")
    os.makedirs(pdir, exist_ok=True)
    for f in os.listdir(pdir):          # drop stale parts from earlier runs with other plans
        if f.endswith(".stl"):
            os.remove(os.path.join(pdir, f))
    entries = []
    for p in parts:
        e = p.meta()
        if p.mesh is not None:
            rel = f"parts/{p.name}.stl"
            p.mesh.export(os.path.join(out_dir, rel))
            e["file"] = rel
        entries.append(e)
    if oml is not None:
        oml.export(os.path.join(out_dir, "oml.stl"))
    man = dict(frame=FRAME, plan=plan, parts=entries, oml="oml.stl" if oml is not None else None, **(extra or {}))
    with open(os.path.join(out_dir, "manifest.json"), "w") as f:
        json.dump(_clean(man), f, indent=1)
    return man


def read(out_dir: str, load_meshes: bool = True):
    man = json.load(open(os.path.join(out_dir, "manifest.json")))
    parts = []
    for e in man["parts"]:
        e = dict(e)
        f = e.pop("file", None)
        mesh = trimesh.load(os.path.join(out_dir, f), force="mesh") if (f and load_meshes) else None
        parts.append(Part(mesh=mesh, **e))
    return man, parts


def export_glb(path: str, parts: list[Part], oml_alpha: float = 1.0):
    """Coloured GLB of the whole assembly (view in any glTF viewer / browser)."""
    scene = trimesh.Scene()
    for p in parts:
        if p.mesh is None:
            continue
        m = p.mesh.copy()
        c = np.array((p.color or [0.6, 0.6, 0.6]) + [1.0])[:4]
        if p.role == "structure":
            c[3] = oml_alpha
        m.visual = trimesh.visual.ColorVisuals(m, face_colors=(c * 255).astype(np.uint8))
        scene.add_geometry(m, node_name=p.name, geom_name=p.name)
    scene.export(path)
