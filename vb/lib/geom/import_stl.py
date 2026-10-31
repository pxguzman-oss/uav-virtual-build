#!/usr/bin/env python3
"""Assemble a model from user STL/STEP parts (YAML mapping), repair them, compute
printed-part masses, merge a watertight OML, and write the same manifest/out layout as
build_model.py (so mass_properties / fit_check / render work unchanged).

  python import_stl.py assembly.yaml --out out/stork_stl [--folder /path/to/stl_pack]

Printed-part mass model (mass_model: shell, the default here):
  m = A_surface * wall_mm * rho  +  (V_enclosed - A_surface * wall_mm) * infill * rho
CG/inertia: exact thin-shell integrals for the skin + uniform solid for the infill.
Use mass_model: solid for parts that are printed 100 % (or whose STL models every wall),
or give `mass:` (weighed grams) to override (geometry then only distributes it).

YAML:
  base_plan: stork.yaml        # materials, groups, bays, tubes, components, reference
  folder: stl                  # where the files live (relative to this YAML; --folder overrides)
  replace_structure: true      # drop parametric printed skins, keep tubes/components
  oml_method: union            # union (manifold3d) | voxel (wrap, closes gaps)
  voxel_pitch: 2.0
  defaults:                    # applied to every part unless overridden
    {material: lw_pla_foamed, mass_model: shell, wall_mm: 0.45, infill: 0.04, group: printed, units: mm}
  parts:
    - name: wing_1_R
      file: "WING 1 R.stl"     # or glob: "WING ? R.stl" -> one part per match, name from file
      material: lw_pla_foamed
      wall_mm: 0.45
      infill: 0.04
      role: structure          # structure (OML) | component | tube
      in_oml: true
      transform: {mirror: y, rotate_deg: [0,0,0], translate: [0,0,0]}  # or center_at: [x,y,z]
  ignore: ["*support*"]        # files in the folder that are deliberately not used
"""
import argparse
import fnmatch
import glob
import json
import os
import re
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geomkit import geometry as G, manifest, parametric, stl_import as SI  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("assembly")
    ap.add_argument("--out", default="out/stl_model")
    ap.add_argument("--folder", default=None, help="folder with the STL/STEP files (overrides YAML folder:)")
    ap.add_argument("--oml-method", choices=["union", "voxel"], default=None)
    ap.add_argument("--voxel-pitch", type=float, default=None)
    a = ap.parse_args(argv)
    t0 = time.perf_counter()
    asm = yaml.safe_load(open(a.assembly))
    root = os.path.dirname(os.path.abspath(a.assembly))
    plan = parametric.load_plan(os.path.join(root, asm["base_plan"]), asm.get("set", [])) if asm.get("base_plan") else \
        dict(materials={}, groups={}, airframe={})
    plan["name"] = asm.get("name", plan.get("name", "imported"))

    base_parts, info = parametric.build_parts(plan) if asm.get("base_plan") else ([], {})
    if asm.get("replace_structure", True):
        base_parts = [p for p in base_parts if p.role != "structure"]

    folder = a.folder or asm.get("folder", ".")
    folder = folder if os.path.isabs(folder) else os.path.join(root, folder)
    defaults = dict(units=asm.get("units", "auto"), group="printed", mass_model="shell",
                    material="lw_pla_foamed", wall_mm=0.45, infill=0.04)
    defaults.update(asm.get("defaults", {}))
    specs = []
    for spec in asm["parts"]:
        if "glob" in spec:
            hits = sorted(glob.glob(os.path.join(folder, spec["glob"])))
            if not hits:
                print(f"WARNING glob {spec['glob']!r} matched nothing in {folder}")
            for h in hits:
                nm = re.sub(r"[^A-Za-z0-9]+", "_", os.path.splitext(os.path.basename(h))[0]).strip("_")
                specs.append(dict(spec, name=nm, file=h))
        else:
            specs.append(spec)

    log = {}
    imported = []
    used = set()
    groups = plan.get("groups", {})
    for spec in specs:
        spec = {**defaults, **spec}
        tp = time.perf_counter()
        f = spec["file"] if os.path.isabs(spec["file"]) else os.path.join(folder, spec["file"])
        used.add(os.path.abspath(f))
        raw = SI.load_any(f)
        k, unit, why = SI.detect_units(raw, spec.get("expected_extent_mm"), spec.get("units", "auto"))
        raw.apply_scale(k)
        fixed, rlog = SI.repair(raw)
        placed = SI.place(fixed, spec.get("transform"))
        log[spec["name"]] = dict(file=os.path.relpath(f, root), unit=unit, unit_reason=why,
                                 in_watertight=bool(raw.is_watertight), out_watertight=bool(placed.is_watertight),
                                 repair=rlog, extents_mm=np.round(placed.extents, 1).tolist(),
                                 area_mm2=round(float(placed.area), 1),
                                 volume_mm3=round(float(placed.volume), 1) if placed.is_watertight else None,
                                 material=spec.get("material"), mass_model=spec["mass_model"],
                                 wall_mm=spec.get("wall_mm"), infill=spec.get("infill"),
                                 time_s=round(time.perf_counter() - tp, 2))
        role = spec.get("role", "structure")
        imported.append(parametric.Part(
            spec["name"], placed, spec["group"], role, spec["mass_model"], spec.get("mass"), spec.get("material"),
            spec.get("wall_mm"), spec.get("infill", 0.0) or 0.0, spec.get("in_oml", role == "structure"),
            spec.get("bay"), None, spec.get("color", groups.get(spec["group"], {}).get("color")),
            spec.get("label", spec["name"])))
    if os.path.isdir(folder):
        ign = asm.get("ignore", [])
        unmapped = sorted(os.path.relpath(p, folder) for p in glob.glob(os.path.join(folder, "**", "*"), recursive=True)
                          if os.path.splitext(p)[1].lower() in (".stl", ".step", ".stp", ".obj", ".3mf")
                          and os.path.abspath(p) not in used
                          and not any(fnmatch.fnmatch(os.path.basename(p), g) for g in ign))
    else:
        unmapped = []
    t1 = time.perf_counter()

    parts = imported + base_parts
    oml_in = [p.mesh for p in parts if p.in_oml]
    method = a.oml_method or asm.get("oml_method", "union")
    oml = None
    if method == "union":
        try:
            oml = SI.union_oml(oml_in)
        except ValueError as e:
            print("union failed:", e, "-> falling back to voxel wrap")
        if oml is not None and oml.body_count != 1:
            print(f"union produced {oml.body_count} bodies (gaps between parts?) -> falling back to voxel wrap")
            oml = None
    if oml is None:
        method = "voxel"
        oml = SI.voxel_wrap(oml_in, a.voxel_pitch or asm.get("voxel_pitch", 2.0))
    t2 = time.perf_counter()

    manifest.write(a.out, parts, plan, oml, extra=dict(info, import_log=log, oml_method=method, unmapped_files=unmapped))
    import trimesh
    trimesh.util.concatenate([p.mesh for p in parts if p.mesh is not None and p.role != "prop"]).export(
        os.path.join(a.out, "assembly.stl"))
    oml_m = oml.copy(); oml_m.apply_scale(1e-3); oml_m.export(os.path.join(a.out, "oml_m.stl"))
    manifest.export_glb(os.path.join(a.out, "assembly.glb"), parts)
    rep = dict(timing_s=dict(import_repair=round(t1 - t0, 2), oml=round(t2 - t1, 2)), oml_method=method,
               oml=G.mesh_report(oml), import_log=log, unmapped_files=unmapped)
    json.dump(manifest._clean(rep), open(os.path.join(a.out, "build_report.json"), "w"), indent=1)
    for n, l in log.items():
        print(f"  {n:14s} {l['unit']:>3s} ({l['unit_reason']}); in watertight={l['in_watertight']}; repair: {', '.join(l['repair'])}  [{l['time_s']} s]")
    if unmapped:
        print(f"WARNING {len(unmapped)} file(s) in {folder} not mapped (add to parts: or ignore:): {unmapped[:12]}")
    span = oml.extents[1]
    ref = asm.get("reference_span_mm") or plan.get("reference", {}).get("span")
    print(f"OML via {method}: watertight={oml.is_watertight} bodies={oml.body_count} faces={len(oml.faces)} "
          f"volume={oml.volume/1e6:.3f} L span={span:.0f} mm" + (f" (ref {ref}, {100*(span/ref-1):+.1f}%)" if ref else ""))
    print(f"import+repair {t1-t0:.2f} s, OML {t2-t1:.2f} s -> {a.out}")
    return 0 if oml.is_watertight else 1


if __name__ == "__main__":
    sys.exit(main())
