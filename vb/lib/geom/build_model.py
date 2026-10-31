#!/usr/bin/env python3
"""Build the parametric model from a YAML build plan.

Outputs (in --out): manifest.json, parts/*.stl, oml.stl (watertight, mm),
oml_m.stl (same in metres, for OpenFOAM/SU2), assembly.glb, build_report.json.

  python build_model.py stork.yaml --out out/stork
  python build_model.py stork.yaml --out out/stork_vtol --set options.vtol=true
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geomkit import geometry as G, manifest, parametric  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("plan")
    ap.add_argument("--out", default="out/model")
    ap.add_argument("--set", action="append", default=[], help="override, e.g. options.vtol=true")
    ap.add_argument("--no-glb", action="store_true")
    a = ap.parse_args(argv)

    t0 = time.perf_counter()
    plan = parametric.load_plan(a.plan, a.set)
    parts, info = parametric.build_parts(plan)
    t1 = time.perf_counter()
    oml = parametric.build_oml(parts)
    t2 = time.perf_counter()
    man = manifest.write(a.out, parts, plan, oml, extra=info)
    oml_m = oml.copy()
    oml_m.apply_scale(1e-3)
    oml_m.export(os.path.join(a.out, "oml_m.stl"))
    if not a.no_glb:
        manifest.export_glb(os.path.join(a.out, "assembly.glb"), parts)
    t3 = time.perf_counter()

    rep = dict(timing_s=dict(parts=round(t1 - t0, 3), oml_union=round(t2 - t1, 3), export=round(t3 - t2, 3)),
               n_parts=len(parts), oml=G.mesh_report(oml),
               parts={p.name: dict(watertight=bool(p.mesh.is_watertight), faces=len(p.mesh.faces))
                      for p in parts if p.mesh is not None})
    bad = [k for k, v in rep["parts"].items() if not v["watertight"]]
    json.dump(rep, open(os.path.join(a.out, "build_report.json"), "w"), indent=1)
    o = rep["oml"]
    print(f"built {len(parts)} parts in {rep['timing_s']['parts']} s; OML union {rep['timing_s']['oml_union']} s; "
          f"export {rep['timing_s']['export']} s")
    print(f"OML: watertight={o['watertight']} bodies={o['bodies']} faces={o['faces']} "
          f"volume={o['volume_mm3']/1e6:.3f} L area={o['area_mm2']/1e4:.1f} dm2 bounds={o['bounds_mm']}")
    if bad:
        print("WARNING non-watertight parts:", bad)
    return 0 if o["watertight"] and o["bodies"] == 1 else 1


if __name__ == "__main__":
    sys.exit(main())
