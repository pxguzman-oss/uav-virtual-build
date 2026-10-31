#!/usr/bin/env python3
"""Fit / clash check for a model directory.

  1. bay fit     : components with `bay:` must lie inside the bay box (+ per-axis margins)
  2. inside OML  : internal parts (components, tubes) must not protrude through the skin
  3. clashes     : pairwise interference between internal parts (fcl broad phase via
                   trimesh.collision.CollisionManager, overlap volume via manifold3d)
  4. prop clearance: rotating discs vs airframe and vs each other (min distance)

  python fit_check.py out/stork [--tol-mm3 1.0] [--prop-clearance 10]
Exit code 1 if anything fails.
"""
import argparse
import json
import os
import sys
import time

import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geomkit import geometry as G, manifest  # noqa: E402


def overlap_volume(a: trimesh.Trimesh, b: trimesh.Trimesh) -> float:
    try:
        return float((G.to_manifold(a) ^ G.to_manifold(b)).volume())
    except ValueError:
        return float("nan")


def outside_volume(a: trimesh.Trimesh, container: trimesh.Trimesh) -> float:
    return float((G.to_manifold(a) - G.to_manifold(container)).volume())


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("model_dir")
    ap.add_argument("--tol-mm3", type=float, default=1.0, help="ignore overlaps smaller than this")
    ap.add_argument("--prop-clearance", type=float, default=10.0, help="required prop clearance, mm")
    ap.add_argument("--allow-outside", nargs="*", default=["gps", "motor", "lift_motor", "vtol_boom", "servo_tail"],
                    help="name prefixes allowed to protrude (externally mounted)")
    ap.add_argument("--prop-exclude", nargs="*", default=["motor", "lift_motor"],
                    help="name prefixes ignored by the rotor clearance check (the motors the rotors sit on)")
    a = ap.parse_args(argv)
    t0 = time.perf_counter()
    man, parts = manifest.read(a.model_dir)
    plan = man["plan"]
    oml = trimesh.load(os.path.join(a.model_dir, man["oml"]), force="mesh")
    bays = plan["airframe"].get("bays", {})
    rep = dict(bay_fit=[], protrusions=[], clashes=[], prop_clearance=[], failures=0)

    # 1 bay fit -------------------------------------------------------------
    for p in parts:
        if not p.bay:
            continue
        bd = bays[p.bay]
        bc, bs = np.asarray(bd["center"], float), np.asarray(bd["size"], float)
        lo, hi = p.mesh.bounds
        margin_lo = lo - (bc - bs / 2)
        margin_hi = (bc + bs / 2) - hi
        margins = np.minimum(margin_lo, margin_hi)
        bay_box = G.box(bs, bc)
        out_v = outside_volume(p.mesh, bay_box)
        ok = out_v <= a.tol_mm3 and bool(np.all(margins >= 0))
        # also check the bay itself sits inside the fuselage skin
        rep["bay_fit"].append(dict(part=p.name, bay=p.bay, ok=ok, outside_mm3=round(out_v, 1),
                                   margin_xyz_mm=np.round(margins, 1).tolist(),
                                   fwd_aft_travel_mm=[round(float(margin_lo[0]), 1), round(float(margin_hi[0]), 1)]))
        rep["failures"] += not ok
    for name, bd in bays.items():
        bay_box = G.box(bd["size"], bd["center"])
        v = outside_volume(bay_box, oml)
        rep["bay_fit"].append(dict(part=f"<bay {name}>", bay="fuselage OML", ok=v <= a.tol_mm3, outside_mm3=round(v, 1)))
        rep["failures"] += v > a.tol_mm3

    # 2 protrusion through OML ---------------------------------------------
    internal = [p for p in parts if p.mesh is not None and p.role in ("component", "tube")]
    for p in internal:
        v = outside_volume(p.mesh, oml)
        if v > a.tol_mm3:
            allowed = any(p.name.startswith(x) for x in a.allow_outside)
            frac = v / p.mesh.volume
            rep["protrusions"].append(dict(part=p.name, outside_mm3=round(v, 1), outside_frac=round(frac, 3), allowed=allowed))
            rep["failures"] += not allowed

    # 3 pairwise clashes (fcl broad/narrow phase, manifold for volume) -------
    cm = trimesh.collision.CollisionManager()
    for p in internal:
        cm.add_object(p.name, p.mesh)
    hit, names = cm.in_collision_internal(return_names=True)
    byname = {p.name: p for p in parts}
    for n1, n2 in sorted(names):
        v = overlap_volume(byname[n1].mesh, byname[n2].mesh)
        if v > a.tol_mm3:
            rep["clashes"].append(dict(a=n1, b=n2, overlap_mm3=round(v, 1)))
            rep["failures"] += 1

    # 4 prop clearance ------------------------------------------------------
    props = [p for p in parts if p.role == "prop" and p.mesh is not None]
    if props:
        airframe = trimesh.collision.CollisionManager()
        airframe.add_object("oml", oml)
        for p in parts:
            if p.role == "component" and p.mesh is not None and not any(p.name.startswith(x) for x in a.prop_exclude):
                airframe.add_object(p.name, p.mesh)
        for p in props:
            d, nm = airframe.min_distance_single(p.mesh, return_name=True)
            ok = d >= a.prop_clearance
            rep["prop_clearance"].append(dict(prop=p.name, nearest=nm, distance_mm=round(float(d), 1), ok=ok))
            rep["failures"] += not ok
        if len(props) > 1:
            pm = trimesh.collision.CollisionManager()
            for p in props:
                pm.add_object(p.name, p.mesh)
            d, nm = pm.min_distance_internal(return_names=True)
            ok = d >= a.prop_clearance
            rep["prop_clearance"].append(dict(prop="prop-to-prop", nearest=list(nm), distance_mm=round(float(d), 1), ok=ok))
            rep["failures"] += not ok

    rep["time_s"] = round(time.perf_counter() - t0, 2)
    json.dump(manifest._clean(rep), open(os.path.join(a.model_dir, "fit_report.json"), "w"), indent=1)

    print("BAY FIT")
    for r in rep["bay_fit"]:
        extra = f" margins xyz {r['margin_xyz_mm']} mm, fwd/aft travel {r['fwd_aft_travel_mm']}" if "margin_xyz_mm" in r else ""
        print(f"  [{'OK ' if r['ok'] else 'BAD'}] {r['part']:22s} in {r['bay']:16s} outside={r['outside_mm3']} mm3{extra}")
    print("PROTRUSIONS through OML" + ("" if rep["protrusions"] else ": none"))
    for r in rep["protrusions"]:
        print(f"  [{'ok ' if r['allowed'] else 'BAD'}] {r['part']:22s} {r['outside_mm3']:9.1f} mm3 ({100*r['outside_frac']:.0f}% of part)"
              + ("  (external mount allowed)" if r["allowed"] else ""))
    print("CLASHES" + ("" if rep["clashes"] else ": none"))
    for r in rep["clashes"]:
        print(f"  [BAD] {r['a']} x {r['b']}: {r['overlap_mm3']} mm3")
    print("PROP CLEARANCE")
    for r in rep["prop_clearance"]:
        print(f"  [{'OK ' if r['ok'] else 'BAD'}] {r['prop']:14s} nearest {r['nearest']} at {r['distance_mm']} mm")
    print(f"{rep['failures']} failure(s); {rep['time_s']} s -> {a.model_dir}/fit_report.json")
    return 1 if rep["failures"] else 0


if __name__ == "__main__":
    sys.exit(main())
