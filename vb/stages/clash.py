"""Stage 3 - fit / clash on the geometry-stage meshes.

  bay fit (component inside its bay, bay inside the OML), protrusions through the skin (external parts
  allowed), pairwise interference (fcl + manifold overlap volume), clearance < min_clearance between
  internal parts, rotating-disc clearance to the airframe and to each other, and whether the battery
  can travel far enough in its bay to fix the CG (needs the mass stage)."""
import json
import shutil
import sys
from pathlib import Path

import numpy as np

LIB = Path(__file__).resolve().parents[1] / "lib" / "geom"
sys.path.insert(0, str(LIB))


def run(ctx):
    import trimesh
    import fit_check
    from geomkit import manifest

    gdir = ctx.updir("geometry")
    gdata = ctx.data("geometry")
    gates_cfg = (ctx.plan.get("gates") or {}).get("clash", {})
    min_clear = gates_cfg.get("min_clearance_mm", 2.0)
    prop_clear = gates_cfg.get("prop_clearance_mm", 10.0)
    allow = list(gdata.get("external_parts", [])) + ["boom", "prop"]
    fit_check.main([str(gdir), "--prop-clearance", str(prop_clear), "--allow-outside", *allow])
    shutil.move(str(gdir / "fit_report.json"), str(ctx.dir / "fit_report.json"))
    rep = json.loads((ctx.dir / "fit_report.json").read_text())

    for r in rep["bay_fit"]:
        if r["part"].startswith("<bay"):
            ctx.gate(f"{r['part']} inside the fuselage OML", r["ok"], True, "bool", "FAIL", note=f"outside {r['outside_mm3']} mm3")
        else:
            m = r.get("margin_xyz_mm") or [None]
            ctx.gate(f"{r['part']} fits {r['bay']} (min margin mm)", min(m) if None not in m else None, 0.0, ">=", "FAIL", "mm",
                     note=f"margins xyz {m} mm")
    bad = [p for p in rep["protrusions"] if not p["allowed"]]
    ctx.gate("internal parts protruding through the skin", len(bad), 0, "<=", "FAIL",
             note=", ".join(f"{p['part']} {100*p['outside_frac']:.0f}%" for p in bad[:6]))
    ctx.gate("component-to-component interference (overlap > 1 mm3)", len(rep["clashes"]), 0, "<=", "FAIL",
             note=", ".join(f"{c['a']} x {c['b']} {c['overlap_mm3']:.0f} mm3" for c in rep["clashes"][:6]))
    for r in rep["prop_clearance"]:
        nm = r["prop"] if isinstance(r["prop"], str) else str(r["prop"])
        ctx.gate(f"{nm} clearance to {r['nearest'] if isinstance(r['nearest'], str) else ' / '.join(r['nearest'])} [mm]",
                 r["distance_mm"], prop_clear, ">=", "FAIL", "mm")

    # ---- clearance between internal parts (fcl distance on AABB-prefiltered pairs) ---------------
    man, parts = manifest.read(str(gdir))
    internal = [p for p in parts if p.mesh is not None and p.role in ("component", "tube")]
    close = []
    clash_pairs = {tuple(sorted((c["a"], c["b"]))) for c in rep["clashes"]}
    for i, a in enumerate(internal):
        la, ha = a.mesh.bounds
        for b in internal[i + 1:]:
            lb, hb = b.mesh.bounds
            if np.any(la - min_clear > hb) or np.any(lb - min_clear > ha):
                continue
            if tuple(sorted((a.name, b.name))) in clash_pairs:
                continue
            cm = trimesh.collision.CollisionManager()
            cm.add_object("a", a.mesh)
            d = cm.min_distance_single(b.mesh)
            if d < min_clear:
                close.append(dict(a=a.name, b=b.name, distance_mm=round(float(d), 2)))
    ctx.gate(f"part pairs closer than {min_clear} mm (wiring/assembly clearance)", len(close), 0, "<=", "WARN",
             note=", ".join(f"{c['a']}/{c['b']} {c['distance_mm']} mm" for c in close[:6]))

    # ---- can the battery fix the CG? -----------------------------------------------------------
    mass = ctx.data("mass", required=False)
    bat = next((r for r in rep["bay_fit"] if "batt" in r["part"].lower() and "fwd_aft_travel_mm" in r), None)
    travel = None
    if mass and bat and mass.get("battery_shift_m") is not None:
        need = mass["battery_shift_m"] * 1000
        fwd, aft = bat["fwd_aft_travel_mm"]
        travel = dict(need_mm=round(need, 1), fwd_mm=fwd, aft_mm=aft)
        ok = (-fwd <= need <= aft)
        tol_mm = mass["cg_tol_m"] * 1000 * mass["mass_kg"] / max(1e-9, sum(
            c["mass_g"] for c in mass["items"] if c["category"] == "energy") / 1000)
        ctx.gate("battery travel in its bay can put the CG on target", ok or abs(need) <= tol_mm, True, "bool", "FAIL",
                 note=f"needs {need:+.0f} mm, bay allows -{fwd:.0f}/+{aft:.0f} mm (else ballast or move other items)")
    data = dict(fit=rep, close_pairs=close, battery_travel=travel, min_clearance_mm=min_clear, prop_clearance_mm=prop_clear)
    summ = dict(clashes=len(rep["clashes"]), protrusions_not_allowed=len(bad), close_pairs=len(close),
                prop_min_clearance_mm=min([r["distance_mm"] for r in rep["prop_clearance"]], default=None),
                bay_failures=sum(not r["ok"] for r in rep["bay_fit"]))
    return dict(summary=summ, data=data)
