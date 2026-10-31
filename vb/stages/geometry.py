"""Stage 1 - geometry: parametric airframe (or real STL/STEP parts) from the plan -> watertight OML,
per-part meshes, geometry-based mass properties (printed shells scaled to the plan masses), balance chart,
Blender Workbench renders (matplotlib fallback).

quick: 2 views (iso, xray_side) | standard: 6 views | full: 7 views (+front)"""
import json
import os
import shutil
import sys
import time
from pathlib import Path

import numpy as np
import yaml

from .. import env
from ..adapters import plan2geom

LIB = Path(__file__).resolve().parents[1] / "lib" / "geom"
sys.path.insert(0, str(LIB))

VIEWS = {"quick": "iso,xray_side", "standard": "iso,top,side,xray,xray_side,xray_top",
         "full": "iso,top,side,front,xray,xray_side,xray_top"}


def run(ctx):
    from geomkit import geometry as G, manifest, massprops as MP, parametric  # noqa
    import balance_chart
    import render

    plan = ctx.plan
    d = ctx.dir
    t = {}
    t0 = time.time()
    g = plan2geom.build(plan)
    yaml.safe_dump(json.loads(json.dumps(g)), open(d / "geom_plan.yaml", "w"), sort_keys=False)
    imp = plan["airframe"].get("cad_import")
    if imp:
        import import_stl
        asm = dict(imp)
        asm["base_plan"] = str(d / "geom_plan.yaml")
        folder = asm.get("folder")
        if folder and not os.path.isabs(folder):
            asm["folder"] = str((ctx.plan_path.parent / folder).resolve())
        for p in asm.get("parts", []):
            if "file" in p and not os.path.isabs(p["file"]) and not asm.get("folder"):
                p["file"] = str((ctx.plan_path.parent / p["file"]).resolve())
        (d / "cad_import.yaml").write_text(yaml.safe_dump(asm, sort_keys=False))
        rc = import_stl.main([str(d / "cad_import.yaml"), "--out", str(d)])
        man, parts = manifest.read(str(d))
        oml = G.boolean([p.mesh for p in parts if p.in_oml and p.mesh is not None], "union")
        info = man.get("surfaces") and {"surfaces": man["surfaces"]} or {"surfaces": {}}
        ctx.note(f"geometry from CAD import ({len(parts)} parts, importer rc={rc})")
    else:
        parts, info = parametric.build_parts(g)
        oml = parametric.build_oml(parts)
        manifest.write(str(d), parts, g, oml, extra=info)
    t["build"] = round(time.time() - t0, 2)
    oml_m = oml.copy(); oml_m.apply_scale(1e-3); oml_m.export(str(d / "oml_m.stl"))
    try:
        manifest.export_glb(str(d / "assembly.glb"), parts)
        ctx.add_file(d / "assembly.glb", "coloured assembly (glTF)")
    except Exception as e:
        ctx.note(f"GLB export failed: {e}")
    rep = G.mesh_report(oml)
    bad_parts = [p.name for p in parts if p.mesh is not None and not p.mesh.is_watertight]

    # ---- geometry mass model (masses from the plan; geometry distributes them) --------------------
    t0 = time.time()
    res = MP.compute(parts, g)
    summ = MP.summarize(res, g, info)
    json.dump(manifest._clean(summ), open(d / "mass_properties.json", "w"), indent=1)
    open(d / "jsbsim_mass_balance.xml", "w").write(MP.jsbsim_xml(summ))
    # raw geometric estimate of every printed shell (no mass target) -> soft-input cross-check
    raw = {}
    for p in parts:
        if p.role == "structure" and p.mesh is not None and p.mass_model == "shell":
            q = parametric.Part(**{**p.__dict__, "mass": None})
            m, _, _, _ = MP.part_raw(q, g["materials"])
            raw[p.name] = dict(geometry_g=round(m, 1), plan_g=p.mass)
    t["mass"] = round(time.time() - t0, 2)

    # ---- balance chart + renders ------------------------------------------------------------------
    t0 = time.time()
    try:
        balance_chart.main(str(d))
        ctx.add_file(d / "balance.png", "Balance chart: every mass, CG vs target, moments by group", "figure")
    except Exception as e:
        ctx.note(f"balance chart failed: {e}")
    t["balance"] = round(time.time() - t0, 2)
    t0 = time.time()
    views = VIEWS.get(ctx.fidelity, VIEWS["standard"])
    backend = ["--backend", "blender", "--engine", "BLENDER_WORKBENCH"] if env.blender() else ["--backend", "matplotlib"]
    title = plan["meta"]["name"] + (f" ({ctx.variant})" if ctx.variant != "base" else "")
    rargs = [str(d)] + backend + ["--views", views, "--res", "1600", "1000", "--title", title]
    rendered = []
    try:
        render.main(rargs)
        rendered = views.split(",")
    except SystemExit as e:
        ctx.note(f"blender render failed ({e}); falling back to matplotlib")
        try:
            render.main([str(d), "--backend", "matplotlib", "--views", views, "--res", "1600", "1000", "--title", title])
            rendered = views.split(",")
        except Exception as e2:
            ctx.note(f"matplotlib render failed: {e2}")
    shutil.rmtree(d / "renders" / "raw", ignore_errors=True)
    for v in rendered:
        ctx.add_file(d / "renders" / f"{v}.png", f"render: {v}", "render")
    t["render"] = round(time.time() - t0, 2)

    # ---- gates -------------------------------------------------------------------------------------
    ctx.gate("OML watertight (needed for CFD and fit checks)", bool(rep["watertight"]), True, "bool", "FAIL")
    ctx.gate("OML is one connected body", rep["bodies"], 1, "<=", "WARN", note="separate bodies = parts not touching")
    ctx.gate("non-watertight part meshes", len(bad_parts), 0, "<=", "WARN", note=", ".join(bad_parts[:5]))
    for k, r in raw.items():
        if r["plan_g"]:
            dev = r["plan_g"] / r["geometry_g"] - 1 if r["geometry_g"] else None
            ctx.gate(f"printed {k}: plan mass vs geometry shell estimate", abs(dev) if dev is not None else None, 0.25, "<=", "WARN",
                     note=f"plan {r['plan_g']:.0f} g vs geometry {r['geometry_g']:.0f} g (weigh the part)",
                     drivers=["/components (structure_printed masses)", "/materials (printed density)"])
    b = summ["body_axes_about_cg"]
    data = dict(mass_kg=summ["mass_kg"], cg_m=[v / 1000 for v in summ["cg_struct_mm"]],
                inertia_frd=dict(Ixx=b["Ixx"], Iyy=b["Iyy"], Izz=b["Izz"], Ixz=b["Ixz"], Ixy=b["Ixy"], Iyz=b["Iyz"]),
                mac_m=summ.get("mac", {}).get("mac_mm", 0) / 1000 or None,
                x_le_mac_m=summ.get("mac", {}).get("x_le_mac_mm", 0) / 1000,
                printed_raw=raw, oml=dict(watertight=bool(rep["watertight"]), bodies=rep["bodies"],
                                          volume_L=rep["volume_mm3"] / 1e6, wetted_area_m2=rep["area_mm2"] / 1e6,
                                          bounds_m=(np.asarray(rep["bounds_mm"]) / 1000).tolist()),
                parts=[dict(name=p["name"], group=p["group"], mass_g=p["mass_g"], cg_mm=p["cg_mm"]) for p in summ["parts"]],
                external_parts=g["external_parts"], renders=rendered, mode="cad_import" if imp else "parametric")
    timings = dict(t, total=round(sum(t.values()), 2))
    (d / "timings.json").write_text(json.dumps(timings))
    summary = dict(mass_g=round(summ["mass_kg"] * 1000, 1), cg_mm=[round(v, 1) for v in summ["cg_struct_mm"]],
                   oml_watertight=bool(rep["watertight"]), oml_bodies=rep["bodies"], wetted_area_m2=round(rep["area_mm2"] / 1e6, 3),
                   renders=len(rendered), timings_s=timings)
    return dict(summary=summary, data=data)
