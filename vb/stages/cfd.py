"""Stage 6b - CFD (OpenFOAM v1912 apt, snappyHexMesh + simpleFoam k-omega SST, half model).

quick: skipped | standard: analysis.cfd.alphas_standard (default 1 alpha) | full: alphas_full (default 3).
Geometry kind: analysis.cfd.kind = wing | wingbody | wingbodytail. The first alpha meshes + solves, the
others re-use the mesh. Each point is compared like-for-like with AeroBuildup / ASB VLM / AVL on the
identical geometry (named question: does the low-order model predict lift and pitching moment?).
Fully turbulent RANS over-predicts the drag of laminar-flow sections (~50 %, see README) -> CD is info only."""
import json
import os
import shutil
import sys
import types
from pathlib import Path

import numpy as np

from .. import env
from ..adapters import geom2aero
from ..figures import style, save, PALETTE, INK2

LIB = Path(__file__).resolve().parents[1] / "lib" / "aero"


def run(ctx):
    plan = ctx.plan
    cfg = plan.get("analysis", {}).get("cfd", {}) or {}
    if ctx.fidelity == "quick":
        ctx.note("CFD skipped at fidelity quick")
        return dict(skipped=True, summary=dict(skipped="fidelity quick"), data={})
    if not env.openfoam_bashrc() or not shutil.which("simpleFoam"):
        raise RuntimeError("OpenFOAM (simpleFoam) not installed - run setup/install.sh")
    os.environ["VB_FOAM_SHIM"] = env.foam_shim() or ""
    os.environ["VB_OPENFOAM_BASHRC"] = env.openfoam_bashrc()
    sys.path.insert(0, str(LIB))
    import cfd_case
    import cfd_geom
    import aerosandbox as asb

    mass = ctx.data("mass")
    from .. import plan as P
    # moment reference = design CG (shape-only cache key -> variants share the clean-airframe CFD); else the CG
    cg_t = P.val(plan.get("analysis", {}).get("cg_target_x"))
    cg = [cg_t, 0.0, 0.0] if cg_t is not None else mass["cg_m"]
    g, sa = geom2aero.build(plan, cg, mass["mass_kg"])
    g["vtol"] = False                                   # CFD answers clean-airframe questions
    full = ctx.fidelity == "full"
    kind = cfg.get("kind_full", cfg.get("kind", "wing")) if full else cfg.get("kind", "wing")
    alphas = cfg.get("alphas_full", [0, 4, 8]) if ctx.fidelity == "full" else cfg.get("alphas_standard", [4])
    V = float(plan["envelope"]["v_cruise"])
    gdir = ctx.dir / "stl"
    gdir.mkdir(exist_ok=True)
    comps = {"wing": cfd_geom.wing_surface(g)}
    if kind in ("wingbody", "wingbodytail"):
        comps["fuselage"] = cfd_geom.fuselage_surface(g)
    if kind == "wingbodytail":
        comps["vtail"] = cfd_geom.vtail_surface(g, +1)
    stls = []
    for nm, (Vv, F) in comps.items():
        p = gdir / f"{nm}.stl"
        cfd_geom.write_stl(str(p), nm, Vv, F)
        stls.append(str(p))
    case = ctx.dir / "case"
    base_args = dict(stl=stls, case=str(case), V=V, nu=1.5e-5, sref=g["ref"]["S"], cref=g["ref"]["c"],
                     cofr=[cg[0], 0.0, cg[2]], half=True, model=cfg.get("model", "kOmegaSST"),
                     surf_level=list(cfg.get("surf_level_full", cfg.get("surf_level", [5, 6])) if full else cfg.get("surf_level", [5, 6])), base=cfg.get("base", 0.2), up=3.0, down=5.0, side=2.5,
                     layers=cfg.get("layers", 3), first_layer=0.0, expansion=1.25, tu=0.001, visc_ratio=10.0,
                     iters=int(cfg.get("iters", 400)), np=int(cfg.get("np", os.cpu_count() or 2)), keep=False,
                     second_order_turb=False, avg=int(cfg.get("avg", 50)), mesh_only=False)
    points = []
    for i, al in enumerate(alphas):
        a = types.SimpleNamespace(**base_args, alpha=float(al))
        try:
            r = cfd_case.run(a) if i == 0 else cfd_case.solve(a)
        except Exception as e:
            ctx.note(f"alpha {al}: {e}")
            r = {"error": str(e)}
        points.append(dict(alpha=al, CL=r.get("CL"), CD=r.get("CD"), Cm=r.get("Cm"), converged=r.get("converged"),
                           cells=r.get("cells"), mesh_ok=r.get("mesh_ok"), yplus=r.get("yplus"),
                           solver_s=r.get("solver_exec_s"), timing=r.get("timing_s"), error=r.get("error")))
        (ctx.dir / "cfd_points.json").write_text(json.dumps(points, indent=1, default=str))
    # ---- like-for-like low-order references --------------------------------------------------
    # The flight model's LINEAR lift and Cm come from AVL on the full geometry (vb/liftmodel.py), so the
    # like-for-like "model" reference here is AVL on the SAME configuration as the CFD (wing / wing-body /
    # wing-body-tail) and the same moment reference. Raw AeroBuildup and ASB VLM are shown for information.
    ap_full = sa.build_asb(g)
    wings = [ap_full.wings[0]] + ([ap_full.wings[1]] if kind == "wingbodytail" else [])
    fus = ap_full.fuselages[:1] if kind in ("wingbody", "wingbodytail") else []
    ap_ = asb.Airplane(name=kind, xyz_ref=ap_full.xyz_ref, wings=wings, fuselages=fus, s_ref=g["ref"]["S"],
                       c_ref=g["ref"]["c"], b_ref=g["ref"]["b"])
    al_line = np.linspace(min(alphas) - 4, max(alphas) + 4, 13)
    avl_exe = env.avl()
    adir = ctx.dir / "avl_ref"
    adir.mkdir(exist_ok=True)
    sa.write_avl(g, str(adir / "cfdcfg.avl"), str(adir / "avl_airfoils"), include_vtail=(kind == "wingbodytail"),
                 include_body=(kind != "wing"))
    avl_r, _ = sa.run_avl(avl_exe, str(adir / "cfdcfg.avl"), [round(float(a), 3) for a in al_line], str(adir), tag="ref")
    refs = []
    for al, ar in zip(al_line, avl_r):
        r = sa.run_buildup(ap_, g, V, [float(al)])
        v = asb.VortexLatticeMethod(ap_, asb.OperatingPoint(velocity=V, alpha=float(al)), xyz_ref=ap_.xyz_ref,
                                    spanwise_resolution=10, chordwise_resolution=8).run()
        refs.append(dict(alpha=float(al), ab_CL=float(r["CL"][0]), ab_CD=float(r["CD"][0]), ab_Cm=float(r["Cm"][0]),
                         vlm_CL=float(np.ravel(v["CL"])[0]), vlm_Cm=float(np.ravel(v["Cm"])[0]),
                         model_CL=ar.get("CLtot"), model_Cm=ar.get("Cmtot")))
    # mesh quality (checkMesh): the real number of failed checks is recorded; a few skew faces at junctions
    # are tolerated (INFO), anything else is a WARN (review m4)
    import re
    try:
        txt = (case / "log.checkMesh").read_text()
        nfail = int((re.search(r"Failed (\d+) mesh checks", txt) or [0, 0])[1])
        nskew = int((re.search(r"(\d+) highly skew faces", txt) or [0, 0])[1])
        only_skew = nfail == 1 and nskew > 0
        if nfail == 0:
            ctx.gate("checkMesh: failed mesh checks", 0, 0, "<=", "WARN", note="mesh OK")
        elif only_skew and nskew <= 50:
            ctx.gate("checkMesh: failed mesh checks (only a few highly skew faces: tolerated, not a PASS)", nfail, 0, "info", "INFO",
                     note=f"{nfail} failed check: {nskew} highly skew faces (wing-body junction)")
        else:
            ctx.gate("checkMesh: failed mesh checks", nfail, 0, "<=", "WARN", note=f"{nfail} failed checks, {nskew} highly skew faces")
    except FileNotFoundError:
        ctx.gate("checkMesh log present", False, True, "bool", "WARN", note="log.checkMesh missing")
    # gates: CFD vs the MODEL actually used by the flight model (AVL lift line on the same configuration)
    A_ = [r["alpha"] for r in refs]
    for p in points:
        if p["CL"] is None:
            ctx.gate(f"CFD alpha {p['alpha']} completed", False, True, "bool", "WARN", note=p.get("error") or "")
            continue
        mcl = float(np.interp(p["alpha"], A_, [r["model_CL"] for r in refs]))
        mcm = float(np.interp(p["alpha"], A_, [r["model_Cm"] for r in refs]))
        abcl = float(np.interp(p["alpha"], A_, [r["ab_CL"] for r in refs]))
        vcl = float(np.interp(p["alpha"], A_, [r["vlm_CL"] for r in refs]))
        p.update(model_CL=mcl, model_Cm=mcm, ab_CL=abcl, vlm_CL=vcl)
        ctx.gate(f"CFD alpha {p['alpha']}: converged (force oscillation < 0.5 %)", bool(p["converged"]), True, "bool", "WARN",
                 note=f"{p['cells']} cells")
        if abs(mcl) > 0.05:
            err = abs(p["CL"] / mcl - 1)
            ctx.gate(f"CFD alpha {p['alpha']}: abs(CL_cfd / CL_model - 1), model = flight-model lift line on the same {kind}",
                     err, 0.10, "<=", "FAIL" if err > 0.20 else "WARN",
                     note=f"CFD {p['CL']:.3f} vs model {mcl:.3f} (raw AeroBuildup {abcl:.3f} = {100*(abcl/p['CL']-1):+.0f} % vs CFD, "
                          f"ASB VLM {vcl:.3f}); fully turbulent RANS loses some lift vs free transition; >20 % = FAIL",
                     drivers=["/airframe/lifting_surfaces/wing"])
        ctx.gate(f"CFD alpha {p['alpha']}: abs(Cm_cfd - Cm_model)", abs(p["Cm"] - mcm), 0.03, "<=", "WARN",
                 note=f"CFD {p['Cm']:.3f} vs model (AVL) {mcm:.3f}, both about x = {1000*cg[0]:.0f} mm, z = {1000*cg[2]:.0f} mm "
                      f"(design CG; the flight model uses the mass-stage CG)")
    okp = [p for p in points if p["CL"] is not None]
    if len(okp) >= 2:
        a0, a1 = okp[0], okp[-1]
        cla_cfd = (a1["CL"] - a0["CL"]) / ((a1["alpha"] - a0["alpha"]) * np.pi / 180)
        cla_mod = (a1["model_CL"] - a0["model_CL"]) / ((a1["alpha"] - a0["alpha"]) * np.pi / 180)
        ctx.gate("CFD lift slope / model lift slope: abs(ratio-1)", abs(cla_cfd / cla_mod - 1), 0.15, "<=", "WARN",
                 note=f"CFD {cla_cfd:.2f} /rad vs model {cla_mod:.2f} /rad ({a0['alpha']}..{a1['alpha']} deg)")
    else:
        ctx.gate("CFD lift slope checked (needs >= 2 alphas)", False, True, "bool", "WARN",
                 note="only one CFD alpha: lift slope not checked")
    # figure
    plt = style()
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.6))
    A = [r["alpha"] for r in refs]
    ax[0].plot(A, [r["ab_CL"] for r in refs], color=PALETTE[0], lw=1, ls="--", label="AeroBuildup raw (not used)")
    ax[0].plot(A, [r["vlm_CL"] for r in refs], color=PALETTE[1], lw=1, ls=":", label="ASB VLM (inviscid)")
    ax[0].plot(A, [r["model_CL"] for r in refs], color=PALETTE[2], lw=2, label="flight-model lift line (AVL)")
    ax[1].plot(A, [r["ab_CD"] for r in refs], color=PALETTE[0], label="AeroBuildup")
    ax[2].plot(A, [r["ab_Cm"] for r in refs], color=PALETTE[0], label="AeroBuildup")
    ax[2].plot(A, [r["vlm_Cm"] for r in refs], color=PALETTE[1], lw=1, ls=":", label="ASB VLM")
    ax[2].plot(A, [r["model_Cm"] for r in refs], color=PALETTE[2], lw=2, label="flight-model Cm line (AVL)")
    ok = [p for p in points if p["CL"] is not None]
    for k, axx in (("CL", ax[0]), ("CD", ax[1]), ("Cm", ax[2])):
        axx.plot([p["alpha"] for p in ok], [p[k] for p in ok], "s", color=PALETTE[7], ms=7, label="OpenFOAM SST")
        axx.set(xlabel="alpha [deg]", ylabel=k)
        axx.legend(loc="best", fontsize=7.5)
    ax[0].set_title(f"CFD vs low order: {kind}, V {V:g} m/s", loc="left")
    ax[1].set_title("CD (RANS fully turbulent: high)", loc="left")
    ax[2].set_title(f"Cm about x = {1000 * cg[0]:.0f} mm, z = {1000 * cg[2]:.0f} mm (design CG)", loc="left")
    save(fig, ctx.dir / "cfd_vs_lowfi.png")
    ctx.add_file(ctx.dir / "cfd_vs_lowfi.png", "CFD points vs AeroBuildup/VLM on the same geometry", "figure")
    # tidy: keep logs/results, drop the big processor/mesh directories
    for p in case.glob("processor*"):
        shutil.rmtree(p, ignore_errors=True)
    shutil.rmtree(case / "constant" / "polyMesh", ignore_errors=True)
    data = dict(kind=kind, V=V, points=points, lowfi_ref=refs,
                cl_ratio_mean=float(np.mean([p["CL"] / p["model_CL"] for p in ok if abs(p.get("model_CL", 0)) > 0.05])) if ok else None,
                cl_ratio_reference="CFD / flight-model lift line (AVL, same configuration)",
                moment_ref_m=[cg[0], 0.0, cg[2]])
    summ = dict(kind=kind, alphas=alphas, CL=[p["CL"] for p in points], converged=[p["converged"] for p in points],
                cells=points[0].get("cells") if points else None,
                wall_min=round(sum((p.get("timing") or {}).get("snappyHexMesh", 0) + (p.get("timing") or {}).get("simpleFoam", 0)
                                   for p in points[:1]) / 60, 1))
    return dict(summary=summ, data=data)
