"""Stage 2 - mass / CG / inertia.

Two independent models of the same component list:
  plan      : point masses + box self-inertia straight from the plan (always available)
  geometry  : meshes from the geometry stage (printed shells, tubes, boxes; masses scaled to the plan)
analysis.mass_model selects the one passed downstream (default geometry when it ran); the other is a
cross-check gate. Gates: AUW vs target, CG vs design CG (+-tol x MAC), lateral CG, model agreement."""
import json

import numpy as np

from .. import plan as P
from ..figures import style, save, PALETTE, INK2


def run(ctx):
    plan = ctx.plan
    mp = P.mass_properties(plan)
    Ifrd = P.design_to_frd_inertia(mp["inertia"])
    plan_model = dict(mass_kg=float(mp["mass"]), cg_m=mp["cg"].tolist(),
                      inertia_frd=dict(Ixx=Ifrd[0, 0], Iyy=Ifrd[1, 1], Izz=Ifrd[2, 2], Ixz=-Ifrd[0, 2],
                                       Ixy=-Ifrd[0, 1], Iyz=-Ifrd[1, 2]))
    geo = ctx.data("geometry", required=False)
    want = plan.get("analysis", {}).get("mass_model", "geometry")
    use = "geometry" if (want == "geometry" and geo) else "plan"
    if want == "geometry" and not geo:
        ctx.note("geometry stage unavailable -> using the point/box plan model (inertia less accurate)")
    M = geo if use == "geometry" else plan_model
    w = P.main_wing(plan)
    mg = P.mac_geometry(w)
    mac, xle = mg["mac"], mg["x_le_mac"]
    cg = np.asarray(M["cg_m"])
    an = plan.get("analysis", {})
    cg_t = P.val(an.get("cg_target_x"))
    tol = an.get("cg_tolerance_mac", 0.03) * mac
    cg_pct = (cg[0] - xle) / mac
    bat = [c for c in P.expanded_components(plan) if c.get("category") == "energy"]
    m_bat = sum(c["mass"] for c in bat)
    shift = None
    if cg_t is not None:
        err = cg[0] - cg_t
        ctx.gate("CG x vs design CG [mm] (tolerance +-%.1f mm = %.0f%% MAC)" % (tol * 1000, 100 * tol / mac),
                 round(err * 1000, 1), [-tol * 1000, tol * 1000], "range", "FAIL", "mm",
                 note=f"CG {cg[0]*1000:.1f} mm vs target {cg_t*1000:.1f} mm "
                      + ("(nose-heavy)" if err < 0 else "(tail-heavy)"),
                 drivers=["/components (positions, estimated masses)", "/analysis/cg_target_x"])
        if m_bat:
            shift = -err * M["mass_kg"] / m_bat
            ctx.note(f"battery shift to hit the CG target: {shift*1000:+.0f} mm in x")
    ctx.gate("lateral CG offset [mm]", abs(cg[1]) * 1000, 2.0, "<=", "WARN", "mm")
    auw_t = P.val(an.get("auw_target"))
    if auw_t:
        ctx.gate("AUW vs planned AUW", abs(M["mass_kg"] / auw_t - 1), 0.05, "<=", "WARN",
                 note=f"{M['mass_kg']:.3f} kg vs {auw_t:.3f} kg")
    if geo:
        dm = abs(geo["mass_kg"] / plan_model["mass_kg"] - 1)
        dcg = np.linalg.norm(np.subtract(geo["cg_m"], plan_model["cg_m"])) * 1000
        ctx.gate("plan point-mass model vs geometry model: mass", dm, 0.03, "<=", "WARN",
                 note=f"{plan_model['mass_kg']*1000:.0f} g vs {geo['mass_kg']*1000:.0f} g")
        ctx.gate("plan point-mass model vs geometry model: CG distance [mm]", dcg, 8.0, "<=", "WARN", "mm",
                 note="positions of printed parts/tubes in the plan are estimates")
        for k in ("Ixx", "Iyy", "Izz"):
            r = plan_model["inertia_frd"][k] / geo["inertia_frd"][k]
            ctx.gate(f"inertia {k}: plan model / geometry model", r, [0.75, 1.33], "range", "WARN",
                     note="point masses without dims under-estimate inertia")
    soft_mass = [c["name"] for c in plan["components"]
                 if isinstance(c["mass"], dict) and P.SOFT_WORDS.search(str(c["mass"].get("source", "")))]
    m_soft = sum(c["mass"] for c in P.expanded_components(plan) if c["name"] in soft_mass or
                 any(c["name"] == P.mirror_name(s) for s in soft_mass))
    ctx.gate("share of AUW that is estimated (weigh parts, calibration C0)", m_soft / M["mass_kg"], 0.30, "<=", "WARN",
             drivers=[f"/components/{n}/mass" for n in soft_mass])

    # ---- mass table + figure --------------------------------------------------------------------
    rows = []
    for c in P.expanded_components(plan):
        pm = plan_comp(plan, c["name"]).get("mass")
        src = pm.get("source", "") if isinstance(pm, dict) else ""
        rows.append(dict(name=c["name"], category=c.get("category"), mass_g=round(c["mass"] * 1000, 1),
                         x_mm=round(c["position"][0] * 1000), y_mm=round(c["position"][1] * 1000),
                         z_mm=round(c["position"][2] * 1000), source=src or "(no source)"))
    by_cat = {}
    for r in rows:
        by_cat[r["category"]] = by_cat.get(r["category"], 0) + r["mass_g"]
    plt = style()
    fig, ax = plt.subplots(figsize=(8, 3.6))
    cats = sorted(by_cat, key=lambda k: by_cat[k])
    ax.barh(cats, [by_cat[c] for c in cats], color=PALETTE[0], height=0.6)
    for i, c in enumerate(cats):
        ax.text(by_cat[c] + 8, i, f"{by_cat[c]:.0f} g ({100*by_cat[c]/sum(by_cat.values()):.0f} %)", va="center", color=INK2)
    ax.set_xlabel("mass [g]")
    ax.set_xlim(0, max(by_cat.values()) * 1.3)
    ax.set_title(f"Mass by category - AUW {sum(by_cat.values()):.0f} g (plan component list)", loc="left")
    save(fig, ctx.dir / "mass_breakdown.png")
    ctx.add_file(ctx.dir / "mass_breakdown.png", "Mass breakdown by category", "figure")
    (ctx.dir / "mass_table.json").write_text(json.dumps(rows, indent=1))

    cases = []
    for auw in an.get("vtol_auw_cases_kg", []) or []:
        cases.append(dict(mass_kg=auw, note="VTOL AUW case (propulsion hover table)"))
    data = dict(model=use, mass_kg=M["mass_kg"], cg_m=list(map(float, cg)), inertia_frd=M["inertia_frd"],
                mac_m=mac, x_le_mac_m=xle, cg_pct_mac=cg_pct, cg_target_m=cg_t, cg_tol_m=tol,
                battery_shift_m=shift, plan_model=plan_model, geometry_model=geo and dict(
                    mass_kg=geo["mass_kg"], cg_m=geo["cg_m"], inertia_frd=geo["inertia_frd"]),
                by_category_g=by_cat, items=rows, mass_cases=cases, soft_mass_items=soft_mass)
    summ = dict(model=use, mass_g=round(M["mass_kg"] * 1000, 1), cg_mm=[round(v * 1000, 1) for v in cg],
                cg_pct_mac=round(100 * cg_pct, 1), Ixx=round(M["inertia_frd"]["Ixx"], 4), Iyy=round(M["inertia_frd"]["Iyy"], 4),
                Izz=round(M["inertia_frd"]["Izz"], 4), Ixz=round(M["inertia_frd"]["Ixz"], 5),
                battery_shift_mm=None if shift is None else round(shift * 1000))
    return dict(summary=summ, data=data)


def plan_comp(plan, name):
    for c in plan["components"]:
        if c["name"] == name or P.mirror_name(c["name"]) == name:
            return c
    return {}
