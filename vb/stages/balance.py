"""Stage 3b - auto-balance: ranked ways to put the CG on target and, for a quadplane, the lift-thrust
centre on the CG. Uses the mass stage (CG of the selected mass model), the aero stage (neutral point ->
static margin) and the plan's bays / travel limits.

Moves it may propose, cheapest first: slide the battery; re-arrange internal parts (least total travel,
parts may not pass through each other); move the lift motors/booms; move the wing on the fuselage; extend
the battery bay; swap the battery (catalog + parts-list alternatives, filtered by meta.sourcing.policy);
ballast as a last resort. Each option carries its edits, the resulting CG / static margin / thrust-centre
offset / AUW / range estimate, pros and cons, and a variant overlay so it can be run through the WHOLE
pipeline (tools/balance_apply.py). Nothing here changes the plan: the team picks.

Outputs: balance_options.json, balance_options.md, fusion_change_list.csv, options/<id>.overlay.yaml,
balance_options.png."""
import csv
import json
from pathlib import Path

import numpy as np
import yaml

from .. import plan as P
from ..figures import style, save, PALETTE, INK2
from ..lib import balance as B

ROOT = Path(__file__).resolve().parents[2]


def run(ctx):
    plan = ctx.plan
    cfg = (plan.get("analysis") or {}).get("balance") or {}
    mass = ctx.data("mass")
    aero = ctx.data("aero_lowfi", required=False)
    if not aero:
        ctx.note("aero stage not available: static margin of the options is not evaluated")
    policy = ((plan.get("meta") or {}).get("sourcing") or {}).get("policy", "any")
    cat_f = ROOT / "data" / "catalog" / "batteries.yaml"
    catalog = yaml.safe_load(cat_f.read_text()) if cat_f.exists() else {}
    T = B.targets(plan, mass, aero)
    prop = ctx.data("propulsion", required=False) or {}
    tw = next((r.get("TW_max") for r in (prop.get("vtol") or {}).get("hover_table") or [] if r.get("is_plan_auw")), None)
    res = B.generate(plan, mass, aero, T, cfg, catalog, policy, hover_tw=tw)
    base, opts = res["base"], res["options"]
    lc = B.lift_centre(plan)

    ctx.gate("as drawn: CG inside the allowed window", base["in_window"], True, "bool", "WARN",
             note=f"CG {base['cg_x']*1000:.1f} mm vs target {T['target']*1000:.1f} mm, window {T['lo']*1000:.1f}..{T['hi']*1000:.1f} mm "
                  f"({T['source']})")
    if lc:
        ctx.gate("as drawn: CG to lift-thrust centre [mm]", round(base["thrust_centre_offset_mm"], 1),
                 [-round(res["thrust_tol_mm"], 1), round(res["thrust_tol_mm"], 1)], "range", "WARN", "mm",
                 note=f"front lift motors carry {100*base['front_share']:.1f} % of the weight")
    real = [o for o in opts if o["kind"] != "none"]
    feas = [o for o in opts if o["feasible"]]
    cheap = [o for o in feas if o["kind"] in ("none", "battery", "internals")]
    no_ballast = [o for o in feas if "ballast" not in o["kind"]]
    ctx.gate("a balanced layout exists without ballast, new parts or frame changes", bool(cheap), True, "bool", "WARN",
             note=(f"best: {cheap[0]['id']} - {cheap[0]['title']}" if cheap else
                   f"internal parts can move the CG only {res['reach']['max_cg_shift_fwd_mm']:+.1f}..{res['reach']['max_cg_shift_aft_mm']:+.1f} mm; "
                   f"needed {-base['cg_err_mm']:+.1f} mm"))
    ctx.gate("at least one option reaches the CG window without a new clash", bool(feas), True, "bool", "FAIL",
             note=f"{len(feas)} of {len(opts)} options" + ("" if feas else ": change the layout or the target by hand"))
    best = feas[0] if feas else None
    ctx.gate("recommended option", best["id"] if best else None, None, "info", "INFO", note=best["title"] if best else "none")

    # ---- files ------------------------------------------------------------------------------------
    od = ctx.dir / "options"
    od.mkdir(exist_ok=True)
    rows = []
    for o in opts:
        m = o["metrics"]
        rows.append(dict(id=o["id"], title=o["title"], kind=o["kind"], feasible=o["feasible"], score=o["score"], effort=o["effort"],
                         cg_err_mm=round(m["cg_err_mm"], 1), static_margin_pct=None if m.get("static_margin") is None else round(100 * m["static_margin"], 1),
                         thrust_centre_offset_mm=None if "thrust_centre_offset_mm" not in m else round(m["thrust_centre_offset_mm"], 1),
                         d_mass_g=round(m.get("d_mass_g", 0.0), 1), range_change_pct=m["range_change_pct"], pros=o["pros"], cons=o["cons"],
                         edits=o["edits"], change_list=B.change_list(plan, o)))
        if o["kind"] != "none":
            ov = B.overlay(ctx.raw_plan, ctx.variant, o["edits"], f"{o['id']} {o['title']}")
            (od / f"{o['id']}.overlay.yaml").write_text(yaml.safe_dump(json.loads(json.dumps(ov)), sort_keys=False))
    (ctx.dir / "balance_options.json").write_text(json.dumps(dict(target=T, base={k: v for k, v in base.items() if not k.startswith("_")},
                                                                 reach=res["reach"], options=rows, source_variant=ctx.variant,
                                                                 sourcing_policy=policy), indent=1, default=float))
    with open(ctx.dir / "fusion_change_list.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["option", "item", "cad_body", "change", "from", "to", "delta", "why"])
        for r in rows:
            for c in r["change_list"]:
                w.writerow([r["id"], c["item"], c.get("cad_body", ""), c["change"], c["frm"], c["to"], c["delta"], c["why"]])
    md = [f"# Balance options - {plan['meta']['name']} ({ctx.variant})", "",
          f"Target CG x = {T['target']*1000:.1f} mm from the wing-root leading edge, window {T['lo']*1000:.1f} to {T['hi']*1000:.1f} mm "
          f"({T['source']}). As drawn: {base['cg_x']*1000:.1f} mm ({base['cg_err_mm']:+.1f} mm), AUW {base['mass_kg']*1000:.0f} g.", "",
          "| # | option | CG error mm | static margin % | CG to thrust centre mm | AUW g | range % | effort | reaches target |",
          "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        md.append(f"| {r['id']} | {r['title']} | {r['cg_err_mm']:+.1f} | {r['static_margin_pct'] if r['static_margin_pct'] is not None else '-'} | "
                  f"{r['thrust_centre_offset_mm'] if r['thrust_centre_offset_mm'] is not None else '-'} | {r['d_mass_g']:+.0f} | "
                  f"{r['range_change_pct']:+.1f} | {r['effort']:g} | {'yes' if r['feasible'] else 'NO'} |")
    for r in rows:
        md += ["", f"## {r['id']} - {r['title']}", "", "Pros: " + ("; ".join(r["pros"]) or "-"), "", "Cons: " + ("; ".join(r["cons"]) or "-")]
        if r["change_list"]:
            md += ["", "| item | change | from | to | delta | why |", "|---|---|---|---|---|---|"]
            md += [f"| {c['item']} | {c['change']} | {c['frm']} | {c['to']} | {c['delta']} | {c['why']} |" for c in r["change_list"]]
    md += ["", "Every option except 0 has an overlay in options/. To test one through the whole pipeline:", "",
           f"    python tools/balance_apply.py <plan.yaml> --from {ctx.dir} --option <id>", "",
           "Range and static-margin figures here are estimates; the re-run gives the real numbers."]
    (ctx.dir / "balance_options.md").write_text("\n".join(md) + "\n")
    for fn, cap in (("balance_options.md", "Ranked balance options with pros and cons"), ("fusion_change_list.csv", "Change list for Fusion"),
                    ("balance_options.json", "Balance options (machine readable)")):
        ctx.add_file(ctx.dir / fn, cap, "report")

    # ---- figure -----------------------------------------------------------------------------------
    plt = style()
    fig, ax = plt.subplots(figsize=(10, 0.55 * len(rows) + 1.6))
    ax.axvspan(T["lo"] * 1000, T["hi"] * 1000, color="#1baf7a", alpha=0.18, lw=0, label="allowed CG window")
    ax.axvline(T["target"] * 1000, color="#1baf7a", lw=1.2)
    if lc:
        ax.axvline(lc["x"] * 1000, color=INK2, lw=1, ls=":", label="lift-thrust centre (as drawn)")
    yy = np.arange(len(rows))[::-1]
    for y, r, o in zip(yy, rows, opts):
        ax.scatter([o["metrics"]["cg_x"] * 1000], [y], s=70, color=PALETTE[0] if r["feasible"] else "#e34948", zorder=3)
        ax.text(o["metrics"]["cg_x"] * 1000, y + 0.28, f"{r['cg_err_mm']:+.1f} mm", ha="center", fontsize=7, color=INK2)
    ax.set_yticks(yy, [f"{r['id']}  {r['title'][:58]}" for r in rows], fontsize=7)
    ax.set_ylim(-0.6, len(rows) - 0.3)
    ax.set(xlabel="CG x [mm aft of the wing-root leading edge]", title="Where each option puts the CG (red = does not reach the window or clashes)")
    ax.yaxis.grid(False); ax.legend(fontsize=7, loc="upper left")
    save(fig, ctx.dir / "balance_options.png")
    ctx.add_file(ctx.dir / "balance_options.png", "CG of every balance option against the allowed window", "figure")

    summ = dict(cg_error_as_drawn_mm=round(base["cg_err_mm"], 1), options=len(real), reach_target=len([o for o in feas if o["kind"] != "none"]),
                recommended=best and f"{best['id']} - {best['title']}", sourcing_policy=policy,
                internal_reach_mm=[round(res["reach"]["max_cg_shift_fwd_mm"], 1), round(res["reach"]["max_cg_shift_aft_mm"], 1)])
    data = dict(target=T, base={k: v for k, v in base.items() if not k.startswith("_")}, options=[{k: v for k, v in r.items() if k != "edits"} for r in rows],
                reach=res["reach"], movables=res["movables"], wing_attached=res["wing_attached"], recommended=best and best["id"])
    return dict(summary=summ, data=data, tables=dict(options=[{k: r[k] for k in ("id", "title", "feasible", "cg_err_mm", "d_mass_g", "range_change_pct")} for r in rows]))
