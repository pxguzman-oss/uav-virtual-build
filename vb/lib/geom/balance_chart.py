#!/usr/bin/env python3
"""CG / balance chart from a model directory (needs mass_properties.json).

Top: side silhouette (projected OML) with every mass as a bubble (area ~ mass),
     CG, CG target +/- tolerance band and the MAC bar.
Bottom: moment of each group about the target CG (g*m): what makes it nose/tail heavy.

  python balance_chart.py out/stork  ->  out/stork/balance.png
"""
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import trimesh  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# validated categorical palette (dataviz skill reference instance, light mode), fixed order
PALETTE = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
GROUP_ORDER = ["printed", "carbon", "power", "propulsion", "servos", "avionics", "misc", "vtol"]
INK, INK2, GRID = "#0b0b0b", "#52514e", "#d9d8d4"


def main(model_dir, out=None):
    man = json.load(open(os.path.join(model_dir, "manifest.json")))
    mp = json.load(open(os.path.join(model_dir, "mass_properties.json")))
    plan = man["plan"]
    oml = trimesh.load(os.path.join(model_dir, man["oml"]), force="mesh")
    groups = [g for g in GROUP_ORDER if any(p["group"] == g for p in mp["parts"])]
    groups += sorted({p["group"] for p in mp["parts"]} - set(groups))
    # fixed group->colour map: one green only (power), 'misc' neutral grey, target band neutral
    fixed = {"printed": PALETTE[0], "carbon": PALETTE[1], "power": PALETTE[2], "propulsion": PALETTE[3],
             "servos": PALETTE[4], "avionics": PALETTE[6], "misc": "#8a8984", "vtol": PALETTE[7]}
    spare = [c for c in PALETTE if c not in fixed.values()] + PALETTE
    gcol = {g: fixed.get(g) or spare[i % len(spare)] for i, g in enumerate(groups)}

    fig, (ax, bx) = plt.subplots(2, 1, figsize=(16, 10), dpi=100, gridspec_kw=dict(height_ratios=[2.2, 1]))
    fig.patch.set_facecolor("#fcfcfb")
    for a in (ax, bx):
        a.set_facecolor("#fcfcfb")
        for s in ("top", "right"):
            a.spines[s].set_visible(False)
        a.spines["left"].set_color(GRID); a.spines["bottom"].set_color(GRID)
        a.tick_params(colors=INK2)

    # silhouette: union of OML triangles projected onto the x-z plane
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    tri = oml.triangles[:, :, [0, 2]]
    e1, e2 = tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
    area2 = np.abs(e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0])
    polys = [Polygon(t) for t in tri[area2 > 1e-6]]
    shape = unary_union(polys)
    geoms = getattr(shape, "geoms", [shape])
    for g in geoms:
        x, z = g.exterior.xy
        ax.fill(x, z, color="#eceae6", zorder=1)
        ax.plot(x, z, color="#9a9890", lw=1.2, zorder=2)

    ref = plan.get("reference", {})
    tgt, tol = ref.get("cg_target_x"), ref.get("cg_tolerance", 5)
    if "mac" in mp:
        m = mp["mac"]
        zb = oml.bounds[0][2] - 18
        ax.plot([m["x_le_mac_mm"], m["x_le_mac_mm"] + m["mac_mm"]], [zb, zb], color=INK2, lw=2, solid_capstyle="butt")
        ax.text(m["x_le_mac_mm"] + m["mac_mm"] / 2, zb - 8, f"MAC {m['mac_mm']:.0f} mm", ha="center", va="top", color=INK2, fontsize=11)
    if tgt is not None:
        ax.axvspan(tgt - tol, tgt + tol, color=INK2, alpha=0.10, zorder=0, lw=0)
        ax.axvline(tgt, color=INK2, lw=1.3, ls="--", zorder=3)
        ax.text(tgt - tol - 4, oml.bounds[0][2] - 4, f"target {tgt} ± {tol} mm", color=INK2, ha="right", va="top", fontsize=11)

    parts = sorted(mp["parts"], key=lambda p: -p["mass_g"])
    mmax = max(p["mass_g"] for p in parts)
    for p in parts:
        x, _, z = p["cg_mm"]
        s = 2400 * p["mass_g"] / mmax + 12
        ax.scatter(x, z, s=s, color=gcol[p["group"]], alpha=0.85, edgecolors="#fcfcfb", linewidths=2, zorder=4)
    # direct labels: merge mirrored L/R items, label only the heavy ones, stagger offsets
    import re
    agg = {}
    for p in parts:
        k = re.sub(r"_(L|R)$", "", p["name"])
        a = agg.setdefault(k, [0.0, np.zeros(3)])
        a[1] = (a[1] * a[0] + np.asarray(p["cg_mm"]) * p["mass_g"]) / (a[0] + p["mass_g"])
        a[0] += p["mass_g"]
    heavy = sorted([(k, m, c) for k, (m, c) in agg.items() if m >= 40], key=lambda t: t[2][0])
    zt, zb_ = oml.bounds[1][2], oml.bounds[0][2]
    fz = 0.5 * (zt + zb_)
    lanes = [fz + 85, zb_ - 45, fz + 120, zb_ - 80]          # label rows above/below the fuselage
    last = {i: -1e9 for i in range(len(lanes))}
    min_dx = 0.12 * (oml.bounds[1][0] - oml.bounds[0][0])
    for i, (k, m, c) in enumerate(heavy):
        lane = min(range(len(lanes)), key=lambda L: (last[L] + min_dx > c[0], L))   # first lane with room
        xl = max(c[0], last[lane] + min_dx)
        last[lane] = xl
        ax.annotate(f"{k.replace('_', ' ')} {m:.0f} g", (c[0], c[2]), xytext=(xl, lanes[lane]), textcoords="data",
                    ha="center", va="center", fontsize=10, color=INK,
                    bbox=dict(boxstyle="round,pad=0.2", fc="#fcfcfb", ec="none", alpha=0.9),
                    arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8, shrinkA=2, shrinkB=4))
    ax.set_ylim(min(lanes) - 40, max(zt, max(lanes)) + 30)
    cg = mp["cg_struct_mm"]
    ax.scatter(cg[0], cg[2], s=260, marker="P", color=INK, edgecolors="#fcfcfb", linewidths=2, zorder=6)
    ax.annotate(f"CG x={cg[0]:.1f} mm ({mp.get('cg_pct_mac', float('nan')):.1f}% MAC)", (cg[0], cg[2]),
                xytext=(cg[0] - 120, max(lanes) + 5), textcoords="data", ha="right", fontsize=12, fontweight="bold",
                color=INK, arrowprops=dict(arrowstyle="-", color=INK, lw=1))
    ax.set_aspect("equal")
    ax.set_xlabel("x aft of wing-root LE [mm]", color=INK2)
    ax.set_ylabel("z [mm]", color=INK2)
    handles = [plt.Line2D([], [], marker="o", ls="", color=gcol[g], markersize=10, label=g) for g in groups]
    ax.legend(handles=handles, loc="lower left", bbox_to_anchor=(0, 1.0), frameon=False, ncol=len(groups), fontsize=11,
              labelcolor=INK, borderaxespad=0.2)
    err = mp.get("cg_error_x_mm", 0)
    if tgt is None:
        verdict = ""
    elif abs(err) <= tol:
        verdict = f", CG within tolerance ({err:+.1f} mm)"
    else:
        verdict = (f", CG {abs(err):.1f} mm {'forward' if err < 0 else 'aft'} of target "
                   f"({'nose' if err < 0 else 'tail'}-heavy)")
        if mp.get("battery_shift_to_target_mm") is not None:
            verdict += f" - move battery {mp['battery_shift_to_target_mm']:+.0f} mm"
    title = (f"{plan.get('name', '').title()}{' VTOL' if plan.get('options', {}).get('vtol') else ''} balance - "
             f"{mp['mass_kg']*1000:.0f} g{verdict}")
    ax.set_title(title, loc="left", fontsize=16, fontweight="bold", color=INK, pad=40)

    # moments about target per group
    xt = tgt if tgt is not None else cg[0]
    mom = {g: sum(p["mass_g"] * (p["cg_mm"][0] - xt) for p in mp["parts"] if p["group"] == g) / 1000.0 for g in groups}
    order = sorted(groups, key=lambda g: mom[g])
    vals = [mom[g] for g in order]
    bx.barh(order, vals, color=[gcol[g] for g in order], height=0.6, edgecolor="#fcfcfb", linewidth=2)
    for i, v in enumerate(vals):
        bx.text(v + (0.3 if v >= 0 else -0.3), i, f"{v:+.1f}", va="center", ha="left" if v >= 0 else "right", color=INK, fontsize=10)
    bx.axvline(0, color=INK2, lw=1)
    lim = max(abs(min(vals)), abs(max(vals))) * 1.25
    bx.set_xlim(-lim, lim)
    bx.set_xlabel("moment about CG target [g·m]   (<0 nose-heavy, >0 tail-heavy; sum = total mass x CG error)", color=INK2)
    bx.grid(axis="x", color=GRID, lw=0.8)
    bx.set_axisbelow(True)
    fig.tight_layout()
    out = out or os.path.join(model_dir, "balance.png")
    fig.savefig(out, facecolor=fig.get_facecolor())
    print("wrote", out)


if __name__ == "__main__":
    main(*sys.argv[1:3])
