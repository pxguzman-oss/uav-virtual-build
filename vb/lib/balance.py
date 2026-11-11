"""Auto-balance: generate, evaluate and rank ways to put the CG on target (and, for a quadplane, the
lift-thrust centre on the CG).

Everything here works on the RESOLVED plan dict and is pure (no files), so it can be unit-tested and
re-used by tools/balance_apply.py. An option is a list of EDITS:

  {"kind": "move",   "name": comp, "dx": m [, "dy": m, "dz": m], "why": str}      component CG shift
  {"kind": "set",    "name": comp, "fields": {...}, "why": str}                     e.g. mass/dims/part_number of a swap
  {"kind": "add",    "component": {...}, "why": str}                                ballast
  {"kind": "unit",   "unit": name, "dx": m, "why": str}                             propulsion unit positions
  {"kind": "boom",   "name": boom, "dx": m, "why": str}                             boom root/tip (supports stay on the spars)
  {"kind": "path",   "path": "/airframe/...", "value": v, "old": v0, "why": str}    any other plan value

apply(plan, edits) returns the edited plan; overlay(raw, variant, edits) returns the variant overlay that
reproduces it from the raw plan file, so every option can be re-run through the whole pipeline."""
from __future__ import annotations

import copy
import math

import numpy as np

from .. import plan as P

FIXED_WORDS = ("camera", "airspeed", "pitot", "gps", "antenna", "compass", "lidar", "rangefinder", "gimbal")
PURCHASED = ("propulsion", "energy", "avionics", "servo", "payload")


# ------------------------------------------------------------------------------------------ plan editing
def comp_by_name(plan, name):
    for c in plan["components"]:
        if c["name"] == name:
            return c
    for c in plan["components"]:
        if c.get("mirror_y") and P.mirror_name(c["name"]) == name:
            return c
    raise KeyError(name)


def _set_path(node, path, value):
    parts = [p for p in path.split("/") if p]
    for i, p in enumerate(parts[:-1]):
        if isinstance(node, list):
            node = next(x for x in node if isinstance(x, dict) and x.get("name") == p) if not p.isdigit() else node[int(p)]
        else:
            node = node.setdefault(p, {})
    last = parts[-1]
    if isinstance(node, list):
        node[int(last)] = value
    else:
        node[last] = value


def apply(plan, edits):
    out = copy.deepcopy(plan)
    for e in edits:
        k = e["kind"]
        if k == "move":
            c = comp_by_name(out, e["name"])
            c["position"] = [round(c["position"][0] + e.get("dx", 0.0), 5), round(c["position"][1] + e.get("dy", 0.0), 5),
                             round(c["position"][2] + e.get("dz", 0.0), 5)]
        elif k == "set":
            comp_by_name(out, e["name"]).update(copy.deepcopy(e["fields"]))
        elif k == "add":
            out["components"].append(copy.deepcopy(e["component"]))
        elif k == "unit":
            for u in out["propulsion"]["units"]:
                if u["name"] == e["unit"]:
                    u["positions"] = [[round(p[0] + e["dx"], 5), p[1], p[2]] for p in u["positions"]]
        elif k == "boom":
            for b in out["airframe"].get("booms", []):
                if b["name"] == e["name"]:
                    b["root"] = [round(b["root"][0] + e["dx"], 5), b["root"][1], b["root"][2]]
                    b["tip"] = [round(b["tip"][0] + e["dx"], 5), b["tip"][1], b["tip"][2]]
        elif k == "path":
            _set_path(out, e["path"], copy.deepcopy(e["value"]))
        else:
            raise ValueError(f"unknown edit kind {k}")
    return out


def overlay(raw, variant, edits, title=""):
    """Variant overlay (for raw['variants'][new]) = the source variant's overlay + the edits."""
    src = copy.deepcopy((raw.get("variants") or {}).get(variant) or {}) if variant and variant != "base" else {}
    resolved = apply(P.resolve(raw, None if variant in (None, "base") else variant), edits)
    added = {c["name"]: c for c in src.get("components_add", [])}
    added_units = {u["name"]: u for u in src.get("propulsion_add", [])}
    patch = {}
    touched_c, touched_u, touched_b = set(), set(), set()
    for e in edits:
        if e["kind"] in ("move", "set"):
            touched_c.add(comp_by_name(resolved, e["name"])["name"])
        elif e["kind"] == "unit":
            touched_u.add(e["unit"])
        elif e["kind"] == "boom":
            touched_b.add(e["name"])
        elif e["kind"] == "add":
            patch.setdefault("components_add", []).append(copy.deepcopy(e["component"]))
        elif e["kind"] == "path":
            _overlay_path(patch, resolved, e["path"])
    for n in sorted(touched_c):
        c = comp_by_name(resolved, n)
        fields = {k: copy.deepcopy(v) for k, v in c.items()}
        if n in added:
            added[n].clear(); added[n].update(fields)
        else:
            base = next((x for x in raw["components"] if x["name"] == n), {})
            patch.setdefault("components", []).append({"name": n, **{k: v for k, v in fields.items() if base.get(k) != v}})
    for n in sorted(touched_u):
        u = next(x for x in resolved["propulsion"]["units"] if x["name"] == n)
        if n in added_units:
            added_units[n]["positions"] = copy.deepcopy(u["positions"])
        else:
            patch.setdefault("propulsion", {}).setdefault("units", []).append({"name": n, "positions": copy.deepcopy(u["positions"])})
    for n in sorted(touched_b):
        b = next(x for x in resolved["airframe"]["booms"] if x["name"] == n)
        patch.setdefault("airframe", {}).setdefault("booms", []).append({"name": n, "root": b["root"], "tip": b["tip"]})
    extra_add = patch.pop("components_add", [])
    out = P._merge(src, patch)
    if extra_add:
        out.setdefault("components_add", []).extend(extra_add)
    out.setdefault("meta", {})["phase"] = (src.get("meta", {}).get("phase", "") + " | balance option: " + title).strip(" |")
    return out


def _overlay_path(patch, resolved, path):
    """Copy the value at `path` of the edited plan into the overlay, keeping named-list merge semantics."""
    parts = [p for p in path.split("/") if p]
    src, dst = resolved, patch
    for i, p in enumerate(parts):
        last = i == len(parts) - 1
        if isinstance(src, list):
            item = next(x for x in src if isinstance(x, dict) and x.get("name") == p)
            ex = next((x for x in dst if x.get("name") == p), None)
            if ex is None:
                ex = {"name": p}
                dst.append(ex)
            src, dst = item, ex
            continue
        nxt = src[p]
        if last or not isinstance(nxt, (dict, list)) or (isinstance(nxt, list) and not all(isinstance(x, dict) and "name" in x for x in nxt)):
            dst[p] = copy.deepcopy(nxt)
            return
        dst = dst.setdefault(p, [] if isinstance(nxt, list) else {})
        src = nxt


# ------------------------------------------------------------------------------------------ evaluation
def targets(plan, mass, aero):
    """CG target and allowed window [m]. The design CG of the plan (+- tolerance) is the authority; the
    static-margin range only sets the window when the plan has no design CG. The static margin of every
    option is still reported against analysis.static_margin_range."""
    an = plan.get("analysis") or {}
    mac = mass["mac_m"]
    t = P.val(an.get("cg_target_x"))
    tol = an.get("cg_tolerance_mac", 0.03) * mac
    sm = an.get("static_margin_range", [0.05, 0.15])
    x_np = (aero or {}).get("x_np")
    if t is not None:
        lo, hi, src = t - tol, t + tol, "design CG +- tolerance"
    elif x_np is not None:
        lo, hi = x_np - sm[1] * mac, x_np - sm[0] * mac
        t, tol, src = 0.5 * (lo + hi), 0.5 * (hi - lo), "middle of the static-margin range (no design CG in the plan)"
    else:
        raise ValueError("no CG target: give analysis.cg_target_x or run the aero stage")
    return dict(target=t, lo=lo, hi=hi, tol=tol, mac=mac, x_np=x_np, sm_range=sm, source=src)


def lift_centre(plan):
    lifts = P.units(plan, "lift")
    if not lifts or not lifts[0].get("positions"):
        return None
    pos = np.array(lifts[0]["positions"], float)
    xc = pos[:, 0].mean()
    f = pos[:, 0] < xc
    return dict(x=float(xc), xf=float(pos[f, 0].mean()), xr=float(pos[~f, 0].mean()))


def boxes(plan):
    """Internal boxed components (for overlap checks): name -> (centre, half-size)."""
    out = {}
    for c in P.expanded_components(plan):
        if c.get("dims") and c.get("geom") is not False and c.get("shape", "box") in ("box", None) \
                and c.get("category") in PURCHASED and not c.get("external"):
            out[c["name"]] = (np.array(c["position"], float), np.array(c["dims"], float) / 2)
    return out


def overlaps(plan, tol=0.0005):
    b = boxes(plan)
    names = sorted(b)
    out = []
    for i, a in enumerate(names):
        for c in names[i + 1:]:
            pen = (b[a][1] + b[c][1]) - np.abs(b[a][0] - b[c][0])
            if np.all(pen > tol):
                out.append((a, c, float(pen.min() * 1000)))
    return out


def bay_violations(plan):
    bays = plan["airframe"]["fuselage"].get("bays") or {}
    out = []
    for c in plan["components"]:
        if c.get("bay") in bays and c.get("dims"):
            b = bays[c["bay"]]
            over = np.abs(np.subtract(c["position"], b["center"])) + np.array(c["dims"]) / 2 - np.array(b["size"]) / 2
            if over.max() > 0.0005:
                out.append((c["name"], c["bay"], float(over.max() * 1000)))
    return out


def evaluate(plan, base, T, cg_offset, aero=None, np_shift=0.0):
    """Metrics of an (edited) plan. `base` = evaluate() of the unedited plan (None for the baseline)."""
    mp = P.mass_properties(plan)
    cg = mp["cg"] + cg_offset
    M = float(mp["mass"])
    err = float(cg[0] - T["target"])
    m = dict(mass_kg=M, cg_x=float(cg[0]), cg_y=float(cg[1]), cg_err_mm=err * 1000, in_window=bool(T["lo"] - 1e-4 <= cg[0] <= T["hi"] + 1e-4),
             cg_pct_mac=None, static_margin=None)
    if T.get("x_np") is not None:
        m["static_margin"] = float((T["x_np"] + np_shift - cg[0]) / T["mac"])
    lc = lift_centre(plan)
    if lc:
        m["thrust_centre_offset_mm"] = float((cg[0] - lc["x"]) * 1000)
        m["front_share"] = float((lc["xr"] - cg[0]) / (lc["xr"] - lc["xf"]))
        m["motor_load_ratio"] = float(max(m["front_share"], 1 - m["front_share"]) / 0.5)
    ov, bv = overlaps(plan), bay_violations(plan)
    if base is not None:
        ov = [o for o in ov if (o[0], o[1]) not in {(a, b) for a, b, _ in base["_overlaps"]}]
        bv = [b for b in bv if b[0] not in {x[0] for x in base["_bay"]}]
        m["d_mass_g"] = (M - base["mass_kg"]) * 1000
    m["_overlaps"], m["_bay"] = ov, bv
    m["new_overlaps"] = [f"{a} x {b} ({p:.1f} mm)" for a, b, p in ov] if base is not None else []
    m["new_bay_violations"] = [f"{n} out of {b} by {p:.1f} mm" for n, b, p in bv] if base is not None else []
    return m


# ------------------------------------------------------------------------------------------ movable parts
def movables(plan, cfg):
    """Components the optimiser may slide in x: name -> dict(m_eff, x, lo, hi, weight, why_bounds)."""
    bays = plan["airframe"]["fuselage"].get("bays") or {}
    default_travel = cfg.get("default_travel_m", 0.03)
    out = {}
    for c in plan["components"]:
        flag = c.get("movable")
        nm = c["name"].lower()
        if flag is False or c.get("fixed"):
            continue
        if flag is None:
            if c.get("category") not in ("energy", "avionics", "payload", "propulsion", "ballast"):
                continue
            if c.get("external") or c.get("shape") == "cylinder" or any(w in nm for w in FIXED_WORDS):
                continue
            if c.get("geom") is False or abs(c["position"][1]) > 0.08:
                continue
        x = c["position"][0]
        half = (c.get("dims") or [0, 0, 0])[0] / 2
        if c.get("move_range_x"):
            lo, hi, why = c["move_range_x"][0], c["move_range_x"][1], "parts list travel"
        elif c.get("bay") in bays:
            b = bays[c["bay"]]
            lo, hi, why = b["center"][0] - b["size"][0] / 2 + half, b["center"][0] + b["size"][0] / 2 - half, f"bay {c['bay']}"
        else:
            lo, hi, why = x - default_travel, x + default_travel, f"assumed +-{default_travel*1000:.0f} mm (no bay / travel given)"
        lo, hi = min(lo, x), max(hi, x)
        out[c["name"]] = dict(m=P.val(c["mass"]) * (2 if c.get("mirror_y") else 1), x=x, lo=lo, hi=hi, why=why,
                              w=0.5 if c.get("category") == "energy" else 1.0, cat=c.get("category"))
    return out


def solve_moves(plan, mov, need_kgm, clearance=0.002, only=None):
    """LP: smallest total weighted travel so that sum(m_i dx_i) = need_kgm, inside the travel limits, without
    parts that share a y-z footprint passing through each other. Returns ({name: dx}, achieved_kgm)."""
    from scipy.optimize import linprog
    names = [n for n in sorted(mov) if only is None or n in only]
    if not names:
        return {}, 0.0
    n = len(names)
    bx = boxes(plan)
    A, b = [], []
    for i, a in enumerate(names):
        if a not in bx:
            continue
        for other in bx:
            if other == a:
                continue
            pa, ha = bx[a]
            po, ho = bx[other]
            if abs(pa[1] - po[1]) >= ha[1] + ho[1] or abs(pa[2] - po[2]) >= ha[2] + ho[2]:
                continue                                    # footprints do not overlap in y-z: free to pass
            gap = abs(pa[0] - po[0]) - (ha[0] + ho[0])
            room = max(gap - clearance, 0.0) if gap > 0 else 0.0
            sgn = 1.0 if po[0] > pa[0] else -1.0            # a may move towards `other` by at most room (+ other's own move)
            row = np.zeros(2 * n)
            row[i], row[n + i] = sgn, -sgn
            if other in names:
                j = names.index(other)
                if j < i:
                    continue
                row[j], row[n + j] = -sgn, sgn
            A.append(row); b.append(room)
    m = np.array([mov[k]["m"] for k in names])
    w = np.array([mov[k]["w"] for k in names])
    bounds = [(0, mov[k]["hi"] - mov[k]["x"]) for k in names] + [(0, mov[k]["x"] - mov[k]["lo"]) for k in names]
    kw = dict(A_ub=np.array(A) if A else None, b_ub=np.array(b) if b else None, bounds=bounds, method="highs")
    r = linprog(np.concatenate([w, w]), A_eq=[np.concatenate([m, -m])], b_eq=[need_kgm], **kw)
    if r.status != 0:                                       # not reachable: go as far as the limits allow
        s = 1.0 if need_kgm > 0 else -1.0
        r = linprog(-s * np.concatenate([m, -m]) + 1e-6 * np.concatenate([w, w]), **kw)
        if r.status != 0:
            return {}, 0.0
    dx = r.x[:n] - r.x[n:]
    dx[np.abs(dx) < 2e-4] = 0.0
    return {k: float(round(v, 4)) for k, v in zip(names, dx) if v}, float(m @ dx)


# ------------------------------------------------------------------------------------------ option generators
def wing_attached(plan, cfg):
    names = set(cfg.get("wing_attached") or [])
    wing = P.main_wing(plan)
    spars = {s["name"] for s in wing.get("spars", [])}
    servos = {cs["servo"] for cs in wing.get("control_surfaces", [])}
    booms = plan["airframe"].get("booms", [])
    hw = plan["airframe"]["fuselage"].get("max_width", 0.1) / 2
    for c in plan["components"]:
        n = c["name"]
        gs = c.get("geom_spar")
        gs = set(gs if isinstance(gs, list) else [gs]) if gs else set()
        if "tail" in n.lower() or str(c.get("geom_part", "")).startswith(("vtail", "htail", "fin")):
            continue
        if str(c.get("geom_part", "")).startswith("wing") or (gs and gs <= spars) or n in servos or c.get("mount") == "wing" \
                or abs(c["position"][1]) > hw + 0.01:
            names.add(n)
    for b in booms:
        names.add(b["name"])
        names.update(P.mirror_name(t) if t not in {c["name"] for c in plan["components"]} else t for t in b.get("tip_mass_components", []))
    return names


def ballast_station(plan, cfg, nose):
    st = (cfg.get("ballast_stations") or {}).get("nose" if nose else "tail")
    if st:
        return list(st)
    fus = plan["airframe"]["fuselage"]
    x0 = fus.get("x_nose", 0.0)
    return [round(x0 + 0.03, 4), 0.0, 0.0] if nose else [round(x0 + fus.get("length", 1.0) - 0.05, 4), 0.0, 0.0]


def battery_alternatives(plan, cfg, catalog, policy):
    """Alternative packs: same cell with one more / one fewer parallel group, catalog cells in the same S x P,
    and anything listed under analysis.balance.alternatives.battery (from the parts list)."""
    en = plan["energy"]
    cell = en["cell"]
    S, Pp = en["series"], en["parallel"]
    bat = next((c for c in plan["components"] if c.get("category") == "energy"), None)
    if not bat:
        return []
    m0 = P.val(bat["mass"])
    over = m0 / (S * Pp * cell["mass"])                       # pack overhead (nickel, wrap, leads)
    E0 = S * Pp * cell["capacity_Ah"] * cell.get("v_nom", 3.6)
    alts = []
    for p2 in (Pp - 1, Pp + 1):
        if p2 >= 1:
            alts.append(dict(label=f"{S}S{p2}P {cell['model']}", mass=round(S * p2 * cell["mass"] * over, 4), energy_ratio=p2 / Pp,
                             fields=dict(part_number=f"{S}S{p2}P {cell['model']}"), energy_patch=dict(parallel=p2),
                             dims_scale=[1.0, p2 / Pp, 1.0], origin=bat.get("origin"), note="same cell, pack rebuilt"))
    for c in (catalog or {}).get("cells", []):
        if c["model"] == cell["model"]:
            continue
        e = S * Pp * c["capacity_Ah"] * c.get("v_nom", 3.6)
        alts.append(dict(label=f"{S}S{Pp}P {c['model']}", mass=round(S * Pp * c["mass"] * over, 4), energy_ratio=e / E0,
                         fields=dict(part_number=f"{S}S{Pp}P {c['model']}"), energy_patch=dict(cell={k: c[k] for k in c if k in (
                             "model", "capacity_Ah", "r_dc", "i_cont", "v_nom", "v_max", "v_min", "mass")}),
                         origin=c.get("origin"), origin_verified=c.get("origin_verified", False), note=c.get("source", "catalog")))
    for a in ((cfg.get("alternatives") or {}).get("battery") or []):
        alts.append(dict(label=a.get("part_number", "alternative pack"), mass=P.val(a["mass"]), energy_ratio=a.get("energy_ratio", 1.0),
                         fields={k: a[k] for k in ("part_number", "dims") if k in a}, energy_patch=a.get("energy"),
                         origin=a.get("origin"), origin_verified=a.get("origin_verified", False), note="parts-list alternative"))
    return [a for a in alts if origin_ok(a.get("origin"), policy)[0]]


US_NAMES = {"us", "usa", "united states", "u.s.", "u.s.a.", "united states of america"}
NDAA_COVERED = {"china", "prc", "people's republic of china", "russia", "iran", "north korea", "dprk"}


def origin_ok(origin, policy):
    """(allowed, reason). A screening aid for the parts list, not a legal compliance determination."""
    pol = (policy or "any").lower()
    if pol == "any":
        return True, ""
    o = str(origin or "").strip().lower()
    if not o:
        return False, "origin unknown"
    country = o.split("(")[0].strip()
    if pol == "us_made":
        return country in US_NAMES, f"origin {origin}"
    if pol == "ndaa":
        return country not in NDAA_COVERED, f"origin {origin}"
    return True, ""


def generate(plan, mass, aero, T, cfg, catalog=None, policy="any", hover_tw=None):
    """-> list of options [{id, title, kind, edits, metrics, pros, cons, effort, score, feasible}] sorted best first."""
    cg_offset = np.array(mass["cg_m"]) - np.array(mass["plan_model"]["cg_m"])
    base = evaluate(plan, None, T, cg_offset, aero)
    M = base["mass_kg"]
    need = (T["target"] - base["cg_x"]) * M                   # kg m of moment to add (+ = move mass aft)
    mov = movables(plan, cfg)
    lc = lift_centre(plan)
    thr_tol = cfg.get("thrust_centre_tol_frac", 0.05) * ((lc["xr"] - lc["xf"]) if lc else 0.0)
    opts = []

    def add(kind, title, edits, effort, notes=(), np_shift=0.0, extra=None):
        p2 = apply(plan, edits)
        m = evaluate(p2, base, T, cg_offset, aero, np_shift)
        opts.append(dict(kind=kind, title=title, edits=edits, metrics=m, effort=effort, notes=list(notes), extra=extra or {}))

    # 0. as designed
    add("none", "Keep the layout as drawn", [], 0.0)

    bat = next((c for c in plan["components"] if c.get("category") == "energy" and c["name"] in mov), None)
    # A. slide the battery only
    if bat and abs(need) > 1e-6:
        mv, got = solve_moves(plan, mov, need, only={bat["name"]})
        if mv:
            add("battery", f"Slide the battery {abs(mv[bat['name']])*1000:.0f} mm {'aft' if mv[bat['name']] > 0 else 'forward'}",
                [dict(kind="move", name=bat["name"], dx=mv[bat["name"]], why="put the CG on target")], 1.0,
                notes=[f"battery travel limited by {mov[bat['name']]['why']}"] if abs(got - need) > 1e-4 else [])
    # B. re-arrange internals (least total travel)
    if abs(need) > 1e-6 and len(mov) > 1:
        mv, got = solve_moves(plan, mov, need)
        if mv and (len(mv) > 1 or not bat or bat["name"] not in mv):
            add("internals", "Re-arrange internal parts (" + ", ".join(f"{k} {v*1000:+.0f} mm" for k, v in mv.items()) + ")",
                [dict(kind="move", name=k, dx=v, why="put the CG on target with the least total travel") for k, v in mv.items()],
                1.0 + 0.75 * sum(k != (bat or {}).get("name") for k in mv))
            if abs(got - need) > 1e-4 * max(M, 1):           # internals alone fall short: top up with the least ballast
                p2 = apply(plan, [dict(kind="move", name=k, dx=v) for k, v in mv.items()])
                cg2 = evaluate(p2, base, T, cg_offset, aero)["cg_x"]
                xb = ballast_station(plan, cfg, nose=need < 0)
                if abs(xb[0] - T["target"]) > 0.02:
                    mb = M * (T["target"] - cg2) / (xb[0] - T["target"])
                    if mb > 0.0005:
                        comp = dict(name="ballast", category="ballast", mass=dict(value=round(mb, 4), source="auto-balance: ballast for the remainder"),
                                    position=xb, dims=[0.02, 0.02, 0.01], geom=False)
                        add("internals+ballast", f"Move internal parts as far as they go, then {mb*1000:.0f} g of ballast in the "
                            f"{'nose' if need < 0 else 'tail'}",
                            [dict(kind="move", name=k, dx=v, why="take up as much of the CG error as the travel allows") for k, v in mv.items()]
                            + [dict(kind="add", component=comp, why="ballast only for what the moves cannot reach")],
                            3.0 + 0.75 * sum(k != (bat or {}).get("name") for k in mv), notes=["dead weight, but far less than ballast alone"])
        max_fwd = solve_moves(plan, mov, -1e3)[1]
        max_aft = solve_moves(plan, mov, 1e3)[1]
    else:
        max_fwd = max_aft = 0.0
    reach = dict(max_cg_shift_fwd_mm=max_fwd / M * 1000, max_cg_shift_aft_mm=max_aft / M * 1000)

    # C. quadplane: put the lift-thrust centre on the design CG (+ battery to hold the CG)
    if lc and abs(T["target"] - lc["x"]) > 0.3 * thr_tol:
        lift = P.units(plan, "lift")[0]
        pods = [c for c in plan["components"] if c.get("category") == "propulsion" and c.get("shape") == "cylinder"
                and any(np.linalg.norm(np.subtract(q, [c["position"][0], abs(c["position"][1]), c["position"][2]])[:2]) < 0.06
                        for q in [[p[0], abs(p[1]), p[2]] for p in lift["positions"]])]
        m_l = sum(P.val(c["mass"]) * (2 if c.get("mirror_y") else 1) for c in pods)
        dxl = T["target"] - lc["x"]
        edits = [dict(kind="unit", unit=lift["name"], dx=dxl, why="lift-thrust centre onto the design CG")]
        edits += [dict(kind="move", name=c["name"], dx=dxl, why="lift motor pod moves with its motor") for c in pods]
        edits += [dict(kind="boom", name=b["name"], dx=dxl, why="boom slides in its wing clamps") for b in plan["airframe"].get("booms", [])]
        rest = need - m_l * dxl
        if bat and abs(rest) > 1e-6:
            mv, _ = solve_moves(plan, mov, rest, only={bat["name"]})
            edits += [dict(kind="move", name=k, dx=v, why="hold the CG on target after moving the lift motors") for k, v in mv.items()]
        add("lift_system", f"Move the lift motors/booms {abs(dxl)*1000:.0f} mm {'aft' if dxl > 0 else 'forward'} so they straddle the CG",
            edits, 6.0, notes=["re-drill / re-print the boom clamps; re-check prop-to-wing clearance (clash stage) and boom vibration (structures)"])

    # D. move the wing along the fuselage
    wa = wing_attached(plan, cfg)
    m_non = sum(P.val(c["mass"]) * (2 if c.get("mirror_y") else 1) for c in plan["components"] if c["name"] not in wa)
    d = -need / m_non if m_non > 0 else 0.0                    # wing moves aft by d == everything else moves forward by d
    if abs(d) > 0.002 and abs(d) < cfg.get("max_wing_shift_m", 0.08):
        edits = [dict(kind="move", name=c["name"], dx=-d, why=f"wing moved {d*1000:+.0f} mm along the fuselage") for c in plan["components"]
                 if c["name"] not in wa]
        fus = plan["airframe"]["fuselage"]
        if "x_nose" in fus:
            edits.append(dict(kind="path", path="/airframe/fuselage/x_nose", old=fus["x_nose"], value=round(fus["x_nose"] - d, 5), why="fuselage datum"))
        if fus.get("stations"):
            edits.append(dict(kind="path", path="/airframe/fuselage/stations", old="(stations)", why="fuselage datum",
                              value=[[round(s[0] - d, 5)] + list(s[1:]) for s in fus["stations"]]))
        for bn, b in (fus.get("bays") or {}).items():
            edits.append(dict(kind="path", path=f"/airframe/fuselage/bays/{bn}/center", old=b["center"], why="bay moves with the fuselage",
                              value=[round(b["center"][0] - d, 5), b["center"][1], b["center"][2]]))
        tail_arm = None
        for s in plan["airframe"]["lifting_surfaces"]:
            if s.get("kind", "wing") != "wing":
                x0 = P.val(s.get("x_le_root"), 0.0)
                edits.append(dict(kind="path", path=f"/airframe/lifting_surfaces/{s['name']}/x_le_root", old=x0, value=round(x0 - d, 5),
                                  why="tail moves with the fuselage"))
                tail_arm = tail_arm or (x0 + 0.25 * P.val(s["root_chord"]) - base["cg_x"])
        for u in P.units(plan, "cruise"):
            edits.append(dict(kind="unit", unit=u["name"], dx=-d, why="pusher moves with the fuselage"))
        np_shift = 0.0
        notes = ["re-print the wing saddle / centre fuselage; re-run aero, CFD and SITL (tail arm changes)"]
        if tail_arm and T.get("x_np") is not None:
            x_acw = mass["x_le_mac_m"] + 0.25 * T["mac"]
            np_shift = -d * (T["x_np"] - x_acw) / tail_arm
            notes.append(f"tail arm {tail_arm*1000:.0f} -> {(tail_arm-d)*1000:.0f} mm ({-100*d/tail_arm:+.1f} % tail volume)")
        add("wing", f"Move the wing {abs(d)*1000:.0f} mm {'aft' if d > 0 else 'forward'} on the fuselage", edits, 8.0, notes, np_shift,
            extra=dict(summary_rows=[dict(item="wing and everything mounted on it (" + ", ".join(sorted(wa)) + ")", change="move along the fuselage",
                                          frm="", to="", delta="%+.1f mm (%s)" % (d * 1000, "aft" if d > 0 else "forward"),
                                          why="puts the design CG under the aircraft's actual CG; fuselage, tail and everything in them stay put",
                                          cad_body="")]))

    # E. lengthen the battery bay when the battery alone runs out of travel
    if bat and abs(need) > 1e-6:
        bm = mov[bat["name"]]
        want = need / bm["m"]
        over = (bm["x"] + want) - (bm["hi"] if want > 0 else bm["lo"])
        bay = (plan["airframe"]["fuselage"].get("bays") or {}).get(bat.get("bay") or "")
        if bay and ((want > 0 and over > 0.001) or (want < 0 and over < -0.001)) and abs(over) <= cfg.get("max_bay_extension_m", 0.06):
            ext = math.ceil(abs(over) * 1000 / 5) * 0.005 * (1 if over > 0 else -1)
            newc = [round(bay["center"][0] + ext / 2, 5), bay["center"][1], bay["center"][2]]
            news = [round(bay["size"][0] + abs(ext), 5), bay["size"][1], bay["size"][2]]
            edits = [dict(kind="path", path=f"/airframe/fuselage/bays/{bat['bay']}/center", old=bay["center"], value=newc, why="longer battery bay"),
                     dict(kind="path", path=f"/airframe/fuselage/bays/{bat['bay']}/size", old=bay["size"], value=news, why="longer battery bay"),
                     dict(kind="move", name=bat["name"], dx=want, why="battery slides into the extended bay")]
            add("bay", f"Extend the battery bay {abs(ext)*1000:.0f} mm {'aft' if ext > 0 else 'forward'} and slide the battery "
                       f"{abs(want)*1000:.0f} mm", edits, 5.0,
                notes=["frame change: check the fuselage outer shape still encloses the bay (clash stage) and re-print that section"])

    # F. swap the battery
    for a in battery_alternatives(plan, cfg, catalog, policy):
        if not bat:
            break
        fields = dict(a["fields"], mass=dict(value=a["mass"], source=f"estimate: {a['note']}"))
        if a.get("dims_scale") and bat.get("dims"):
            fields["dims"] = [round(v * s, 4) for v, s in zip(bat["dims"], a["dims_scale"])]
        edits = [dict(kind="set", name=bat["name"], fields=fields, why=f"battery swap to {a['label']}")]
        for k, v in (a.get("energy_patch") or {}).items():
            edits.append(dict(kind="path", path=f"/energy/{k}", old=plan["energy"].get(k), value=v, why="pack definition follows the swap"))
        p2 = apply(plan, edits)
        need2 = (T["target"] - evaluate(p2, base, T, cg_offset, aero)["cg_x"]) * (M + a["mass"] - P.val(bat["mass"]))
        mv, _ = solve_moves(p2, movables(p2, cfg), need2)
        edits += [dict(kind="move", name=k, dx=v, why="re-centre the CG with the new pack") for k, v in mv.items()]
        add("swap", f"Swap the battery for {a['label']} ({a['mass']*1000:.0f} g)", edits, 3.0,
            notes=[f"origin: {a.get('origin') or 'unknown'}" + ("" if a.get("origin_verified") else " (unverified)"),
                   "re-run propulsion, thermal and SITL: voltage sag, current per cell and hover time change"],
            extra=dict(energy_ratio=a["energy_ratio"]))

    # G. ballast (last resort)
    if abs(need) > 1e-6:
        xb = ballast_station(plan, cfg, nose=need < 0)
        den = (xb[0] - T["target"])
        if abs(den) > 0.02:
            mb = M * (T["target"] - base["cg_x"]) / den
            if mb > 0.0005:
                comp = dict(name="ballast", category="ballast", mass=dict(value=round(mb, 4), source="auto-balance: last-resort ballast"),
                            position=xb, dims=[0.02, 0.02, 0.01], geom=False)
                add("ballast", f"Add {mb*1000:.0f} g of ballast in the {'nose' if need < 0 else 'tail'}", [dict(kind="add", component=comp,
                    why="last resort: dead mass to put the CG on target")], 2.0, notes=["dead weight: costs range and hover margin on every flight"])

    def power_ratio(m_new):
        """Cruise drag at the new weight / drag now, from the aero stage's polar at the plan's cruise speed
        (falls back to weight^1.5, i.e. flying at the best L/D speed, without a polar)."""
        if not aero or not all(k in aero for k in ("CD0", "k", "S")):
            return (m_new / M) ** 1.5
        q = 0.5 * 1.225 * plan["envelope"]["v_cruise"] ** 2 * aero["S"]
        cd = lambda m_: aero["CD0"] + aero["k"] * (m_ * 9.80665 / q - aero.get("CL_minD", 0.0)) ** 2
        return cd(m_new) / cd(M)

    for o in opts:
        finish(o, base, M, T, thr_tol, lc, hover_tw, power_ratio)
    # an in-window layout needs no change: "keep as drawn" then leads and the rest are alternatives
    opts.sort(key=lambda o: (not o["feasible"], not (o["kind"] == "none" and o["feasible"]), o["score"]))
    for i, o in enumerate(o_ for o_ in opts if o_["kind"] != "none"):
        o["id"] = chr(ord("A") + i)
    for o in opts:
        if o["kind"] == "none":
            o["id"] = "0"
    return dict(base=base, need_kgm=need, options=opts, reach=reach, movables=mov, thrust_tol_mm=thr_tol * 1000, wing_attached=sorted(wa))


def finish(o, base, M, T, thr_tol, lc, hover_tw=None, power_ratio=None):
    m = o["metrics"]
    dm = m.get("d_mass_g", 0.0)
    e_ratio = o["extra"].get("energy_ratio", 1.0)
    pr = power_ratio(M + dm / 1000) if power_ratio else ((M + dm / 1000) / M) ** 1.5
    m["range_change_pct"] = round(100 * (e_ratio / pr - 1), 1)
    thr_ok = True
    if lc:
        thr_ok = abs(m["thrust_centre_offset_mm"]) <= thr_tol * 1000 + 1e-6
    m["thrust_centre_ok"] = thr_ok
    o["feasible"] = bool(m["in_window"] and not m["new_overlaps"] and not m["new_bay_violations"])
    pros, cons = [], []
    if m["in_window"]:
        pros.append(f"CG {m['cg_err_mm']:+.1f} mm from target (inside the +-{T['tol']*1000:.0f} mm window)")
    else:
        cons.append(f"CG still {m['cg_err_mm']:+.1f} mm from target (window +-{T['tol']*1000:.0f} mm)")
    if m.get("static_margin") is not None:
        sm = T.get("sm_range", [0.05, 0.15])
        ok_sm = sm[0] <= m["static_margin"] <= sm[1]
        (pros if ok_sm else cons).append(f"static margin {100*m['static_margin']:.0f} % MAC" + ("" if ok_sm else
                                         f" (outside the plan's {100*sm[0]:.0f}-{100*sm[1]:.0f} % range: "
                                         + ("sluggish in pitch, more trim drag" if m["static_margin"] > sm[1] else "twitchy in pitch") + ")"))
    if lc:
        (pros if thr_ok else cons).append(f"CG {m['thrust_centre_offset_mm']:+.0f} mm from the lift-thrust centre "
                                          f"(front motors carry {100*m['front_share']:.0f} %)")
    if abs(dm) > 0.5:
        (cons if dm > 0 else pros).append(f"AUW {dm:+.0f} g")
    tw_pen = 0.0
    if hover_tw:
        m["hover_TW"] = round(hover_tw * M / (M + dm / 1000), 2)
        if abs(dm) > 20:
            (cons if dm > 0 else pros).append(f"hover thrust-to-weight {hover_tw:.2f} -> {m['hover_TW']:.2f}")
        if dm > 20:
            tw_pen = 20 * (max(1.6 - m["hover_TW"], 0.0) - max(1.6 - hover_tw, 0.0))
    if abs(m["range_change_pct"]) >= 0.5:
        (pros if m["range_change_pct"] > 0 else cons).append(f"range about {m['range_change_pct']:+.0f} % (estimate)")
    if o["kind"] in ("battery", "internals"):
        pros.append("no re-print, no new parts")
    if o["kind"] in ("ballast", "internals+ballast"):
        cons.append("ballast is a last resort")
    if o["kind"] in ("lift_system", "wing", "bay"):
        cons.append("frame change: re-print and re-validate")
    if o["kind"] == "swap":
        cons.append("new part to buy and test")
    if o["kind"] == "none" and m["in_window"]:
        pros.append("nothing to change")
    cons += m["new_overlaps"] + m["new_bay_violations"] + o["notes"]
    o["pros"], o["cons"] = pros, cons
    o["score"] = round(o["effort"] + 0.4 * max(-m["range_change_pct"], 0) - min(0.05 * max(m["range_change_pct"], 0), 1.0)
                       + 0.15 * abs(m["cg_err_mm"]) + (0.0 if thr_ok else 2.0 + 0.05 * abs(m.get("thrust_centre_offset_mm", 0)))
                       + tw_pen + (3.0 if "ballast" in o["kind"] else 0.0) + 10 * (len(m["new_overlaps"]) + len(m["new_bay_violations"])), 2)


def change_list(plan, option):
    """Rows for the Fusion change list: what to move/change, from -> to, why."""
    rows = []
    if option.get("extra", {}).get("summary_rows"):
        return list(option["extra"]["summary_rows"])
    for e in option["edits"]:
        k = e["kind"]
        if k == "move":
            c = comp_by_name(plan, e["name"])
            old = c["position"]
            new = [old[0] + e.get("dx", 0), old[1] + e.get("dy", 0), old[2] + e.get("dz", 0)]
            rows.append(dict(item=c["name"] + (" (+ mirrored twin)" if c.get("mirror_y") else ""), change="move",
                             frm="x %.1f mm" % (old[0] * 1000), to="x %.1f mm" % (new[0] * 1000),
                             delta="%+.1f mm (%s)" % (e.get("dx", 0) * 1000, "aft" if e.get("dx", 0) > 0 else "forward"), why=e["why"],
                             cad_body=c.get("cad_body", "")))
        elif k == "set":
            c = comp_by_name(plan, e["name"])
            for f, v in e["fields"].items():
                rows.append(dict(item=c["name"], change=f"replace {f}", frm=str(P.val(c.get(f)) if f == "mass" else c.get(f)),
                                 to=str(P.val(v) if f == "mass" else v), delta="", why=e["why"], cad_body=c.get("cad_body", "")))
        elif k == "add":
            c = e["component"]
            rows.append(dict(item=c["name"], change="add", frm="-", to="%.0f g at x %.0f mm" % (P.val(c["mass"]) * 1000, c["position"][0] * 1000),
                             delta="", why=e["why"], cad_body=""))
        elif k == "unit":
            rows.append(dict(item=f"propulsion unit '{e['unit']}' (motor axes)", change="move", frm="", to="",
                             delta="%+.1f mm (%s)" % (e["dx"] * 1000, "aft" if e["dx"] > 0 else "forward"), why=e["why"], cad_body=""))
        elif k == "boom":
            rows.append(dict(item=f"boom '{e['name']}' (+ mirrored twin)", change="slide in clamps", frm="", to="",
                             delta="%+.1f mm (%s)" % (e["dx"] * 1000, "aft" if e["dx"] > 0 else "forward"), why=e["why"], cad_body=""))
        elif k == "path":
            if e["path"].endswith("/stations"):
                continue
            rows.append(dict(item=e["path"], change="set", frm=str(e.get("old")), to=str(e["value"]), delta="", why=e["why"], cad_body=""))
    return rows
