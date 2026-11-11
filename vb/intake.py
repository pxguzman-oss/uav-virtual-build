"""CAD + parts list -> build plan.

Inputs
  * a STEP assembly exported from the CAD tool (Fusion: File > Export > STEP), every part and bought
    component a named body, and/or a folder with one STEP per part
  * a parts list (CSV or XLSX, see docs/PARTS_LIST.md): one row per part with id, category, mass or
    print settings, and optional thermal / sourcing / travel columns
  * a base plan with everything neither CAD nor a parts list can give (aerofoil and planform as analysed,
    motor / propeller / cell data, mission, environment)
  * the frame: where the wing-root leading edge is in CAD coordinates and which CAD axes are aft/right/up

Matching: a body belongs to the row whose id starts its name (P012_battery -> P012), else the row whose
cad_name equals it, else the closest name (>= 0.8 similarity). Nothing is guessed silently: unmatched
bodies, rows without a body and without a position, rows without a mass, and the frame when it was only
suggested, all go to the QUESTIONS list of the intake report and make the intake exit non-zero.

Printed parts: mass = slicer weight when given, else from the CAD volume with the print settings
(thin-wall bodies: density x volume; solid bodies: walls + infill). CG = CAD centroid. Bought parts:
mass from the list, CG = CAD centroid of the body, size = its bounding box."""
from __future__ import annotations

import copy
import csv
import difflib
import json
from pathlib import Path

import numpy as np
import yaml

from . import cadio, plan as P

COLS = ["id", "name", "plan_name", "cad_name", "category", "role", "qty", "mass_g", "mass_source", "material", "print_mode", "wall_mm",
        "infill", "slicer_g", "bay", "movable", "move_min_mm", "move_max_mm", "power_w", "t_max_c", "external", "origin",
        "origin_verified", "vendor_part", "x_mm", "y_mm", "z_mm", "extra", "notes"]
ALIAS = {"part": "name", "part_name": "name", "description": "name", "item": "name", "part_id": "id", "number": "id", "quantity": "qty",
         "mass": "mass_g", "weight_g": "mass_g", "weight": "mass_g", "unit_mass_g": "mass_g", "power": "power_w", "max_temp_c": "t_max_c",
         "country": "origin", "made_in": "origin", "part_number": "vendor_part", "model": "vendor_part", "slicer_weight_g": "slicer_g"}
CATS = {"structure_printed", "structure_tube", "propulsion", "energy", "avionics", "servo", "payload", "wiring", "hardware", "ballast", "other"}


def _num(v):
    try:
        return None if v in (None, "") or (isinstance(v, float) and np.isnan(v)) else float(v)
    except (TypeError, ValueError):
        return None


def _yes(v):
    s = str(v).strip().lower()
    return True if s in ("y", "yes", "true", "1") else False if s in ("n", "no", "false", "0") else None


def read_parts(path):
    path = Path(path)
    if path.suffix.lower() in (".xlsx", ".xlsm", ".xls"):
        import pandas as pd
        rows = pd.read_excel(path).to_dict("records")
    else:
        with open(path, newline="", encoding="utf-8-sig") as f:
            rows = list(csv.DictReader(f))
    out, problems = [], []
    for n, r in enumerate(rows, start=2):
        d = {}
        for k, v in r.items():
            key = str(k).strip().lower().replace(" ", "_")
            key = ALIAS.get(key, key)
            d[key] = "" if v is None or (isinstance(v, float) and np.isnan(v)) else (v.strip() if isinstance(v, str) else v)
        if not str(d.get("name", "")).strip() and not str(d.get("id", "")).strip():
            continue
        d["_row"] = n
        d["id"] = str(d.get("id", "")).strip()
        d["qty"] = int(_num(d.get("qty")) or 1)
        cat = str(d.get("category", "")).strip().lower()
        if cat not in CATS:
            problems.append(f"row {n} '{d.get('name')}': category '{cat}' is not one of {sorted(CATS)}")
            cat = "other"
        d["category"] = cat
        try:
            d["extra"] = yaml.safe_load(d["extra"]) if isinstance(d.get("extra"), str) and d["extra"].strip() else {}
        except yaml.YAMLError as e:
            problems.append(f"row {n}: 'extra' is not valid YAML ({e})")
            d["extra"] = {}
        out.append(d)
    ids = [r["id"] for r in out if r["id"]]
    for i in sorted({i for i in ids if ids.count(i) > 1}):
        problems.append(f"id {i} appears on more than one row")
    return out, problems


def match(bodies, rows):
    """-> ({row_index: [bodies]}, unmatched_bodies, how{body.path: reason})"""
    got, how, un = {}, {}, []
    idn = {cadio.norm(r["id"]): i for i, r in enumerate(rows) if r["id"]}
    cadn = {cadio.norm(r["cad_name"]): i for i, r in enumerate(rows) if r.get("cad_name")}
    names = {cadio.norm(r.get("name", "")): i for i, r in enumerate(rows)}
    for b in bodies:
        segs = [cadio.norm(s) for s in b.path.split("/")] + [cadio.norm(b.name)]
        hit = None
        for s in segs:
            for k, i in idn.items():
                if s == k or s.startswith(k + "_"):
                    hit, why = i, f"id {rows[i]['id']}"
            if hit is None and s in cadn:
                hit, why = cadn[s], "cad_name"
        if hit is None:
            best = max(((difflib.SequenceMatcher(None, s, k).ratio(), i) for s in segs for k, i in names.items() if k), default=(0, None))
            if best[0] >= 0.8:
                hit, why = best[1], f"name similarity {best[0]:.2f}"
        if hit is None:
            un.append(b)
        else:
            got.setdefault(hit, []).append(b)
            how[b.path] = why
    return got, un, how


def printed_mass(b, row, materials, defaults):
    """(mass_kg, cg_cad_mm, source) of a printed body."""
    sg = _num(row.get("slicer_g"))
    mat = row.get("material") or defaults.get("material")
    rho = (materials.get(mat) or {}).get("density")
    wall = (_num(row.get("wall_mm")) or defaults.get("wall_mm", 0.45)) / 1000.0
    infill = _num(row.get("infill"))
    infill = defaults.get("infill", 0.03) if infill is None else infill
    V, A = b.volume * 1e-9, b.area * 1e-6
    mean_t = 2 * V / max(A, 1e-12)
    mode = str(row.get("print_mode") or "").lower() or ("solid" if mean_t < 3 * wall else "shell")
    if mode == "solid":
        m, cg, how = (rho or 0) * V, b.centroid, f"CAD volume {V*1e6:.1f} cm3 x {rho} kg/m3 (thin-wall body, mean wall {mean_t*1000:.2f} mm)"
    else:
        ms, mi = (rho or 0) * A * wall, (rho or 0) * infill * max(V - A * wall, 0.0)
        m = ms + mi
        cg = (ms * b.surf_centroid + mi * b.centroid) / max(m, 1e-12)
        how = f"shell model: {A*1e4:.0f} cm2 x {wall*1000:.2f} mm wall + {100*infill:.0f} % infill of {V*1e6:.0f} cm3, {rho} kg/m3"
    if sg is not None:
        return sg / 1000.0, cg, f"slicer weight {sg:g} g (geometry model would give {m*1000:.0f} g)", m
    if not rho:
        return None, cg, f"no density for material '{mat}'", None
    return m, cg, "estimate: " + how, m


def build(bodies, rows, base_plan, frame, frame_suggested=False, defaults=None, policy=None, name=None, merge=None):
    """-> (plan dict, report dict). base_plan is a RESOLVED-free raw plan (variants already merged by the caller)."""
    defaults = defaults or {}
    plan = copy.deepcopy(base_plan)
    plan.pop("variants", None)
    if merge:
        plan = P._merge(plan, merge)
    mats = plan["materials"]
    got, unmatched, how = match(bodies, rows)
    comps, questions, table, cad_map = [], [], [], {}
    lift_pts, cruise_pts = [], []
    for i, r in enumerate(rows):
        bs = got.get(i, [])
        base = (r.get("plan_name") or cadio.norm(r.get("name") or r["id"]))
        cat = r["category"]
        placed = []                                   # (name, position, dims, mirror, body list, mass, mass_src)
        pts = [(b, frame.point(b.centroid)) for b in bs]
        if not bs:
            xyz = [_num(r.get(k)) for k in ("x_mm", "y_mm", "z_mm")]
            if xyz[0] is None:
                questions.append(f"'{r.get('name')}' ({r['id']}) has no CAD body and no x_mm/y_mm/z_mm: where is it?")
                continue
            placed.append((base.replace("{pos}", ""), [xyz[0] / 1000, (xyz[1] or 0) / 1000, (xyz[2] or 0) / 1000], None, False, [], None, None))
        else:
            if len(bs) != r["qty"]:
                questions.append(f"'{r.get('name')}' ({r['id']}): parts list says qty {r['qty']}, CAD has {len(bs)} bodies")
            used = set()
            pairs, singles = [], []
            for a, (ba, pa) in enumerate(pts):
                if a in used:
                    continue
                tw = next((c for c, (bc, pc) in enumerate(pts) if c != a and c not in used and abs(pc[1] + pa[1]) < 2e-3
                           and abs(pc[0] - pa[0]) < 2e-3 and abs(pc[2] - pa[2]) < 2e-3 and abs(pa[1]) > 2e-3), None)
                if tw is not None:
                    used |= {a, tw}
                    pairs.append((a, tw) if pa[1] > 0 else (tw, a))
                else:
                    used.add(a)
                    singles.append(a)
            pairs.sort(key=lambda p_: pts[p_[0]][1][0])
            for k, (a, tw) in enumerate(pairs):
                tag = ("FR", "RR")[k] if len(pairs) == 2 else ("R" if len(pairs) == 1 else f"{k+1}R")
                nm = base.replace("{pos}", tag) if "{pos}" in base else (base if len(pairs) == 1 and r.get("plan_name") else f"{base}_{tag}")
                placed.append((nm, pts[a][1], frame.extents(pts[a][0].bbox), True, [pts[a][0], pts[tw][0]], None, None))
            for k, a in enumerate(singles):
                nm = base.replace("{pos}", str(k + 1)) if "{pos}" in base else (base if len(singles) == 1 else f"{base}_{k+1}")
                placed.append((nm, pts[a][1], frame.extents(pts[a][0].bbox), False, [pts[a][0]], None, None))
        for nm, pos, dims, mirror, blist, _, _ in placed:
            c = dict(name=nm, category=cat)
            if r.get("vendor_part"):
                c["part_number"] = str(r["vendor_part"])
            if cat == "structure_printed" and blist:
                m, cg_cad, src, m_geo = printed_mass(blist[0], r, mats, defaults)
                if m is None:
                    questions.append(f"'{r.get('name')}' ({r['id']}): {src}; give material, or slicer_g")
                    continue
                pos = frame.point(cg_cad)
                if mirror and pos[1] < 0:
                    pos = pos * [1, -1, 1]
                c.update(mass=dict(value=round(float(m), 4), source=src, **({} if "slicer" in src else {"uncertainty": 0.25})),
                         material=r.get("material") or defaults.get("material"), shape="distributed")
            else:
                mg = _num(r.get("mass_g"))
                if mg is None:
                    questions.append(f"'{r.get('name')}' ({r['id']}) has no mass_g: weigh it or give the datasheet mass")
                    continue
                c["mass"] = dict(value=round(mg / 1000.0, 5), source=str(r.get("mass_source") or "parts list (no source given: estimate)"))
                if r.get("material"):
                    c["material"] = r["material"]
            c["position"] = [round(float(v), 4) for v in pos]
            if dims is not None:
                c["dims"] = [round(float(v), 4) for v in dims]
            else:
                c["geom"] = False
            if mirror:
                c["mirror_y"] = True
            role = str(r.get("role") or "").lower()
            if role in ("lift_motor", "cruise_motor") and dims is not None:
                ax = [0, 0, 1] if role == "lift_motor" else [1, 0, 0]
                c.update(shape="cylinder", axis=ax, external=True)
                (lift_pts if role == "lift_motor" else cruise_pts).append((c["position"], mirror))
            for key, col, conv in (("bay", "bay", str), ("thermal_limit_C", "t_max_c", _num), ("power_W", "power_w", _num),
                                   ("origin", "origin", str)):
                if r.get(col) not in ("", None):
                    c[key] = conv(r[col])
            for key, col in (("external", "external"), ("movable", "movable"), ("origin_verified", "origin_verified")):
                v = _yes(r.get(col))
                if v is not None:
                    c[key] = v
            lo, hi = _num(r.get("move_min_mm")), _num(r.get("move_max_mm"))
            if lo is not None and hi is not None:
                c["move_range_x"] = [lo / 1000, hi / 1000]
            if blist:
                c["cad_body"] = "; ".join(b.path for b in blist)
                cad_map[nm] = [b.path for b in blist]
            if role:
                c["role"] = role
            c.update(r.get("extra") or {})
            if c.get("geom") is False:
                c.pop("dims", None) if cat == "structure_printed" else None
            comps.append(c)
            table.append(dict(id=r["id"], part=r.get("name"), component=nm, bodies=len(blist), matched_by=how.get(blist[0].path, "-") if blist else "no CAD body",
                              mass_g=round(P.val(c["mass"]) * 1000 * (2 if mirror else 1), 1), x_mm=round(c["position"][0] * 1000, 1),
                              mirrored=mirror, mass_source=c["mass"]["source"]))
    for b in unmatched:
        questions.append(f"CAD body '{b.path}' ({b.volume/1000:.1f} cm3) matches no parts-list row: add a row or tell me to ignore it")
    if frame_suggested:
        questions.append("the frame (wing-root leading edge position and aft/right/up axes) was only guessed from body names: confirm it "
                         f"against the top-view render: {frame.spec}")
    plan["components"] = comps
    # propulsion unit positions from the motor bodies (ArduPilot quad-X order: front-right, rear-left, front-left, rear-right)
    for u in plan["propulsion"]["units"]:
        if u["role"] == "lift" and lift_pts:
            pts4 = []
            for pos, mir in lift_pts:
                pts4.append(list(pos))
                if mir:
                    pts4.append([pos[0], -pos[1], pos[2]])
            if len(pts4) == u["count"] == 4:
                xm = np.mean([p[0] for p in pts4])
                f_ = lambda fwd, right: next(p for p in pts4 if (p[0] < xm) == fwd and (p[1] > 0) == right)
                zt = 0.02
                u["positions"] = [[round(p[0], 4), round(p[1], 4), round(p[2] + zt, 4)] for p in (f_(True, True), f_(False, False), f_(True, False), f_(False, True))]
            else:
                questions.append(f"lift unit expects {u['count']} motors, CAD has {len(pts4)} lift_motor bodies")
        if u["role"] == "cruise" and cruise_pts:
            p0 = cruise_pts[0][0]
            u["positions"] = [[round(p0[0] + 0.018, 4), round(p0[1], 4), round(p0[2], 4)]]     # prop plane ~18 mm behind the motor centre (assumed)
    if policy:
        plan.setdefault("meta", {})["sourcing"] = dict(policy=policy, note="asked at intake")
    if name:
        plan["meta"]["name"] = name
    plan["meta"]["intake"] = dict(frame=frame.spec, frame_confirmed=not frame_suggested, bodies=len(bodies), rows=len(rows))
    mp = P.mass_properties(dict(plan, components=[dict(c, mass=P.val(c["mass"])) for c in comps])) if comps else None
    rep = dict(frame=frame.spec, frame_suggested=frame_suggested, matched=table, questions=questions, cad_map=cad_map,
               unmatched_bodies=[b.path for b in unmatched],
               totals=None if mp is None else dict(mass_g=round(float(mp["mass"]) * 1000, 1), cg_mm=[round(float(v) * 1000, 1) for v in mp["cg"]]))
    return plan, rep


def measure(bodies, frame, wing_keys=("wing",), fus_keys=("fuselage", "fuse", "nose", "tail_cone", "tailcone")):
    """Planform numbers read off the CAD bodies, to compare with the base plan (never applied silently)."""
    out = {}
    wings = [b for b in bodies if any(k in b.key for k in wing_keys)]
    if wings:
        lo = np.min([frame.bbox(b.bbox)[0] for b in wings], 0)
        hi = np.max([frame.bbox(b.bbox)[1] for b in wings], 0)
        out["wing_span_m"] = round(float(hi[1] - lo[1]), 4)
        out["wing_x_le_min_m"] = round(float(lo[0]), 4)
        out["wing_chord_bbox_m"] = round(float(hi[0] - lo[0]), 4)
    fus = [b for b in bodies if any(k in b.key for k in fus_keys)]
    if fus:
        lo = np.min([frame.bbox(b.bbox)[0] for b in fus], 0)
        hi = np.max([frame.bbox(b.bbox)[1] for b in fus], 0)
        out.update(fuselage_length_m=round(float(hi[0] - lo[0]), 4), fuselage_x_nose_m=round(float(lo[0]), 4),
                   fuselage_width_m=round(float(hi[1] - lo[1]), 4), fuselage_height_m=round(float(hi[2] - lo[2]), 4))
    return out


def compare(meas, plan):
    rows = []
    w = P.main_wing(plan)
    fus = plan["airframe"]["fuselage"]
    for key, val, tol in (("wing_span_m", P.val(w["span"]), 0.02), ("fuselage_length_m", fus.get("length"), 0.03),
                          ("fuselage_x_nose_m", fus.get("x_nose"), 0.015), ("fuselage_width_m", fus.get("max_width"), 0.1),
                          ("fuselage_height_m", fus.get("max_height"), 0.1)):
        if key in meas and val is not None:
            d = meas[key] - val
            ok = abs(d) <= (tol if key.endswith("x_nose_m") else tol * abs(val))
            rows.append(dict(quantity=key, cad=meas[key], plan=val, differs=not ok))
    return rows


def write_report(path, rep, meas_rows, plan_path):
    L = ["# CAD + parts list intake", "", f"Plan written: `{plan_path}`", "",
         f"Frame: origin (wing-root leading edge) at CAD {rep['frame']['origin_mm']} mm; aft = CAD {rep['frame']['x_aft']}, "
         f"right = CAD {rep['frame']['y_right']}, up = CAD {rep['frame']['z_up']}" + (" - GUESSED, confirm" if rep["frame_suggested"] else ""), ""]
    if rep["totals"]:
        L += [f"Total mass {rep['totals']['mass_g']:.0f} g, CG at x/y/z = {rep['totals']['cg_mm']} mm (point-mass model; the mass stage refines it).", ""]
    L += ["## Questions to settle before trusting the run", ""] + ([f"{i+1}. {q}" for i, q in enumerate(rep["questions"])] or ["None."]) + [""]
    L += ["## Parts matched", "", "| id | part | plan component | CAD bodies | matched by | mass g | x mm | mass source |", "|---|---|---|---|---|---|---|---|"]
    L += [f"| {r['id']} | {r['part']} | {r['component']}{' (mirrored pair)' if r['mirrored'] else ''} | {r['bodies']} | {r['matched_by']} | "
          f"{r['mass_g']} | {r['x_mm']} | {r['mass_source']} |" for r in rep["matched"]]
    if meas_rows:
        L += ["", "## CAD geometry vs the base plan", "", "| quantity | CAD | plan | |", "|---|---|---|---|"]
        L += [f"| {r['quantity']} | {r['cad']} | {r['plan']} | {'DIFFERS: update the plan or the CAD' if r['differs'] else 'ok'} |" for r in meas_rows]
    Path(path).write_text("\n".join(L) + "\n")
