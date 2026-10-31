"""Adapter: master plan (+ mass-stage CG) -> geometry dict `g` of the aero library (vb/lib/aero/stork_aero.py).

Supported configuration: one main wing + V-tail (+ fuselage stations, optional VTOL booms/lift rotors).
Other tail layouts raise NotImplementedError (see README known limitations)."""
from __future__ import annotations

import copy

import numpy as np

from .. import plan as P


def build(plan, cg_m, mass_kg):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib" / "aero"))
    import stork_aero as sa

    af = plan["airframe"]
    w = P.main_wing(plan)
    vts = [s for s in af["lifting_surfaces"] if s.get("kind") in ("v_tail", "vtail")]
    if not vts:
        raise NotImplementedError("aero_lowfi supports wing + V-tail only (add htail/fin support in vb/lib/aero)")
    vt = vts[0]
    mg = P.mac_geometry(w)
    an = plan.get("analysis", {}).get("aero", {}) or {}
    g = {
        "name": P.slug(plan["meta"]["name"]) + "_" + plan["meta"].get("variant", "base"),
        "source_yaml": "build plan",
        "vtol": bool(af.get("booms")),
        "wing": dict(airfoil=w["airfoil"].lower(), span=w["span"], cr=w["root_chord"], ct=w["tip_chord"],
                     sweep_c4_deg=w.get("sweep_c4_deg", 0.0) or 0.0, dihedral_deg=w.get("dihedral_deg", 0.0),
                     twist_tip_deg=w.get("twist_tip_deg", 0.0), z_root=w.get("z_le_root", 0.0)),
        "fuse": dict(stations=[list(s) for s in af["fuselage"]["stations"]], exponent=af["fuselage"].get("exponent", 2.0)),
        "vtail": dict(airfoil=vt["airfoil"].lower(), x_le=vt.get("x_le_root", 0.0), z=vt.get("z_le_root", 0.0),
                      cr=vt["root_chord"], ct=vt["tip_chord"], panel_span=vt["span"], dihedral_deg=vt.get("dihedral_deg", 35),
                      sweep_le_deg=vt.get("sweep_le_deg", 0.0)),
        "ref": dict(S=mg["area"], b=w["span"], c=mg["mac"], x_cg=float(cg_m[0])),
        "zref": float(cg_m[2]),
        "controls": {},
        "parasite": copy.deepcopy(sa.DEFAULTS["parasite"]),
        "assumed": ["moment reference = CG from the mass stage", "control hinge lines default to 1 - chord_fraction"],
    }
    for cs in w.get("control_surfaces", []):
        k = {"aileron": "aileron", "flap": "flap", "flaperon": "aileron", "elevon": "aileron"}.get(cs["function"])
        if k:
            g["controls"][k] = dict(y0=cs["y_start"], y1=cs["y_end"], hinge_xc=cs.get("hinge_x_c", 1 - cs["chord_fraction"]),
                                    max_deg=cs["max_defl_deg"])
    for cs in vt.get("control_surfaces", []):
        g["controls"]["ruddervator"] = dict(s0=cs["y_start"] / vt["span"], s1=min(cs["y_end"] / vt["span"], 1.0),
                                            hinge_xc=cs.get("hinge_x_c", 1 - cs["chord_fraction"]), max_deg=cs["max_defl_deg"])
    if "ruddervator" not in g["controls"]:
        raise ValueError("V-tail has no control surface")
    if "excrescence_factor" in an:
        g["parasite"]["excrescence_factor"] = an["excrescence_factor"]
    if an.get("prop_parking"):
        g["parasite"]["vtol_prop_parking"] = an["prop_parking"]
    if af.get("booms"):
        b = af["booms"][0]
        r0, r1 = np.asarray(b["root"]), np.asarray(b["tip"])
        lift = P.units(plan, "lift")
        pods = [c for c in plan["components"] if c["name"] in b.get("tip_mass_components", [])]
        dims = pods[0].get("dims", [0.028, 0.028, 0.022]) if pods else [0.028, 0.028, 0.022]
        g["vtol_items"] = dict(boom_side=b["tube"]["od"], boom_len=float(np.linalg.norm(r1 - r0)) + 0.03,
                               boom_y=abs(float(r0[1])), boom_z=float(r0[2]), boom_xc=float((r0[0] + r1[0]) / 2),
                               motor_d=max(dims[0], dims[1]), motor_len=dims[2],
                               prop_d=lift[0]["prop"]["diameter"] if lift else 0.18, n_rotors=lift[0]["count"] if lift else 4)
    return g, sa
