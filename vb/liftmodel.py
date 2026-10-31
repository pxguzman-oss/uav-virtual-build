"""Lift model shared by the aero, flight-model and structures stages (review finding C1).

ONE CLmax / Vs source for every stage:

* linear lift line  : AVL (vortex lattice + fuselage BODY) on the SAME full geometry as the flight model.
                      AeroBuildup's own lift line sits ~0.1 CL higher (no wing->tail downwash, NeuralFoil
                      CL0); its table keeps only the NON-linear (stall) shape: anchor_table().
* 3-D CLmax         : min( AeroBuildup stall peak after anchoring,
                           Raymer cap 0.9 x Clmax_2D(NeuralFoil, Re at Vs on the MAC) x cos(sweep_c/4),
                           plan wing cl_max when the plan gives one )
                      Raymer, "Aircraft Design: A Conceptual Approach", 6th ed., eq. 12.15 (high-AR wings).
* trimmed CLmax     : untrimmed + tail download needed to trim at CLmax (AVL ruddervator derivatives).
* uncertainty band  : pessimistic = nominal x (1 - u); u = plan cl_max uncertainty (relative) or 0.10.
                      Safety gates (hand-launch margin, AIRSPEED_MIN, turn stall) use the PESSIMISTIC value.
"""
from __future__ import annotations

import math

import numpy as np

DEG = math.pi / 180
RAYMER_3D_FACTOR = 0.9
DEFAULT_CLMAX_UNCERTAINTY = 0.10


def lin_fit(alpha_deg, CL, lo=-2.0, hi=6.0):
    """(CLa per rad, CL0) of a table over [lo, hi] deg."""
    a = np.asarray(alpha_deg, float)
    c = np.asarray(CL, float)
    m = (a >= lo) & (a <= hi)
    p = np.polyfit(a[m] * DEG, c[m], 1)
    return float(p[0]), float(p[1])


def stall_index(alpha_deg, CL, lo=-5.0, hi=20.0):
    a = np.asarray(alpha_deg, float)
    c = np.asarray(CL, float)
    return int(np.argmax(np.where((a >= lo) & (a <= hi), c, -9)))


def anchor_table(alpha_deg, CL_ab, CD_ab, CLa_anchor, CL0_anchor, fit=(-2.0, 6.0)):
    """Replace the LINEAR part of an AeroBuildup CL(alpha) table by the anchor line; keep its stall shape.

    CL(a) = CL_AB(a) + [CL0_anchor - CL0_AB] + [CLa_anchor - CLa_AB] * clip(a, -10 deg, a_CLmax_AB)
    CD is re-mapped so the drag POLAR CD(CL) stays AeroBuildup's (profile + induced drag depend on lift,
    not on alpha): CD(a) = CD_AB at the AeroBuildup alpha that gives the same CL (pre-stall branch);
    post-stall and below -8 deg the AeroBuildup CD(a) is offset for continuity."""
    a = np.asarray(alpha_deg, float)
    cl = np.asarray(CL_ab, float)
    cd = np.asarray(CD_ab, float)
    CLa_ab, CL0_ab = lin_fit(a, cl, *fit)
    i_s = stall_index(a, cl)
    a_s = a[i_s]
    d = (CL0_anchor - CL0_ab) + (CLa_anchor - CLa_ab) * np.clip(a, -10.0, a_s) * DEG
    cl1 = cl + d
    br = (a >= -8.0) & (a <= a_s)
    order = np.argsort(cl[br])
    cd1 = cd.copy()
    cd1[br] = np.interp(cl1[br], cl[br][order], cd[br][order])
    j_hi = np.where(br)[0].max()
    j_lo = np.where(br)[0].min()
    post = a > a_s
    cd1[post] = cd[post] + (cd1[j_hi] - cd[j_hi])
    pre = a < -8.0
    cd1[pre] = cd[pre] + (cd1[j_lo] - cd[j_lo])
    return cl1, cd1, dict(CLa_AB=CLa_ab, CL0_AB=CL0_ab, CLa_anchor=CLa_anchor, CL0_anchor=CL0_anchor, alpha_CLmax_deg=float(a_s))


def cap_table(alpha_deg, CL, clmax_target, knee_frac=0.75):
    """Scale the positive-alpha part of CL(alpha) above a knee (knee_frac x target) so the peak equals
    clmax_target. Continuous, monotonic in CL, keeps the stall alpha and the post-stall shape."""
    a = np.asarray(alpha_deg, float)
    cl = np.array(CL, float)
    i = stall_index(a, cl)
    cmax = cl[i]
    k = knee_frac * clmax_target
    if cmax <= k + 1e-6:
        return cl
    s = (clmax_target - k) / (cmax - k)
    m = (a > 0) & (cl > k)
    cl[m] = k + (cl[m] - k) * s
    return cl


def monotone_post_stall_cm(alpha_deg, Cm, alpha_stall_deg, a_end=30.0):
    """No pitch-UP near/after the stall: Cm(alpha) is made non-increasing between alpha_stall - 4 deg and a_end.
    AeroBuildup has no change of wing->tail downwash at the stall and NeuralFoil is not trustworthy past
    Clmax, so its post-stall Cm bump is treated as an artefact (review M3). The real post-stall pitch
    behaviour is UNKNOWN until the F1 stall test."""
    a = np.asarray(alpha_deg, float)
    cm = np.array(Cm, float)
    prev, n_changed, max_bump = None, 0, 0.0
    a_from = alpha_stall_deg - 4.0          # the AeroBuildup bump starts ~2 deg before the CL peak
    for i in np.argsort(a):
        if a[i] <= a_from:
            prev = cm[i]
            continue
        if a[i] > a_end:
            break
        if prev is not None and cm[i] > prev:
            max_bump = max(max_bump, cm[i] - prev)
            cm[i] = prev
            n_changed += 1
        prev = cm[i]
    return cm, dict(points_changed=n_changed, max_raw_rise=float(max_bump))


def vs(mass_kg, S, CLmax, rho=1.225, g=9.80665):
    return math.sqrt(2 * mass_kg * g / (rho * S * CLmax))


def section_clmax(airfoil, Re, model_size="large"):
    """2-D Clmax (NeuralFoil) and the viscous 2-D lift slope (0..6 deg)."""
    import aerosandbox as asb
    af = asb.Airfoil(airfoil)
    al = np.arange(-2, 20.01, 0.25)
    r = af.get_aero_from_neuralfoil(alpha=al, Re=Re, model_size=model_size)
    cl = np.asarray(r["CL"], float)
    i = int(np.argmax(cl))
    m = (al >= 0) & (al <= 6)
    slope = float(np.polyfit(al[m] * DEG, cl[m], 1)[0])
    return dict(Clmax_2d=float(cl[i]), alpha_Clmax_2d_deg=float(al[i]), Cla_2d_per_rad=slope,
                confidence=float(np.asarray(r.get("analysis_confidence", [np.nan] * len(al)))[i]), Re=float(Re))


def uncertainty_of(clmax_plan_node):
    """Relative CLmax uncertainty from a plan sourced_number (relative if < 1, else absolute -> relative)."""
    if isinstance(clmax_plan_node, dict) and clmax_plan_node.get("uncertainty") is not None:
        u = float(clmax_plan_node["uncertainty"])
        v = float(clmax_plan_node.get("value", 1.0))
        return u if u < 0.5 else u / v
    return DEFAULT_CLMAX_UNCERTAINTY
