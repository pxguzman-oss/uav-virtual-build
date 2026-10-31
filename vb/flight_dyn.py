"""Linear small-perturbation modes (Etkin-style dimensional derivatives, stability axes, level trim)."""
import math

import numpy as np

G = 9.80665


def modes(der, mass, Ixx, Iyy, Izz, Ixz, S, b, c, V, rho=1.225, CL=None, CD=None):
    """der: CLa, CD0/oswald or CD, Cma, Cmq, Cmadot, CYb, CYp, CYr, Clb, Clp, Clr, Cnb, Cnp, Cnr (per rad)."""
    q = 0.5 * rho * V * V
    m = mass
    CL = CL if CL is not None else m * G / (q * S)
    AR = b * b / S
    if CD is None:
        CD = der.get("CD0", 0.03) + CL * CL / (math.pi * der.get("oswald", 0.8) * AR)
    CDa = 2 * CL * der.get("CLa", 5) / (math.pi * der.get("oswald", 0.8) * AR)
    Xu = -2 * CD * q * S / (m * V)
    Xw = (CL - CDa) * q * S / (m * V)
    Zu = -2 * CL * q * S / (m * V)
    Zw = -(der["CLa"] + CD) * q * S / (m * V)
    Mw = der["Cma"] * q * S * c / (Iyy * V)
    Mq = der["Cmq"] * q * S * c * c / (2 * V * Iyy)
    Mwd = (der.get("Cmadot") or 0.0) * q * S * c * c / (2 * V * V * Iyy)
    A_lon = np.array([[Xu, Xw, 0, -G],
                      [Zu, Zw, V, 0],
                      [Mwd * Zu, Mw + Mwd * Zw, Mq + Mwd * V, 0],
                      [0, 0, 1, 0]])
    Yv = der["CYb"] * q * S / (m * V)
    Yp = der.get("CYp", 0) * q * S * b / (2 * m * V)
    Yr = der.get("CYr", 0) * q * S * b / (2 * m * V)
    L = np.array([der["Clb"] * q * S * b / V, der["Clp"] * q * S * b * b / (2 * V), der["Clr"] * q * S * b * b / (2 * V)])
    N = np.array([der["Cnb"] * q * S * b / V, der.get("Cnp", 0) * q * S * b * b / (2 * V), der["Cnr"] * q * S * b * b / (2 * V)])
    den = 1 - Ixz * Ixz / (Ixx * Izz)
    Lp_ = (L / Ixx + Ixz / Ixx * N / Izz) / den
    Np_ = (N / Izz + Ixz / Izz * L / Ixx) / den
    A_lat = np.array([[Yv, Yp, Yr - V, G],
                      [Lp_[0], Lp_[1], Lp_[2], 0],
                      [Np_[0], Np_[1], Np_[2], 0],
                      [0, 1, 0, 0]])
    out = {}
    ev = np.linalg.eigvals(A_lon)
    cplx = sorted([e for e in ev if e.imag > 1e-6], key=lambda e: abs(e))
    if len(cplx) == 2:
        out["phugoid"] = _osc(cplx[0])
        out["short_period"] = _osc(cplx[1])
    else:
        out["longitudinal_eigs"] = [complex(e) for e in ev]
        real = sorted([e.real for e in ev], key=abs)
        if cplx:
            out["phugoid"] = _osc(cplx[0])
        out["short_period"] = dict(real_roots=real[-2:], zeta=1.0, wn=float(abs(real[-1])))
    ev = np.linalg.eigvals(A_lat)
    cplx = [e for e in ev if e.imag > 1e-6]
    real = sorted([e.real for e in ev if abs(e.imag) <= 1e-6], key=abs)
    if cplx:
        out["dutch_roll"] = _osc(cplx[0])
    if len(real) >= 2:
        out["spiral"] = _real(real[0])
        out["roll"] = _real(real[-1])
    out["A_lon"] = A_lon.tolist()
    out["A_lat"] = A_lat.tolist()
    out["trim"] = dict(V=V, CL=CL, CD=CD)
    return out


def _osc(e):
    wn = abs(e)
    zeta = -e.real / wn
    return dict(wn=float(wn), zeta=float(zeta), period_s=float(2 * math.pi / e.imag), eig=[float(e.real), float(e.imag)])


def _real(s):
    d = dict(eig=float(s), tau_s=float(-1 / s) if s < 0 else None, stable=bool(s < 0))
    if s > 0:
        d["time_to_double_s"] = float(math.log(2) / s)
    else:
        d["time_to_half_s"] = float(math.log(2) / -s) if s < 0 else None
    return d
