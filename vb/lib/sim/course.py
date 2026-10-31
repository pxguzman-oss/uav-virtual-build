#!/usr/bin/env python3
"""Canal-like S-curve course: straights + arcs with radii 300/150/80/50 m, both directions.
Returns a dense centreline (local N,E metres from home) and waypoints at a chosen spacing."""
import math
import numpy as np

# (type, length_or_radius, turn_deg)  turn>0 = right turn
DEFAULT_SEGMENTS = [
    ("S", 250, 0), ("A", 300, 60), ("S", 150, 0), ("A", 150, -90), ("S", 150, 0),
    ("A", 80, 90), ("S", 120, 0), ("A", 50, -120), ("S", 150, 0), ("A", 80, 60),
    ("S", 200, 0),
]


def centreline(start_ne=(150.0, 0.0), heading_deg=0.0, segments=DEFAULT_SEGMENTS, ds=1.0):
    n, e = start_ne
    h = math.radians(heading_deg)
    pts = [(n, e, 0.0, 0.0)]  # n, e, curvature radius (0=straight), signed
    for typ, a, b in segments:
        if typ == "S":
            k = int(a / ds)
            for _ in range(k):
                n += ds * math.cos(h); e += ds * math.sin(h)
                pts.append((n, e, 0.0, 0))
        else:
            R, turn = a, math.radians(b)
            L = R * abs(turn)
            k = int(L / ds)
            dpsi = turn / k
            for _ in range(k):
                h += dpsi / 2
                step = 2 * R * math.sin(abs(dpsi) / 2)
                n += step * math.cos(h); e += step * math.sin(h)
                h += dpsi / 2
                pts.append((n, e, R, math.copysign(1, turn)))
    return np.array(pts)


def waypoints(cl, spacing=25.0):
    """resample the centreline every `spacing` metres of arc length (always keep the end)"""
    d = np.r_[0, np.cumsum(np.hypot(np.diff(cl[:, 0]), np.diff(cl[:, 1])))]
    s = np.arange(0, d[-1], spacing)
    s = np.r_[s, d[-1]]
    return np.c_[np.interp(s, d, cl[:, 0]), np.interp(s, d, cl[:, 1])], d[-1]


def cross_track(cl, n, e):
    """signed-free cross-track distance of points (n,e) to the dense centreline + local bend radius"""
    from scipy.spatial import cKDTree
    tree = cKDTree(cl[:, :2])
    dist, idx = tree.query(np.c_[n, e])
    return dist, cl[idx, 2], idx


def compensate(wps, cl, l1_dist):
    """Shift waypoints that lie on arcs INWARD by L1^2/(2R). Measured in SITL (and consistent with
    L1 theory): ArduPlane L1 tracking a dense-waypoint arc settles OUTSIDE the bend by ~L1^2/(2R)
    (L1 = damping*period*V/pi). Pre-distorting the path inward cancels the steady-state offset.
    (A first attempt shifting outward doubled the error: results/mission_comp15.)"""
    from scipy.spatial import cKDTree
    tree = cKDTree(cl[:, :2])
    out = []
    for n, e in wps:
        _, i = tree.query((n, e))
        R, sgn = cl[i, 2], cl[i, 3]
        if R <= 0:
            out.append((n, e)); continue
        j0, j1 = max(i - 2, 0), min(i + 2, len(cl) - 1)
        h = math.atan2(cl[j1, 1] - cl[j0, 1], cl[j1, 0] - cl[j0, 0])
        off = min(l1_dist ** 2 / (2 * R), 0.5 * R)
        # right turn (sgn>0): centre to the right -> outward = left = (sin h, -cos h)
        ln, le = math.sin(h), -math.cos(h)
        out.append((n - sgn * off * ln, e - sgn * off * le))
    return np.array(out)
