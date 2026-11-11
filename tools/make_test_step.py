#!/usr/bin/env python3
"""Synthetic 'Fusion export' of the Stork VTOL for testing the CAD intake: one STEP assembly with a named
solid per part (boxes and cylinders at the positions of examples/stork_phase0.buildplan.yaml, variant vtol,
with the electronics of the Flightory VTOL parts list), written in a deliberately DIFFERENT frame
(mm, CAD X = right, Y = up, Z = aft, origin near the nose) so the frame handling is exercised.

  python tools/make_test_step.py examples/stork_vtol_flightory/stork_vtol_assembly.step

This is a stand-in: the real Stork STEP pack is a paid download and is not in this repo."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vb import cadio  # noqa: E402

FRAME = cadio.Frame(origin_mm=(0, 5, 335), x_aft="+z", y_right="+x", z_up="+y")
# id_name, kind, centre [m] (design frame), dims [m] (design axes), axis for cylinders
B = [
    ("P001_Fuselage print", "box", [0.1525, 0, 0.0], [0.975, 0.095, 0.100]),
    ("P002_Wing print", "box", [0.085, 0.4288, 0.024], [0.19, 0.7625, 0.018]),
    ("P002_Wing print", "box", [0.085, -0.4288, 0.024], [0.19, 0.7625, 0.018]),
    ("P003_V-tail print", "box", [0.53, 0.12, 0.10], [0.12, 0.25, 0.01]),
    ("P003_V-tail print", "box", [0.53, -0.12, 0.10], [0.12, 0.25, 0.01]),
    ("P004_Hatch", "box", [-0.15, 0, 0.045], [0.20, 0.06, 0.004]),
    ("P010_Main spar", "cyl", [0.0575, 0, 0.028], [0.010, 1.0, 0.010], "y"),
    ("P011_Secondary spar", "cyl", [0.120, 0, 0.027], [0.008, 1.0, 0.008], "y"),
    ("P012_Wing spar 6x500", "cyl", [0.080, 0.56, 0.026], [0.006, 0.5, 0.006], "y"),
    ("P012_Wing spar 6x500", "cyl", [0.080, -0.56, 0.026], [0.006, 0.5, 0.006], "y"),
    ("P013_V-tail spars", "box", [0.52, 0, 0.08], [0.03, 0.30, 0.008]),
    ("P015_VTOL boom", "box", [0.05, 0.30, 0.010], [0.50, 0.010, 0.010]),
    ("P015_VTOL boom", "box", [0.05, -0.30, 0.010], [0.50, 0.010, 0.010]),
    ("P020_Lift motor pod", "cyl", [-0.20, 0.30, 0.026], [0.028, 0.028, 0.022], "z"),
    ("P020_Lift motor pod", "cyl", [-0.20, -0.30, 0.026], [0.028, 0.028, 0.022], "z"),
    ("P020_Lift motor pod", "cyl", [0.30, 0.30, 0.026], [0.028, 0.028, 0.022], "z"),
    ("P020_Lift motor pod", "cyl", [0.30, -0.30, 0.026], [0.028, 0.028, 0.022], "z"),
    ("P021_Pusher motor", "cyl", [0.650, 0, 0.014], [0.030, 0.035, 0.035], "x"),
    ("P023_Pusher ESC", "box", [0.360, 0, -0.012], [0.035, 0.018, 0.007]),
    ("P024_Lift ESC 4in1", "box", [-0.100, 0, -0.020], [0.045, 0.042, 0.008]),
    ("P030_Battery 4S2P", "box", [-0.210, 0, -0.004], [0.090, 0.046, 0.073]),
    ("P040_Flight controller", "box", [0.020, 0, -0.017], [0.052, 0.032, 0.019]),
    ("P041_GPS", "box", [0.230, 0, 0.040], [0.020, 0.020, 0.008]),
    ("P042_Receiver", "box", [0.110, 0, -0.018], [0.020, 0.013, 0.004]),
    ("P043_Video transmitter", "box", [0.140, 0, 0.010], [0.033, 0.033, 0.011]),
    ("P044_FPV camera", "box", [-0.310, 0, -0.008], [0.019, 0.019, 0.020]),
    ("P050_Aileron servo", "box", [0.058, 0.56, 0.027], [0.023, 0.025, 0.010]),
    ("P050_Aileron servo", "box", [0.058, -0.56, 0.027], [0.023, 0.025, 0.010]),
    ("P051_Ruddervator servo", "box", [0.420, 0.015, 0.008], [0.023, 0.012, 0.025]),
    ("P051_Ruddervator servo", "box", [0.420, -0.015, 0.008], [0.023, 0.012, 0.025]),
]


def to_cad(p):
    return FRAME.R.T @ np.asarray(p, float) * 1000.0 + FRAME.origin


def main(out):
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Pnt, gp_Ax2, gp_Dir
    bodies, count = [], {}
    for row in B:
        name, kind, c, d = row[:4]
        count[name] = count.get(name, 0) + 1
        label = f"{name} v3:{count[name]}"            # Fusion style: version + instance suffix
        c, d = np.array(c, float), np.array(d, float)
        if kind == "box":
            lo, hi = to_cad(c - d / 2), to_cad(c + d / 2)
            lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)
            s = BRepPrimAPI_MakeBox(gp_Pnt(*lo), *(hi - lo)).Shape()
        else:
            ax = "xyz".index(row[4])
            L = d[ax]
            r = max(v for i, v in enumerate(d) if i != ax) / 2
            e = np.zeros(3); e[ax] = 1.0
            p0 = to_cad(c - e * L / 2)
            dcad = FRAME.R.T @ e
            s = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*p0), gp_Dir(*dcad)), r * 1000, L * 1000).Shape()
        bodies.append((label, s))
    cadio.write_step(out, bodies)
    print(f"wrote {out}: {len(bodies)} bodies, frame {FRAME.spec}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "stork_vtol_assembly.step")
