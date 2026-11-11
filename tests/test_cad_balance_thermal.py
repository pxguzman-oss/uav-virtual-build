#!/usr/bin/env python3
"""Hand-checkable tests of the CAD intake, auto-balance and thermal code (no CFD / SITL needed).

  .venv/bin/python tests/test_cad_balance_thermal.py
"""
import copy
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "vb" / "lib"))
from vb import plan as P  # noqa: E402
from vb.lib import balance as B  # noqa: E402
import thermal as TH  # noqa: E402

PLAN = ROOT / "examples" / "stork_phase0.buildplan.yaml"
EX = ROOT / "examples" / "stork_vtol_flightory"
ok = 0


def check(name, cond, detail=""):
    global ok
    if not cond:
        print(f"FAIL {name} {detail}")
        sys.exit(1)
    ok += 1
    print(f"ok   {name} {detail}")


# ---------------------------------------------------------------- thermal network vs closed forms
net = TH.Network()
net.node("a", 50.0)
ph = dict(name="p", duration=2000.0, T_amb=20.0, links=[], amb=[("a", 0.5)], Q={"a": 10.0})
t, T, _ = net.simulate([ph], [20.0], dt=1.0)
check("thermal steady state = T_amb + Q/G", abs(T[-1, 0] - 40.0) < 0.01, f"{T[-1,0]:.3f} C vs 40")
tau = 50.0 / 0.5
k = int(np.argmin(abs(t - tau)))
check("thermal transient = 63.2 % of the rise after one time constant", abs((T[k, 0] - 20) / 20 - (1 - math.exp(-1))) < 0.01,
      f"{(T[k,0]-20)/20:.3f} vs 0.632")
net2 = TH.Network(); net2.node("part", 10.0); net2.node("air", 5.0)
ph2 = dict(name="p", duration=1.0, T_amb=30.0, links=[("part", "air", 0.2)], amb=[("air", 1.0)], Q={"part": 4.0})
Ts = net2.steady(ph2)
check("two nodes in series: part = T_amb + Q (1/G1 + 1/G2)", abs(Ts[0] - (30 + 4 * (1 / 0.2 + 1 / 1.0))) < 1e-6, f"{Ts[0]:.2f} C")
check("flat-plate h at 13 m/s over 0.975 m is 30-60 W/m2K", 30 < TH.h_forced_plate(13.0, 0.975) < 60, f"{TH.h_forced_plate(13.0, 0.975):.1f}")

# ---------------------------------------------------------------- balance on the Stork VTOL
raw = P.load_raw(PLAN)
plan = P.resolve(raw, "vtol")
mp = P.mass_properties(plan)
mass = dict(mass_kg=float(mp["mass"]), cg_m=mp["cg"].tolist(), plan_model=dict(cg_m=mp["cg"].tolist()),
            mac_m=P.mac_geometry(P.main_wing(plan))["mac"], x_le_mac_m=P.mac_geometry(P.main_wing(plan))["x_le_mac"])
T = B.targets(plan, mass, dict(x_np=0.087))
check("target is the design CG of the plan", abs(T["target"] - 0.047) < 1e-9 and abs(T["tol"] - 0.03 * mass["mac_m"]) < 1e-9)

raw_h = copy.deepcopy(raw)                                     # 200 g in the nose -> nose heavy
raw_h["variants"]["vtol"]["components_add"].append(dict(name="nose_payload", category="payload", mass=0.200, position=[-0.30, 0, 0],
                                                         geom=False, movable=False))
heavy = P.resolve(raw_h, "vtol")
mp2 = P.mass_properties(heavy)
mass2 = dict(mass, mass_kg=float(mp2["mass"]), cg_m=mp2["cg"].tolist(), plan_model=dict(cg_m=mp2["cg"].tolist()))
res = B.generate(heavy, mass2, dict(x_np=0.087), T, {}, None, "any", hover_tw=1.6)
by = {o["kind"]: o for o in res["options"]}
M, cg = mass2["mass_kg"], mass2["cg_m"][0]
check("the 200 g nose payload makes it nose heavy", cg < T["lo"], f"CG {cg*1000:.1f} mm, window from {T['lo']*1000:.1f}")
bal = by["ballast"]
mb = P.val(bal["edits"][0]["component"]["mass"])
xb = bal["edits"][0]["component"]["position"][0]
check("ballast mass = M (t - cg) / (x_b - t)", abs(mb - M * (T["target"] - cg) / (xb - T["target"])) < 2e-4, f"{mb*1000:.1f} g at x {xb*1000:.0f} mm")
check("ballast puts the CG on target", abs(bal["metrics"]["cg_err_mm"]) < 0.2, f"{bal['metrics']['cg_err_mm']:+.2f} mm")
w = by["wing"]
check("wing shift puts the CG on target", abs(w["metrics"]["cg_err_mm"]) < 0.2, f"{w['title']}")
for o in res["options"]:
    if o["kind"] == "none":
        continue
    ov = B.overlay(raw_h, "vtol", o["edits"], "t")
    r2 = copy.deepcopy(raw_h); r2["variants"]["t"] = ov
    p_ov, p_ap = P.resolve(r2, "t"), B.apply(heavy, [e for e in o["edits"]])
    if o is by.get("ballast") or "ballast" in o["kind"]:
        continue
    a = {c["name"]: c["position"] for c in p_ov["components"]}
    b = {c["name"]: c["position"] for c in p_ap["components"]}
    check(f"overlay of option '{o['kind']}' reproduces the edited plan", a == b and
          [u["positions"] for u in p_ov["propulsion"]["units"]] == [u["positions"] for u in p_ap["propulsion"]["units"]]
          and not P.validate(p_ov))
mov = B.movables(plan, {})
check("battery and ESCs are movable, camera / GPS / motors are not", "battery" in mov and "camera" not in mov and "gps" not in mov
      and "motor_cruise" not in mov, str(sorted(mov)))
mv, got = B.solve_moves(plan, mov, 0.6 * 0.02, only={"battery"})
check("battery slide = needed moment / battery mass", abs(mv["battery"] - 0.02) < 2e-4, f"{mv['battery']*1000:.1f} mm for 12 g m")
mv, got = B.solve_moves(plan, mov, 5.0)
bx = B.boxes(B.apply(plan, [dict(kind="move", name=k, dx=v) for k, v in mv.items()]))
check("parts pushed to their limits do not pass through each other", not B.overlaps(B.apply(plan, [dict(kind="move", name=k, dx=v) for k, v in mv.items()])))
check("sourcing: any allows everything", B.origin_ok("China", "any")[0] and B.origin_ok(None, "any")[0])
check("sourcing: us_made allows USA only, unknown fails", B.origin_ok("USA", "us_made")[0] and not B.origin_ok("Taiwan", "us_made")[0]
      and not B.origin_ok("", "us_made")[0])
check("sourcing: ndaa screens covered countries", not B.origin_ok("China", "ndaa")[0] and B.origin_ok("Taiwan", "ndaa")[0])

# ---------------------------------------------------------------- CAD intake (needs cadquery-ocp)
try:
    from vb import cadio, intake
    import OCP  # noqa: F401
except Exception as e:  # noqa: BLE001
    print(f"skip CAD tests ({e})")
else:
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt
    with tempfile.TemporaryDirectory() as td:
        f = Path(td) / "t.step"
        cadio.write_step(f, [("P001_box v2:1", BRepPrimAPI_MakeBox(gp_Pnt(10, 20, 30), 100, 50, 20).Shape())])
        b = cadio.read_step(f)[0]
        check("STEP round trip keeps name, volume and centroid", b.key == "p001_box" and abs(b.volume - 100 * 50 * 20) < 1 and
              np.allclose(b.centroid, [60, 45, 40], atol=1e-3), f"{b.path}: {b.volume:.0f} mm3 at {b.centroid.round(2)}")
        s2 = cadio.translated(b.shape, [5, 0, -3])
        cadio.write_step(f, [("moved", s2)])
        check("translated body moves by the vector", np.allclose(cadio.read_step(f)[0].centroid, [65, 45, 37], atol=1e-3))
    fr = cadio.Frame((0, 5, 335), "+z", "+x", "+y")
    check("frame: CAD (300, 31, 135) mm -> design (-0.200, 0.300, 0.026) m", np.allclose(fr.point([300, 31, 135]), [-0.2, 0.3, 0.026]))
    check("frame: design vector back to CAD", np.allclose(fr.vec_to_cad_mm([0.01, 0, 0]), [0, 0, 10]))
    step = EX / "stork_vtol_assembly.step"
    if step.exists():
        bodies = cadio.read_step(step)
        rows, problems = intake.read_parts(EX / "parts_list.csv")
        base = P._merge({k: v for k, v in raw.items() if k != "variants"}, raw["variants"]["vtol"])
        pl, rep = intake.build(bodies, rows, base, fr, False, defaults=dict(material="lw_pla_230C"), policy="any", name="t")
        want = sum((float(r["slicer_g"] or 0) if r["category"] == "structure_printed" else float(r["mass_g"])) * r["qty"] for r in rows)
        check("intake: every body matched, no questions", not rep["unmatched_bodies"] and not rep["questions"] and not problems, str(rep["questions"][:2]))
        check("intake: total mass = sum of the parts list", abs(rep["totals"]["mass_g"] - want) < 0.5, f"{rep['totals']['mass_g']} g vs {want:.1f} g")
        names = {c["name"]: c for c in pl["components"]}
        check("intake: four lift motors become two mirrored pairs, front and rear", names["lift_pod_FR"]["mirror_y"] and
              abs(names["lift_pod_FR"]["position"][0] + 0.2) < 1e-3 and abs(names["lift_pod_RR"]["position"][0] - 0.3) < 1e-3)
        lift = next(u for u in pl["propulsion"]["units"] if u["role"] == "lift")
        check("intake: lift unit in ArduPilot quad-X order (FR, RL, FL, RR)", [(p[0] < 0.05, p[1] > 0) for p in lift["positions"]] ==
              [(True, True), (False, False), (True, False), (False, True)])
        check("intake: generated plan validates", not P.validate(pl), str(P.validate(pl)[:2]))
        sugg, notes = cadio.suggest_frame(bodies)
        check("frame guess finds aft / right / up axes", (sugg.spec["x_aft"], sugg.spec["y_right"], sugg.spec["z_up"]) == ("+z", "+x", "+y"))
print(f"\n{ok} checks passed")
