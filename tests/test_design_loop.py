#!/usr/bin/env python3
"""Desk-tier regression: CAD + parts list -> plan -> design loop, checked against known Stork numbers.
No CFD, no SITL; about 4 minutes on 2 cores.

  .venv/bin/python tests/test_design_loop.py
"""
import json
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EX = ROOT / "examples" / "stork_vtol_flightory"
PY = sys.executable
FRAME = "origin=0,5,335; x_aft=+z; y_right=+x; z_up=+y"
ok = 0


def check(name, cond, detail=""):
    global ok
    if not cond:
        print(f"FAIL {name} {detail}")
        sys.exit(1)
    ok += 1
    print(f"ok   {name} {detail}")


def sh(*args):
    r = subprocess.run([PY, *map(str, args)], cwd=ROOT, capture_output=True, text=True)
    return r.returncode, r.stdout + r.stderr


with tempfile.TemporaryDirectory() as td:
    td = Path(td)
    # ---- 1. intake of the nose-payload parts list --------------------------------------------------
    rc, log = sh("tools/intake.py", "--cad", EX / "stork_vtol_assembly.step", "--parts", EX / "parts_list_with_payload.csv", "--base",
                 ROOT / "examples" / "stork_phase0.buildplan.yaml", "--variant", "vtol", "--frame", FRAME, "--policy", "any", "--merge",
                 EX / "datasheets.yaml", "--name", "Loop test", "--out", td / "work")
    check("intake exits 0 with no open questions", rc == 0, log[-300:] if rc else "")
    rep = json.loads((td / "work" / "intake.json").read_text())
    check("intake total mass 2294 g", abs(rep["totals"]["mass_g"] - 2293.9) < 1.0, f"{rep['totals']['mass_g']} g")
    rc, log = sh("tools/intake.py", "--cad", EX / "stork_vtol_assembly.step", "--parts", EX / "parts_list.csv", "--base",
                 ROOT / "examples" / "stork_phase0.buildplan.yaml", "--variant", "vtol", "--merge", EX / "datasheets.yaml", "--name", "Q test",
                 "--out", td / "q")
    check("intake without --frame and --policy exits 2 and asks about both", rc == 2 and "sourcing policy" in log and "guessed" in log)

    # ---- 2. design loop --------------------------------------------------------------------------------
    plan = td / "work" / "loop_test.buildplan.yaml"
    rc, log = sh("tools/design_loop.py", plan, "--out", td / "out", "--max-options", "3")
    check("design loop finds a candidate that clears the layout phase (exit 0)", rc == 0, log[-600:] if rc else "")
    cmp_ = json.loads((td / "out" / "loop_test" / "design_loop" / "comparison.json").read_text())
    c = {x["id"]: x for x in cmp_["candidates"]}
    b = c["0"]["m"]
    check("as drawn: 34.5 mm nose heavy and outside the window", abs(b["cg_err_mm"] + 34.5) < 1.5 and not b["cg_in_window"], f"{b['cg_err_mm']} mm")
    check("as drawn: the layout phase fails", b["phases"]["layout"]["fail"] >= 1)
    proven = [x for i, x in c.items() if i != "0"]
    check("three options were proven", len(proven) == 3, str([x["title"][:30] for x in proven]))
    for x in proven:
        check(f"option {x['id']} proven: CG inside the window, layout phase clear, nothing crashed",
              x["m"]["cg_in_window"] and x["m"]["phases"]["layout"]["fail"] == 0 and not any(p["crashed"] for p in x["m"]["phases"].values()),
              f"CG {x['m']['cg_err_mm']:+.1f} mm")
        check(f"option {x['id']}: proven CG within 1 mm of the balance stage's estimate", abs(x["m"]["cg_err_mm"] - x["estimate"]["cg_err_mm"]) < 1.0)
        check(f"option {x['id']} fixes the CG and the lift-motor load findings", any("CG x vs design" in f for f in x["fixed"])
              and any("most-loaded lift motor" in f for f in x["fixed"]))
    check("the recommendation is a proven option, not the layout as drawn", cmp_["recommended"] not in (None, "base"), str(cmp_["recommended"]))
    ball = [x for x in proven if x["kind"] == "ballast"]
    if ball:
        check("ballast alone is not the recommendation when another option clears the layout", cmp_["recommended"] != ball[0]["variant"])
    dec = (td / "out" / "loop_test" / "design_loop" / "DECISION.md").read_text()
    check("decision brief has the comparison, a recommendation and the still-open list", all(k in dec for k in (
        "## Candidates, each proven", "## Recommendation", "## Still open whichever option is chosen", "## Next")))

    # ---- 3. report of a proven candidate ---------------------------------------------------------------
    v = cmp_["recommended"]
    rd = json.loads((td / "out" / "loop_test" / v / "report" / "result.json").read_text())["data"]
    check("report carries the three phases", [p["phase"] for p in rd["phases"]] == ["layout", "sizing", "flight"])
    check("report: layout phase not FAIL; CFD and SITL shown as not run in a desk check", rd["phases"][0]["status"] != "FAIL"
          and {"cfd", "sitl"} <= set(rd["phases"][2]["missing"]), str([(p["phase"], p["status"], p["missing"]) for p in rd["phases"]]))
    kinds = {a["kind"] for a in rd["actions"]}
    check("report: findings are grouped by who acts", {"cad", "material", "part"} <= kinds, str(sorted(kinds)))
    check("report: a desk check is never GO", "GO" not in rd["overall"].replace("NO-GO", "") or "INCOMPLETE" in rd["overall"], rd["overall"])
    check("ArduPilot parameter file withheld (design not approved)", not (td / "out" / "loop_test" / v / "report" / "derived.param").exists())
    rc, log = sh("tools/design_loop.py", plan, "--out", td / "out", "--options", "none", "--fly", "base", "--fidelity", "quick")
    check("flight proof is refused for a design that fails the layout phase", "NOT flying" in log, log[-200:])
print(f"\n{ok} checks passed")
