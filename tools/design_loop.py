#!/usr/bin/env python3
"""The design loop in one command: desk-check the design, prove every promising balance option with the
real pipeline, compare them side by side, and (optionally) flight-prove the chosen one.

  python tools/design_loop.py PLAN.yaml [--variant V]                  # desk check + prove the top options
  python tools/design_loop.py PLAN.yaml --options B,C                  # prove these options only
  python tools/design_loop.py PLAN.yaml --fly best [--fidelity standard]   # then CFD + SITL on the recommended one
  python tools/design_loop.py PLAN.yaml --fly balB                     # ... or on a named variant

What it does
  1. Desk check of the design as drawn (layout + sizing phases, about a minute, no CFD or SITL).
  2. If the layout does not balance (or --options is given): adds the best balance options to the plan as
     variants and desk-checks each one, so the comparison shows PROVEN numbers, not the balance stage's estimates.
  3. Writes out/<plan>/design_loop/DECISION.md and comparison.json: one row per candidate with CG, static margin,
     hover margin, range, hot parts, clashes, FAIL counts per phase, what it fixes and what it newly breaks.
  4. Recommends the candidate with the fewest layout FAILs, then the fewest sizing FAILs and new problems, then
     the lowest cost score (effort, range and hover margin lost; ballast is penalised as a last resort).
     The team decides; nothing is approved here.
  5. --fly runs the whole pipeline (CFD, flight model, SITL, report) on one candidate, refused while that
     candidate still has a layout FAIL (use run.py directly to force it).
Exit code: 0 = a candidate with no layout FAIL exists, 2 = none does, 1 = something broke."""
import argparse
import json
import re
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
import run as _run  # noqa: E402,F401  (re-executes under the project venv when needed)
from vb import plan as P  # noqa: E402
from vb.dag import Runner  # noqa: E402
from vb.stages import NAMES, PHASES, DESK  # noqa: E402
import balance_apply  # noqa: E402


def gid(stage, check):
    return stage + ": " + re.sub(r"[-+]?\d+(\.\d+)?", "#", check)


def collect(run_dir):
    """Key numbers and gate sets of one desk-checked variant."""
    R = {}
    for s in NAMES:
        f = run_dir / s / "result.json"
        R[s] = json.loads(f.read_text()) if f.exists() else None
    d = lambda s: (R.get(s) or {}).get("data") or {}
    sm = lambda s: (R.get(s) or {}).get("summary") or {}
    fails = {gid(s, g["check"]): f"{g['check'][:110]} ({s})" for s in R if R[s] for g in R[s].get("gates", []) if g["status"] in ("FAIL", "ERROR")}
    for s in R:                                              # a stage that crashed is a finding, never a silent gap
        if R[s] and R[s].get("status") == "error":
            fails[f"{s}: stage crashed"] = f"{s} stage crashed: {str(R[s].get('error'))[:90]}"
    per_phase = {}
    for ph in PHASES:
        gs = [g for s in ph["stages"] if R.get(s) for g in R[s].get("gates", [])]
        crashed = [s for s in ph["stages"] if R.get(s) and R[s].get("status") == "error"]
        per_phase[ph["name"]] = dict(fail=sum(g["status"] in ("FAIL", "ERROR") for g in gs) + len(crashed),
                                     warn=sum(g["status"] == "WARN" for g in gs), crashed=crashed)
    m, v, th, pr = d("mass"), d("vtol"), d("thermal"), d("propulsion")
    rb = (pr.get("cruise_unit") or {}).get("range_band") or {}
    mi = (pr.get("missions") or [{}])[0]
    cg_t = m.get("cg_target_m")
    out = dict(
        auw_g=None if not m else round(m["mass_kg"] * 1000), cg_mm=None if not m else round(m["cg_m"][0] * 1000, 1),
        cg_err_mm=None if not m or cg_t is None else round((m["cg_m"][0] - cg_t) * 1000, 1),
        cg_in_window=None if not m or cg_t is None else abs(m["cg_m"][0] - cg_t) <= m["cg_tol_m"] + 1e-6,
        static_margin_pct=sm("aero_lowfi").get("static_margin_pct"), stall_pess=sm("aero_lowfi").get("V_stall_pessimistic"),
        thrust_centre_mm=sm("vtol").get("cg_to_thrust_centre_mm"), tw_isa=sm("vtol").get("TW_isa"), tw_worst=sm("vtol").get("TW_worst_air"),
        range_km=[round((rb.get("pessimistic_drag") or {}).get("range_km", 0)), round((rb.get("nominal") or {}).get("range_km", 0))] if rb else None,
        mission_margin_pct=None if "margin" not in mi else round(100 * mi["margin"]),
        hot_parts=None if not th else sum(r["peak_nominal_C"] > r["limit_C"] for r in th.get("parts", [])),
        clashes=sm("clash").get("clashes"), bay_failures=sm("clash").get("bay_failures"),
        phases=per_phase, fails=fails, n_fail=sum(p["fail"] for p in per_phase.values()), n_warn=sum(p["warn"] for p in per_phase.values()))
    return out


def desk(plan, variant, out, quiet=True):
    r = Runner(plan, variant, "quick", out, (), [n for n in NAMES if n in DESK], None, quiet, None)
    r.run()
    return r.run_dir


def fmt(v, suffix=""):
    return "-" if v is None else (f"{v}{suffix}" if not isinstance(v, list) else "-".join(str(x) for x in v) + suffix)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("plan")
    ap.add_argument("--variant", default=None)
    ap.add_argument("--options", default=None, help="comma-separated option ids to prove (default: the best --max-options that reach the target)")
    ap.add_argument("--max-options", type=int, default=4)
    ap.add_argument("--fly", default=None, help="'best', an option id or a variant name: run CFD + SITL + report on it")
    ap.add_argument("--fidelity", default="standard", choices=["quick", "standard", "full"], help="fidelity of the --fly run")
    ap.add_argument("--out", default=str(ROOT / "out"))
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args(argv)
    t0 = time.time()
    plan_p = Path(a.plan).resolve()
    base_v = a.variant or "base"
    print(f"[1/4] desk check of the design as drawn ({base_v})")
    bdir = desk(plan_p, a.variant, a.out, not a.verbose)
    base = collect(bdir)
    bo_f = bdir / "balance" / "balance_options.json"
    cands = [dict(id="0", variant=base_v, title="As drawn", effort=0.0, kind="none", m=base, pros=[], cons=[])]
    if bo_f.exists():
        bo = json.loads(bo_f.read_text())
        opts = [o for o in bo["options"] if o["id"] != "0"]
        need = (not base["cg_in_window"]) or base["phases"]["layout"]["fail"] > 0 or any(
            "lift-thrust centre" in k or "most-loaded lift motor" in k for k in base["fails"])
        if a.options:
            pick = [o for o in opts if o["id"] in set(a.options.split(","))]
        elif need:
            pick = [o for o in opts if o["feasible"]][:a.max_options]
        else:
            pick = []
        if pick:
            print(f"[2/4] proving {len(pick)} balance option(s): " + ", ".join(o["id"] for o in pick))
            added = balance_apply.add_variants(plan_p, bdir / "balance", [o["id"] for o in pick])
            for (oid, vname, title), o in zip(added, pick):
                print(f"      {oid}: {title}")
                vdir = desk(plan_p, vname, a.out, not a.verbose)
                cands.append(dict(id=oid, variant=vname, title=title, effort=o["score"], kind=o["kind"], m=collect(vdir),
                                  pros=o["pros"], cons=o["cons"], estimate=dict(cg_err_mm=o["cg_err_mm"], range_change_pct=o["range_change_pct"],
                                                                                 static_margin_pct=o["static_margin_pct"])))
        else:
            print("[2/4] " + ("no option reaches the target: see the balance stage" if need else
                              "the layout balances as drawn: no option needs proving (use --options to compare alternatives anyway)"))
    for c in cands:
        c["fixed"] = sorted(v for k, v in base["fails"].items() if k not in c["m"]["fails"]) if c["id"] != "0" else []
        c["new"] = sorted(v for k, v in c["m"]["fails"].items() if k not in base["fails"]) if c["id"] != "0" else []
    order = sorted(cands, key=lambda c: (c["m"]["phases"]["layout"]["fail"], not c["m"]["cg_in_window"], c["m"]["phases"]["sizing"]["fail"],
                                         len(c["new"]), c["effort"]))
    best = order[0]
    ok = best["m"]["phases"]["layout"]["fail"] == 0 and best["m"]["cg_in_window"]
    common = sorted(set.intersection(*[set(c["m"]["fails"]) for c in cands])) if cands else []

    # ---- decision brief -----------------------------------------------------------------------------
    plan_name = P.load_raw(plan_p)["meta"]["name"]
    odir = Path(a.out) / P.slug(plan_name) / "design_loop"
    odir.mkdir(parents=True, exist_ok=True)
    L = [f"# Design decision brief - {plan_name} ({base_v})", "",
         f"Desk check only (layout and sizing phases, no CFD or flight simulation). {time.strftime('%Y-%m-%d %H:%M')}. "
         "Numbers are uncalibrated until the team's measurements are fed back.", "",
         "## Where the design stands as drawn", "",
         f"- Weight {fmt(base['auw_g'], ' g')}; CG {fmt(base['cg_err_mm'], ' mm')} from the design CG "
         f"({'inside' if base['cg_in_window'] else 'OUTSIDE'} the allowed window).",
         f"- Phase 1 layout: {base['phases']['layout']['fail']} FAIL, {base['phases']['layout']['warn']} WARN. "
         f"Phase 2 sizing: {base['phases']['sizing']['fail']} FAIL, {base['phases']['sizing']['warn']} WARN.", ""]
    if len(cands) > 1:
        L += ["## Candidates, each proven by a desk check", "",
              "| # | change | CG error mm | static margin % | CG to lift-thrust centre mm | hover T/W (std day / worst air) | range km (pessimistic-nominal) | parts over temperature | layout FAIL | sizing FAIL | fixes | new problems |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for c in cands:
            m = c["m"]
            L.append(f"| {c['id']}{' (recommended)' if c is best and ok else ''} | {c['title']} | {fmt(m['cg_err_mm'])} | {fmt(m['static_margin_pct'])} | "
                     f"{fmt(m['thrust_centre_mm'])} | {fmt(m['tw_isa'])} / {fmt(m['tw_worst'])} | {fmt(m['range_km'])} | {fmt(m['hot_parts'])} | "
                     f"{m['phases']['layout']['fail']} | {m['phases']['sizing']['fail']} | {len(c['fixed'])} | {len(c['new'])} |")
        L += ["", "## Recommendation", "",
              (f"**{best['id']} - {best['title']}** (variant `{best['variant']}`): fewest layout FAILs, then fewest sizing FAILs and new problems, "
               "then least effort. This is a ranking rule, not a decision: the team chooses." if ok else
               "**No candidate clears the layout phase.** Change the layout or the parts list by hand and run the loop again."), ""]
        for c in cands[1:]:
            L += [f"### {c['id']} - {c['title']}", "", "Pros: " + ("; ".join(c["pros"]) or "-"), "", "Cons: " + ("; ".join(c["cons"]) or "-"), ""]
            if c.get("estimate"):
                L += [f"Estimate vs proven: CG error {c['estimate']['cg_err_mm']:+.1f} -> {fmt(c['m']['cg_err_mm'])} mm; "
                      f"static margin {fmt(c['estimate']['static_margin_pct'])} -> {fmt(c['m']['static_margin_pct'])} %.", ""]
            if c["fixed"]:
                L += ["Fixes: " + "; ".join(c["fixed"][:8]), ""]
            if c["new"]:
                L += ["NEW problems: " + "; ".join(c["new"][:8]), ""]
    else:
        L += ["## Balance", "", "The layout balances as drawn; no change is needed for balance. Alternatives (battery swaps, trims) are in "
              f"`{bdir}/balance/balance_options.md`.", ""]
    if common:
        L += ["## Still open whichever option is chosen", "", "These FAILs are in every candidate, so no balance option removes them:", ""]
        L += [f"- {cands[0]['m']['fails'][k]}" for k in common[:20]] + [""]
    L += ["## Next", "",
          f"1. Team picks a candidate. Reports per candidate: `{a.out}/{P.slug(plan_name)}/<variant>/report/index.html` "
          "(opens with the three questions and a what-to-do-next list grouped by who acts).",
          f"2. Flight proof (CFD + SITL): `python tools/design_loop.py {a.plan} {'--variant ' + a.variant + ' ' if a.variant else ''}--fly "
          f"{best['variant'] if ok else '<variant>'}`",
          f"3. Moved CAD for the chosen option: `python tools/balance_apply.py {a.plan} --from {bdir}/balance --option <id> --step-out balanced.step "
          "--cad-map <intake folder>/cad_map.json`",
          "4. When the team approves a design that is GO or CONDITIONAL GO, set `meta.approval` in the plan and re-run the report to release the ArduPilot parameter file.", ""]
    (odir / "DECISION.md").write_text("\n".join(L))
    (odir / "comparison.json").write_text(json.dumps(dict(plan=str(plan_p), base_variant=base_v, recommended=best["variant"] if ok else None,
                                                          candidates=[{k: v for k, v in c.items()} for c in cands], common_fails=common), indent=1, default=str))
    print(f"[3/4] decision brief: {odir / 'DECISION.md'}")
    print(f"      {'#':3s} {'CG err':>7s} {'SM %':>6s} {'T/W':>10s} {'range km':>9s} {'layout F':>8s} {'sizing F':>8s} {'new':>4s}  change")
    for c in cands:
        m = c["m"]
        print(f"      {c['id']:3s} {fmt(m['cg_err_mm']):>7s} {fmt(m['static_margin_pct']):>6s} {fmt(m['tw_isa']) + '/' + fmt(m['tw_worst']):>10s} "
              f"{fmt(m['range_km']):>9s} {m['phases']['layout']['fail']:8d} {m['phases']['sizing']['fail']:8d} {len(c['new']):4d}  "
              f"{c['title'][:70]}{'   <- recommended' if c is best and ok else ''}")

    # ---- flight proof ------------------------------------------------------------------------------
    rc = 0 if ok else 2
    if a.fly:
        tgt = best if a.fly == "best" else next((c for c in cands if a.fly in (c["id"], c["variant"])), None)
        vname = tgt["variant"] if tgt else a.fly
        if tgt and (tgt["m"]["phases"]["layout"]["fail"] or not tgt["m"]["cg_in_window"]):
            print(f"[4/4] NOT flying '{vname}': it still has a layout FAIL. Fix the layout first (python run.py forces it).")
            rc = 2
        else:
            print(f"[4/4] flight proof of '{vname}' at {a.fidelity} fidelity (CFD + SITL): this is the long step")
            r = Runner(plan_p, None if vname == "base" else vname, a.fidelity, a.out, (), None, None, not a.verbose, None)
            r.run()
            rep = r.run_dir / "report" / "result.json"
            if rep.exists():
                print("      OVERALL:", json.loads(rep.read_text())["data"].get("overall"), "-", r.run_dir / "report" / "index.html")
    else:
        print("[4/4] flight proof not requested (add --fly best)")
    print(f"done in {(time.time() - t0) / 60:.1f} min")
    return rc


if __name__ == "__main__":
    sys.exit(main())
