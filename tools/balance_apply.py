#!/usr/bin/env python3
"""Turn balance options into plan variants (so each can be run through the whole pipeline) and, when the
plan came from a CAD intake, write the re-positioned STEP assembly for an option.

  python tools/balance_apply.py PLAN.yaml --from out/<plan>/<variant>/balance --option B        # adds variant <variant>_balB
  python tools/balance_apply.py PLAN.yaml --from ... --all                                      # every option that reaches the target
  python tools/balance_apply.py PLAN.yaml --from ... --option B --step-out balanced_B.step --cad-map work/cad_map.json

The plan file is edited in place (variants only; a .bak copy is written first). Then:
  python run.py PLAN.yaml --variant <variant>_balB [--fidelity standard]

--step-out moves every CAD body whose component moved in the option (mirrored twins included) and writes a flat,
named STEP to import back into Fusion. Frame changes (wing, bay, boom moves) are in the change list, not in the
STEP: they need a designer."""
import argparse
import json
import shutil
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from vb import plan as P  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("plan")
    ap.add_argument("--from", dest="src", required=True, help="balance stage folder")
    ap.add_argument("--option", action="append", default=[])
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--step-out", default=None)
    ap.add_argument("--cad-map", default=None)
    a = ap.parse_args(argv)
    src = Path(a.src)
    bo = json.loads((src / "balance_options.json").read_text())
    ids = [o["id"] for o in bo["options"] if o["feasible"] and o["id"] != "0"] if a.all else a.option
    if not ids:
        sys.exit("no option selected (use --option ID or --all)")
    plan_p = Path(a.plan)
    raw = P.load_raw(plan_p)
    sv = bo.get("source_variant") or "base"
    added = []
    for i in ids:
        ovf = src / "options" / f"{i}.overlay.yaml"
        if not ovf.exists():
            sys.exit(f"option {i} has no overlay in {src / 'options'}")
        name = (f"{sv}_bal{i}" if sv != "base" else f"bal{i}")
        raw.setdefault("variants", {})[name] = yaml.safe_load(ovf.read_text())
        added.append(name)
    errs = []
    for n in added:
        try:
            errs += [f"{n}: {e}" for e in P.validate(P.resolve(raw, n))]
        except Exception as e:  # noqa: BLE001
            errs.append(f"{n}: {e}")
    if errs:
        sys.exit("option variants do not validate:\n  " + "\n  ".join(errs[:10]))
    shutil.copy(plan_p, plan_p.with_suffix(plan_p.suffix + ".bak"))
    head = "".join(l for l in plan_p.read_text().splitlines(True)[:40] if l.startswith("#"))
    plan_p.write_text(head + yaml.safe_dump(json.loads(json.dumps(raw)), sort_keys=False, width=160))
    print("variants added:", ", ".join(added))
    for n in added:
        print(f"  python run.py {plan_p} --variant {n}")

    if a.step_out:
        if len(ids) != 1:
            sys.exit("--step-out needs exactly one --option")
        if not a.cad_map:
            sys.exit("--step-out needs --cad-map (cad_map.json written by tools/intake.py)")
        from vb import cadio
        cm = json.loads(Path(a.cad_map).read_text())
        frame = cadio.Frame(**cm["frame"])
        opt = next(o for o in bo["options"] if o["id"] == ids[0])
        moves = {}
        for e in opt["edits"]:
            if e["kind"] == "move":
                moves[e["name"]] = [e.get("dx", 0.0), e.get("dy", 0.0), e.get("dz", 0.0)]
        bodies = []
        for c in cm["cad"]:
            bodies += cadio.read_folder(c) if Path(c).is_dir() else cadio.read_step(c)
        path_to_comp = {p: n for n, paths in cm["components"].items() for p in paths}
        out, moved = [], 0
        for b in bodies:
            comp = path_to_comp.get(b.path)
            shape = b.shape
            if comp in moves:
                shape = cadio.translated(shape, frame.vec_to_cad_mm(moves[comp]))
                moved += 1
            out.append((b.path.split("/")[-1], shape))
        cadio.write_step(a.step_out, out)
        skipped = [e for e in opt["edits"] if e["kind"] not in ("move",)]
        print(f"wrote {a.step_out}: {len(out)} bodies, {moved} moved" + (f"; {len(skipped)} edit(s) are not body moves "
              "(swaps, frame changes, ballast): see fusion_change_list.csv" if skipped else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
