"""DAG orchestrator: cache keys, stage subprocesses, timeouts, error isolation, run log."""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

from . import env, plan as P
from .stages import STAGES, BY_NAME, NAMES, ancestors, descendants

ROOT = Path(__file__).resolve().parent.parent


def code_hash(paths):
    import hashlib
    h = hashlib.sha256()
    for p in paths:
        p = ROOT / p
        files = sorted(p.rglob("*")) if p.is_dir() else [p]
        for f in files:
            if f.is_file() and f.suffix in (".py", ".yaml", ".json", ".parm", ".c", ".txt") and "__pycache__" not in f.parts:
                h.update(str(f.relative_to(ROOT)).encode())
                h.update(f.read_bytes())
    return h.hexdigest()[:16]


def git_info():
    """Pipeline git hash + dirty flag (tracked changes or untracked, non-ignored files)."""
    try:
        h = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
        st = subprocess.run(["git", "-C", str(ROOT), "status", "--porcelain"], capture_output=True, text=True).stdout
        dirty = [l[3:] for l in st.splitlines() if l.strip()]
        return dict(git_hash=h or "?", git_dirty=bool(dirty), dirty_files=dirty[:20])
    except Exception:
        return dict(git_hash="?", git_dirty=None, dirty_files=[])


def read_result(d):
    f = Path(d) / "result.json"
    if f.exists():
        try:
            return json.loads(f.read_text())
        except Exception:
            return None
    return None


class Runner:
    def __init__(self, plan_path, variant=None, fidelity="standard", out=None, force=(), only=None, until=None,
                 quiet=False, calibration=None):
        self.plan_path = Path(plan_path).resolve()
        self.raw = P.load_raw(self.plan_path)
        self.variant = variant or "base"
        self.fidelity = fidelity
        self.plan = P.resolve(self.raw, None if self.variant == "base" else self.variant)
        self.calibration = str(Path(calibration).resolve()) if calibration else None
        if self.calibration:
            self.plan = P.apply_calibration(self.plan, self.calibration)
        self.plan_slug = P.slug(self.plan["meta"]["name"])
        self.out_root = Path(out or ROOT / "out").resolve()
        self.run_dir = self.out_root / self.plan_slug / (self.variant + ("_calibrated" if self.calibration else ""))
        self.cache_dir = self.out_root / self.plan_slug / "_cache"
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.quiet = quiet
        sel = list(NAMES)
        if until:
            keep = ancestors(until) | {until}
            sel = [s for s in sel if s in keep]
        if only:
            sel = [s for s in sel if s in set(only)]
        self.selected = sel
        forced = set()
        for f in force or ():
            if f == "all":
                forced |= set(NAMES)
            else:
                forced |= {f} | descendants(f)
        self.forced = forced
        self.log = []
        self._ups = {}
        self.git = git_info()

    # ------------------------------------------------------------------ keys
    def static_inputs(self, st):
        """Code hash + hash of the plan subset the stage depends on (no upstream data)."""
        inputs = dict(stage=st["name"], code=code_hash(st["code"]),
                      plan={k: P.get_ptr(self.plan, k) for k in st["keys"]} if st["keys"] != ["/"] else self.plan,
                      fidelity=self.fidelity if st["fid"] else None)
        if st["name"] == "cfd":
            inputs.update(self._cfd_inputs())
            inputs.pop("plan")
        return inputs

    def _cfd_inputs(self):
        out = {}
        cfg = self.plan.get("analysis", {}).get("cfd", {}) or {}
        cfdk = cfg.get("kind_full", cfg.get("kind", "wing")) if self.fidelity == "full" else cfg.get("kind", "wing")
        ls = self.plan["airframe"]["lifting_surfaces"]
        out["geom"] = [P.main_wing(self.plan)] if cfdk == "wing" else [ls, self.plan["airframe"].get("fuselage")]
        cg_t = P.val((self.plan.get("analysis") or {}).get("cg_target_x"))
        if cg_t is None:
            m = read_result(self.run_dir / "mass") or {}
            cg = (m.get("data") or {}).get("cg_m")
            cg_t = cg[0] if cg else None
        out["moment_ref_x_mm"] = None if cg_t is None else round(cg_t * 1000)
        out["alphas"] = cfg.get("alphas_full" if self.fidelity == "full" else "alphas_standard")
        out["cfg"] = {k: v for k, v in cfg.items() if k not in ("alphas_standard", "alphas_full")}
        return out

    def provenance(self, st):
        si = self.static_inputs(st)
        return dict(code_hash=si["code"], plan_subset_hash=P.sha({k: v for k, v in si.items() if k != "code"}, 16),
                    fidelity=self.fidelity, fidelity_matters=bool(st["fid"]))

    def key(self, st):
        inputs = dict(stage=st["name"], code=code_hash(st["code"]),
                      plan={k: P.get_ptr(self.plan, k) for k in st["keys"]} if st["keys"] != ["/"] else self.plan,
                      fidelity=self.fidelity if st["fid"] else None)
        if st["name"] == "cfd":
            # CFD case depends only on the external shape + moment reference: shared by variants whose
            # analysed geometry is identical (e.g. clean wing for base and VTOL)
            cfg = self.plan.get("analysis", {}).get("cfd", {}) or {}
            cfdk = cfg.get("kind_full", cfg.get("kind", "wing")) if self.fidelity == "full" else cfg.get("kind", "wing")
            ls = self.plan["airframe"]["lifting_surfaces"]
            inputs["geom"] = [P.main_wing(self.plan)] if cfdk == "wing" else [ls, self.plan["airframe"].get("fuselage")]
            cg_t = P.val((self.plan.get("analysis") or {}).get("cg_target_x"))
            if cg_t is None:
                m = read_result(self.run_dir / "mass") or {}
                cg = (m.get("data") or {}).get("cg_m")
                cg_t = cg[0] if cg else None
            inputs["moment_ref_x_mm"] = None if cg_t is None else round(cg_t * 1000)
            inputs["alphas"] = cfg.get("alphas_full" if self.fidelity == "full" else "alphas_standard")
        else:
            ups = {}
            for d in st["deps"] + st["soft"]:
                r = read_result(self.run_dir / d)
                ups[d] = r.get("data_hash") if r and r.get("status") == "ok" else "absent"
            inputs["upstream"] = ups
            self._ups[st["name"]] = ups
            if st["name"] == "sitl" and os.environ.get("VB_SITL_TESTS"):
                inputs["sitl_subset"] = os.environ["VB_SITL_TESTS"]
        return P.sha(inputs, 20)

    # ------------------------------------------------------------------ run one stage
    def run_stage(self, st):
        name = st["name"]
        sdir = self.run_dir / name
        t0 = time.time()
        if name == "report":
            self.write_expected()     # keys every result must carry for a consistent report (computed NOW)
        # dependencies
        for d in st["deps"]:
            r = read_result(self.run_dir / d)
            if not r or r.get("status") != "ok":
                self._write_blocked(sdir, name, f"blocked: upstream '{d}' {'missing' if not r else r.get('status')}")
                return self._record(name, "blocked", 0, f"upstream {d} not ok")
        key = self.key(st)
        prev = read_result(sdir)
        if name not in self.forced and not st["cheap"] and prev and prev.get("key") == key and prev.get("status") in ("ok", "skipped"):
            return self._record(name, "cached", 0, "same key in stage dir", prev)
        store = self.cache_dir / name / key
        if name not in self.forced and not st["cheap"] and (store / "result.json").exists():
            if sdir.exists():
                shutil.rmtree(sdir)
            shutil.copytree(store, sdir, symlinks=True)
            r = read_result(sdir)
            return self._record(name, "cached", time.time() - t0, f"copied from {store.relative_to(self.out_root)}", r)
        # fresh run in a subprocess
        if sdir.exists():
            shutil.rmtree(sdir)
        sdir.mkdir(parents=True)
        prov = dict(self.provenance(st), upstream=self._ups.get(name, {}), **{k: v for k, v in self.git.items() if k != "dirty_files"},
                    plan_hash=P.sha(self.plan, 16), sitl_subset=os.environ.get("VB_SITL_TESTS") if name == "sitl" else None,
                    created=time.strftime("%Y-%m-%d %H:%M:%S"))
        spec = dict(stage=name, key=key, plan_path=str(self.plan_path), variant=self.variant, fidelity=self.fidelity,
                    calibration=self.calibration, provenance=prov,
                    stage_dir=str(sdir), run_dir=str(self.run_dir), out_root=str(self.out_root))
        spec_f = sdir / "spec.json"
        spec_f.write_text(json.dumps(spec, indent=1))
        timeout = st["timeout"] * (1.5 if self.fidelity == "full" else 1.0)
        logf = open(sdir / "stage.log", "w")
        print(f"\n=== [{time.strftime('%H:%M:%S')}] {name} (fidelity {self.fidelity}, key {key[:10]}) ===", flush=True)
        envv = dict(os.environ, PYTHONPATH=str(ROOT) + os.pathsep + os.environ.get("PYTHONPATH", ""),
                    MPLBACKEND="Agg", PYTHONUNBUFFERED="1")
        try:
            p = subprocess.Popen([sys.executable, "-m", "vb.core", str(spec_f)], cwd=str(ROOT), env=envv,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
            deadline = time.time() + timeout
            for line in p.stdout:
                logf.write(line); logf.flush()
                if not self.quiet:
                    sys.stdout.write("  " + line); sys.stdout.flush()
                if time.time() > deadline:
                    break
            try:
                p.wait(timeout=max(5, deadline - time.time()))
            except subprocess.TimeoutExpired:
                import signal
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
                logf.write(f"\nTIMEOUT after {timeout:.0f} s\n")
        except Exception as e:  # orchestrator-level failure is recorded, never fatal
            logf.write(f"\norchestrator error: {e}\n")
        logf.close()
        r = read_result(sdir)
        if r is None:
            tail = (sdir / "stage.log").read_text()[-3000:]
            r = dict(stage=name, status="error", key=key, verdict="ERROR", gates=[], files=[], data={}, summary={},
                     error="stage process produced no result.json (crash or timeout)", log_tail=tail,
                     wall_s=round(time.time() - t0, 1), data_hash="none")
            (sdir / "result.json").write_text(json.dumps(r, indent=1))
        if r.get("status") in ("ok", "skipped") and not st["cheap"]:
            if store.exists():
                shutil.rmtree(store)
            store.parent.mkdir(parents=True, exist_ok=True)
            shutil.copytree(sdir, store, symlinks=True)
        return self._record(name, r.get("status"), time.time() - t0, r.get("error", ""), r)

    def _write_blocked(self, sdir, name, why):
        if sdir.exists():
            shutil.rmtree(sdir)
        sdir.mkdir(parents=True)
        (sdir / "result.json").write_text(json.dumps(dict(stage=name, status="blocked", verdict="BLOCKED", error=why,
                                                          gates=[], files=[], data={}, summary={}, data_hash="none"), indent=1))

    def _record(self, name, status, wall, note, r=None):
        g = (r or {}).get("gates", [])
        rec = dict(stage=name, status=status, wall_s=round(wall, 1), note=note,
                   verdict=(r or {}).get("verdict", "BLOCKED" if status == "blocked" else "?"),
                   n_fail=sum(x["status"] == "FAIL" for x in g), n_warn=sum(x["status"] == "WARN" for x in g))
        self.log.append(rec)
        print(f"--- {name:12s} {status:8s} {rec['verdict']:7s} FAIL {rec['n_fail']} WARN {rec['n_warn']} "
              f"({rec['wall_s']} s) {note[:120] if status not in ('ok',) else ''}", flush=True)
        return rec

    # ------------------------------------------------------------------ run all
    def write_expected(self):
        """What a CONSISTENT set of stage results must carry (checked by the report stage, review M5/M6)."""
        exp = dict(fidelity=self.fidelity, plan_hash=P.sha(self.plan, 16), git=self.git,
                   stages={s["name"]: dict(self.provenance(s), key=self.key(s)) for s in STAGES if s["name"] != "report"})
        (self.run_dir / "expected_provenance.json").write_text(json.dumps(exp, indent=1))

    def run(self):
        t0 = time.time()
        print(f"plan {self.plan_path.name} | variant {self.variant} | fidelity {self.fidelity} | out {self.run_dir}")
        print(f"stages: {', '.join(self.selected)}" + (f" | forced: {', '.join(sorted(self.forced))}" if self.forced else ""))
        for name in self.selected:
            self.run_stage(BY_NAME[name])
        total = time.time() - t0
        runlog = dict(plan=str(self.plan_path), variant=self.variant, fidelity=self.fidelity, started=time.ctime(t0),
                      total_wall_s=round(total, 1), stages=self.log, tool_versions=env.versions())
        hist = self.run_dir / "runs.jsonl"
        with open(hist, "a") as f:
            f.write(json.dumps(runlog) + "\n")
        (self.run_dir / "last_run.json").write_text(json.dumps(runlog, indent=1))
        print(f"\n=== done in {total/60:.1f} min ===")
        for rec in self.log:
            print(f"  {rec['stage']:12s} {rec['status']:8s} {rec['verdict']:8s} FAIL {rec['n_fail']:2d} WARN {rec['n_warn']:2d}  {rec['wall_s']:7.1f} s")
        rep = self.run_dir / "report" / "index.html"
        if rep.exists():
            print(f"report: {rep}")
        return runlog
