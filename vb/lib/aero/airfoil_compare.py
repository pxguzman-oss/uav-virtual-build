#!/usr/bin/env python
"""
2D airfoil comparison: XFOIL 6.99 (apt) vs NeuralFoil (AeroSandbox) for the S3021
at Re = 100k/200k/300k, Ncrit = 9, flap 0 deg and +10 deg (plain flap, hinge 75 % c).

Outputs:
  out/airfoil/polars_<tool>_Re<re>_f<flap>.csv
  out/airfoil/summary.json (CLmax, alpha_stall, CDmin, CL/CD max, timings)
  out/airfoil/s3021_polars.png
Usage:
  python airfoil_compare.py [--airfoil s3021] [--xfoil /usr/bin/xfoil]
"""
import argparse, json, os, subprocess, tempfile, time
import numpy as np
import aerosandbox as asb

HERE = os.path.dirname(os.path.abspath(__file__))


def write_dat(af, path):
    with open(path, "w") as f:
        f.write(f"{af.name}\n")
        for x, y in af.coordinates:
            f.write(f" {x:.6f} {y:.6f}\n")


class _Silence:
    """Silence Fortran stdout (xfoil-python prints every BL iteration)."""
    def __enter__(self):
        import sys; sys.stdout.flush()
        self.fd = os.dup(1); self.null = os.open(os.devnull, os.O_WRONLY); os.dup2(self.null, 1)
    def __exit__(self, *a):
        os.dup2(self.fd, 1); os.close(self.null); os.close(self.fd)


def run_xfoil(af, Re, ncrit=9.0, flap_deg=0.0, hinge_x=0.75, a_min=-6, a_max=18, da=0.5, max_iter=150):
    """XFOIL 6.99 via xfoil-python (compiled from github.com/DARcorporation/xfoil-python).
    The flap is applied geometrically (AeroSandbox add_control_surface) so that XFOIL and NeuralFoil
    see the identical deflected section. Two sweeps from 0 deg (up, then down) for robust convergence."""
    import xfoil
    from xfoil.model import Airfoil as XAirfoil
    g = af.add_control_surface(deflection=flap_deg, hinge_point_x=hinge_x) if flap_deg else af
    g = g.repanel(n_points_per_side=120)
    t0 = time.time()
    with _Silence():
        x = xfoil.XFoil()
        x.airfoil = XAirfoil(g.coordinates[:, 0], g.coordinates[:, 1])
        x.repanel(n_nodes=200)
        x.Re = Re; x.n_crit = ncrit; x.max_iter = max_iter
        rows = []
        for lo, hi, step in ((0.0, a_max, da), (-da, a_min, -da)):
            x.reset_bls()
            for a in np.arange(lo, hi + step / 2, step):
                cl, cd, cm, cp = x.a(a)
                if np.isfinite(cl):
                    rows.append([a, cl, cd, np.nan, cm, np.nan, np.nan])
    dt = time.time() - t0
    rows = np.array(sorted(rows, key=lambda r: r[0])) if rows else np.zeros((0, 7))
    return rows, dt


def run_neuralfoil(af, Re, ncrit=9.0, flap_deg=0.0, hinge_x=0.75, alphas=None, model_size="xlarge"):
    alphas = np.arange(-6, 18.01, 0.5) if alphas is None else alphas
    cs = [asb.ControlSurface(name="flap", deflection=flap_deg, hinge_point=hinge_x)] if flap_deg else None
    t0 = time.time()
    r = af.get_aero_from_neuralfoil(alpha=alphas, Re=Re, n_crit=ncrit, model_size=model_size,
                                    control_surfaces=cs, include_360_deg_effects=False)
    dt = time.time() - t0
    out = np.column_stack([alphas, r["CL"], r["CD"], np.full_like(alphas, np.nan), r["CM"],
                           r["Top_Xtr"], r["Bot_Xtr"]])
    return out, dt, np.asarray(r["analysis_confidence"])


def summarize(p):
    if len(p) < 3:
        return {}
    a, cl, cd = p[:, 0], p[:, 1], p[:, 2]
    i = int(np.argmax(cl))
    ld = cl / cd
    j = int(np.argmax(ld))
    # lift slope over -2..4 deg
    m = (a >= -2) & (a <= 4)
    cla = float(np.polyfit(a[m], cl[m], 1)[0] * 180 / np.pi) if m.sum() > 2 else None
    a0 = float(-np.polyfit(a[m], cl[m], 1)[1] / np.polyfit(a[m], cl[m], 1)[0]) if m.sum() > 2 else None
    return dict(CLmax=float(cl[i]), alpha_CLmax=float(a[i]), CDmin=float(cd.min()),
                CL_at_CDmin=float(cl[int(np.argmin(cd))]), LD_max=float(ld[j]), CL_at_LDmax=float(cl[j]),
                alpha_L0=a0, cl_alpha_per_rad=cla, n_points=int(len(a)), alpha_range=[float(a.min()), float(a.max())])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--airfoil", default="s3021")
    ap.add_argument("--xfoil", default="xfoil")
    ap.add_argument("--res", default="100000,200000,300000")
    ap.add_argument("--flaps", default="0,10")
    ap.add_argument("--ncrit", type=float, default=9.0)
    a = ap.parse_args()
    outdir = os.path.join(HERE, "out", "airfoil")
    os.makedirs(outdir, exist_ok=True)
    af = asb.Airfoil(a.airfoil)
    dat = os.path.join(outdir, f"{a.airfoil}.dat")
    write_dat(af, dat)
    res = [float(x) for x in a.res.split(",")]
    flaps = [float(x) for x in a.flaps.split(",")]
    summary = {"airfoil": a.airfoil, "ncrit": a.ncrit, "hinge_x": 0.75, "cases": []}
    polars = {}
    # warm-up NeuralFoil (first call loads weights)
    t0 = time.time(); run_neuralfoil(af, 2e5, alphas=np.array([0.0])); t_nf_load = time.time() - t0
    for Re in res:
        for fl in flaps:
            px, tx = run_xfoil(af, Re, a.ncrit, fl)
            pn, tn, conf = run_neuralfoil(af, Re, a.ncrit, fl)
            runs = [("xfoil", px, tx), ("neuralfoil", pn, tn)]
            if fl:
                # NeuralFoil on the geometrically deflected section (Kulfan fit of flapped coordinates)
                afg = af.add_control_surface(deflection=fl, hinge_point_x=0.75)
                pg, tg, _ = run_neuralfoil(afg, Re, a.ncrit, 0.0)
                runs.append(("neuralfoil_geomflap", pg, tg))
            for tool, p, t in runs:
                fn = os.path.join(outdir, f"polars_{tool}_Re{int(Re/1000)}k_f{int(fl)}.csv")
                np.savetxt(fn, p, delimiter=",", header="alpha,CL,CD,CDp,CM,xtr_top,xtr_bot", comments="")
                polars[(tool, Re, fl)] = p
                s = summarize(p)
                s.update(tool=tool, Re=Re, flap_deg=fl, wall_s=round(t, 3))
                if tool == "neuralfoil":
                    s["min_confidence"] = float(conf.min())
                summary["cases"].append(s)
                print(f"{tool:10s} Re={Re:7.0f} flap={fl:4.1f}  t={t:6.2f}s  " +
                      "  ".join(f"{k}={v:.4g}" for k, v in s.items() if isinstance(v, float) and k not in ("Re", "flap_deg", "wall_s")))
    # agreement metrics at matched alphas (-2..8 deg), each NeuralFoil variant vs XFOIL
    for (tool, Re, fl), pn in polars.items():
        if tool == "xfoil":
            continue
        px = polars[("xfoil", Re, fl)]
        if len(px) < 3:
            continue
        m = (px[:, 0] >= -2) & (px[:, 0] <= 8)
        cln = np.interp(px[m, 0], pn[:, 0], pn[:, 1]); cdn = np.interp(px[m, 0], pn[:, 0], pn[:, 2])
        summary.setdefault("agreement_vs_xfoil_m2_to_8deg", []).append(dict(
            tool=tool, Re=Re, flap_deg=fl, CL_rms_diff=round(float(np.sqrt(np.mean((cln - px[m, 1]) ** 2))), 4),
            CD_mean_rel_diff=round(float(np.mean((cdn - px[m, 2]) / px[m, 2])), 4)))
    summary["neuralfoil_first_call_load_s"] = round(t_nf_load, 2)
    json.dump(summary, open(os.path.join(outdir, "summary.json"), "w"), indent=1)
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 3, figsize=(15, 5))
        cols = {100000: "tab:blue", 200000: "tab:orange", 300000: "tab:green"}
        for (tool, Re, fl), p in polars.items():
            if len(p) == 0: continue
            ls = {"xfoil": "-", "neuralfoil": "--"}.get(tool, ":")
            lw = 2 if fl == 0 else 1
            lab = f"{tool} Re{int(Re/1e3)}k f{int(fl)}"
            c = cols.get(Re, "k")
            ax[0].plot(p[:, 0], p[:, 1], ls, color=c, lw=lw, label=lab)
            ax[1].plot(p[:, 2], p[:, 1], ls, color=c, lw=lw)
            ax[2].plot(p[:, 0], p[:, 4], ls, color=c, lw=lw)
        ax[0].set_xlabel("alpha [deg]"); ax[0].set_ylabel("CL"); ax[0].legend(fontsize=6)
        ax[1].set_xlabel("CD"); ax[1].set_ylabel("CL"); ax[1].set_xlim(0, 0.04)
        ax[2].set_xlabel("alpha [deg]"); ax[2].set_ylabel("CM c/4")
        for x in ax: x.grid(alpha=.3)
        fig.suptitle(f"{a.airfoil.upper()} Ncrit={a.ncrit}: XFOIL (solid) vs NeuralFoil (dashed); thick=flap 0, thin=flap 10")
        fig.tight_layout(); fig.savefig(os.path.join(outdir, f"{a.airfoil}_polars.png"), dpi=110)
    except Exception as e:
        print("plot failed", e)


if __name__ == "__main__":
    main()
