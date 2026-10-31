"""Cross-check the beam-theory scripts with CalculiX (ccx 2.21, `apt install calculix-ccx`).

Model A: 10x8 mm round CFRP tube, 0.5 m cantilever (main spar half), tip load 20 N
         -> static tip deflection + first 3 natural frequencies.
Model B: 10x10x1 mm square CFRP boom, 0.257 m overhang (front VTOL arm) with a
         70.6 g point mass (motor+prop+mount) at the tip -> first natural frequency.
Elements: B32R quadratic beams (ccx expands them internally to C3D20R bricks).
"""
import math
import re
import subprocess
from pathlib import Path

E, NU, RHO = 1.2e11, 0.3, 1550.0
WD = Path(__file__).resolve().parent / "out" / "ccx"
WD.mkdir(parents=True, exist_ok=True)


def beam_inp(name, L, nel, section_lines, tip_load=None, tip_mass=None, step="static"):
    nn = 2 * nel + 1
    lines = ["*HEADING", name, "*NODE"]
    lines += [f"{i+1}, {L*i/(nn-1):.6f}, 0, 0" for i in range(nn)]
    lines.append("*ELEMENT, TYPE=B32R, ELSET=EB")
    lines += [f"{e+1}, {2*e+1}, {2*e+2}, {2*e+3}" for e in range(nel)]
    if tip_mass:
        lines += ["*ELEMENT, TYPE=MASS, ELSET=EM", f"{nel+1}, {nn}", "*MASS, ELSET=EM", f"{tip_mass}"]
    lines += ["*NSET, NSET=ROOT", "1", "*NSET, NSET=TIP", f"{nn}",
              "*MATERIAL, NAME=CF", "*ELASTIC", f"{E}, {NU}", "*DENSITY", f"{RHO}"]
    lines += section_lines
    lines += ["*BOUNDARY", "ROOT, 1, 6, 0.0"]
    if step == "static":
        lines += ["*STEP", "*STATIC", "*CLOAD", f"TIP, 3, {tip_load}", "*NODE PRINT, NSET=TIP", "U", "*END STEP"]
    else:
        lines += ["*STEP", "*FREQUENCY", "4", "*END STEP"]
    (WD / f"{name}.inp").write_text("\n".join(lines) + "\n")
    r = subprocess.run(["ccx", "-i", name], cwd=WD, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(r.stdout[-2000:] + r.stderr[-2000:])
    return (WD / f"{name}.dat").read_text()


def freqs(dat):
    out = []
    blk = dat.split("E I G E N V A L U E   O U T P U T")[1]
    for line in blk.splitlines():
        m = re.match(r"\s+(\d+)\s+([-\d.E+]+)\s+([-\d.E+]+)\s+([-\d.E+]+)\s+([-\d.E+]+)", line)
        if m:
            out.append(float(m.group(4)))      # cycles/time column
    return out


# ---- Model A -------------------------------------------------------------------
L, F = 0.5, 20.0
ro, t = 0.005, 0.001
I = math.pi / 64 * (0.010 ** 4 - 0.008 ** 4)
A = math.pi / 4 * (0.010 ** 2 - 0.008 ** 2)
secA = ["*BEAM SECTION, ELSET=EB, MATERIAL=CF, SECTION=PIPE", f"{ro}, {t}", "0, 1, 0"]
dat = beam_inp("spar_static", L, 20, secA, tip_load=F)
uz = float(re.findall(r"^\s+\d+\s+[-\d.E+]+\s+[-\d.E+]+\s+([-\d.E+]+)", dat.split("displacements")[1], re.M)[0])
d_an = F * L ** 3 / (3 * E * I)
sig_an = F * L * 0.005 / I
print(f"Model A  10x8 tube cantilever 0.5 m, 20 N tip load")
print(f"   tip deflection: CalculiX {uz*1000:.2f} mm | Euler-Bernoulli {d_an*1000:.2f} mm | diff {(uz/d_an-1)*100:+.1f} %")
print(f"   root bending stress (beam theory) {sig_an/1e6:.0f} MPa")
datf = beam_inp("spar_modal", L, 20, secA, step="freq")
fa = freqs(datf)
m = RHO * A
f_an = [(bl ** 2) / (2 * math.pi) * math.sqrt(E * I / (m * L ** 4)) for bl in (1.8751, 4.6941, 7.8548)]
print(f"   frequencies CalculiX {[round(f,1) for f in fa[:4]]} Hz (pairs = two bending planes)")
print(f"   analytic bending      {[round(f,1) for f in f_an]} Hz")

# ---- Model B -------------------------------------------------------------------
Lb, mt = 0.257, 0.0706
Ib = (0.010 ** 4 - 0.008 ** 4) / 12
Ab = 0.010 ** 2 - 0.008 ** 2
secB = ["*BEAM SECTION, ELSET=EB, MATERIAL=CF, SECTION=BOX", "0.010, 0.010, 0.001, 0.001, 0.001, 0.001", "0, 1, 0"]
datb = beam_inp("boom_modal", Lb, 16, secB, tip_mass=mt, step="freq")
fb = freqs(datb)
k = 3 * E * Ib / Lb ** 3
f_b_an = math.sqrt(k / (mt + 0.24 * RHO * Ab * Lb)) / 2 / math.pi
print(f"Model B  10x10x1 square boom, 257 mm overhang, 70.6 g tip mass")
print(f"   f1 CalculiX {fb[0]:.1f} Hz | Rayleigh (structures.py) {f_b_an:.1f} Hz | diff {(fb[0]/f_b_an-1)*100:+.1f} %")
print(f"   (hover 1P of the lift props is 185-230 Hz -> boom runs super-critical; it passes f1 on spool-up)")
