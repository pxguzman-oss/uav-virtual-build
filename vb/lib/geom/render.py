#!/usr/bin/env python3
"""Render views of a model directory and annotate them.

Backends
  blender    : system Blender (apt 4.0.2 on Ubuntu 24.04) running render_blender.py
  bpy        : official `bpy` wheel (pip, Blender 5.x, has OpenImageDenoise) in its own venv
  matplotlib : pure-python fallback, no OpenGL needed (flat-shaded, fast, less pretty)

  python render.py out/stork --backend blender --engine CYCLES --samples 32
  python render.py out/stork --backend bpy --views iso,xray
  python render.py out/stork --backend matplotlib

Outputs <out>/<view>.png (composited on a light background, x-ray views labelled)
and <out>/render_timings.json.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import time

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

DEFAULT_BPY_PY = os.environ.get("VB_BPY_PYTHON", "")   # only for --backend bpy (python with the bpy module); not installed by install.sh


# --------------------------------------------------------------------------- helpers
def font(size, bold=False):
    for p in ([
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
    ]):
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default(size=size)


def background(w, h, top=(236, 240, 245), bottom=(186, 196, 210)):
    t = np.linspace(0, 1, h)[:, None, None]
    arr = (np.array(top)[None, None, :] * (1 - t) + np.array(bottom)[None, None, :] * t)
    arr = np.repeat(arr, w, axis=1).astype(np.uint8)
    return Image.fromarray(arr, "RGB").convert("RGBA")


def composite(png, title=None, subtitle=None):
    fg = Image.open(png).convert("RGBA")
    bg = background(*fg.size)
    bg.alpha_composite(fg)
    if title:
        d = ImageDraw.Draw(bg)
        d.text((28, 22), title, fill=(30, 35, 45), font=font(30, True))
        if subtitle:
            d.text((28, 60), subtitle, fill=(80, 88, 100), font=font(19))
    return bg


def annotate(img: Image.Image, labels, masses, info_lines, min_gap=30, layout="columns"):
    """Place labels with leader lines to each component.
    layout=columns: left/right columns (perspective / planform views)
    layout=rows   : rows above and below the aircraft sorted by x (side views: short,
                    near-vertical leaders that do not cross)."""
    W, H = img.size
    box_h = 24 * len(info_lines) + 14 + 24 if info_lines else 0
    W, H = img.size
    d = ImageDraw.Draw(img)
    fs = max(14, int(H / 52))
    f = font(fs)
    fb = font(fs + 2, True)
    comps = [l for l in labels if l["group"] != "marker" and 0 <= l["px"][0] <= W and 0 <= l["px"][1] <= H]
    marks = [l for l in labels if l["group"] == "marker"]
    cx = np.median([l["px"][0] for l in comps]) if comps else W / 2
    left = sorted([l for l in comps if l["px"][0] <= cx], key=lambda l: l["px"][1])
    right = sorted([l for l in comps if l["px"][0] > cx], key=lambda l: l["px"][1])

    def column(items, x_text, align_right, bot):
        n = len(items)
        if not n:
            return
        top = 110
        gap = min(min_gap, (bot - top) / max(n - 1, 1))
        ys = np.clip(np.array([l["px"][1] for l in items], float), top, bot)
        for i in range(1, n):                      # push down to keep the gap
            ys[i] = max(ys[i], ys[i - 1] + gap)
        if ys[-1] > bot:                           # then pull back up from the bottom
            ys[-1] = bot
            for i in range(n - 2, -1, -1):
                ys[i] = min(ys[i], ys[i + 1] - gap)
        for l, y in zip(items, ys):
            m = masses.get(l["name"])
            txt = f"{l['label'].replace('_', ' ')}" + (f"  {m:.0f} g" if m is not None else "")
            tw = d.textlength(txt, font=f)
            col = tuple(int(255 * c) for c in (l.get("color") or [0.3, 0.3, 0.3])[:3])
            ax, ay = l["px"]
            if align_right:            # text ends at x_text, swatch before it, line leaves from the swatch
                tx = x_text - tw
                lx = tx - 24
            else:                      # swatch at x_text-18, text after it, line leaves from text end
                tx = x_text
                lx = tx + tw + 6
            d.line([(lx, y), (ax, ay)], fill=(60, 70, 85, 190), width=2)
            d.ellipse([ax - 5, ay - 5, ax + 5, ay + 5], fill=col + (255,), outline=(20, 20, 20))
            d.rectangle([tx - 18, y - 6, tx - 6, y + 6], fill=col + (255,))
            d.text((tx, y - 11), txt, fill=(25, 30, 40), font=f)

    def rows(items):
        if not items:
            return
        ys_all = np.array([l["px"][1] for l in items])
        y_mid = float(np.median(ys_all))
        ytop, ybot = ys_all.min() - 70, ys_all.max() + 70
        groups_ = ([l for l in items if l["px"][1] <= y_mid], [l for l in items if l["px"][1] > y_mid])
        ops = []
        for gi, grp in enumerate(groups_):
            grp = sorted(grp, key=lambda l: l["px"][0])
            txts = []
            for l in grp:
                m = masses.get(l["name"])
                txts.append(f"{l['label'].replace('_', ' ')}" + (f"  {m:.0f} g" if m is not None else ""))
            widths = [d.textlength(t, font=f) + 30 for t in txts]
            # two staggered sub-rows; pack left->right, then shift back inside the image
            sub = [[], []]
            last_end = [-1e9, -1e9]
            for l, t, wdt in zip(grp, txts, widths):
                k = 0 if last_end[0] <= last_end[1] else 1
                x0 = max(l["px"][0] - wdt / 2, last_end[k] + 12, 30)
                last_end[k] = x0 + wdt
                sub[k].append((l, t, x0, wdt))
            for k in range(2):
                over = (sub[k][-1][2] + sub[k][-1][3] - (W - 30)) if sub[k] else 0
                if over > 0:
                    sub[k] = [(l, t, x0 - over, wdt) for l, t, x0, wdt in sub[k]]
                for l, t, x0, wdt in sub[k]:
                    y = (ytop - 34 * k) if gi == 0 else (ybot + 34 * k)
                    ops.append((l, t, x0, wdt, y, gi))
        # leaders first, then boxes on top (outer-row leaders pass behind inner-row boxes)
        for l, t, x0, wdt, y, gi in ops:
            d.line([(x0 + wdt / 2, y + (10 if gi == 0 else -10)), tuple(l["px"])], fill=(60, 70, 85, 190), width=2)
        for l, t, x0, wdt, y, gi in ops:
            col = tuple(int(255 * c) for c in (l.get("color") or [0.3, 0.3, 0.3])[:3])
            ax_, ay_ = l["px"]
            d.ellipse([ax_ - 5, ay_ - 5, ax_ + 5, ay_ + 5], fill=col + (255,), outline=(20, 20, 20))
            d.rounded_rectangle([x0, y - 13, x0 + wdt, y + 13], 6, fill=(255, 255, 255, 235), outline=col + (255,))
            d.rectangle([x0 + 7, y - 5, x0 + 17, y + 5], fill=col + (255,))
            d.text((x0 + 22, y - 11), t, fill=(25, 30, 40), font=f)

    if layout == "rows":
        rows(comps)
    else:
        column(left, 40 + 18, align_right=False, bot=H - box_h - 30)
        column(right, W - 40, align_right=True, bot=H - 30)
    # CG / target labels: each on the OUTER side of the pair (screen x), so the label order
    # matches the true fore/aft order of the two markers; CG below, target above
    pos_ = {mk["label"]: mk["px"] for mk in marks}
    for mk in marks:
        x, y = mk["px"]
        is_cg = mk["label"] == "CG"
        other = pos_.get("CG target" if is_cg else "CG")
        left_side = other is not None and x < other[0]
        col = (215, 160, 0) if is_cg else (0, 150, 60)
        tw = d.textlength(mk["label"], font=fb)
        tx = x - tw - 12 if left_side else x + 12
        ty = y + 10 if is_cg else y - 34
        d.text((tx, ty), mk["label"], fill=col, font=fb, stroke_width=3, stroke_fill=(255, 255, 255))
    # info box bottom-left
    if info_lines:
        f2 = font(17)
        bw = max(d.textlength(t, font=f2) for t in info_lines) + 24
        bh = 24 * len(info_lines) + 14
        x0, y0 = 28, H - bh - 24
        d.rounded_rectangle([x0, y0, x0 + bw, y0 + bh], 10, fill=(255, 255, 255, 225), outline=(150, 160, 175))
        for i, t in enumerate(info_lines):
            d.text((x0 + 12, y0 + 8 + 24 * i), t, fill=(25, 30, 40), font=f2)
    return img


def info_from_mass(mp):
    if not mp:
        return []
    b = mp["body_axes_about_cg"]
    cg = mp["cg_struct_mm"]
    lines = [f"Mass {mp['mass_kg']*1000:.0f} g   CG x={cg[0]:.1f} mm z={cg[2]:.1f} mm"]
    if "cg_target_x_mm" in mp:
        lines.append(f"Target x={mp['cg_target_x_mm']} mm  ({mp['cg_error_x_mm']:+.1f} mm, {mp.get('cg_pct_mac', float('nan')):.1f}% MAC)")
    lines.append(f"Ixx {b['Ixx']:.4f}  Iyy {b['Iyy']:.4f}  Izz {b['Izz']:.4f}  Ixz {b['Ixz']:+.4f} kg m^2")
    return lines


# --------------------------------------------------------------------------- backends
def run_blender(a, raw_dir):
    script = os.path.join(HERE, "render_blender.py")
    common = [a.model_dir, raw_dir, "--engine", a.engine, "--samples", str(a.samples), "--res", *map(str, a.res),
              "--views", a.views] + (["--threads", str(a.threads)] if a.threads else []) + (["--outline"] if a.outline else []) + (["--shadow"] if a.shadow else [])
    if a.backend == "blender":
        exe = a.blender or shutil.which("blender")
        if not exe:
            raise SystemExit("blender not found (apt-get install blender, or use --backend bpy/matplotlib)")
        cmd = [exe, "-b", "--factory-startup", "-noaudio", "-P", script, "--"] + common
        if a.engine != "CYCLES" and shutil.which("xvfb-run"):   # GL engines need a display
            cmd = ["xvfb-run", "-a", "-s", "-screen 0 1920x1080x24"] + cmd
    else:
        cmd = [a.bpy_python, script] + common
    t0 = time.perf_counter()
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0 or not os.path.exists(os.path.join(raw_dir, "render_timings.json")):
        sys.stderr.write(p.stdout[-3000:] + p.stderr[-3000:])
        raise SystemExit(f"render backend failed ({p.returncode})")
    tim = json.load(open(os.path.join(raw_dir, "render_timings.json")))
    tim["wall_total_s"] = round(time.perf_counter() - t0, 2)
    return tim


def run_matplotlib(a, raw_dir):
    """Flat-shaded fallback using matplotlib's 3D Poly3DCollection (painter's algorithm)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    from geomkit import manifest
    man, parts = manifest.read(a.model_dir)
    mp_path = os.path.join(a.model_dir, "mass_properties.json")
    mp = json.load(open(mp_path)) if os.path.exists(mp_path) else None
    views = {"iso": (30, -125), "top": (90, -90), "side": (0, -90), "front": (0, 180), "rear": (25, 40),
             "xray": (35, -125), "xray_top": (90, -90), "xray_side": (0, -90)}
    light = np.array([-0.4, -0.5, 0.75]); light /= np.linalg.norm(light)
    tim = {}
    for view in a.views.split(","):
        t0 = time.perf_counter()
        xray = view.startswith("xray")
        fig = plt.figure(figsize=(a.res[0] / 100, a.res[1] / 100), dpi=100)
        ax = fig.add_axes([0, 0, 1, 1], projection="3d")
        ax.set_proj_type("ortho")
        labels = []
        for p in parts:
            if p.mesh is None:
                continue
            m = p.mesh
            if len(m.faces) > 6000:
                try:   # needs the optional fast_simplification wheel
                    m = m.simplify_quadric_decimation(face_count=6000)
                except Exception:
                    pass
            col = np.array((p.color or [0.6, 0.6, 0.6])[:3])
            shade = 0.45 + 0.55 * np.clip(m.face_normals @ light, 0, 1)
            fc = np.clip(col[None, :] * shade[:, None], 0, 1)
            alpha = 1.0
            if p.role == "structure" and xray:
                alpha, fc = 0.12, np.tile([0.55, 0.7, 0.95], (len(fc), 1))
            elif p.role == "prop":
                alpha = 0.15
            elif p.role in ("component", "tube") and not xray:
                continue                              # hidden inside the skin anyway
            pc = Poly3DCollection(m.triangles, facecolors=np.c_[fc, np.full(len(fc), alpha)], linewidths=0)
            ax.add_collection3d(pc)
            if p.role == "component" and xray:
                labels.append((p, m.centroid))
        lo, hi = man_bounds(parts)
        c, r = (lo + hi) / 2, (hi - lo).max() / 2
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.set_box_aspect((1, 1, 1), zoom=1.9 if views.get(view, (30,))[0] not in (0, 90) else 1.35)
        ax.view_init(*views.get(view, (30, -125)))
        ax.set_axis_off()
        fig.patch.set_alpha(0); ax.patch.set_alpha(0)
        fig.canvas.draw()
        lab = []
        if xray:
            from mpl_toolkits.mplot3d import proj3d
            pts = [(p, c3) for p, c3 in labels]
            if mp:
                pts.append(("CG", np.array(mp["cg_struct_mm"])))
                if "cg_target_x_mm" in mp:
                    pts.append(("CG target", np.array([mp["cg_target_x_mm"], 0, mp["cg_struct_mm"][2]])))
            for p, c3 in pts:
                x2, y2, _ = proj3d.proj_transform(*c3, ax.get_proj())
                X, Y = ax.transData.transform((x2, y2))
                if isinstance(p, str):
                    lab.append(dict(name=p, label=p, group="marker", px=[X, a.res[1] - Y]))
                    ax.scatter(*c3, s=80, c="gold" if p == "CG" else "limegreen", edgecolors="k", depthshade=False)
                else:
                    lab.append(dict(name=p.name, label=p.label or p.name, group=p.group, color=p.color, px=[X, a.res[1] - Y]))
        fig.savefig(os.path.join(raw_dir, f"{view}.png"), transparent=True, dpi=100)
        plt.close(fig)
        json.dump(lab, open(os.path.join(raw_dir, f"{view}_labels.json"), "w"))
        tim[view] = round(time.perf_counter() - t0, 2)
    return dict(engine="matplotlib", res=a.res, timings_s=tim)


def man_bounds(parts):
    b = np.array([p.mesh.bounds for p in parts if p.mesh is not None and p.role != "prop"])
    return b[:, 0].min(0), b[:, 1].max(0)


# --------------------------------------------------------------------------- main
TITLES = {"iso": "Isometric", "top": "Top view", "side": "Side view (left)", "front": "Front view", "rear": "Rear 3/4",
          "xray": "X-ray: component placement", "xray_top": "X-ray: top", "xray_side": "X-ray: side"}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("model_dir")
    ap.add_argument("--out", default=None, help="default <model_dir>/renders")
    ap.add_argument("--backend", default="blender", choices=["blender", "bpy", "matplotlib"])
    ap.add_argument("--engine", default="CYCLES", choices=["CYCLES", "EEVEE", "BLENDER_WORKBENCH"])
    ap.add_argument("--samples", type=int, default=32)
    ap.add_argument("--res", type=int, nargs=2, default=[1600, 1000])
    ap.add_argument("--views", default="iso,top,side,front,xray,xray_top")
    ap.add_argument("--threads", type=int, default=0)
    ap.add_argument("--outline", action="store_true", help="Freestyle outlines (blender backends)")
    ap.add_argument("--shadow", action="store_true", help="ground shadow catcher (Cycles; slow, use with bpy/OIDN)")
    ap.add_argument("--blender", default=None, help="path to blender executable")
    ap.add_argument("--bpy-python", default=DEFAULT_BPY_PY, help="python with the bpy wheel")
    ap.add_argument("--title", default=None)
    ap.add_argument("--reuse-raw", action="store_true", help="skip rendering, re-annotate existing raw/*.png")
    a = ap.parse_args(argv)

    out = a.out or os.path.join(a.model_dir, "renders")
    raw = os.path.join(out, "raw")
    os.makedirs(raw, exist_ok=True)
    if a.reuse_raw and os.path.exists(os.path.join(raw, "render_timings.json")):
        tim = json.load(open(os.path.join(raw, "render_timings.json")))
    elif a.backend == "matplotlib":
        tim = run_matplotlib(a, raw)
        json.dump(tim, open(os.path.join(raw, "render_timings.json"), "w"))
    else:
        tim = run_blender(a, raw)
    tim["backend"] = a.backend

    man = json.load(open(os.path.join(a.model_dir, "manifest.json")))
    mp_path = os.path.join(a.model_dir, "mass_properties.json")
    mp = json.load(open(mp_path)) if os.path.exists(mp_path) else None
    masses = {p["name"]: p["mass_g"] for p in mp["parts"]} if mp else {}
    name = a.title or man["plan"].get("name", "model").title()
    vtol = man["plan"].get("options", {}).get("vtol")
    t0 = time.perf_counter()
    for view in a.views.split(","):
        img = composite(os.path.join(raw, f"{view}.png"), f"{name}{' VTOL' if vtol else ''} - {TITLES.get(view, view)}",
                        f"{a.backend}/{tim.get('engine')} {a.res[0]}x{a.res[1]}")
        if view.startswith("xray"):
            labels = json.load(open(os.path.join(raw, f"{view}_labels.json")))
            img = annotate(img, labels, masses, info_from_mass(mp),
                           layout="rows" if view in ("xray_side",) else "columns")
        img.convert("RGB").save(os.path.join(out, f"{view}.png"))
    tim["annotate_s"] = round(time.perf_counter() - t0, 2)
    json.dump(tim, open(os.path.join(out, "render_timings.json"), "w"), indent=1)
    print(json.dumps(tim))
    return 0


if __name__ == "__main__":
    sys.exit(main())
