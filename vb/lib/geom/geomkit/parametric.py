"""Parametric airframe builder: YAML build plan -> list of Part objects.

Everything is plain numpy/trimesh meshes; booleans go through manifold3d so the
outer mould line (OML) comes out watertight by construction.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, field

import numpy as np
import trimesh
import yaml

from . import geometry as G


# --------------------------------------------------------------------------- data
@dataclass
class Part:
    name: str
    mesh: trimesh.Trimesh | None
    group: str = "misc"
    role: str = "component"          # structure | component | tube | prop | marker
    mass_model: str = "solid"        # solid | shell | point
    mass: float | None = None        # g; None -> from material/geometry
    material: str | None = None
    wall_mm: float | None = None
    infill: float = 0.0
    in_oml: bool = False
    bay: str | None = None
    point: list | None = None        # for point masses
    color: list | None = None
    label: str | None = None
    extra: dict = field(default_factory=dict)

    def meta(self) -> dict:
        d = {k: v for k, v in self.__dict__.items() if k not in ("mesh",)}
        return d


# --------------------------------------------------------------------------- config
def load_plan(path: str, overrides: list[str] | None = None) -> dict:
    plan = yaml.safe_load(open(path))
    for ov in overrides or []:
        key, val = ov.split("=", 1)
        node = plan
        keys = key.split(".")
        for k in keys[:-1]:
            node = node.setdefault(k, {})
        node[keys[-1]] = yaml.safe_load(val)
    return plan


# --------------------------------------------------------------------------- lifting surfaces
class LiftingSurface:
    """Straight-tapered panel described in its own span coordinate s (mm, along the
    panel), so spars/servos can be attached by (s, chord fraction)."""

    def __init__(self, airfoil, root_chord, tip_chord, span, root_le, sweep_le_deg=None,
                 sweep_qc_deg=None, dihedral_deg=0.0, twist_tip_deg=0.0, side=+1, s0=0.0, n_af=100):
        self.af = G.resample_airfoil(G.load_airfoil(airfoil), n_af)
        self.cr, self.ct, self.b = root_chord, tip_chord, span
        self.root_le = np.asarray(root_le, float)
        if sweep_le_deg is None:  # derive LE sweep from quarter-chord sweep
            qc = np.radians(sweep_qc_deg or 0.0)
            self.tan_le = np.tan(qc) + 0.25 * (root_chord - tip_chord) / span
        else:
            self.tan_le = np.tan(np.radians(sweep_le_deg))
        self.dih = dihedral_deg
        self.twist = twist_tip_deg
        self.side = side
        self.s0 = s0
        d = np.radians(dihedral_deg)
        self.dir = np.array([0.0, side * np.cos(d), np.sin(d)])

    def chord(self, s):
        return self.cr + (self.ct - self.cr) * s / self.b

    def le(self, s):
        return self.root_le + self.dir * s + np.array([self.tan_le * s, 0, 0])

    def twist_at(self, s):
        return self.twist * s / self.b

    def section(self, s):
        roll = self.side * self.dih
        return G.wing_section(self.af, self.chord(s), self.le(s), self.twist_at(s), roll)

    def camber_point(self, s, frac):
        """3D point on the mean camber line at span s and chord fraction frac."""
        af = self.af
        i_le = int(np.argmin(af[:, 0]))
        up, lo = af[: i_le + 1][::-1], af[i_le:]
        zc = 0.5 * (np.interp(frac, up[:, 0], up[:, 1]) + np.interp(frac, lo[:, 0], lo[:, 1]))
        sec = G.wing_section(np.array([[frac, zc]]), self.chord(s), self.le(s), self.twist_at(s), self.side * self.dih)
        return sec[0]

    def thickness(self, s, frac):
        af = self.af
        i_le = int(np.argmin(af[:, 0]))
        up, lo = af[: i_le + 1][::-1], af[i_le:]
        return (np.interp(frac, up[:, 0], up[:, 1]) - np.interp(frac, lo[:, 0], lo[:, 1])) * self.chord(s)

    def mesh(self, n_sections=8):
        ss = np.linspace(self.s0, self.b, n_sections + 1)
        secs = [self.section(s) for s in ss]
        if self.side < 0:
            secs = secs  # orientation fixed inside loft()
        return G.loft(secs, cap_2d=self.af)


def _surfaces(plan: dict) -> dict[str, LiftingSurface]:
    af = plan["airframe"]
    w = af["wing"]
    fus_half = 0.5 * max(st[1] for st in af["fuselage"]["stations"])
    half = 0.5 * w["span"]
    surf = {}
    for side, tag in ((+1, "R"), (-1, "L")):
        surf[f"wing_{tag}"] = LiftingSurface(
            w["airfoil"], w["root_chord"], w["tip_chord"], half, [0, 0, w["z_root"]],
            sweep_qc_deg=w.get("sweep_quarter_chord_deg", 0), dihedral_deg=w.get("dihedral_deg", 0),
            twist_tip_deg=w.get("twist_tip_deg", 0), side=side, s0=0.5 * fus_half)
    v = af["vtail"]
    for side, tag in ((+1, "R"), (-1, "L")):
        surf[f"vtail_{tag}"] = LiftingSurface(
            v["airfoil"], v["root_chord"], v["tip_chord"], v["panel_span"], [v["x_root_le"], 0, v["z_root"]],
            sweep_le_deg=v.get("sweep_le_deg", 0), dihedral_deg=v["dihedral_deg"], side=side, s0=5)
    return surf


def _fuselage(plan) -> trimesh.Trimesh:
    f = plan["airframe"]["fuselage"]
    e = f.get("exponent", 2.5)
    n = 64
    base = G.superellipse(1.0, 1.0, n, e)
    secs = []
    st = np.asarray(f["stations"], float)
    # smooth, shape-preserving (PCHIP) interpolation between stations; cosine
    # clustering of sections toward the nose and tail for a rounded nose cap
    from scipy.interpolate import PchipInterpolator
    iw, ih, iz = (PchipInterpolator(st[:, 0], st[:, k]) for k in (1, 2, 3))
    u = 0.5 * (1 - np.cos(np.linspace(0, np.pi, f.get("n_sections", 60))))
    xs = np.unique(np.concatenate([st[:, 0], st[0, 0] + u * (st[-1, 0] - st[0, 0])]))
    for x in xs:
        wdt, hgt, zc = float(iw(x)), float(ih(x)), float(iz(x))
        p = base * [wdt, hgt]
        secs.append(np.c_[np.full(n, x), p[:, 0], p[:, 1] + zc])
    return G.loft(secs, cap_2d=base)


# --------------------------------------------------------------------------- builder
def build_parts(plan: dict) -> tuple[list[Part], dict]:
    af = plan["airframe"]
    groups = plan.get("groups", {})
    col = lambda g: groups.get(g, {}).get("color", [0.6, 0.6, 0.6])
    parts: list[Part] = []
    if not (af.get("wing") and af.get("fuselage") and af.get("vtail")):   # pure-import plan: no parametric airframe
        return _non_structure(plan, {}, parts, col), {"surfaces": {}}
    surf = _surfaces(plan)

    # ---- printed OML structure
    pr = af["fuselage"]["printed"]
    parts.append(Part("fuselage", _fuselage(plan), "printed", "structure", "shell", None, pr["material"],
                      pr["wall_mm"], pr.get("infill", 0.0), True, color=col("printed"), label="Fuselage"))
    for key in ("wing_L", "wing_R", "vtail_L", "vtail_R"):
        src = af["wing"] if key.startswith("wing") else af["vtail"]
        nsec = src.get("sections_per_side", 6)
        pr = src["printed"]
        parts.append(Part(key, surf[key].mesh(nsec), "printed", "structure", "shell", None, pr["material"],
                          pr["wall_mm"], pr.get("infill", 0.0), True, color=col("printed"), label=key.replace("_", " ")))
    parts = _non_structure(plan, surf, parts, col)
    info = {"surfaces": {k: dict(root_chord=s.cr, tip_chord=s.ct, span=s.b, tan_le=s.tan_le) for k, s in surf.items()}}
    return parts, info


def _non_structure(plan, surf, parts, col):
    af = plan.get("airframe", {})
    # ---- carbon tubes
    for t in plan.get("tubes", []):
        if "attach" in t:   # placed along a lifting surface: s range + chord fraction
            ls = surf[t["attach"]]
            p0 = ls.camber_point(t["s0"], t["chord_frac"])
            p1 = ls.camber_point(t["s1"], t["chord_frac"])
            center, axis, length = 0.5 * (p0 + p1), p1 - p0, float(np.linalg.norm(p1 - p0))
        else:
            center, axis, length = np.asarray(t["center"], float), np.asarray(t["axis"], float), t["length"]
        m = G.tube(t["od"], t["wall"], length, center, axis, sections=32)
        parts.append(Part(t["name"], m, "carbon", "tube", "solid", t.get("mass"), t.get("material", "cfrp_tube"),
                          color=col("carbon"), label=t["name"], extra=dict(od=t["od"], wall=t["wall"], length=length)))

    # ---- components
    for c in plan.get("components", []):
        c = copy.deepcopy(c)
        if "attach" in c:
            ls = surf[c["attach"]]
            c["center"] = ls.camber_point(c["s"], c["chord_frac"]).tolist()
        shape = c.get("shape", "box")
        if shape == "box":
            m = G.box(c["size"], c["center"])
        elif shape == "cylinder":
            m = G.cylinder(c["d"], c["length"], c["center"], c.get("axis", [0, 0, 1]))
        elif shape == "mesh":
            m = trimesh.load(c["file"], force="mesh")
        else:
            m = None
        parts.append(Part(c["name"], m, c.get("group", "misc"), "component", "point" if m is None else "solid",
                          c.get("mass"), c.get("material"), bay=c.get("bay"), point=c.get("center"),
                          color=c.get("color", col(c.get("group", "misc"))), label=c.get("label", c["name"])))

    # ---- propeller (render + CFD actuator disk, not part of OML)
    p = af.get("propeller")
    if p:
        disk = G.cylinder(p["diameter"], 2.0, [p["x"], 0, p["z"]], [1, 0, 0], sections=72)
        parts.append(Part("prop_disk", disk, "propulsion", "prop", "solid", p.get("mass", 0.0), color=[0.3, 0.3, 0.3],
                          label="10x6 pusher", in_oml=False))

    # ---- VTOL pack
    if plan.get("options", {}).get("vtol"):
        v = plan["vtol"]
        b = v["boom"]
        for tag, sgn in (("L", -1), ("R", +1)):
            y = sgn * b["y"]
            m = G.square_tube(b["outer"], b["wall"], b["length"], [b["x_center"], y, b["z"]], [1, 0, 0])
            parts.append(Part(f"vtol_boom_{tag}", m, "vtol", "tube", "solid", b.get("mass"), "cfrp_tube", in_oml=True,
                              color=col("carbon"), label=f"VTOL boom {tag}"))
            for end, xe in (("F", b["x_center"] - b["length"] / 2 + 15), ("A", b["x_center"] + b["length"] / 2 - 15)):
                lm = v["lift_motor"]
                zc = b["z"] + b["outer"] / 2 + lm["length"] / 2
                parts.append(Part(f"lift_motor_{end}{tag}", G.cylinder(lm["d"], lm["length"], [xe, y, zc]), "vtol",
                                  "component", "solid", lm["mass"], color=col("vtol"), label=f"lift motor {end}{tag}"))
                parts.append(Part(f"lift_prop_{end}{tag}", G.cylinder(v["prop_diameter"], 1.5, [xe, y, zc + lm["length"] / 2 + 6]),
                                  "vtol", "prop", "solid", 0.0, color=[0.3, 0.3, 0.3], label=None))
                if v.get("lift_esc_mass"):
                    parts.append(Part(f"lift_esc_{end}{tag}", G.box([30, 12, 6], [xe + (25 if end == "F" else -25), y, b["z"]]),
                                      "vtol", "component", "solid", v["lift_esc_mass"], color=col("vtol")))

    return parts


def build_oml(parts: list[Part]) -> trimesh.Trimesh:
    """Union of all in_oml parts -> single watertight outer mould line."""
    return G.boolean([p.mesh for p in parts if p.in_oml and p.mesh is not None], "union")
