"""Stage 5c - thermal: lumped network of every heat source over the mission timeline.

Nodes  : battery, ESCs, flight controller / radios / payload computers (each in its bay's air),
         one air node and one skin node per fuselage bay, motors (outside, in their own airflow).
Sources: battery I2R, ESC conduction + switching, motor copper + iron loss (propulsion stage),
         electronics power (components[].power_W, else avionics.avionics_power_W shared out), sun on the skin.
Paths  : part -> bay air (natural convection + radiation), bay air -> printed skin -> outside air
         (flat-plate forced convection at flight speed, natural + radiation at rest), optional vents.
Timeline: ground soak in the sun -> the mission segments of the propulsion stage (hover, climb, cruise,
         loiter, hover landing) -> post-landing soak. Hottest and coldest environment case of the plan.
Two networks: NOMINAL and PESSIMISTIC (losses x1.25, convection x0.7). A limit broken by the nominal
network is a FAIL, one broken only by the pessimistic network a WARN. Everything within 10 K of its
limit is listed as a candidate for detailed CFD / a thermocouple on the first hover.

The printed structure matters as much as the electronics: LW-PLA softens near 55 C, so bay air, parts
touching the print and motor mounts are checked against the glass-transition temperature of the material."""
import math
import sys
from pathlib import Path

import numpy as np

from .. import plan as P
from ..figures import style, save, PALETTE, INK2

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))

H_PART, H_WALL, H_REST_EXT = 10.0, 7.0, 8.0      # W/m2K: part->bay air (nat. conv + radiation), air->wall, skin->still air
H_MOTOR_RUN, H_MOTOR_WIND, H_MOTOR_STILL = 60.0, 35.0, 10.0
CP = dict(energy=1000.0, propulsion=900.0, avionics=900.0, payload=900.0, motor=450.0, print=1800.0)
DEFAULT_LIMIT = dict(energy=60.0, esc=90.0, motor=120.0, avionics=70.0, payload=70.0, vtx=85.0)
K_PRINT = 0.10                                     # W/mK foamed LW-PLA wall (solid PLA ~0.13-0.2)


def box_area(d):
    a, b, c = d
    return 2 * (a * b + b * c + a * c)


def rho_at(T_C=15.0, alt=0.0):
    p = 101325 * (1 - 2.25577e-5 * alt) ** 5.25588
    return p / (287.05 * (T_C + 273.15))


def comp_power(c):
    e = c.get("electrical") or {}
    v = c.get("power_W", e.get("power_W"))
    return None if v is None else float(P.val(v))


def limit_of(c, is_motor=False):
    if c.get("thermal_limit_C") is not None:
        return float(c["thermal_limit_C"]), "plan"
    n = c["name"].lower()
    if is_motor:
        return DEFAULT_LIMIT["motor"], "default"
    if "esc" in n:
        return DEFAULT_LIMIT["esc"], "default"
    if "vtx" in n or "video" in n:
        return DEFAULT_LIMIT["vtx"], "default"
    return DEFAULT_LIMIT.get(c.get("category"), 70.0), "default"


def section_at(stations, x):
    st = np.array(stations, float)
    return float(np.interp(x, st[:, 0], st[:, 1])), float(np.interp(x, st[:, 0], st[:, 2]))


def run(ctx):
    import thermal as TH
    plan = ctx.plan
    an = (plan.get("analysis") or {}).get("thermal") or {}
    prop = ctx.data("propulsion")
    vt = ctx.data("vtol", required=False) or {}
    comps = P.expanded_components(plan)
    fus = plan["airframe"]["fuselage"]
    bays = dict(fus.get("bays") or {})
    stations = fus.get("stations") or [[fus.get("x_nose", 0), fus.get("max_width", 0.1), fus.get("max_height", 0.1), 0],
                                       [fus.get("x_nose", 0) + fus.get("length", 1), fus.get("max_width", 0.1), fus.get("max_height", 0.1), 0]]
    skin = fus.get("skin") or {}
    skin_mat = plan["materials"].get(skin.get("material", ""), {})
    t_skin = float(skin.get("thickness", 0.0006)) * max(int(skin.get("walls", 1)), 1)
    tg = skin_mat.get("tg_C")
    L_fus = float(fus.get("length", 1.0))
    envs = plan.get("environment_cases") or [dict(name="ISA", temperature_C=15, altitude_msl=0)]
    solar = lambda e: float(P.val(e.get("solar_heating_dT_C"), 0.0) or 0.0)
    hot = max(envs, key=lambda e: e.get("temperature_C", 15) + solar(e))
    cold = min(envs, key=lambda e: e.get("temperature_C", 15))
    pack_S, pack_P = plan["energy"]["series"], plan["energy"]["parallel"]
    r_cells = pack_S * P.val(plan["energy"]["cell"]["r_dc"]) / pack_P
    v_bus_nom = pack_S * plan["energy"]["cell"].get("v_nom", 3.6)
    P_av = float(plan.get("avionics", {}).get("avionics_power_W", 4.0))

    # ---- classify components --------------------------------------------------------------------
    units = plan["propulsion"]["units"]
    motor_of, esc_of = {}, {}
    prop_comps = [c for c in comps if c.get("category") == "propulsion"]
    for u in units:
        mm = P.val(u["motor"].get("mass"), 0.03)
        names = []
        for p_ in u.get("positions") or []:
            cand = [c for c in prop_comps if "esc" not in c["name"].lower() and c["mass"] >= 0.6 * mm
                    and np.linalg.norm(np.subtract(c["position"], p_)) < 0.08]
            if cand:
                names.append(min(cand, key=lambda c: np.linalg.norm(np.subtract(c["position"], p_)))["name"])
        motor_of[u["name"]] = sorted(set(names))
        ec = (u.get("esc") or {}).get("component")
        if not ec:
            escs = [c["name"] for c in prop_comps if "esc" in c["name"].lower()]
            ec = next((n_ for n_ in escs if u["name"].lower() in n_.lower() or u["role"] in n_.lower()), None)
        esc_of[u["name"]] = ec
    motors = {n_ for v in motor_of.values() for n_ in v}
    escs = {v for v in esc_of.values() if v}

    def bay_of(c):
        if c.get("bay") in bays:
            return c["bay"]
        for bn, b in bays.items():
            if all(abs(c["position"][i] - b["center"][i]) <= b["size"][i] / 2 for i in range(3)):
                return bn
        return "_fuselage"

    def is_internal(c):
        if c["name"] in motors:
            return False
        if c["name"] in escs:
            return True
        return (not c.get("external") and c.get("geom") is not False and abs(c["position"][1]) < 0.08
                and c.get("category") in ("energy", "avionics", "payload", "propulsion"))

    def area_of(c):
        return box_area(c["dims"]) if c.get("dims") else 6 * (c["mass"] / 1500.0) ** (2 / 3)

    internal = [c for c in comps if is_internal(c)]
    batt = [c["name"] for c in internal if c.get("category") == "energy"]
    elec = [c for c in internal if c.get("category") in ("avionics", "payload")]
    given = {c["name"]: comp_power(c) for c in elec if comp_power(c) is not None}
    rest = [c for c in elec if c["name"] not in given]
    left = max(P_av - sum(given.values()), 0.0)
    a_rest = sum(area_of(c) for c in rest)
    q_elec = {**given, **{c["name"]: left * area_of(c) / a_rest for c in rest}}
    if rest:
        ctx.note(f"electronics without power_W share the remaining {left:.1f} W of avionics.avionics_power_W by surface area "
                 f"({', '.join(c['name'] for c in rest)}): put power_W on the parts list to localise the heat")

    used_bays = sorted({bay_of(c) for c in internal})
    bay_geo = {}
    for bn in used_bays:
        if bn == "_fuselage":
            xs = [s[0] for s in stations]
            cx, Lb = 0.5 * (min(xs) + max(xs)), 0.6 * L_fus
        else:
            cx, Lb = bays[bn]["center"][0], bays[bn]["size"][0]
        w, h = section_at(stations, cx)
        bay_geo[bn] = dict(L=Lb, A_wall=0.9 * 2 * (w + h) * Lb, A_top=w * Lb, vol=0.7 * w * h * Lb, w=w, h=h)

    def build(scale_q, scale_h, env, hmap=None):
        """Network + phase list for one environment. hmap: part -> part-to-air coefficient from the last iterate."""
        hmap = hmap or {}
        T_amb = float(env.get("temperature_C", 15))
        rho = rho_at(T_amb, env.get("altitude_msl", 0))
        rr = rho / 1.225
        net = TH.Network()
        for bn in used_bays:
            g = bay_geo[bn]
            net.node(f"air:{bn}", max(rho * g["vol"] * TH.AIR_CP, 5.0))
            net.node(f"skin:{bn}", g["A_wall"] * t_skin * skin_mat.get("density", 800) * CP["print"])
        for c in internal:
            net.node(c["name"], c["mass"] * CP.get(c.get("category"), 900.0))
        for mn in sorted(motors):
            c = next(x for x in comps if x["name"] == mn)
            net.node(mn, c["mass"] * CP["motor"])

        hov = next((r for r in (prop.get("vtol") or {}).get("hover_table") or [] if r.get("is_plan_auw")), None)
        worst_hov = (vt.get("cases") or [None, None])[1] if scale_q > 1 else None
        cu = prop.get("cruise_unit") or {}
        cr, wot = cu.get("cruise") or {}, (cu.get("static_wot") or {}).get("nominal") or {}
        lift = next((u for u in units if u["role"] == "lift"), None)
        cruise = next((u for u in units if u["role"] == "cruise"), None)
        vc = plan["envelope"]["v_cruise"]

        def state(kind):
            """-> (airspeed, Q dict, motors running set)"""
            Q, running = {}, set()
            k = kind.lower()
            if "vtol" in k and hov and lift:
                Im = (worst_hov or {}).get("I_motor") or hov["I_motor"]
                loss = (worst_hov or {}).get("motor_loss_W") or hov["motor_loss_W"]
                Ptot = (worst_hov or {}).get("hover_P_W") or hov["P_total_W"]
                for mn in motor_of[lift["name"]]:
                    Q[mn] = loss; running.add(mn)
                if esc_of.get(lift["name"]):
                    Q[esc_of[lift["name"]]] = lift["count"] * Im ** 2 * lift["esc"].get("r_on", 0.004) + 0.01 * Ptot + 0.5
                Ib, V = Ptot / v_bus_nom, 0.0
            elif "climb" in k or k.startswith("launch"):
                for mn in motor_of.get(cruise["name"], []):
                    Q[mn] = wot.get("motor_loss_W", 0.0); running.add(mn)
                if esc_of.get(cruise["name"]):
                    Q[esc_of[cruise["name"]]] = wot.get("I_motor", 0) ** 2 * cruise["esc"].get("r_on", 0.004) + 0.01 * wot.get("P_elec_per_motor", 0) + 0.3
                Ib, V = wot.get("I_bus_total", 0.0), vc
            elif any(s in k for s in ("cruise", "loiter", "survey")):
                for mn in motor_of.get(cruise["name"], []):
                    Q[mn] = cr.get("motor_loss_W", 0.0); running.add(mn)
                if esc_of.get(cruise["name"]):
                    Q[esc_of[cruise["name"]]] = cr.get("I_motor", 0) ** 2 * cruise["esc"].get("r_on", 0.004) + 0.01 * cr.get("P_elec_per_motor", 0) + 0.3
                Ib, V = cr.get("I_bus_total", 0.0), vc
            elif "soak" in k:
                Ib, V = P_av / v_bus_nom, 0.0
            else:                                   # glide / fixed-wing landing
                Ib, V = P_av / v_bus_nom, vc * 0.8
            for bn_ in batt:
                Q[bn_] = Ib ** 2 * r_cells / max(len(batt), 1)
            Q.update(q_elec)
            return V, {k_: v * scale_q for k_, v in Q.items()}, running

        def phase(name, kind, dur):
            V, Q, running = state(kind)
            links, amb = [], []
            h_ext = TH.h_forced_plate(V, L_fus, rr) * (1.0 if scale_h >= 1 else 0.8) + H_REST_EXT * (1.0 if V < 0.3 else 0.6)
            for bn in used_bays:
                g = bay_geo[bn]
                g_in = 1.0 / (1.0 / (H_WALL * scale_h * g["A_wall"]) + t_skin / (K_PRINT * g["A_wall"]))
                links.append((f"air:{bn}", f"skin:{bn}", g_in))
                amb.append((f"skin:{bn}", h_ext * g["A_wall"]))
                Q[f"skin:{bn}"] = solar(env) * H_REST_EXT * g["A_top"] * (1.3 if scale_q > 1 else 1.0)
                va = float((an.get("vents") or {}).get(bn, 0.0))
                if va > 0 and V > 0.3:
                    amb.append((f"air:{bn}", rho * TH.AIR_CP * 0.3 * va * V))
            for c in internal:
                links.append((c["name"], f"air:{bay_of(c)}", hmap.get(c["name"], H_PART) * scale_h * area_of(c)))
            for mn in sorted(motors):
                c = next(x for x in comps if x["name"] == mn)
                d = c.get("dims") or [0.03, 0.03, 0.03]
                dia, hgt = (d[1], d[0]) if (c.get("axis") or [1, 0, 0])[0] else (d[0], d[2])
                A = 2.0 * (math.pi * dia * hgt + 2 * math.pi * dia ** 2 / 4)          # x2: ventilated bell, both faces
                h = H_MOTOR_RUN if mn in running else (H_MOTOR_WIND if V > 0.3 else H_MOTOR_STILL)
                amb.append((mn, h * (scale_h if scale_h < 1 else 1.0) * A * math.sqrt(rr)))
            return dict(name=name, kind=kind, duration=float(dur), T_amb=T_amb, links=links, amb=amb, Q=Q, V=V)

        mission = (prop.get("missions") or [{}])[0]
        phases = [phase("ground soak (sun, avionics on)", "soak", an.get("ground_soak_s", 900.0))]
        for i, s in enumerate(mission.get("segments") or []):
            if s.get("time_s"):
                phases.append(phase(f"{i+1} {s['kind'].split(' (')[0]}", s["kind"], s["time_s"]))
        phases.append(phase("post-landing soak", "soak", an.get("post_land_soak_s", 300.0)))
        return net, phases, T_amb, phase

    def simulate(scale_q, scale_h, env):
        """A hot small part cools better than the 10 W/m2K starting value (natural convection ~ dT^0.25, radiation
        ~ T^3): three fixed-point passes on the part-to-air coefficient, from the peak temperature difference."""
        hmap = {}
        for _ in range(4):
            net, phases, T_amb, phase = build(scale_q, scale_h, env, hmap)
            t, T, ph = net.simulate(phases, [T_amb] * len(net.names), dt=2.0)
            new = {}
            for c in internal:
                i, ia = net.idx(c["name"]), net.idx(f"air:{bay_of(c)}")
                k = int(T[:, i].argmax())
                dT = max(float(T[k, i] - T[k, ia]), 1.0)
                Lc = max(min(c.get("dims") or [0.03] * 3), 0.01) if c.get("dims") else 0.03
                Tp, Ta = T[k, i] + 273.15, T[k, ia] + 273.15
                h = 1.32 * (dT / max(Lc * 3, 0.02)) ** 0.25 + 0.8 * 5.67e-8 * (Tp ** 2 + Ta ** 2) * (Tp + Ta)
                new[c["name"]] = float(min(max(h, 6.0), 40.0))
            if hmap and max(abs(new[k_] - hmap[k_]) for k_ in new) < 0.3:
                hmap = new
                break
            hmap = new
        return net, phases, t, T, ph, phase, hmap

    runs = {}
    for tag, sq, sh, env in (("hot_nominal", 1.0, 1.0, hot), ("hot_pessimistic", 1.25, 0.7, hot), ("cold_nominal", 1.0, 1.0, cold)):
        runs[tag] = simulate(sq, sh, env)

    net, phases, t, T, ph, phase_fn, hmap_n = runs["hot_nominal"]
    netp, _, tp, Tp, php, phase_fn_p, _ = runs["hot_pessimistic"]
    rows, cands = [], []
    for c in internal + [next(x for x in comps if x["name"] == mn) for mn in sorted(motors)]:
        nm = c["name"]
        lim, src = limit_of(c, nm in motors)
        i = net.idx(nm)
        pk, pkp = float(T[:, i].max()), float(Tp[:, i].max())
        when = phases[int(ph[int(T[:, i].argmax())])]["name"]
        rows.append(dict(part=nm, limit_C=lim, limit_source=src, peak_nominal_C=round(pk, 1), peak_pessimistic_C=round(pkp, 1),
                         hottest_phase=when, margin_pess_K=round(lim - pkp, 1)))
        sev_note = f"hottest in '{when}'; pessimistic network {pkp:.0f} C; limit from {src}"
        ctx.gate(f"{nm} peak temperature in '{hot['name']}' [C]", round(pk, 1), lim, "<=", "FAIL", "C", note=sev_note,
                 drivers=[f"/components/{nm}", "/environment_cases"])
        if pk <= lim and pkp > lim:
            ctx.gate(f"{nm} peak temperature, pessimistic network [C]", round(pkp, 1), lim, "<=", "WARN", "C",
                     note="losses x1.25, convection x0.7, sun x1.3: inside the uncertainty of a lumped model")
        if lim - pkp < 10.0:
            cands.append(nm)

    # ---- printed structure --------------------------------------------------------------------------
    struct_rows = []
    if tg:
        for bn in used_bays:
            ia = net.idx(f"air:{bn}")
            pk, pkp = float(T[:, ia].max()), float(Tp[:, ia].max())
            struct_rows.append(dict(bay=bn, air_peak_nominal_C=round(pk, 1), air_peak_pessimistic_C=round(pkp, 1)))
            ctx.gate(f"air in '{bn.strip('_')}' vs printed-skin softening (Tg {tg:g} C - 10) [C]", round(pk, 1), tg - 10, "<=", "FAIL", "C",
                     note=f"pessimistic {pkp:.0f} C. The print around the bay sees this temperature on its inside face",
                     drivers=[f"/materials/{skin.get('material')}/tg_C", "/environment_cases"])
            if pk <= tg - 10 < pkp:
                ctx.gate(f"air in '{bn.strip('_')}' vs printed-skin softening, pessimistic network [C]", round(pkp, 1), tg - 10, "<=", "WARN", "C")
        hot_parts = [r for r in rows if r["part"] not in motors and r["peak_nominal_C"] > tg - 5]
        ctx.gate(f"parts hotter than the print can touch (Tg {tg:g} C - 5)", len(hot_parts), 0, "<=", "WARN",
                 note=", ".join(f"{r['part']} {r['peak_nominal_C']:.0f} C" for r in hot_parts[:6])
                      + (". Mount these on standoffs or a PETG/ASA tray, not directly on LW-PLA" if hot_parts else ""))
    mount_mat = an.get("motor_mount_material") or ("petg" if "petg" in plan["materials"] else skin.get("material"))
    tg_m = (plan["materials"].get(mount_mat) or {}).get("tg_C")
    if tg_m and motors:
        T_amb = float(hot.get("temperature_C", 15))
        worst = max(((mn, T_amb + 0.6 * (float(T[:, net.idx(mn)].max()) - T_amb)) for mn in motors), key=lambda x: x[1])
        ctx.gate(f"motor mount temperature (estimate) vs mount material '{mount_mat}' Tg {tg_m:g} C - 10 [C]", round(worst[1], 1), tg_m - 10,
                 "<=", "WARN", "C", note=f"worst: {worst[0]}; mount assumed at 60 % of the motor's rise above ambient. "
                 "Set analysis.thermal.motor_mount_material if the mounts are not this material", drivers=["/analysis/thermal/motor_mount_material"])

    # ---- sustained hover from the end-of-cruise state -------------------------------------------------
    hover_res = None
    vt_idx = [k for k, p_ in enumerate(phases) if "vtol" in p_["kind"].lower()]
    if vt_idx:
        k_land = vt_idx[-1]
        T_start = Tp[np.where(php == k_land)[0][0] - 1]
        hp = phase_fn_p("sustained hover", phases[k_land]["kind"], an.get("sustained_hover_s", 600.0))
        th_, Th_, _ = netp.simulate([hp], T_start, dt=2.0)
        first = None
        hover_heated = set(batt) | {n_ for n_ in hp["Q"] if n_ in motors or n_ in escs}
        for r in rows:
            if r["part"] not in hover_heated:
                continue
            j = netp.idx(r["part"])
            over = np.where(Th_[:, j] > r["limit_C"])[0]
            if len(over) and (first is None or th_[over[0]] < first[1]):
                first = (r["part"], float(th_[over[0]]))
        planned = sum(phases[k]["duration"] for k in vt_idx)
        hover_res = dict(first_limit=first, planned_hover_s=planned, simulated_s=float(th_[-1]))
        ctx.gate("continuous hover before the battery, a lift ESC or a lift motor reaches its limit [s] (pessimistic, after cruise)",
                 first[1] if first else float(th_[-1]), 2.0 * max(phases[k_land]["duration"], 1.0), ">=", "WARN", "s",
                 note=(f"first: {first[0]}" if first else f"no limit reached in {th_[-1]:.0f} s")
                      + f"; the mission plans {phases[k_land]['duration']:.0f} s of hover for the landing. TEAM CRITERION: 2x margin")

    # ---- cold case ----------------------------------------------------------------------------------
    netc, phc, tc, Tc, phc_i, _, _ = runs["cold_nominal"]
    if batt and cold.get("temperature_C", 15) < 15:
        ib = netc.idx(batt[0])
        k_to = np.where(phc_i == 1)[0][0] if (phc_i == 1).any() else 0
        ctx.gate(f"battery temperature at take-off in '{cold['name']}' [C]", round(float(Tc[k_to, ib]), 1), an.get("battery_min_C", 10.0),
                 ">=", "WARN", "C", note="Li-ion internal resistance roughly doubles near 0 C: less hover thrust margin and early "
                                         "low-voltage failsafe. Keep the pack warm until launch")

    ctx.gate("parts within 10 K of a limit in the pessimistic network (candidates for detailed CFD / thermocouple test)", len(cands), 0,
             "info", "INFO", note=", ".join(cands) or "none")

    # ---- what to do about it: vent sizing for forward flight, advice for hover / ground --------------
    rec, advice_sink = [], []
    T_amb_h = float(hot.get("temperature_C", 15))
    rho_h = rho_at(T_amb_h, hot.get("altitude_msl", 0))
    vc = plan["envelope"]["v_cruise"]
    cruise_ph = next((p_ for p_ in phases if any(s_ in p_["kind"].lower() for s_ in ("cruise", "loiter"))), None)
    for bn in used_bays:
        members = [c for c in internal if bay_of(c) == bn]
        if not members or cruise_ph is None:
            continue
        g = bay_geo[bn]
        Qb = sum(cruise_ph["Q"].get(c["name"], 0.0) for c in members)
        g_wall = next(gl for a_, b_, gl in cruise_ph["links"] if a_ == f"air:{bn}")
        g_ext = next(gl for a_, gl in cruise_ph["amb"] if a_ == f"skin:{bn}")
        g_tot = 1.0 / (1.0 / g_wall + 1.0 / g_ext)
        sun = cruise_ph["Q"].get(f"skin:{bn}", 0.0) / g_ext
        self_rise = {c["name"]: cruise_ph["Q"].get(c["name"], 0.0) / (hmap_n.get(c["name"], H_PART) * area_of(c)) for c in members}
        needs_sink = sorted(n_ for n_, dT_ in self_rise.items()
                            if dT_ > limit_of(next(c for c in members if c["name"] == n_))[0] - T_amb_h - 5.0)
        ok_parts = [c for c in members if c["name"] not in needs_sink]
        allow_parts = min((limit_of(c)[0] - self_rise[c["name"]] for c in ok_parts), default=None)
        air_now = T_amb_h + sun + Qb / g_tot
        row = dict(bay=bn.strip("_"), heat_in_cruise_W=round(Qb, 1), bay_air_cruise_C=round(air_now, 1),
                   air_limit_for_parts_C=None if allow_parts is None else round(allow_parts, 1),
                   air_limit_for_print_C=None if not tg else tg - 10, needs_own_heat_sink=needs_sink)
        if needs_sink:
            advice_sink.append(f"{', '.join(needs_sink)}: runs {max(self_rise[n_] for n_ in needs_sink):.0f} K above the air around it on "
                               "its own heat. No bay vent fixes that: give it a heat sink in moving air or mount it outside the bay")
        for key, lim_air in (("vent_area_for_parts_cm2", allow_parts), ("vent_area_for_print_cm2", (tg - 10) if tg else None)):
            if lim_air is None:
                continue
            if air_now <= lim_air:
                row[key] = 0.0
            elif lim_air - T_amb_h <= 1.0:
                row[key] = None
                row[key.replace("_cm2", "_note")] = (f"not fixable by ventilation: the limit ({lim_air:.0f} C) is at or below the "
                                                     f"{T_amb_h:.0f} C ambient of this case. Use a higher-temperature material or part")
            else:
                g_need = Qb / (lim_air - T_amb_h) - g_tot
                row[key] = round(max(g_need, 0.0) / (rho_h * TH.AIR_CP * 0.3 * vc) * 1e4, 2)
        rec.append(row)
    advice = list(advice_sink)
    for r in rows:
        if r["part"] in motors or r["margin_pess_K"] >= 0:
            continue
        ph_ = r["hottest_phase"].lower()
        if "soak" in ph_:
            advice.append(f"{r['part']}: peaks on the ground with no airflow. Shade the aircraft, keep ground time powered-up short, "
                          "or give the part a heat path to the outside (aluminium plate through the skin)")
        elif any(k_ in ph_ for k_ in ("launch", "land", "vtol")):
            advice.append(f"{r['part']}: peaks while hovering, where a ram-air vent does nothing. Heat-sink it into the lift-prop "
                          "wash, shorten the hover, or move it out of the closed bay")
        else:
            advice.append(f"{r['part']}: peaks in forward flight. Add a ram-air inlet and an exit to its bay "
                          "(analysis.thermal.vents: {bay: area_m2}) and re-run")
    for a_ in advice:
        ctx.note(a_)

    # ---- figures ------------------------------------------------------------------------------------
    plt = style()
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.2), gridspec_kw=dict(width_ratios=[1.6, 1]))
    show = sorted(rows, key=lambda r: r["margin_pess_K"])[:7]
    for j, r in enumerate(show):
        ax[0].plot(t / 60, T[:, net.idx(r["part"])], color=PALETTE[j % len(PALETTE)], label=r["part"])
    for bn in used_bays:
        ax[0].plot(t / 60, T[:, net.idx(f"air:{bn}")], color=INK2, lw=0.9, ls=":")
    edges = np.cumsum([0] + [p_["duration"] for p_ in phases]) / 60
    for k, p_ in enumerate(phases):
        if "vtol" in p_["kind"].lower() or "soak" in p_["kind"].lower():
            ax[0].axvspan(edges[k], edges[k + 1], color="#eda100" if "vtol" in p_["kind"].lower() else "#8a8984", alpha=0.12, lw=0)
    if tg:
        ax[0].axhline(tg - 10, color="#e34948", lw=1, ls="--")
        ax[0].text(0.5 * edges[-1], tg - 10, f"print softening limit, Tg - 10 = {tg-10:g} C", va="bottom", ha="center", fontsize=7,
                   color="#e34948", bbox=dict(facecolor="#fcfcfb", edgecolor="none", pad=1))
    ax[0].set(xlabel="time [min]", ylabel="temperature [C]",
              title=f"'{hot['name']}' ({hot.get('temperature_C')} C + sun), nominal network. Dotted: bay air; amber: hover; grey: soak")
    ax[0].legend(fontsize=7, ncol=2)
    yy = np.arange(len(rows))
    ax[1].barh(yy, [r["peak_pessimistic_C"] for r in rows], color="#eda100", height=0.6, label="pessimistic")
    ax[1].barh(yy, [r["peak_nominal_C"] for r in rows], color=PALETTE[0], height=0.6, label="nominal")
    ax[1].scatter([r["limit_C"] for r in rows], yy, marker="|", s=220, color="#e34948", zorder=5, label="limit")
    ax[1].set_yticks(yy, [r["part"] for r in rows], fontsize=7)
    ax[1].set(xlabel="peak temperature [C]", title="Peak vs limit"); ax[1].yaxis.grid(False); ax[1].legend(fontsize=7)
    save(fig, ctx.dir / "thermal_timeline.png")
    ctx.add_file(ctx.dir / "thermal_timeline.png", "Component temperatures over the mission (hottest case)", "figure")

    data = dict(hot_case=hot["name"], cold_case=cold["name"], parts=rows, bays=struct_rows, cfd_candidates=cands,
                sustained_hover=hover_res, vent_sizing=rec, advice=advice, part_to_air_h=hmap_n, motor_map=motor_of, esc_map=esc_of,
                heat_sources_W={p_["name"]: {k: round(v, 2) for k, v in p_["Q"].items() if v > 0.01} for p_ in phases},
                assumptions=dict(h_part=H_PART, h_wall=H_WALL, h_rest_ext=H_REST_EXT, k_print=K_PRINT,
                                 pessimistic="losses x1.25, internal convection x0.7, forced external x0.8, sun x1.3",
                                 skin_thickness_m=t_skin, vents=an.get("vents") or {}))
    hottest = min(rows, key=lambda r: r["margin_pess_K"]) if rows else None
    summ = dict(hot_case=hot["name"], parts_modelled=len(rows), over_limit_nominal=sum(r["peak_nominal_C"] > r["limit_C"] for r in rows),
                over_limit_pessimistic=sum(r["margin_pess_K"] < 0 for r in rows),
                tightest_part=hottest and f"{hottest['part']} ({hottest['peak_nominal_C']:.0f} C nominal / {hottest['peak_pessimistic_C']:.0f} C pessimistic vs {hottest['limit_C']:.0f} C)",
                hottest_bay_air_C=max((b["air_peak_nominal_C"] for b in struct_rows), default=None), cfd_candidates=len(cands))
    return dict(summary=summ, data=data, tables=dict(parts=rows, vent_sizing=rec))
