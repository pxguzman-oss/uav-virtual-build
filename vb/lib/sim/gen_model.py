#!/usr/bin/env python3
"""
gen_model.py - turn a DroneLab aircraft-model JSON (schema/aircraft_model.schema.json)
into simulator back-end models:

  <out>/jsbsim/aircraft/<name>/<name>.xml   JSBSim FDM (aero tables, mass/inertia, contacts,
                                            actuators, external-force propulsion hooks)
  <out>/<name>_aero_tables.csv              the synthesised/merged CL/CD/Cm(alpha) tables
  <out>/<name>_simplane.json                fallback coefficients for ArduPilot's built-in
                                            SIM_Plane (--model plane-vtail:<file>) -- see notes
  <out>/<name>_model.parm                   ArduPilot params that must agree with the model
                                            (SERVOn_FUNCTION, BATT_CAPACITY, AIRSPEED_* ...)
  <out>/<name>_static_perf.json             quick static-performance numbers (trim, Vs, L/D, Pmin)

Usage:  gen_model.py models/stork_p0.json -o generated/stork_p0
"""
import argparse, json, math, os, sys
import numpy as np

try:
    import jsonschema
except ImportError:  # pragma: no cover
    jsonschema = None

HERE = os.path.dirname(os.path.abspath(__file__))
SCHEMA = os.path.join(HERE, "..", "schema", "aircraft_model.schema.json")
RHO0 = 1.225
G = 9.80665
FT = 0.3048
LBF = 4.4482216


# --------------------------------------------------------------------------- aero tables
def synth_alpha_tables(geo, d):
    """Build CL/CD/Cm vs alpha (deg, -180..180) from derivatives + a simple stall model,
    blending to flat-plate behaviour at large |alpha|."""
    AR = geo["span_m"] ** 2 / geo["wing_area_m2"]
    k = 1.0 / (math.pi * d.get("oswald", 0.8) * AR)
    CL0, CLa = d["CL0"], d["CLa"]
    CLmax = d.get("CLmax", 1.2)
    a_st = math.radians(d.get("alpha_stall_deg", 12))
    drop = d.get("CL_post_stall_drop", 0.3)
    CLmin = -0.7 * CLmax
    a_st_neg = (CLmin - CL0) / CLa - math.radians(2)
    CD0 = d["CD0"]
    Cm0, Cma = d["Cm0"], d["Cma"]

    alphas = sorted(set(list(np.arange(-180, -30, 10)) + list(np.arange(-30, 30.5, 1.0)) +
                    list(np.arange(30, 181, 10))))
    CL, CD, Cm = [], [], []
    for adeg in alphas:
        a = math.radians(adeg)
        lin = CL0 + CLa * a
        # pre-stall with smooth round-off over the last ~4 deg
        a_knee = a_st - math.radians(4)
        if a <= a_knee and a >= a_st_neg:
            cl_att = lin
        elif a > a_knee:
            cl_knee = CL0 + CLa * a_knee
            if a <= a_st:
                t = (a - a_knee) / (a_st - a_knee)
                cl_att = cl_knee + (CLmax - cl_knee) * (2 * t - t * t)
            else:
                # post stall: drop then decay
                t = min((a - a_st) / math.radians(4), 1.0)
                cl_att = CLmax - drop * t
        else:
            cl_att = max(lin, CLmin) if a > a_st_neg - math.radians(4) else CLmin + 0.3 * (a - a_st_neg)
        cl_fp = 2 * math.sin(a) * math.cos(a)  # flat plate
        # blend weight: 0 attached (|a|<stall), 1 fully separated beyond ~ stall+15 deg
        if a >= 0:
            w = np.clip((a - a_st) / math.radians(15), 0, 1)
        else:
            w = np.clip((a_st_neg - a) / math.radians(15), 0, 1)
        if a > a_st + math.radians(4) or a < a_st_neg - math.radians(4):
            cl = (1 - w) * cl_att + w * cl_fp * (0.9 if a > 0 else 0.8)
        else:
            cl = cl_att
        cd_att = CD0 + k * (min(max(lin, CLmin), CLmax)) ** 2
        if a > a_st:  # separated-flow drag rise just past stall
            cd_att += 0.8 * (a - a_st)
        cd_fp = CD0 + 1.25 * math.sin(a) ** 2
        cd = (1 - w) * cd_att + w * cd_fp
        cd = max(cd, cd_att)
        cm_att = Cm0 + Cma * a
        cm_fp = Cm0 - 0.6 * math.sin(a) * (1 if abs(adeg) <= 90 else -1) - 0.0
        # mild nose-down break after stall
        if a > a_st:
            cm_att += -0.05 * min((a - a_st) / math.radians(5), 1.0)
        cm = (1 - w) * cm_att + w * cm_fp
        CL.append(round(cl, 5)); CD.append(round(cd, 5)); Cm.append(round(cm, 5))
    return alphas, CL, CD, Cm


def merge_tables(geo, aero):
    d = aero.get("derivatives", {})
    if d:
        a, CL, CD, Cm = synth_alpha_tables(geo, d)
    else:
        a, CL, CD, Cm = [], [], [], []
    t = aero.get("alpha_tables")
    if t:
        # supplied data wins inside its range; synthesised flat-plate outside
        ta = np.array(t["alpha_deg"], float)
        lo, hi = ta.min(), ta.max()
        if not a:
            a = sorted(set(list(np.arange(-180, lo, 10)) + list(ta) + list(np.arange(hi + 10, 181, 10))))
            out = []
            for adeg in a:
                r = math.radians(adeg)
                out.append((2 * math.sin(r) * math.cos(r) * 0.9, 0.05 + 1.25 * math.sin(r) ** 2, -0.6 * math.sin(r)))
            CL, CD, Cm = [o[0] for o in out], [o[1] for o in out], [o[2] for o in out]
        aa = sorted(set([x for x in a if x < lo or x > hi] + list(ta)))
        CLn, CDn, Cmn = [], [], []
        for x in aa:
            if lo <= x <= hi:
                CLn.append(float(np.interp(x, ta, t["CL"]))); CDn.append(float(np.interp(x, ta, t["CD"])));
                Cmn.append(float(np.interp(x, ta, t["Cm"])))
            else:
                CLn.append(float(np.interp(x, a, CL))); CDn.append(float(np.interp(x, a, CD)));
                Cmn.append(float(np.interp(x, a, Cm)))
        a, CL, CD, Cm = aa, CLn, CDn, Cmn
    return a, CL, CD, Cm


# --------------------------------------------------------------------------- JSBSim XML
def x3(tag, v, unit="M"):
    return (f'<{tag} unit="{unit}"><x>{v[0]:.5f}</x><y>{v[1]:.5f}</y><z>{v[2]:.5f}</z></{tag}>')


def loc(name, v, unit="M"):
    return (f'<location name="{name}" unit="{unit}"><x>{v[0]:.5f}</x><y>{v[1]:.5f}</y><z>{v[2]:.5f}</z></location>')


def table1d(var, xs, ys):
    rows = "\n".join(f"            {x:10.6f} {y:10.6f}" for x, y in zip(xs, ys))
    return (f"<table>\n          <independentVar lookup=\"row\">{var}</independentVar>\n"
            f"          <tableData>\n{rows}\n          </tableData>\n        </table>")


def func(name, desc, factors, tbl=None, const=None):
    inner = "".join(f"<property>{f}</property>" for f in factors)
    if tbl is not None:
        inner += "\n        " + tbl
    if const is not None:
        inner += f"<value>{const:.6g}</value>"
    return (f'      <function name="{name}">\n        <description>{desc}</description>\n'
            f'        <product>{inner}</product>\n      </function>\n')


def jsbsim_xml(m, tables):
    geo, mass, aero = m["geometry"], m["mass"], m["aero"]
    d = aero.get("derivatives", {})
    name = m["name"]
    a_deg, CL, CD, Cm = tables
    a_rad = [math.radians(x) for x in a_deg]
    I = mass["inertia_kgm2"]
    out = []
    w = out.append
    w('<?xml version="1.0"?>')
    w(f'<fdm_config name="{name}" version="2.0" release="ALPHA">')
    w('  <fileheader><author>DroneLab gen_model.py</author>'
      f'<description>Auto-generated from {name} model JSON. Do not hand edit.</description></fileheader>')
    w('  <metrics>')
    w(f'    <wingarea unit="M2"> {geo["wing_area_m2"]} </wingarea>')
    w(f'    <wingspan unit="M"> {geo["span_m"]} </wingspan>')
    w(f'    <chord unit="M"> {geo["mac_m"]} </chord>')
    w(f'    <htailarea unit="M2"> {geo.get("htail_area_m2", 0.0)} </htailarea><htailarm unit="M"> 0 </htailarm>')
    w(f'    <vtailarea unit="M2"> {geo.get("vtail_area_m2", 0.0)} </vtailarea><vtailarm unit="M"> 0 </vtailarm>')
    w('    ' + loc("AERORP", aero["moment_ref_m"]))
    w('    ' + loc("EYEPOINT", mass["cg_m"]))
    w('    ' + loc("VRP", [0, 0, 0]))
    w('  </metrics>')
    w('  <mass_balance>')
    w(f'    <ixx unit="KG*M2"> {I["Ixx"]} </ixx><iyy unit="KG*M2"> {I["Iyy"]} </iyy><izz unit="KG*M2"> {I["Izz"]} </izz>')
    w(f'    <ixz unit="KG*M2"> {I.get("Ixz", 0)} </ixz><ixy unit="KG*M2"> {I.get("Ixy", 0)} </ixy><iyz unit="KG*M2"> {I.get("Iyz", 0)} </iyz>')
    w(f'    <emptywt unit="KG"> {mass["mass_kg"]} </emptywt>')
    w('    ' + loc("CG", mass["cg_m"]))
    w('  </mass_balance>')
    # ground contacts
    w('  <ground_reactions>')
    for c in m.get("ground", {}).get("contacts", []):
        w(f'    <contact type="STRUCTURE" name="{c["name"]}">')
        w('      ' + loc("", c["position_m"]).replace(' name=""', ''))
        w(f'      <static_friction> {c.get("static_friction", 0.6)} </static_friction>')
        w(f'      <dynamic_friction> {c.get("dynamic_friction", 0.5)} </dynamic_friction>')
        w(f'      <spring_coeff unit="N/M"> {c.get("spring_n_per_m", 3000)} </spring_coeff>')
        w(f'      <damping_coeff unit="N/M/SEC"> {c.get("damping_n_s_per_m", 60)} </damping_coeff>')
        w('    </contact>')
    w('  </ground_reactions>')
    w('  <propulsion/>')
    # external reactions: one force + one torque per motor, plus launch & hand support
    w('  <external_reactions>')
    for mo in m["propulsion"]["motors"]:
        dvec = mo["thrust_dir_body"]
        w(f'    <force name="{mo["name"]}" frame="BODY" unit="LBS">')
        w('      ' + x3("location", mo["position_m"]))
        w(f'      <direction><x>{dvec[0]}</x><y>{dvec[1]}</y><z>{dvec[2]}</z></direction>')
        w('    </force>')
        w(f'    <moment name="{mo["name"]}_torque" frame="BODY" unit="LBSFT">')
        w(f'      <direction><x>{dvec[0]}</x><y>{dvec[1]}</y><z>{dvec[2]}</z></direction>')
        w('    </moment>')
    w('    <force name="launch" frame="BODY" unit="LBS">')
    w('      ' + x3("location", mass["cg_m"]))
    w('      <direction><x>1</x><y>0</y><z>0</z></direction>')
    w('    </force>')
    w('    <force name="hand" frame="LOCAL" unit="LBS">')
    w('      ' + x3("location", mass["cg_m"]))
    w('      <direction><x>0</x><y>0</y><z>-1</z></direction>')
    w('    </force>')
    w('  </external_reactions>')
    # flight controls: one actuator per physical surface, command set in rad by the bridge
    w('  <flight_control name="FCS">')
    for s in m["surfaces"]:
        w(f'    <property value="0">fcs/{s["name"]}-cmd-rad</property>')
    for s in m["surfaces"]:
        n = s["name"]
        mx = math.radians(s["max_deflection_deg"])
        mn = -math.radians(s.get("min_deflection_deg", s["max_deflection_deg"]))
        w(f'    <channel name="{n}">')
        w(f'      <actuator name="fcs/{n}-actuator">')
        w(f'        <input>fcs/{n}-cmd-rad</input>')
        w(f'        <lag> {1.0 / max(s.get("lag_s", 0.02), 1e-3):.3f} </lag>')
        w(f'        <rate_limit> {math.radians(s.get("rate_limit_deg_s", 300)):.4f} </rate_limit>')
        w(f'        <clipto><min> {mn:.5f} </min><max> {mx:.5f} </max></clipto>')
        w(f'        <output>fcs/{n}-pos-rad</output>')
        w('      </actuator>')
        w('    </channel>')
    w('  </flight_control>')
    # aerodynamics
    w('  <aerodynamics>')
    w('    <alphalimits unit="DEG"><min>-180</min><max>180</max></alphalimits>')
    w('    <function name="aero/beta-clipped-rad"><description>beta clipped +-25deg for linear terms</description>'
      '<table><independentVar lookup="row">aero/beta-rad</independentVar><tableData>'
      ' -3.1416 -0.4363\n -0.4363 -0.4363\n 0.4363 0.4363\n 3.1416 0.4363 </tableData></table></function>')
    qS = ["aero/qbar-psf", "metrics/Sw-sqft"]
    qSb = qS + ["metrics/bw-ft"]
    qSc = qS + ["metrics/cbarw-ft"]
    tCL = table1d("aero/alpha-rad", a_rad, CL)
    tCD = table1d("aero/alpha-rad", a_rad, CD)
    tCm = table1d("aero/alpha-rad", a_rad, Cm)
    # LIFT
    w('    <axis name="LIFT">')
    w(func("aero/coefficient/CL_alpha", "Lift vs alpha (table)", qS, tbl=tCL))
    if d.get("CLq"):
        w(func("aero/coefficient/CL_q", "Lift due to pitch rate", qS + ["aero/ci2vel", "velocities/q-aero-rad_sec"], const=d["CLq"]))
    for s in m["surfaces"]:
        v = s["derivatives_per_rad"].get("CL", 0)
        if v:
            w(func(f'aero/coefficient/CL_{s["name"]}', f'Lift due to {s["name"]}', qS + [f'fcs/{s["name"]}-pos-rad'], const=v))
    w('    </axis>')
    # DRAG
    w('    <axis name="DRAG">')
    w(func("aero/coefficient/CD_alpha", "Drag vs alpha (table, incl. induced)", qS, tbl=tCD))
    w(func("aero/coefficient/CD_beta", "Drag due to sideslip", qS + ["aero/beta-rad", "aero/beta-rad"], const=d.get("CD_beta2", 0.3)))
    for s in m["surfaces"]:
        v = s["derivatives_per_rad"].get("CD_abs", 0)
        if v:
            w(f'      <function name="aero/coefficient/CD_{s["name"]}"><description>drag due to |{s["name"]}|</description>'
              f'<product><property>aero/qbar-psf</property><property>metrics/Sw-sqft</property>'
              f'<abs><property>fcs/{s["name"]}-pos-rad</property></abs><value>{v}</value></product></function>')
    w('    </axis>')
    # SIDE
    w('    <axis name="SIDE">')
    w(func("aero/coefficient/CY_beta", "Side force due to beta", qS + ["aero/beta-clipped-rad"], const=d.get("CYb", -0.3)))
    if d.get("CYp"):
        w(func("aero/coefficient/CY_p", "Side force due to roll rate", qS + ["aero/bi2vel", "velocities/p-aero-rad_sec"], const=d["CYp"]))
    if d.get("CYr"):
        w(func("aero/coefficient/CY_r", "Side force due to yaw rate", qS + ["aero/bi2vel", "velocities/r-aero-rad_sec"], const=d["CYr"]))
    for s in m["surfaces"]:
        v = s["derivatives_per_rad"].get("CY", 0)
        if v:
            w(func(f'aero/coefficient/CY_{s["name"]}', f'Side force due to {s["name"]}', qS + [f'fcs/{s["name"]}-pos-rad'], const=v))
    w('    </axis>')
    # ROLL
    w('    <axis name="ROLL">')
    w(func("aero/coefficient/Cl_beta", "Roll due to beta (dihedral effect)", qSb + ["aero/beta-clipped-rad"], const=d.get("Clb", -0.05)))
    w(func("aero/coefficient/Cl_p", "Roll damping", qSb + ["aero/bi2vel", "velocities/p-aero-rad_sec"], const=d.get("Clp", -0.45)))
    w(func("aero/coefficient/Cl_r", "Roll due to yaw rate", qSb + ["aero/bi2vel", "velocities/r-aero-rad_sec"], const=d.get("Clr", 0.1)))
    for s in m["surfaces"]:
        v = s["derivatives_per_rad"].get("Cl", 0)
        if v:
            w(func(f'aero/coefficient/Cl_{s["name"]}', f'Roll due to {s["name"]}', qSb + [f'fcs/{s["name"]}-pos-rad'], const=v))
    w('    </axis>')
    # PITCH
    w('    <axis name="PITCH">')
    w(func("aero/coefficient/Cm_alpha", "Pitch moment vs alpha (table)", qSc, tbl=tCm))
    w(func("aero/coefficient/Cm_q", "Pitch damping", qSc + ["aero/ci2vel", "velocities/q-aero-rad_sec"], const=d.get("Cmq", -12)))
    if d.get("Cmadot"):
        w(func("aero/coefficient/Cm_adot", "Pitch due to alpha rate", qSc + ["aero/ci2vel", "aero/alphadot-rad_sec"], const=d["Cmadot"]))
    for s in m["surfaces"]:
        v = s["derivatives_per_rad"].get("Cm", 0)
        if v:
            w(func(f'aero/coefficient/Cm_{s["name"]}', f'Pitch due to {s["name"]}', qSc + [f'fcs/{s["name"]}-pos-rad'], const=v))
    w('    </axis>')
    # YAW
    w('    <axis name="YAW">')
    w(func("aero/coefficient/Cn_beta", "Weathercock stability", qSb + ["aero/beta-clipped-rad"], const=d.get("Cnb", 0.06)))
    w(func("aero/coefficient/Cn_p", "Yaw due to roll rate", qSb + ["aero/bi2vel", "velocities/p-aero-rad_sec"], const=d.get("Cnp", -0.03)))
    w(func("aero/coefficient/Cn_r", "Yaw damping", qSb + ["aero/bi2vel", "velocities/r-aero-rad_sec"], const=d.get("Cnr", -0.08)))
    for s in m["surfaces"]:
        v = s["derivatives_per_rad"].get("Cn", 0)
        if v:
            w(func(f'aero/coefficient/Cn_{s["name"]}', f'Yaw due to {s["name"]}', qSb + [f'fcs/{s["name"]}-pos-rad'], const=v))
    w('    </axis>')
    w('  </aerodynamics>')
    w('</fdm_config>')
    return "\n".join(out) + "\n"


# --------------------------------------------------------------------------- SIM_Plane fallback
def simplane_json(m, tables):
    """ArduPilot SIM_Plane (libraries/SITL/SIM_Plane.cpp) hard-codes mass=2.0 kg, treats the
    aero moment (N m) directly as angular acceleration (i.e. unit inertia), and uses a thrust
    that is linear in throttle (2 kg*g/0.7 at full).  We pre-scale coefficients so that
    translational and angular ACCELERATIONS match our aircraft.  Propulsion/battery are NOT
    representative -> use only for control-logic / mission-logic tests."""
    geo, mass, aero = m["geometry"], m["mass"], m["aero"]
    d = aero.get("derivatives", {})
    I = mass["inertia_kgm2"]
    S, b, c = geo["wing_area_m2"], geo["span_m"], geo["mac_m"]
    AR = b * b / S
    kmass = 2.0 / mass["mass_kg"]
    S2 = S * kmass
    b2 = math.sqrt(AR * S2)
    fl = S * b / (S2 * b2 * I["Ixx"])
    fm = S * c / (S2 * c * I["Iyy"])
    fn = S * b / (S2 * b2 * I["Izz"])
    surf = m["surfaces"]

    def ctl(kind_names, coef):
        tot = 0.0
        for s in surf:
            if s.get("kind") in kind_names:
                tot += abs(s["derivatives_per_rad"].get(coef, 0)) * math.radians(s["max_deflection_deg"])
        return tot
    # SIM_Plane vtail decode gives elevator=(ch2-ch1)/2 -> half throw per surface at full stick
    c_l_da = ctl({"aileron", "flaperon", "elevon"}, "Cl")
    c_m_de = ctl({"elevator", "ruddervator", "elevon"}, "Cm") * (0.5 if any(s.get("kind") == "ruddervator" for s in surf) else 1)
    c_n_dr = ctl({"rudder", "ruddervator"}, "Cn") * (0.5 if any(s.get("kind") == "ruddervator" for s in surf) else 1)
    out = {
        "s": round(S2, 5), "b": round(b2, 5), "c": c,
        "c_lift_0": d.get("CL0", 0.3), "c_lift_a": d.get("CLa", 5.0), "c_lift_q": 0, "c_lift_deltae": 0,
        "mcoeff": 50, "oswald": d.get("oswald", 0.8),
        "alpha_stall": round(math.radians(d.get("alpha_stall_deg", 12)), 4),
        "c_drag_q": 0, "c_drag_deltae": 0, "c_drag_p": d.get("CD0", 0.03),
        "c_y_0": 0, "c_y_b": d.get("CYb", -0.3), "c_y_p": 0, "c_y_r": 0, "c_y_deltaa": 0, "c_y_deltar": -0.1,
        "c_l_0": 0,
        "c_l_p": round(d.get("Clp", -0.45) * fl * b / b2, 4), "c_l_b": round(d.get("Clb", -0.05) * fl, 4),
        "c_l_r": round(d.get("Clr", 0.1) * fl * b / b2, 4), "c_l_deltaa": round(c_l_da * fl, 4), "c_l_deltar": 0,
        "c_m_0": round(d.get("Cm0", 0.0) * fm, 4), "c_m_a": round(d.get("Cma", -0.5) * fm, 4),
        "c_m_q": round(d.get("Cmq", -12) * fm, 4), "c_m_deltae": round(c_m_de * fm, 4),
        "c_n_0": 0, "c_n_b": round(d.get("Cnb", 0.06) * fn, 4),
        "c_n_p": round(d.get("Cnp", -0.03) * fn * b / b2, 4), "c_n_r": round(d.get("Cnr", -0.08) * fn * b / b2, 4),
        "c_n_deltaa": 0, "c_n_deltar": round(c_n_dr * fn, 4),
        "deltaa_max": 0.3491, "deltae_max": 0.3491, "deltar_max": 0.3491,
        "CGOffset": [0.0, 0.0, 0.0],
    }
    return out


# --------------------------------------------------------------------------- ArduPilot params
SERVO_FN = {"aileron": 4, "elevator": 19, "rudder": 21, "flap": 2}


def ardupilot_parm(m):
    lines = [f"# AUTO-GENERATED by gen_model.py from model '{m['name']}' -- servo mapping and model-linked params"]
    used = {}
    for s in m["surfaces"]:
        ch = s["pwm"]["channel"]; k = s.get("kind", "other")
        if k == "ruddervator":
            fn = 79 if "left" in s["name"] else 80
        elif k == "elevon":
            fn = 77 if "left" in s["name"] else 78
        elif k == "flaperon":
            fn = 24 if "left" in s["name"] else 25
        else:
            fn = SERVO_FN.get(k, 0)
        used[ch] = fn
        lines.append(f"SERVO{ch}_FUNCTION {fn}   # {s['name']}")
        lines.append(f"SERVO{ch}_REVERSED 0")
        lines.append(f"SERVO{ch}_MIN {s['pwm'].get('min', 1000)}")
        lines.append(f"SERVO{ch}_TRIM {s['pwm'].get('trim', 1500)}")
        lines.append(f"SERVO{ch}_MAX {s['pwm'].get('max', 2000)}")
    lift_i = 0
    for mo in m["propulsion"]["motors"]:
        ch = mo["pwm"]["channel"]
        if mo.get("role", "forward") == "forward":
            fn = 70
        else:
            fn = 33 + lift_i; lift_i += 1
        used[ch] = fn
        lines.append(f"SERVO{ch}_FUNCTION {fn}   # motor {mo['name']}")
        lines.append(f"SERVO{ch}_MIN {mo.get('esc', {}).get('pwm_min', 1000)}")
        lines.append(f"SERVO{ch}_MAX {mo.get('esc', {}).get('pwm_max', 2000)}")
        lines.append(f"SERVO{ch}_TRIM {mo.get('esc', {}).get('pwm_min', 1000)}")
    for ch in range(1, 17):
        if ch not in used:
            lines.append(f"SERVO{ch}_FUNCTION 0")
    bat = m["battery"]
    lines.append(f"BATT_CAPACITY {int(bat['cells_parallel'] * bat['cell_capacity_ah'] * 1000)}")
    for k, v in m.get("ardupilot", {}).items():
        lines.append(f"{k} {v}")
    return "\n".join(lines) + "\n"


# --------------------------------------------------------------------------- static perf
def static_perf(m, tables):
    geo, mass = m["geometry"], m["mass"]
    a_deg, CL, CD, Cm = tables
    S = geo["wing_area_m2"]; W = mass["mass_kg"] * G
    a = np.array(a_deg); CLa = np.array(CL); CDa = np.array(CD)
    sel = (a > -10) & (a < 25)
    CLmax = float(CLa[sel].max())
    Vs = math.sqrt(2 * W / (RHO0 * S * CLmax))
    res = {"CLmax": round(CLmax, 3), "Vs_1g_mps": round(Vs, 2)}
    rows = []
    for V in np.arange(math.ceil(Vs * 1.05 * 2) / 2, 25.01, 0.5):
        cl = 2 * W / (RHO0 * V * V * S)
        # pre-stall branch
        sel2 = (a > -10) & (a <= a[sel][CLa[sel].argmax()])
        cd = float(np.interp(cl, CLa[sel2], CDa[sel2]))
        D = 0.5 * RHO0 * V * V * S * cd
        rows.append({"V": round(float(V), 2), "CL": round(cl, 3), "CD": round(cd, 4), "L/D": round(cl / cd, 2),
                     "D_N": round(D, 3), "P_aero_W": round(D * V, 2)})
    res["polar"] = rows
    best = max(rows, key=lambda r: r["L/D"]); pmin = min(rows, key=lambda r: r["P_aero_W"])
    res["V_best_LD"] = best["V"]; res["LD_max"] = best["L/D"]
    res["V_min_power"] = pmin["V"]; res["P_aero_min_W"] = pmin["P_aero_W"]
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("model")
    ap.add_argument("-o", "--out", default=None)
    args = ap.parse_args()
    m = json.load(open(args.model))
    if jsonschema is not None:
        jsonschema.validate(m, json.load(open(SCHEMA)))
    name = m["name"]
    out = args.out or os.path.join(HERE, "..", "generated", name)
    acdir = os.path.join(out, "jsbsim", "aircraft", name)
    os.makedirs(acdir, exist_ok=True)
    tables = merge_tables(m["geometry"], m["aero"])
    open(os.path.join(acdir, f"{name}.xml"), "w").write(jsbsim_xml(m, tables))
    with open(os.path.join(out, f"{name}_aero_tables.csv"), "w") as f:
        f.write("alpha_deg,CL,CD,Cm\n")
        for r in zip(*tables):
            f.write(",".join(f"{v:.5f}" for v in r) + "\n")
    with open(os.path.join(out, f"{name}_simplane.json"), "w") as f:
        f.write("# AUTO-GENERATED fallback for ArduPilot SIM_Plane (--model plane-vtail:<this file>).\n"
                "# Coefficients pre-scaled for SIM_Plane's fixed 2 kg mass and unit inertia; thrust and battery NOT realistic.\n")
        json.dump(simplane_json(m, tables), f, indent=1)
    open(os.path.join(out, f"{name}_model.parm"), "w").write(ardupilot_parm(m))
    sp = static_perf(m, tables)
    json.dump(sp, open(os.path.join(out, f"{name}_static_perf.json"), "w"), indent=1)
    # keep a copy of the source model next to the outputs (bridge reads it)
    json.dump(m, open(os.path.join(out, f"{name}.json"), "w"), indent=1)
    print(f"generated {out}: JSBSim XML, SIM_Plane json, parm; CLmax={sp['CLmax']} Vs={sp['Vs_1g_mps']} m/s "
          f"L/Dmax={sp['LD_max']}@{sp['V_best_LD']} m/s, Pmin_aero={sp['P_aero_min_W']} W@{sp['V_min_power']} m/s")


if __name__ == "__main__":
    main()
