"""Stage registry: dependencies, cache inputs, timeouts. Execution order = list order (topological).

deps   : hard dependencies (stage is BLOCKED if one of them did not finish ok)
soft   : optional inputs (used when present; the stage records a fallback otherwise)
keys   : plan JSON pointers hashed into the cache key
cheap  : always re-run (<~10 s), so the report never shows stale numbers (PIPELINE_DESIGN 6)
fid    : fidelity changes what the stage does (fidelity is part of the key)
code   : source files/dirs hashed into the key (code changes invalidate the cache)
"""

STAGES = [
    dict(name="ingest", deps=[], soft=[], cheap=True, keys=["/"], fid=False, timeout=300,
         code=["vb/stages/ingest.py", "vb/plan.py", "schema", "vb/lib/balance.py"]),
    dict(name="geometry", deps=["ingest"], soft=[], cheap=False, fid=True, timeout=2400,
         keys=["/airframe", "/components", "/materials", "/propulsion/units", "/analysis/cg_target_x", "/meta/name"],
         code=["vb/stages/geometry.py", "vb/adapters/plan2geom.py", "vb/lib/geom"]),
    dict(name="mass", deps=["ingest"], soft=["geometry"], cheap=True, fid=False, timeout=300,
         keys=["/components", "/materials", "/airframe", "/analysis"], code=["vb/stages/mass.py"]),
    dict(name="clash", deps=["geometry"], soft=["mass"], cheap=True, fid=False, timeout=900,
         keys=["/components", "/airframe", "/propulsion/units", "/gates"], code=["vb/stages/clash.py", "vb/lib/geom/fit_check.py"]),
    dict(name="aero_lowfi", deps=["geometry", "mass"], soft=[], cheap=False, fid=False, timeout=1800,
         keys=["/airframe", "/envelope", "/analysis/aero", "/analysis/static_margin_range", "/propulsion/units"],
         code=["vb/stages/aero_lowfi.py", "vb/adapters/geom2aero.py", "vb/lib/aero/stork_aero.py", "vb/liftmodel.py", "vb/flight_dyn.py", "vb/plan.py"]),
    dict(name="propulsion", deps=["mass"], soft=["aero_lowfi"], cheap=True, fid=False, timeout=600,
         keys=["/propulsion", "/energy", "/envelope", "/environment_cases", "/missions", "/analysis", "/gates"],
         code=["vb/stages/propulsion.py", "vb/lib/struct/propulsion.py", "data/props"]),
    dict(name="vtol", deps=["mass", "propulsion"], soft=["aero_lowfi"], cheap=True, fid=False, timeout=600,
         keys=["/propulsion", "/energy", "/envelope", "/environment_cases", "/airframe", "/analysis", "/avionics", "/gates"],
         code=["vb/stages/vtol.py", "vb/lib/struct/propulsion.py", "data/props"]),
    dict(name="thermal", deps=["mass", "propulsion"], soft=["vtol"], cheap=True, fid=False, timeout=600,
         keys=["/components", "/materials", "/airframe/fuselage", "/propulsion", "/energy", "/avionics", "/environment_cases",
               "/missions", "/analysis", "/gates"],
         code=["vb/stages/thermal.py", "vb/lib/thermal.py"]),
    dict(name="balance", deps=["mass"], soft=["clash", "aero_lowfi", "propulsion", "vtol"], cheap=True, fid=False, timeout=900,
         keys=["/"], code=["vb/stages/balance.py", "vb/lib/balance.py", "data/catalog"]),
    dict(name="structures", deps=["mass", "propulsion"], soft=["aero_lowfi"], cheap=True, fid=False, timeout=600,
         keys=["/airframe", "/materials", "/components", "/envelope", "/environment_cases", "/gates"],
         code=["vb/stages/structures.py", "vb/lib/struct/structures.py"]),
    dict(name="cfd", deps=["mass"], soft=["aero_lowfi"], cheap=False, fid=True, timeout=4 * 3600,
         keys=["/analysis/cfd", "/envelope/v_cruise"],
         code=["vb/stages/cfd.py", "vb/lib/aero/cfd_case.py", "vb/lib/aero/cfd_geom.py", "vb/lib/aero/stork_aero.py"]),
    dict(name="flightmodel", deps=["mass", "aero_lowfi", "propulsion"], soft=["structures", "cfd", "geometry"],
         cheap=True, fid=False, timeout=600,
         keys=["/avionics", "/energy", "/propulsion", "/airframe/launch", "/airframe/landing", "/envelope", "/airframe/lifting_surfaces"],
         code=["vb/stages/flightmodel.py", "vb/adapters/model2sim.py", "vb/lib/sim/gen_model.py", "data/ardupilot", "vb/liftmodel.py"]),
    dict(name="sitl", deps=["flightmodel"], soft=["propulsion"], cheap=False, fid=True, timeout=6 * 3600,
         keys=["/missions", "/environment_cases", "/analysis/sitl", "/envelope"],
         code=["vb/stages/sitl.py", "vb/lib/sim", "data/ardupilot"]),
    dict(name="report", deps=[], soft=["ingest", "geometry", "mass", "clash", "aero_lowfi", "propulsion", "structures",
                                       "cfd", "flightmodel", "sitl", "vtol", "thermal", "balance"], cheap=True, fid=True, timeout=900,
         keys=["/"], code=["vb/stages/report.py"]),
]

# The three questions a design has to answer, cheapest first. A stage belongs to the phase whose question it
# answers; execution order stays the list order above (balance runs late only because it can use the aero and
# hover numbers when they exist).
PHASES = [
    dict(name="layout", question="Does it add up?", stages=["ingest", "geometry", "mass", "clash", "balance"],
         blurb="plan valid, parts fit, weight and CG on target, a balanced layout exists"),
    dict(name="sizing", question="Does it work on paper?", stages=["aero_lowfi", "propulsion", "vtol", "thermal", "structures"],
         blurb="stall and stability, thrust and energy, hover margins, temperatures, loads"),
    dict(name="flight", question="Does it fly in simulation?", stages=["cfd", "flightmodel", "sitl"],
         blurb="CFD cross-check, flight model and ArduPilot parameters, simulated missions and failsafes"),
]
PHASE_OF = {s: p["name"] for p in PHASES for s in p["stages"]}
DESK = [s for p in PHASES[:2] for s in p["stages"]] + ["flightmodel", "report"]      # everything that takes seconds

BY_NAME = {s["name"]: s for s in STAGES}
NAMES = [s["name"] for s in STAGES]


def descendants(name):
    out = set()
    frontier = {name}
    while frontier:
        nxt = set()
        for s in STAGES:
            if s["name"] in out or s["name"] == name:
                continue
            if frontier & set(s["deps"] + s["soft"]):
                nxt.add(s["name"])
        out |= nxt
        frontier = nxt
    return out


def ancestors(name):
    out = set()
    frontier = {name}
    while frontier:
        nxt = set()
        for f in frontier:
            for d in BY_NAME[f]["deps"] + BY_NAME[f]["soft"]:
                if d not in out:
                    nxt.add(d)
        out |= nxt
        frontier = nxt
    return out
