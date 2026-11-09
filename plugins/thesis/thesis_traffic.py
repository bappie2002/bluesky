""" TRAFFIC: the traffic and the disturbances of one experimental run (decisions 26-10-2026).

Traffic (methodology, Traffic and Scenarios):
  - arrival times: Poisson process, exponential gaps with mean 3600 / demand
    (Zeng et al. 2021, Kleinbekman et al. 2020)
  - gate: uniform over the four gates (Chang et al. 2026)
  - type: 50% multicopter, 50% Lift+Cruise
  - state of charge at the gate: uniform 40-80%, the same for both types
  - run: WARMUP s run-in (20 min since 09-11-2026, from a warm-up check; was 1 h) + MEASURE s measured
    (Wille et al. 2026). Only drones that
    arrive in the measured window count; the run stops when they have all landed.
Disturbances (no wind in this study, decision 27-10-2026):
  - error in the predicted flight time to the merge point, per drone normally distributed
    with standard deviation ETA_ERR % (levels 0, 5, 10, 20 %; own assumption), cut at 3 sigma.
    Main comparison: 0 %. Separation analysis: all levels.
  - pad time: 6 s for every drone (decision 05-11-2026; was 2 s standing still + U(2, 6) s to take
    it off, mean 6 s); pad mode LONG: 60 s (Vlaskin et al. 2024), sensitivity case

Random numbers: every quantity has its own stream from one seed. The drones therefore
depend only on (demand, seed): every mechanism, variant, R preset and error level gets
exactly the same drones (common random numbers).

Arrivals at the gate (decision 29-10-2026, option B): every drone passes its entry point at its
drawn time; there is no waiting before the gate, because what happens in the transit airspace is
outside the model. The only rule at the hand-over (decision 03-11-2026): the transit minimum. When
a drone passes its entry point, the previous drone at that point is at least SEP_H (32 m) ahead,
more if the follower would close in while both follow their lane speeds (VPMS handover_time, decision
04-11-2026: MC behind MC 4.2 s, L+C behind L+C 3.1 s, L+C behind MC 5.1 s, MC behind L+C 3.1 s, each
including one update step; L+C behind MC 6.1 s with the entry leg of 100 m before 06-11-2026; before 04-11-2026
4.0, 3.1, 4.9 and 3.1 s). A drone drawn closer
is moved later by the generator (draw). In the speed variant an MC and an L+C use different entry
points (about 55 m apart), so there is no rule between them. Bunches of arrivals are absorbed inside the
structure: the lane rules restore GAP_MIN (48 m), drones fly slower on the arc and hover at its end.
Safety net (decision 06-11-2026): a drone is created at VPMS entry_speed, which is its entry speed except when
the drone ahead in its lane is slower than the hand-over rule assumes; it is then created slower, so that it
stays SEP_H behind, and the event slow_entry is logged and counted (expected none below capacity). When no speed
works (decision 09-11-2026), the drone is held in the transit airspace until one does; also a slow_entry.

Needs VPMS (and ENERGY for the state of charge). Command:
  TRAFFIC demand seed [eta_err_pct] [pad] [warmup] [measure]
  e.g. TRAFFIC 60 1 10         60 drones/h, seed 1, prediction error 10 %
Batch files: TRAFFIC BATCH (main comparison) and TRAFFIC BATCH SEP (separation analysis)
(File name thesis_traffic.py: 'traffic' is already a BlueSky module.) (writes scenario/thesis/batch_<variant>.scn).
"""
import os
import sys
import numpy as np
from bluesky import core, stack, traf, sim
from bluesky.tools.aero import vtas2cas

EXPERIMENTEN = os.environ.get("EXPERIMENTEN", os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "experimenten"))
sys.path.insert(0, os.path.join(EXPERIMENTEN, "config"))
from vehicles import GENERIC_MC as MC, GENERIC_LC as LC   # noqa: E402

# =============================================================================
# PARAMETERS
# =============================================================================
DEMAND_LEVELS = (30, 45, 60, 75, 90)   # drones per hour (33-100% of 90/h at 40 s)
ETA_LEVELS = (0, 5, 10, 20)            # % standard deviation of the prediction error
SEP_TARGETS = (30, 35, 40, 45, 60)     # s, target separations in the separation analysis
REPETITIONS = 100                      # seeds per setting (Song 2023); more if the CI is too wide
WARMUP, MEASURE = 1200.0, 3600.0       # s: run-in and measured hour. Decision 09-11-2026: run-in 20 min (was 1 h, as
                                       #    Wille et al. 2026): the warm-up check (probes_2910/welch.py, 20 seeds at 60 and
                                       #    75 drones/h) shows no build-up after the first 5-10 min
SHARE_LC = 0.5
SOC_RANGE = (0.40, 0.80)
T_PAD_BASE = 6.0                       # s on the pad, the same for every drone (decision 05-11-2026; was
                                       #    2 s standing still + U(2, 6) s to take it off). The random stream
                                       #    "pad" stays in the list, so the other streams keep their draws
T_PAD_LONG = 60.0                      # s, pad mode LONG
TYPE = ("GMC", "GLC")                  # BlueSky types for MC and L+C (as in ENERGY): the generic drones of this
                                       # thesis, with their limits in BlueSky's OpenAP type file (decision 04-11-2026)
P_WAIT = (float(MC.sim_power(np.array([MC.v_endurance]))[0][0]),   # W, waiting outside the
          float(LC.sim_power(np.array([LC.v_arc]))[0][0]))       # structure at the waiting speed
E_BAT = (MC.battery_wh, LC.battery_wh)


def plugin(name, cls):
    for modname, mod in sys.modules.items():
        if modname.endswith("." + name) and hasattr(mod, cls):
            return getattr(mod, cls).implinstance()
    return None


def draw(demand, seed, duration, pad="BASE", eta_pct=0.0, entry_key=None, handover=None):
    """All arrivals of one run. Separate random streams per quantity (common random numbers).
    Hand-over rule (decision 03-11-2026): entry_key(gate, is_lc) gives the entry point of a drone,
    handover(lead_lc, foll_lc) the smallest time (s) behind the previous drone at that point; a drone
    drawn closer is moved later. Without them (None) there is no rule."""
    rng = {k: np.random.default_rng([seed, n]) for n, k in
           enumerate(["time", "gate", "type", "soc", "pad", "eta"])}
    times, t = [], rng["time"].exponential(3600 / demand)
    while t < duration:
        times.append(t)
        t += rng["time"].exponential(3600 / demand)
    n = len(times)
    gate = rng["gate"].integers(4, size=n)
    is_lc = rng["type"].random(n) < SHARE_LC
    times, last = np.array(times), {}
    if handover is not None:
        for k in range(n):                               # hand-over rule per entry point
            key = entry_key(int(gate[k]), bool(is_lc[k]))
            if key in last:
                t_prev, lc_prev = last[key]
                times[k] = max(times[k], t_prev + handover(lc_prev, bool(is_lc[k])))
            last[key] = (times[k], bool(is_lc[k]))
    plan = dict(
        time=times,
        gate=gate,
        is_lc=is_lc,
        soc=rng["soc"].uniform(*SOC_RANGE, size=n),
        t_pad=np.full(n, T_PAD_LONG if pad.upper() == "LONG" else T_PAD_BASE),
        eta_err=np.clip(rng["eta"].standard_normal(n), -3, 3) * eta_pct / 100,
    )
    order = np.argsort(plan["time"], kind="stable")      # a moved drone can pass a later one
    return {k: v[order] for k, v in plan.items()}


def init_plugin():
    Traffic()
    return {"plugin_name": "TRAFFIC", "plugin_type": "sim"}


class Traffic(core.Entity):
    def __init__(self):
        super().__init__()
        self.clear()

    def clear(self):
        self.plan, self.next, self.waiting, self.measured, self.done = None, 0, [], set(), False
        self.held = {}                            # k -> time from which the safety net held the drone in transit
        self.t0, self.warmup, self.measure = 0.0, WARMUP, MEASURE

    def reset(self):
        super().reset()
        self.clear()

    @core.timed_function(name="traffic", dt=0.5)
    def update(self):
        if self.plan is None:
            return
        vpms = plugin("vpms", "Vpms")
        now = sim.simt - self.t0
        while self.next < len(self.plan["time"]) and self.plan["time"][self.next] <= now:
            self.waiting.append(self.next)
            self.next += 1
        for k in list(self.waiting):             # in order of arrival, at the drawn time (option B)
            if self.spawn(k, vpms):              # False: held in transit by the safety net (decision 09-11-2026)
                self.waiting.remove(k)
        self.check_end()

    def spawn(self, k, vpms):
        p = self.plan
        g, lc = int(p["gate"][k]), bool(p["is_lc"][k])
        acid = f"{'LC' if lc else 'MC'}{k + 1:04d}"
        geo = sys.modules[type(vpms).__module__]              # the VPMS module: geometry
        lat, lon, alt, hdg = geo.gate(g, lc)                  # entry point of this gate and type
        v0, slow = vpms.entry_speed(g, lc)                    # V_ENTRY; lower only as the safety net (06-11-2026)
        if v0 is None:                                        # no speed works: hold it in transit (09-11-2026)
            self.held.setdefault(k, sim.simt)
            return False
        if k in self.held:
            slow = (f"held {sim.simt - self.held.pop(k):.1f} s in transit, then {v0:.1f} m/s"
                    + (f" ({slow})" if slow else ""))
        vpms.register(acid, g, lc, sim.simt, float(p["t_pad"][k]), float(p["eta_err"][k]))
        traf.cre(acid, TYPE[lc], lat, lon, hdg, alt, vtas2cas(v0, alt))
        wait = sim.simt - (self.t0 + p["time"][k])
        soc = p["soc"][k] - P_WAIT[lc] * wait / 3600 / E_BAT[lc]
        energy = plugin("energy", "Energy")
        i = traf.id2idx(acid)
        if energy is not None and i >= 0:
            energy.soc[i] = energy.soc_start[i] = soc
        if p["time"][k] >= self.warmup and p["time"][k] < self.warmup + self.measure:
            self.measured.add(acid)
        fl = plugin("flightlog", "FlightLog")
        if fl is not None and fl.files and i >= 0:
            fl.event(i, "measured" if acid in self.measured else "runin",
                     f"arrival {p['time'][k]:.1f} s, waited {wait:.1f} s at gate")
        if slow:                                              # safety net used: counted (decision 06-11-2026)
            vpms.slow_entries.append((sim.simt, acid, v0, slow))
            if fl is not None and fl.files and i >= 0:
                fl.event(i, "slow_entry", slow)
        return True

    def check_end(self):
        if self.done or self.next < len(self.plan["time"]) or self.waiting:
            return
        if not any(acid in self.measured for acid in traf.id):
            self.done = True
            stack.stack("HOLD")
            stack.stack(f"ECHO TRAFFIC finished: {len(self.measured)} measured drones landed")

    @stack.command(name="TRAFFIC")
    def start(self, demand: str, seed: str = "1", eta: float = 0.0, pad: str = "BASE",
              warmup: float = WARMUP, measure: float = MEASURE):
        """TRAFFIC demand seed [eta_err_pct] [BASE|LONG] [warmup_s] [measure_s], or TRAFFIC BATCH [SEP]"""
        if demand.upper() == "BATCH":
            return write_batch(separation=(str(seed).upper() == "SEP"))
        demand = float(demand)
        self.warmup, self.measure = warmup, measure
        vpms = plugin("vpms", "Vpms")
        geo = sys.modules[type(vpms).__module__] if vpms is not None else None
        self.plan = draw(demand, int(seed), warmup + measure, pad, eta,
                         geo.entry_key if geo else None, geo.handover_time if geo else None)
        self.t0, self.next, self.waiting, self.measured, self.done = sim.simt, 0, [], set(), False
        stack.stack("OP")
        n, n_lc = len(self.plan["time"]), int(self.plan["is_lc"].sum())
        rule = (f"hand-over MC-MC {geo.handover_time(False, False):.1f} s, L+C-L+C {geo.handover_time(True, True):.1f} s"
                + (f", L+C behind MC {geo.handover_time(False, True):.1f} s" if geo.VARIANT == "shared" else "")
                if geo else "no hand-over rule")
        fl = plugin("flightlog", "FlightLog")
        if fl is not None and geo is not None:          # settings of this run for the analysis (06-11-2026)
            fl.run_info(variant=geo.VARIANT, target=geo.T_TARGET, design=geo.T_DESIGN, k_spacing=geo.K_SPACING,
                        r_mc=float(geo.arc_radius(False)), r_lc=float(geo.arc_radius(True)),
                        gate_distance=float(geo.gate_distance()), gate_margin=float(geo.GATE_MARGIN),
                        offset=float(geo.entry_offset()) if geo.VARIANT == "speed" else 0.0,
                        demand=demand, seed=int(seed), eta_pct=float(eta), pad=pad.upper(),
                        warmup=float(warmup), measure=float(measure), arrivals=n, arrivals_lc=n_lc,
                        t0=float(sim.simt))   # simulation time of the start: measured hour = t0 + warmup ...
        return True, (f"TRAFFIC: {n} arrivals ({n_lc} L+C) in {warmup + measure:.0f} s, "
                      f"seed {seed}, prediction error {eta} %, pad {pad}, {rule}")


def write_batch(separation=False, variants=("shared", "speed"), folder="scenario/thesis"):
    """Main comparison: every variant x demand level x seed, without prediction error.
    Separation analysis (SEP): every variant x target x error level x demand level x seed."""
    os.makedirs(folder, exist_ok=True)
    targets = SEP_TARGETS if separation else (None,)
    levels = ETA_LEVELS if separation else (0,)
    n = 0
    for variant in variants:
        path = os.path.join(folder, f"batch_{'sep_' if separation else ''}{variant}.scn")
        with open(path, "w") as f:
            for target in targets:
                for eta in levels:
                    for demand in DEMAND_LEVELS:
                        for seed in range(1, REPETITIONS + 1):
                            name = (f"{variant}_t{target}_e{eta}" if separation else variant) + f"_d{demand}_s{seed}"
                            f.write(f"00:00:00.00>SCEN {name}\n")
                            cmds = ["PLUGINS LOAD ENERGY", "PLUGINS LOAD FLIGHTLOG", "PLUGINS LOAD VPMS",
                                    "PLUGINS LOAD TRAFFIC", "DT 0.5", f"VPMS {variant.upper()}"]
                            cmds += [f"VPMS TARGET {target or 40}"]   # always set: globals persist
                            cmds += [f"FLIGHTLOG {name}", f"TRAFFIC {demand} {seed} {eta}", "FF"]
                            for cmd in cmds:
                                f.write(f"00:00:00.00>{cmd}\n")
                            n += 1
    return True, f"Batch files written in {folder}: {n} runs ({'separation analysis' if separation else 'main comparison'})"
