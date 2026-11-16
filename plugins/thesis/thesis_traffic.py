""" TRAFFIC: the traffic of one experimental run.

  - Draws all arrivals of the run at the start (model.draw): per gate a shifted exponential with the transit minimum
    SEP_H / V_ENTRY of the previous drone as the smallest gap; types, state of charge, pad time and prediction error
    from their own random streams, so every case gets the same drones for the same (demand, seed).
  - Gives VPMS the list of arrival times (it may look H_ANNOUNCE ahead in it).
  - Creates every drone at its entry speed at the first update after its drawn time, placed where it is at that update;
    counts a drone created within SEP_H of another (VPMS close_entries, event entry_close; expected 0).
  - Stops (HOLD) when every drone of the measured window has landed.
  - Video mode (TRAFFIC VIDEO ON/OFF, only for the GUI): announced drones as ghost drones AN#### flying to their gate.
Every number and formula: experimenten/config/model.py (imported as M). Commands:
  TRAFFIC demand seed [eta_err_pct] [BASE|LONG] [warmup_s] [measure_s]     e.g. TRAFFIC 60 1
  TRAFFIC VIDEO ON|OFF
(File name thesis_traffic.py: 'traffic' is already a BlueSky module.)
"""
import os
import sys
import numpy as np
from bluesky import core, stack, traf, sim
from bluesky.tools.aero import vtas2cas

EXPERIMENTEN = os.environ.get("EXPERIMENTEN", os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "experimenten"))
sys.path.insert(0, os.path.join(EXPERIMENTEN, "config"))
import model as M   # noqa: E402


def plugin(name, cls):
    for modname, mod in sys.modules.items():
        if modname.endswith("." + name) and hasattr(mod, cls):
            return getattr(mod, cls).implinstance()
    return None


def init_plugin():
    Traffic()
    return {"plugin_name": "TRAFFIC", "plugin_type": "sim"}


class Traffic(core.Entity):
    def __init__(self):
        super().__init__()
        self.clear()

    def clear(self):
        self.plan, self.next, self.waiting, self.measured, self.done = None, 0, [], set(), False
        self.video, self.ghosts = False, {}       # video mode: k -> ghost call sign
        self.t0, self.warmup, self.measure = 0.0, M.WARMUP, M.MEASURE

    def reset(self):
        super().reset()
        self.clear()

    @core.timed_function(name="traffic", dt=M.DT)
    def update(self):
        if self.plan is None:
            return
        vpms = plugin("vpms", "Vpms")
        now = sim.simt - self.t0
        while self.next < len(self.plan["time"]) and self.plan["time"][self.next] <= now:
            self.waiting.append(self.next)
            self.next += 1
        for k in list(self.waiting):             # in order of arrival, at the drawn time
            self.spawn(k, vpms)
            self.waiting.remove(k)
        if self.video:
            self.show_ghosts(vpms, now)
        self.check_end()

    def show_ghosts(self, vpms, now):
        """Video mode: every announced drone as a ghost drone on the extension of its entry line, where
        it would be if it flew at its entry speed; VPMS does not manage it. Only for the GUI."""
        p = self.plan
        for k in range(self.next, len(p["time"])):
            dt = p["time"][k] - now
            if dt > M.H_ANNOUNCE:
                break
            g, lc = int(p["gate"][k]), bool(p["is_lc"][k])
            v = M.V_ENTRY[int(lc)]
            lat, lon = M.to_latlon(*M.offset_xy(g, lc, M.gate_distance() + v * max(dt, 0.0)))
            hdg = (M.GATE_BEARINGS[g] + 180) % 360
            acid = self.ghosts.get(k)
            if acid is None:
                acid = self.ghosts[k] = f"AN{k + 1:04d}"
                traf.cre(acid, M.BLUESKY_TYPES[lc], lat, lon, hdg, M.H_ARC[g % 2], vtas2cas(v, M.H_ARC[g % 2]))
            else:
                i = traf.id2idx(acid)
                if i >= 0:
                    traf.lat[i], traf.lon[i] = lat, lon

    def spawn(self, k, vpms):
        p = self.plan
        g, lc = int(p["gate"][k]), bool(p["is_lc"][k])
        acid = f"{'LC' if lc else 'MC'}{k + 1:04d}"
        _, _, alt, hdg = M.gate(g, lc)                      # entry point of this gate and type
        v0 = M.V_ENTRY[int(lc)]                             # always the entry speed
        t_gate = self.t0 + p["time"][k]                       # it passed its gate at exactly its drawn time:
        lat, lon = M.to_latlon(*M.offset_xy(g, lc, M.gate_distance() - v0 * max(sim.simt - t_gate, 0.0)))
        ghost = self.ghosts.pop(k, None)
        if ghost is not None:
            stack.stack(f"DEL {ghost}")
        vpms.register(acid, g, lc, sim.simt, float(p["t_pad"][k]), float(p["eta_err"][k]), t_gate)
        traf.cre(acid, M.BLUESKY_TYPES[lc], lat, lon, hdg, alt, vtas2cas(v0, alt))
        soc = p["soc"][k]
        energy = plugin("energy", "Energy")
        i = traf.id2idx(acid)
        if energy is not None and i >= 0:
            energy.soc[i] = energy.soc_start[i] = soc
        if p["time"][k] >= self.warmup and p["time"][k] < self.warmup + self.measure:
            self.measured.add(acid)
        fl = plugin("flightlog", "FlightLog")
        if fl is not None and fl.files and i >= 0:
            fl.event(i, "measured" if acid in self.measured else "runin",
                     f"arrival {p['time'][k]:.1f} s, waited 0.0 s at gate")   # (text kept for identical logs)
        if i >= 0:                                            # within SEP_H of another drone?
            x, y = M.to_xy(traf.lat, traf.lon)
            dh = np.hypot(x - x[i], y - y[i])
            dv = np.abs(traf.alt - traf.alt[i])
            near = [j for j in np.flatnonzero((dh < M.SEP_H) & (dv < M.SEP_V))
                    if j != i and not traf.id[j].startswith("AN")]
            if near:
                j = min(near, key=lambda m: dh[m])
                vpms.close_entries.append((sim.simt, acid, traf.id[j], float(dh[j])))
                if fl is not None and fl.files:
                    fl.event(i, "entry_close", f"{traf.id[j]} at {dh[j]:.1f} m")
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
              warmup: float = M.WARMUP, measure: float = M.MEASURE):
        """TRAFFIC demand seed [eta_err_pct] [BASE|LONG] [warmup_s] [measure_s], or TRAFFIC VIDEO ON|OFF"""
        if demand.upper() == "VIDEO":
            self.video = str(seed).upper() not in ("OFF", "0")
            return True, f"TRAFFIC video mode {'on: announced drones shown as ghosts AN####' if self.video else 'off'}"
        demand = float(demand)
        self.warmup, self.measure = warmup, measure
        vpms = plugin("vpms", "Vpms")
        self.plan = M.draw(demand, int(seed), warmup + measure, pad, eta, M.V_ENTRY, M.SEP_H)
        self.t0, self.next, self.waiting, self.measured, self.done = sim.simt, 0, [], set(), False
        p = self.plan
        if vpms is not None:                            # the list of arrivals for VPMS
            vpms.set_schedule(self.t0 + p["time"], p["gate"], p["is_lc"],
                              [f"{'LC' if lc else 'MC'}{k + 1:04d}" for k, lc in enumerate(p["is_lc"])])
        stack.stack("OP")
        n, n_lc = len(self.plan["time"]), int(self.plan["is_lc"].sum())
        rule = (f"gate rule {M.SEP_H / M.V_ENTRY[0]:.2f} s behind an MC, {M.SEP_H / M.V_ENTRY[1]:.2f} s behind "
                f"an L+C; ring {'on' if M.RING_ON else 'off'}; announcement {M.H_ANNOUNCE:g} s" if vpms is not None else "no gate rule")
        fl = plugin("flightlog", "FlightLog")
        if fl is not None and vpms is not None:         # settings of this run for the analysis
            fl.run_info(variant=M.VARIANT, target=M.T_TARGET, design=M.T_DESIGN, k_spacing=M.K_SPACING,
                        r_mc=float(M.arc_radius(False)), r_lc=float(M.arc_radius(True)),
                        gate_distance=float(M.gate_distance()), entry_leg=float(M.ENTRY_LEG),
                        offset=float(M.entry_offset()) if M.VARIANT == "speed" else 0.0,
                        demand=demand, seed=int(seed), eta_pct=float(eta), pad=pad.upper(),
                        warmup=float(warmup), measure=float(measure), arrivals=n, arrivals_lc=n_lc,
                        t0=float(sim.simt),   # simulation time of the start: measured hour = t0 + warmup ...
                        ring=bool(M.RING_ON), horizon=float(M.H_ANNOUNCE), v_mc_max=float(M.V_MC_MAX),
                        turn_mc=float(M.turn_lookahead(False)), turn_lc=float(M.turn_lookahead(True)),
                        gate_rule="per gate SEP_H / V_ENTRY of the previous drone")
        return True, (f"TRAFFIC: {n} arrivals ({n_lc} L+C) in {warmup + measure:.0f} s, "
                      f"seed {seed}, prediction error {eta} %, pad {pad}, {rule}")

