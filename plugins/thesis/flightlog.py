""" FLIGHTLOG: writes what happened into the folder output/<name>/, at three levels:

  flights.csv  one line per drone, written when the drone is deleted (after landing)
  events.csv   one line per event: spawn, change of flight mode, start and end of
               a loss of separation (at the end with its severity), end. Enough to rebuild
               each drone's timeline.

Separation (decision 05-11-2026): min_sep is the smallest horizontal distance to a drone that is
less than SEP_V apart vertically (before: to any drone, also one far above it). A loss of separation
(LoS) is dh < SEP_H and dv < SEP_V. Its severity is how deep the protected cylinder is entered,
min(1 - dh / SEP_H, 1 - dv / SEP_V): 0 at its edge, 1 at the same position; the smaller of the two,
because the drone is out of the cylinder as soon as one of them reaches its minimum. Per episode the
highest value (event los_end: the other drone, severity, dh and dv at that moment); per drone the
number of episodes (n_los) and the highest severity (los_sev).
  trace.csv    one line per drone per second. Only with "FLIGHTLOG name TRACE":
               large, meant for figures and debugging, not for the experiments.

Needs the ENERGY plugin (load ENERGY first).
Other plugins can add their own events with  FlightLog.implinstance().event(i, "name", "info"),
e.g. the VPMS plugin for "arc_in", "direct_to" and "touchdown".
"""
import os
import sys
import numpy as np
from bluesky import core, stack, traf, sim

# =============================================================================
# PARAMETERS
# =============================================================================
DT = 1.0            # update interval (s); events are therefore accurate to 1 s
SEP_H = 32.0        # m: horizontal separation minimum (Metropolis II, decision 23-10-2026)
SEP_V = 7.62        # m: vertical separation minimum (25 ft, Metropolis II)
FOLDER = "output"   # parent folder of the run folders

MODES = ["hover", "rotor", "wing"]          # same numbering as in ENERGY
# Copied from ENERGY every step, so they are still known when a drone is deleted
FROM_ENERGY = ["is_lc", "used_wh", "soc", "soc_start", "power", "mode",
               "wh_hover", "wh_rotor", "wh_wing", "wh_trans", "n_trans"]

COLUMNS = dict(
    flights=["id", "type", "config", "t_spawn", "t_end", "flight_time", "soc_start", "soc_end",
             "energy_wh", "wh_hover", "wh_rotor", "wh_wing", "wh_trans", "n_trans",
             "t_hover", "t_rotor", "t_wing", "min_sep_m", "los_time", "n_los", "los_sev"],
    events=["t", "id", "event", "info", "mode", "tas", "alt", "soc", "energy_wh"],
    trace=["t", "id", "lat", "lon", "alt", "tas", "mode", "power_w", "energy_wh", "soc"],
)


def energy_plugin():
    """The ENERGY plugin, wherever it is loaded from (None if not loaded)."""
    for name, mod in sys.modules.items():
        if name.endswith(".energy") and hasattr(mod, "Energy"):
            return mod.Energy.implinstance()
    return None


def init_plugin():
    FlightLog()
    return {"plugin_name": "FLIGHTLOG", "plugin_type": "sim"}


class FlightLog(core.Entity):
    def __init__(self):
        super().__init__()
        self.files = {}      # "flights", "events" (and "trace") -> open file
        self.ep_other = {}   # acid -> the other drone at the deepest point of its current LoS episode
        with self.settrafarrays():
            self.t_spawn = np.array([])
            self.started = np.array([], dtype=bool)   # spawn event written
            self.t_hover = np.array([])               # time per mode (s)
            self.t_rotor = np.array([])
            self.t_wing = np.array([])
            self.min_sep = np.array([])               # smallest horizontal distance to a drone less than SEP_V apart (m)
            self.los_time = np.array([])              # time inside the separation minima (s)
            self.in_los = np.array([], dtype=bool)
            self.n_los = np.array([], dtype=int)      # number of LoS episodes
            self.los_sev = np.array([])               # highest LoS severity (0 if none)
            self.ep_sev = np.array([])                # current episode: highest severity, dh and dv then
            self.ep_dh = np.array([])
            self.ep_dv = np.array([])
            for name in FROM_ENERGY:
                setattr(self, name, np.array([]))

    def create(self, n=1):
        super().create(n)
        self.t_spawn[-n:] = sim.simt
        self.min_sep[-n:] = np.inf
        self.n_los[-n:] = 0
        self.los_sev[-n:] = 0.0
        self.ep_sev[-n:] = -np.inf

    @core.timed_function(name="flightlog", dt=DT)
    def update(self, dt):
        energy = energy_plugin()
        if traf.ntraf == 0 or not self.files or energy is None:
            return
        old_mode = self.mode.copy()
        for name in FROM_ENERGY:
            getattr(self, name)[:] = getattr(energy, name)

        # New drones: spawn event
        for i in np.flatnonzero(~self.started):
            self.event(i, "spawn")
        # Change of flight mode: the event is the new mode, info is the old one
        for i in np.flatnonzero(self.started & (self.mode != old_mode)):
            self.event(i, MODES[int(self.mode[i])], MODES[int(old_mode[i])])
        self.started[:] = True

        # Time per mode
        self.t_hover += np.where(self.mode == 0, dt, 0.0)
        self.t_rotor += np.where(self.mode == 1, dt, 0.0)
        self.t_wing += np.where(self.mode == 2, dt, 0.0)

        # Separation (flat earth, fine within a few km)
        x = np.radians(traf.lon) * np.cos(np.radians(traf.lat)) * 6371e3
        y = np.radians(traf.lat) * 6371e3
        dh = np.hypot(x[:, None] - x[None, :], y[:, None] - y[None, :])   # horizontal (m)
        dv = np.abs(traf.alt[:, None] - traf.alt[None, :])                 # vertical (m)
        np.fill_diagonal(dh, np.inf)
        close = dv < SEP_V
        self.min_sep = np.minimum(self.min_sep, np.where(close, dh, np.inf).min(axis=1))
        inside = (dh < SEP_H) & close
        sev = np.where(inside, np.minimum(1 - dh / SEP_H, 1 - dv / SEP_V), -np.inf)
        worst = sev.argmax(axis=1)                       # the other drone that is deepest inside
        los = inside.any(axis=1)
        self.los_time += np.where(los, dt, 0.0)
        for i in np.flatnonzero(los & ~self.in_los):     # start of an episode
            self.n_los[i] += 1
            self.ep_sev[i] = -np.inf
            self.ep_other[traf.id[i]] = traf.id[worst[i]]
            self.event(i, "los_start", traf.id[worst[i]])
        for i in np.flatnonzero(los):                    # deepest point of the episode so far
            j = worst[i]
            if sev[i, j] > self.ep_sev[i]:
                self.ep_sev[i], self.ep_dh[i], self.ep_dv[i] = sev[i, j], dh[i, j], dv[i, j]
                self.ep_other[traf.id[i]] = traf.id[j]
        for i in np.flatnonzero(~los & self.in_los):     # end of an episode
            self.end_episode(i)
        self.in_los[:] = los

        # Trace: every drone, every step
        if "trace" in self.files:
            for i in range(traf.ntraf):
                self.write("trace", [sim.simt, traf.id[i], f"{traf.lat[i]:.6f}", f"{traf.lon[i]:.6f}", traf.alt[i],
                                     traf.tas[i], MODES[int(self.mode[i])], self.power[i],
                                     self.used_wh[i], self.soc[i]])

    def event(self, i, name, info=""):
        """Write one event of drone i (index) with its state at that moment."""
        self.write("events", [sim.simt, traf.id[i], name, info,
                              MODES[int(self.mode[i])], traf.tas[i], traf.alt[i],
                              self.soc[i], self.used_wh[i]])

    def end_episode(self, i):
        """End of a LoS episode of drone i: event los_end with its severity; keep the highest per drone."""
        self.los_sev[i] = max(self.los_sev[i], self.ep_sev[i])
        self.event(i, "los_end", f"{self.ep_other.pop(traf.id[i], '')} severity {self.ep_sev[i]:.3f} "
                                 f"dh {self.ep_dh[i]:.1f} m dv {self.ep_dv[i]:.2f} m")

    def delete(self, idx):
        """Called by BlueSky just before drones are removed: write their line."""
        for i in np.atleast_1d(idx):
            if self.files and self.started[i] and self.in_los[i]:
                self.end_episode(i)                      # removed during a LoS: close the episode
            if self.files and self.started[i]:
                self.event(i, "end")
                self.write("flights", [
                    traf.id[i], traf.type[i], "LC" if self.is_lc[i] else "MC",
                    self.t_spawn[i], sim.simt, sim.simt - self.t_spawn[i],
                    self.soc_start[i], self.soc[i], self.used_wh[i],
                    self.wh_hover[i], self.wh_rotor[i], self.wh_wing[i], self.wh_trans[i],
                    int(self.n_trans[i]), self.t_hover[i], self.t_rotor[i], self.t_wing[i],
                    self.min_sep[i], self.los_time[i], int(self.n_los[i]), self.los_sev[i]])
        super().delete(idx)

    def write(self, kind, values):
        """One CSV line: floats with 4 decimals, the rest as text. Text with a comma or a quote is put between
        quotes (standard CSV), so that it stays one column (decision 05-11-2026: event infos such as
        "arrival 64.4 s, waited 0.1 s at gate" from TRAFFIC split into two columns and broke run_report)."""
        def cell(v):
            if isinstance(v, float):
                return f"{v:.4f}"
            text = str(v)
            return '"' + text.replace('"', '""') + '"' if ("," in text or '"' in text) else text
        if kind in self.files:
            self.files[kind].write(",".join(cell(v) for v in values) + "\n")

    @stack.command(name="FLIGHTLOG")
    def start(self, name: str = "", trace: str = ""):
        """FLIGHTLOG [name] [TRACE]: log into output/<name>/ (TRACE: also every second)."""
        self.close()
        name = name or sim.utc.strftime("%Y%m%d_%H%M%S")
        folder = os.path.join(FOLDER, name)
        os.makedirs(folder, exist_ok=True)
        kinds = ["flights", "events"] + (["trace"] if trace.upper() == "TRACE" else [])
        for kind in kinds:
            self.files[kind] = open(os.path.join(folder, kind + ".csv"), "w", buffering=1)   # line by line
            self.files[kind].write(",".join(COLUMNS[kind]) + "\n")
        return True, f"Flight log: {folder} ({', '.join(kinds)})"

    def close(self):
        """Close all files (also on RESET), so everything is on disk."""
        for f in self.files.values():
            f.close()
        self.files = {}

    def reset(self):
        super().reset()
        self.close()
