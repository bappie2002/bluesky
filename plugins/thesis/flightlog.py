""" FLIGHTLOG: one line per drone in a CSV file, written when the drone is
deleted (after landing). Holds what the metrics need: times, energy,
state of charge, hover time and separation.
"""
import os
import sys
import numpy as np
from bluesky import core, stack, traf, sim

# =============================================================================
# PARAMETERS
# =============================================================================
DT = 1.0            # update interval (s)
HOVER_SPEED = 0.5   # m/s: below this airspeed a drone counts as hovering
SEP_H = 30.0        # m: horizontal separation minimum   (to be decided)
SEP_V = 15.0        # m: vertical separation minimum     (to be decided)
FOLDER = "output"   # folder of the CSV files

COLUMNS = ["id", "type", "t_spawn", "t_end", "flight_time", "energy_wh",
           "soc_start", "soc_end", "hover_time", "min_sep_m", "los_time"]


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
        self.file = None
        with self.settrafarrays():
            self.t_spawn = np.array([])
            self.energy_wh = np.array([])    # copied from ENERGY every step
            self.soc_start = np.array([])
            self.soc_now = np.array([])
            self.hover_time = np.array([])
            self.min_sep = np.array([])      # smallest distance to any other drone (m)
            self.los_time = np.array([])     # time inside the separation minima (s)

    def create(self, n=1):
        super().create(n)
        self.t_spawn[-n:] = sim.simt
        self.energy_wh[-n:] = 0.0
        self.soc_start[-n:] = np.nan         # filled at the first update (after SOC is set)
        self.soc_now[-n:] = np.nan
        self.hover_time[-n:] = 0.0
        self.min_sep[-n:] = np.inf
        self.los_time[-n:] = 0.0

    @core.timed_function(name="flightlog", dt=DT)
    def update(self, dt):
        if traf.ntraf == 0:
            return
        # Energy and state of charge from the ENERGY plugin
        energy = energy_plugin()
        if energy is not None:
            self.energy_wh = energy.used_wh.copy()
            self.soc_now = energy.soc.copy()
            self.soc_start = np.where(np.isnan(self.soc_start), self.soc_now, self.soc_start)

        # Hover time
        self.hover_time += np.where(traf.tas < HOVER_SPEED, dt, 0.0)

        # Distances between all drones (flat earth, fine within a few km)
        x = np.radians(traf.lon) * np.cos(np.radians(traf.lat)) * 6371e3
        y = np.radians(traf.lat) * 6371e3
        dh = np.hypot(x[:, None] - x[None, :], y[:, None] - y[None, :])   # horizontal (m)
        dv = np.abs(traf.alt[:, None] - traf.alt[None, :])                 # vertical (m)
        np.fill_diagonal(dh, np.inf)
        self.min_sep = np.minimum(self.min_sep, dh.min(axis=1))
        in_los = ((dh < SEP_H) & (dv < SEP_V)).any(axis=1)
        self.los_time += np.where(in_los, dt, 0.0)

    def delete(self, idx):
        """Called by BlueSky just before drones are removed: write their line."""
        for i in np.atleast_1d(idx):
            self.write_line(i)
        super().delete(idx)

    def write_line(self, i):
        if self.file is None:
            self.open_file()
        values = [traf.id[i], traf.type[i], self.t_spawn[i], sim.simt, sim.simt - self.t_spawn[i],
                  self.energy_wh[i], self.soc_start[i], self.soc_now[i], self.hover_time[i],
                  self.min_sep[i], self.los_time[i]]
        self.file.write(",".join(f"{v:.3f}" if isinstance(v, float) else str(v) for v in values) + "\n")
        self.file.flush()

    @stack.command(name="FLIGHTLOG")
    def open_file(self, name: str = ""):
        """FLIGHTLOG [name]: start a new CSV file (default name: date and time)."""
        if self.file:
            self.file.close()
        os.makedirs(FOLDER, exist_ok=True)
        name = name or sim.utc.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(FOLDER, f"flights_{name}.csv")
        self.file = open(path, "w")
        self.file.write(",".join(COLUMNS) + "\n")
        return True, f"Flight log: {path}"
