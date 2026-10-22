""" ARRIVALS: synthetic arrival traffic at the gates of the droneport.

Before the run, the full arrival list is drawn from a random seed: time, gate,
configuration and state of charge of every drone. The same seed gives the
same list, so every case of the experiment gets exactly the same traffic.
During the run, each drone is created at its gate when its time has come.
"""
import os
import sys
import numpy as np
from bluesky import core, stack, sim

# The speeds come from the energy model (one place for all values), see ENERGY.
EXPERIMENTEN = os.environ.get("EXPERIMENTEN", os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "experimenten"))
sys.path.insert(0, os.path.join(EXPERIMENTEN, "config"))
from vehicles import GENERIC_MC as MC, GENERIC_LC as LC   # noqa: E402

# =============================================================================
# PARAMETERS (geometry and SoC range still to be decided: placeholders)
# =============================================================================
PORT_LAT, PORT_LON = 52.0, 4.0   # droneport (landing pad)
GATE_DIST = 1500.0               # m from the pad to each gate
GATE_BEARINGS = [0, 90, 180, 270]   # degrees: four gates, north, east, south, west
ALT = 60.0                       # m: arrival altitude

SHARE_LC = 0.5                   # share of Lift+Cruise drones (base case 50/50)
TYPE = {"MC": "M200", "LC": "AMZN"}     # BlueSky types (must match ENERGY plugin)
# Speed at the gate (m/s):
#   MC: 8 m/s, but never faster than its best-range speed v_range (9.0 m/s): the
#       multicopter power curve is only valid up to there (decision 21-10-2026)
#   LC: the arc (waiting) speed v_arc = 1.05 x stall speed (12.4 m/s)
SPEED = {"MC": min(8.0, MC.v_range), "LC": LC.v_arc}
SOC_RANGE = (0.40, 0.80)         # state of charge at the gate, drawn uniformly


def arrival_list(demand, duration, seed):
    """Draw all arrivals for one run.
    demand: drones per hour, duration: s, seed: integer.
    Arrival times form a Poisson process: the gaps between drones are
    exponentially distributed with mean 3600 / demand seconds."""
    rng = np.random.default_rng(seed)
    times, t = [], rng.exponential(3600 / demand)
    while t < duration:
        times.append(t)
        t += rng.exponential(3600 / demand)
    n = len(times)
    return dict(
        time=np.array(times),
        gate=rng.integers(len(GATE_BEARINGS), size=n),
        config=np.where(rng.random(n) < SHARE_LC, "LC", "MC"),
        soc=rng.uniform(*SOC_RANGE, size=n),
    )


def gate_position(bearing):
    """Latitude and longitude of the gate at this bearing from the pad."""
    dlat = GATE_DIST * np.cos(np.radians(bearing)) / 111320
    dlon = GATE_DIST * np.sin(np.radians(bearing)) / (111320 * np.cos(np.radians(PORT_LAT)))
    return PORT_LAT + dlat, PORT_LON + dlon


def init_plugin():
    Arrivals()
    return {"plugin_name": "ARRIVALS", "plugin_type": "sim"}


class Arrivals(core.Entity):
    def __init__(self):
        super().__init__()
        self.plan = None     # the arrival list
        self.next = 0        # index of the next drone to create
        self.t0 = 0.0        # simulation time at which the list started

    @core.timed_function(name="arrivals", dt=0.5)
    def update(self):
        while self.plan is not None and self.next < len(self.plan["time"]) \
                and self.t0 + self.plan["time"][self.next] <= sim.simt:
            self.spawn(self.next)
            self.next += 1

    def spawn(self, k):
        cfg = self.plan["config"][k]
        acid = f"{cfg}{k + 1:04d}"
        bearing = GATE_BEARINGS[self.plan["gate"][k]]
        lat, lon = gate_position(bearing)
        heading = (bearing + 180) % 360                   # facing the pad
        alt_ft, spd_kts = ALT / 0.3048, SPEED[cfg] / 0.514444
        stack.stack(f"CRE {acid} {TYPE[cfg]} {lat:.6f} {lon:.6f} {heading} {alt_ft:.0f} {spd_kts:.1f}")
        stack.stack(f"SOC {acid} {self.plan['soc'][k]:.3f}")
        # Temporary route straight to the pad, until the VPMS plugin exists
        stack.stack(f"ADDWPT {acid} {PORT_LAT} {PORT_LON} {alt_ft:.0f} {spd_kts:.1f}")

    @stack.command(name="ARRIVALS")
    def start(self, demand: float, duration: float = 3600, seed: int = 1):
        """ARRIVALS demand [duration] [seed]: drones per hour, run length (s), random seed."""
        self.plan = arrival_list(demand, duration, seed)
        self.next, self.t0 = 0, sim.simt
        stack.stack("OP")    # start the simulation, also when there is no traffic yet
        n, n_lc = len(self.plan["time"]), int((self.plan["config"] == "LC").sum())
        return True, f"{n} arrivals planned ({n_lc} Lift+Cruise) in {duration:.0f} s, seed {seed}"
