""" ENERGY: battery state of charge (SoC) of every drone.

Every second:  energy used = P(configuration, airspeed) x dt,
and that energy is subtracted from the battery. Per drone it also keeps the
power right now, the flight mode (hover, rotor, wing) and how much energy went
into each mode; FLIGHTLOG writes these out.

This file holds NO formulas and NO parameter values. The energy model has one place:
    formulas:    experimenten/plugins/energy_model.py
    parameters:  experimenten/config/vehicles.py   (GENERIC_MC, GENERIC_LC)
The experimenten folder must be next to the bluesky folder (afstuderen/), or set
the environment variable EXPERIMENTEN to its path (e.g. on a cluster).
"""
import os
import sys
import numpy as np
from bluesky import core, stack, traf

EXPERIMENTEN = os.environ.get("EXPERIMENTEN", os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "experimenten"))
sys.path.insert(0, os.path.join(EXPERIMENTEN, "config"))
from vehicles import GENERIC_MC as MC, GENERIC_LC as LC   # noqa: E402  (also loads energy_model)
from energy_model import MODES                           # noqa: E402

# =============================================================================
# PARAMETERS of the plugin (not of the model)
# =============================================================================
DT = 1.0              # update interval (s)
LC_TYPES = {"AMZN"}   # BlueSky types that are Lift+Cruise; all other types are multicopters
                      # (the type sets the flight performance only; the energy comes from the model)
E_TRANS_J, T_TRANS = LC.transition("to_rotor")   # one wing -> rotor transition: energy (J), duration (s)
P_TRANS = E_TRANS_J / T_TRANS                      # mean power during that transition (W)
# Transition (decision 22-10-2026): when a Lift+Cruise leaves its wing, it uses P_TRANS
# for T_TRANS seconds INSTEAD of the rotor curve. Mathur's transition energy already
# contains the deceleration from stall to rotor flight, so charging the rotor curve
# (or the plateau, A8) at the same time would count that deceleration twice.


def init_plugin():
    Energy()
    return {"plugin_name": "ENERGY", "plugin_type": "sim"}


class Energy(core.Entity):
    def __init__(self):
        super().__init__()
        with self.settrafarrays():           # one value per drone, resized by BlueSky
            self.is_lc = np.array([], dtype=bool)    # True = Lift+Cruise
            self.on_wing = np.array([], dtype=bool)  # Lift+Cruise currently on its wing
            self.soc = np.array([])                  # state of charge, 1.0 = full
            self.soc_start = np.array([])            # state of charge at creation (or as set by SOC)
            self.used_wh = np.array([])              # energy used since creation (Wh)
            self.power = np.array([])                # power now (W), P_TRANS during a transition
            self.trans_left = np.array([])           # seconds left of the current transition
            self.mode = np.array([], dtype=int)      # 0 hover, 1 rotor, 2 wing (see MODES)
            self.wh_hover = np.array([])             # energy used per mode (Wh)
            self.wh_rotor = np.array([])
            self.wh_wing = np.array([])
            self.wh_trans = np.array([])             # energy of wing -> rotor transitions (Wh)
            self.n_trans = np.array([], dtype=int)   # number of those transitions

    def create(self, n=1):
        super().create(n)
        self.is_lc[-n:] = [t.upper() in LC_TYPES for t in traf.type[-n:]]
        self.on_wing[-n:] = self.is_lc[-n:] & (traf.tas[-n:] >= LC.v_stall)
        self.soc[-n:] = 1.0
        self.soc_start[-n:] = 1.0

    @core.timed_function(name="energy", dt=DT)
    def update(self, dt):
        p_mc, mode_mc = MC.sim_power(traf.tas)            # every drone as if multicopter
        p_lc, mode_lc = LC.sim_power(traf.tas)            # every drone as if Lift+Cruise
        self.power = np.where(self.is_lc, p_lc, p_mc)     # W per drone
        self.mode = np.where(self.is_lc, mode_lc, mode_mc)
        wing = self.mode == 2

        # transition: starts when a Lift+Cruise leaves its wing, stops early if it returns to it
        left_wing = self.on_wing & ~wing
        self.trans_left = np.where(wing, 0.0, np.where(left_wing, T_TRANS, self.trans_left))
        t_trans = np.minimum(dt, self.trans_left)                     # s of this step in transition
        self.trans_left = np.maximum(self.trans_left - dt, 0.0)
        self.on_wing = wing

        e_flight = self.power * (dt - t_trans) / 3600                 # Wh this step, outside transition
        e_trans = P_TRANS * t_trans / 3600                            # Wh this step, in transition
        self.power = np.where(t_trans > 0, P_TRANS, self.power)

        self.wh_hover += np.where(self.mode == 0, e_flight, 0.0)
        self.wh_rotor += np.where(self.mode == 1, e_flight, 0.0)
        self.wh_wing += np.where(self.mode == 2, e_flight, 0.0)
        self.wh_trans += e_trans
        self.n_trans += left_wing

        used = e_flight + e_trans
        self.used_wh += used
        self.soc -= used / np.where(self.is_lc, LC.battery_wh, MC.battery_wh)

    @stack.command(name="SOC")
    def soc_command(self, acid: "acid", value: float = -1.0):
        """SOC acid [value]: show the state of charge of a drone, or set it (0-1)."""
        if value >= 0:
            self.soc[acid] = self.soc_start[acid] = value
        return True, f"{traf.id[acid]}: SoC {self.soc[acid]:.4f}, used {self.used_wh[acid]:.2f} Wh"
