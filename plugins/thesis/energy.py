""" ENERGY: battery state of charge (SoC) of every drone.

Every DT_ENERGY (1 s): energy used = P(configuration, airspeed) x dt, subtracted from the battery; per drone also the
power now, the flight mode (hover, rotor, wing) and the energy per mode, which FLIGHTLOG writes out.

Transitions of the Lift+Cruise are blocks of energy and time (Mathur et al. 2025, Table 9, scaled with the hover power).
During a block the drone uses the block power INSTEAD of the rotor or wing curve, because the block already contains
the change of speed.
  wing -> rotor: starts when it leaves its wing (airspeed below stall); full block 10 s.
  rotor -> wing: starts when, below stall, it is told to fly at or above stall (the airspeed VPMS asks for, reported
                 with tell(); without VPMS the autopilot target); full block 15 s.
A block is charged in proportion to the change of speed it covers within 0..V_TRANS: a full transition pays the full
block, an L+C that slows to 5.7 m/s behind a multicopter pays 54 %. To the rotors the block closes when the drone no
longer slows down. At most one rotor -> wing block per time off the wing; an L+C back on its wing without a block (below
stall for less than one step) gets it then. Off and on the wing therefore alternate, so an L+C created on its wing
lands with an odd number of transitions (checked in test_vpms.py and analysis/delays.py). A new block replaces one that
is still running.

Every number and formula: experimenten/config/model.py (imported as M). Command: SOC drone [value].
"""
import os
import sys
import numpy as np
from bluesky import core, stack, traf

EXPERIMENTEN = os.environ.get("EXPERIMENTEN", os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "experimenten"))
sys.path.insert(0, os.path.join(EXPERIMENTEN, "config"))
import model as M   # noqa: E402

TO_ROTOR, TO_WING = 1, 2


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
            self.power = np.array([])                # power now (W), block power during a transition
            self.trans_left = np.array([])           # share of the full block that is still open (0-1)
            self.tas_prev = np.array([])             # airspeed at the previous step (m/s), for the speed change
            self.v_top = np.array([])                # airspeed at which an L+C on its wing began to slow down (m/s)
            self.trans_kind = np.array([], dtype=int)  # 0 none, TO_ROTOR or TO_WING
            self.mode = np.array([], dtype=int)      # 0 hover, 1 rotor, 2 wing (M.MODES)
            self.wh_hover = np.array([])             # energy used per mode (Wh)
            self.wh_rotor = np.array([])
            self.wh_wing = np.array([])
            self.wh_trans = np.array([])             # energy of transitions, both directions (Wh)
            self.n_trans = np.array([], dtype=int)   # number of transitions, both directions
            self.v_told = np.array([])               # airspeed VPMS asks for (m/s); NaN = not told, use the
                                                     # autopilot target
            self.wing_started = np.array([], dtype=bool)  # a rotor -> wing block started since it left the wing

    def create(self, n=1):
        super().create(n)
        self.is_lc[-n:] = [t.upper() in M.LC_TYPES for t in traf.type[-n:]]
        self.on_wing[-n:] = self.is_lc[-n:] & (traf.tas[-n:] >= M.LC.v_stall)
        self.v_told[-n:] = np.nan
        self.wing_started[-n:] = False
        self.tas_prev[-n:] = traf.tas[-n:]
        self.v_top[-n:] = traf.tas[-n:]
        self.soc[-n:] = 1.0
        self.soc_start[-n:] = 1.0

    @core.timed_function(name="energy", dt=M.DT_ENERGY)
    def update(self, dt):
        p_mc, mode_mc = M.MC.sim_power(traf.tas)            # every drone as if multicopter
        p_lc, mode_lc = M.LC.sim_power(traf.tas)            # every drone as if Lift+Cruise
        self.power = np.where(self.is_lc, p_lc, p_mc)     # W per drone
        self.mode = np.where(self.is_lc, mode_lc, mode_mc)
        wing = self.mode == 2

        # transitions: a block of fixed power and duration (see the top of this file)
        to_rotor = self.on_wing & ~wing                                  # just left the wing
        told = np.where(np.isnan(self.v_told), traf.aporasas.tas, self.v_told)   # airspeed asked for
        started = self.wing_started & ~to_rotor                          # leaving the wing clears it
        back = self.is_lc & wing & ~self.on_wing                         # back on the wing at this step
        # told to fly on the wing, or back on it without a block: below stall for less than one step of this
        # plugin, so the moment it was told fell between two steps
        to_wing = self.is_lc & ~started & ((~wing & (told >= M.LC.v_stall)) | back)
        self.wing_started = started | to_wing
        # both in one step (it dipped below stall and is already told to go back): two transitions, and the newer
        # one, rotor -> wing, runs
        self.trans_left = np.where(to_wing | to_rotor, 1.0, self.trans_left)
        self.trans_kind = np.where(to_wing, TO_WING, np.where(to_rotor, TO_ROTOR, self.trans_kind))
        # share of a full transition covered this step: the speed gained (to the wing) or lost (to the rotors)
        # within 0..V_TRANS, as a fraction of V_TRANS
        tas = np.asarray(traf.tas, dtype=float)
        gained = np.maximum(np.minimum(tas, M.V_TRANS) - self.tas_prev, 0.0)
        # to the rotors: counted from the speed at which it began to slow down on its wing, so that a full brake
        # from V_TRANS to hover pays the full block wherever the steps of this plugin fall
        lost = np.maximum(np.minimum(np.where(to_rotor, np.maximum(self.v_top, self.tas_prev), self.tas_prev), M.V_TRANS)
                          - tas, 0.0)
        share = np.where(self.trans_kind == TO_WING, gained, np.where(self.trans_kind == TO_ROTOR, lost, 0.0)) / M.V_TRANS
        share = np.minimum(share, self.trans_left)
        t_full = np.where(self.trans_kind == TO_WING, M.T_TO_WING, M.T_TO_ROTOR)
        e_full = np.where(self.trans_kind == TO_WING, M.E_TO_WING_J, M.E_TO_ROTOR_J)
        t_trans = np.minimum(share * t_full, dt)                      # s of this step in transition
        e_trans = e_full * share / 3600                               # Wh this step, in transition
        self.trans_left -= share
        # the block closes when it is used up, when the drone is on its wing at V_TRANS (to the wing) or when it
        # no longer slows down (to the rotors)
        done = (self.trans_left <= 1e-9) | ((self.trans_kind == TO_WING) & (tas >= M.V_TRANS - 1e-6)) | \
               ((self.trans_kind == TO_ROTOR) & (lost <= 0.0) & ~to_rotor)
        self.trans_kind = np.where(done, 0, self.trans_kind)
        self.trans_left = np.where(done, 0.0, self.trans_left)
        self.v_top = np.where(wing & (tas < self.tas_prev), self.v_top, tas)
        self.tas_prev = tas.copy()
        self.on_wing = wing

        e_flight = self.power * (dt - t_trans) / 3600                 # Wh this step, outside transition
        p_block = np.where(t_trans > 0, e_trans * 3600 / np.maximum(t_trans, 1e-9), 0.0)
        self.power = np.where(t_trans > 0, p_block, self.power)

        self.wh_hover += np.where(self.mode == 0, e_flight, 0.0)
        self.wh_rotor += np.where(self.mode == 1, e_flight, 0.0)
        self.wh_wing += np.where(self.mode == 2, e_flight, 0.0)
        self.wh_trans += e_trans
        self.n_trans += to_rotor.astype(int) + to_wing.astype(int)

        used = e_flight + e_trans
        self.used_wh += used
        self.soc -= used / np.where(self.is_lc, M.LC.battery_wh, M.MC.battery_wh)

    def tell(self, i, v):
        """VPMS asks drone i to fly at airspeed v (m/s); a rotor -> wing transition starts from this."""
        self.v_told[i] = v

    @stack.command(name="SOC")
    def soc_command(self, acid: "acid", value: float = -1.0):
        """SOC acid [value]: show the state of charge of a drone, or set it (0-1)."""
        if value >= 0:
            self.soc[acid] = self.soc_start[acid] = value
        return True, f"{traf.id[acid]}: SoC {self.soc[acid]:.4f}, used {self.used_wh[acid]:.2f} Wh"
