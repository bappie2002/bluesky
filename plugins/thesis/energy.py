""" ENERGY: battery state of charge (SoC) of every drone.

Every second:  energy used = P(configuration, airspeed) x dt,
and that energy is subtracted from the battery.
Per drone it also keeps the power right now, the flight mode (hover, rotor,
wing) and how much energy went into each mode; FLIGHTLOG writes these out.
Formulas and parameter choices: experimenten/config/Energiemodel_uitleg.docx
"""
import numpy as np
from bluesky import core, stack, traf

# =============================================================================
# PARAMETERS: change them here (values and reasons: config/aannames.md)
# =============================================================================
RHO = 1.225   # air density (kg/m3)
G = 9.81      # gravity (m/s2)
DT = 1.0      # update interval of this plugin (s)
HOVER_SPEED = 0.5   # m/s: below this airspeed a drone counts as hovering

# Flight modes, numbered 0, 1, 2 (also used by FLIGHTLOG)
#   hover: airspeed below HOVER_SPEED
#   rotor: moving on the rotors (multicopter forward flight, Lift+Cruise below stall)
#   wing:  Lift+Cruise carried by its wing
MODES = ["hover", "rotor", "wing"]

# BlueSky aircraft type that stands for each configuration (for its flight
# performance only; the energy comes from this plugin).
LC_TYPES = {"AMZN"}   # these types are Lift+Cruise; all other types are multicopters

MC = dict(            # generic multicopter
    mass=3.0,             # kg, without payload (arrivals return empty)
    disk_loading=7.2,     # kg/m2 = mass / total rotor area
    eta=0.60,             # hover efficiency = figure of merit x motor efficiency
    battery_wh=188.0,     # Wh
    ref_area_cm2=624.0,   # body reference area (cm2), sets where the curve rises
)

LC = dict(            # generic Lift+Cruise
    mass=5.0,             # kg, without payload
    disk_loading=10.0,    # kg/m2 of the lift rotors
    eta=0.45,             # hover efficiency of the lift rotors
    battery_wh=480.0,     # Wh
    wing_loading=7.0,     # kg/m2 = mass / wing area
    aspect_ratio=6.0,     # span^2 / wing area
    cl_max=0.8,           # maximum lift coefficient
    cd0=0.03,             # zero-lift drag coefficient (incl. booms, stopped rotors)
    oswald=0.8,           # span efficiency e
    eta_prop=0.6,         # pusher propeller x motor efficiency
    p_avionics=10.0,      # W
)

# Shape of slow flight on the lift rotors (Mathur & Atkins 2024, Table 2, Quad)
ROTOR_SHAPE = [270.2, 21.66, -18.97, 5.822, -0.4229]   # c0 ... c4


# =============================================================================
# FORMULAS
# =============================================================================
def hover_power(p):
    """(1) Hover, momentum theory: P_h = (m g)^1.5 / (eta * sqrt(2 rho A)).
    The rotors push air down; A = m / disk_loading is the total rotor area."""
    area = p["mass"] / p["disk_loading"]
    return (p["mass"] * G) ** 1.5 / (p["eta"] * np.sqrt(2 * RHO * area))


def induced(x):
    """(2) Induced power in forward flight divided by its hover value.
    Falls with speed: the rotor gets fresh air. x = V / v_ih."""
    return np.sqrt(-x**2 / 2 + np.sqrt(x**4 / 4 + 1))


def setup_mc(p):
    """Constants of the multicopter curve, computed once."""
    p["P_h"] = hover_power(p)
    p["v_ih"] = np.sqrt(p["disk_loading"] * G / (2 * RHO))   # air speed through the rotor in hover
    # (3) best-endurance and best-range speed (Bauersfeld & Scaramuzza 2024, eq. 18)
    v_e = p["v_ih"] / (0.10188 + 0.071358 * p["v_ih"] + 0.0007381 * p["ref_area_cm2"])
    v_r = p["v_ih"] / (0.041546 + 0.041122 * p["v_ih"] + 0.00053292 * p["ref_area_cm2"])
    # (4) weights k_i, k_p so that P(v_e) = 0.914 P_h and P(v_r) = 1.092 P_h (eq. 17)
    xe, xr = v_e / p["v_ih"], v_r / p["v_ih"]
    p["k_i"], p["k_p"] = np.linalg.solve([[induced(xe) - 1, xe**3],
                                          [induced(xr) - 1, xr**3]], [0.914 - 1, 1.092 - 1])


def mc_power(p, v):
    """(5) Multicopter: P = P_h * [ (1 - k_i) + k_i * induced + k_p * (V/v_ih)^3 ].
    constant part + induced part (falls) + body drag (rises with V^3). V = 0 is hover."""
    x = v / p["v_ih"]
    return p["P_h"] * (1 - p["k_i"] + p["k_i"] * induced(x) + p["k_p"] * x**3)


def setup_lc(p):
    """Constants of the Lift+Cruise curves, computed once."""
    p["P_h"] = hover_power(p)
    p["S"] = p["mass"] / p["wing_loading"]                                   # wing area
    # (6) stall speed: below it the wing cannot carry the drone
    p["v_stall"] = np.sqrt(2 * p["mass"] * G / (RHO * p["S"] * p["cl_max"]))
    # (9) transition wing -> rotors: 2.04 x hover power during 10 s (Mathur & Atkins 2025)
    p["E_tr"] = 2.04 * p["P_h"] * 10.0


def lc_wing_power(p, v):
    """(7) On the wing: P = drag * V / eta_prop + P_avionics, with
    drag = parasite (cd0, grows with speed) + induced (making lift, large at low speed)."""
    v = np.maximum(v, p["v_stall"])          # only valid on the wing
    q = 0.5 * RHO * v**2                      # dynamic pressure
    cl = p["mass"] * G / (q * p["S"])         # lift coefficient needed to carry the weight
    drag = q * p["S"] * (p["cd0"] + cl**2 / (np.pi * p["oswald"] * p["aspect_ratio"]))
    return drag * v / p["eta_prop"] + p["p_avionics"]


def lc_rotor_power(p, v):
    """(8) On the lift rotors: hover power x shape of Mathur's curve (1.0 at V = 0).
    Valid up to 6.5 m/s; between 6.5 m/s and stall the 6.5 m/s value is used (assumption)."""
    v = np.minimum(v, 6.5)
    return p["P_h"] * np.polyval(ROTOR_SHAPE[::-1], v) / ROTOR_SHAPE[0]


setup_mc(MC)
setup_lc(LC)


# =============================================================================
# BLUESKY PART
# =============================================================================
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
            self.power = np.array([])                # power now (W), without transitions
            self.mode = np.array([], dtype=int)      # 0 hover, 1 rotor, 2 wing (see MODES)
            self.wh_hover = np.array([])             # energy used per mode (Wh)
            self.wh_rotor = np.array([])
            self.wh_wing = np.array([])
            self.wh_trans = np.array([])             # energy of wing -> rotor transitions (Wh)
            self.n_trans = np.array([], dtype=int)   # number of those transitions

    def create(self, n=1):
        super().create(n)
        self.is_lc[-n:] = [t.upper() in LC_TYPES for t in traf.type[-n:]]
        self.on_wing[-n:] = self.is_lc[-n:] & (traf.tas[-n:] >= LC["v_stall"])
        self.soc[-n:] = 1.0
        self.soc_start[-n:] = 1.0
        self.used_wh[-n:] = 0.0

    @core.timed_function(name="energy", dt=DT)
    def update(self, dt):
        v = traf.tas                                      # true airspeed (m/s)
        wing = self.is_lc & (v >= LC["v_stall"])          # L+C fast enough for its wing
        p_lc = np.where(wing, lc_wing_power(LC, v), lc_rotor_power(LC, v))
        self.power = np.where(self.is_lc, p_lc, mc_power(MC, v))      # W per drone
        self.mode = np.where(wing, 2, np.where(v < HOVER_SPEED, 0, 1))

        e_flight = self.power * dt / 3600                             # Wh this step
        e_trans = np.where(self.on_wing & ~wing, LC["E_tr"], 0.0) / 3600   # (9) just left the wing
        self.on_wing = wing

        self.wh_hover += np.where(self.mode == 0, e_flight, 0.0)
        self.wh_rotor += np.where(self.mode == 1, e_flight, 0.0)
        self.wh_wing += np.where(self.mode == 2, e_flight, 0.0)
        self.wh_trans += e_trans
        self.n_trans += e_trans > 0

        used = e_flight + e_trans
        self.used_wh += used
        self.soc -= used / np.where(self.is_lc, LC["battery_wh"], MC["battery_wh"])

    @stack.command(name="SOC")
    def soc_command(self, acid: "acid", value: float = -1.0):
        """SOC acid [value]: show the state of charge of a drone, or set it (0-1)."""
        if value >= 0:
            self.soc[acid] = self.soc_start[acid] = value
        return True, f"{traf.id[acid]}: SoC {self.soc[acid]:.4f}, used {self.used_wh[acid]:.2f} Wh"
