""" VPMS: the vertiport point-merge structure of Chang et al. (2026), scaled to cargo drones.

Geometry (methodology, Airspace Structure and Separation; decisions 22, 23 and 26-10-2026):
  - landing pad at PAD_LAT, PAD_LON; merge point 40 m straight above it
  - four gates on the bearings 0, 90, 180 and 270 deg, each at the height of its own arc
    (decision 29-10-2026: drones descend from the transit airspace before the gate), GATE_MARGIN
    outside the outer arc, so the entry leg is level
  - per gate an arc of 150 deg around the pad, flown clockwise, at 100 m (gates 0 and 2)
    or 90 m (gates 1 and 3): neighbouring arcs overlap 60 deg and lie 10 m apart vertically,
    more than the vertical minimum of 7.62 m
  - arc radius r = 2 T_target V / theta: an arc absorbs two target separations
      VARIANT "shared": one arc per gate for both types, radius set by the L+C (379 m)
      VARIANT "speed":  per gate an MC arc (174 m) and an L+C arc (379 m), same height

What a drone does (one state per drone):
  ENTRY   gate -> start of its arc, level, at the arc height. TRAFFIC delivers at most one drone
          per T_GATE at a gate: in the speed variant an MC and an L+C share the entry leg, and
          without this the faster L+C catches up with the MC (decision 29-10-2026). Drones enter
          at their drawn time (option B): bunches are absorbed on the arcs, not before the gate
  ARC     along the arc at its waiting speed. No overtaking (decision 26-10-2026): a drone
          that closes in on the drone ahead on the same arc takes over its speed. For an
          L+C behind a multicopter this means leaving the wing and flying on the rotors.
          Decision 29-10-2026: entry leg and arc form one lane per arc; a follower keeps GAP_MIN
          (64 m = 2 x SEP_H) to the drone ahead, not T_TARGET (the 40 s is a wish at the merge point),
          and a drone that leaves for the merge point stays its leader until SEP_H + 5 m inside the arc
  HOLD    hover at the end of the arc while not cleared (as in Chang et al.)
  DIRECT  cleared: straight to the merge point, descending to 40 m, and decelerating so
          that it arrives there in hover (an L+C finishes its transition before it)
  FINAL   vertical descent: 2 m/s down to 5 m, 0.5 m/s below that; never closer than
          SEP_V above the drone below it, also not above a drone that is still on the pad.
          This gives the minimum time between two landings (17.3 s with a pad time of 6 s).
  PAD     on the pad for its pad time (2 s standing still + time to take it off), then deleted

Sequencing in this file is first-come-first-served (case C1): drones are cleared in the
order in which they entered their arc, each at least T_TARGET after the previous one at
the merge point. A later SEQUENCER plugin only has to replace next_clearance().

Disturbance (decision 27-10-2026: no wind in this study): every drone has its own error in
the predicted flight time to the merge point (eta_err, drawn by TRAFFIC). The sequencer plans
with the prediction, the drone flies the real time, so the real intervals at the merge point
differ from the planned ones. The structure also works with wind (hover = nose into the wind,
position held), for later research; this study does not use it.

Drones are handed to VPMS by the TRAFFIC plugin (register); other drones are ignored.
Events for FLIGHTLOG: arc_in, hold, direct_to, merge, touchdown.
Commands: VPMS SHOW (draw the structure), VPMS SHARED / VPMS SPEED (variant, before traffic),
          VPMS TARGET 40 (target separation in s, before traffic; the arc radius follows it),
          VPMS GATE 10 (minimum time between two drones at the same gate, before traffic),
          VPMS TIME / VPMS DISTANCE (spacing a follower keeps in its lane, ARC_SPACING; open).
"""
import os
import sys
import numpy as np
from bluesky import core, stack, traf, sim
from bluesky.tools import aero
from bluesky.tools.aero import vtas2cas

EXPERIMENTEN = os.environ.get("EXPERIMENTEN", os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "experimenten"))
sys.path.insert(0, os.path.join(EXPERIMENTEN, "config"))
from vehicles import GENERIC_MC as MC, GENERIC_LC as LC   # noqa: E402

# =============================================================================
# PARAMETERS (methodology 23-10-2026; placeholders marked "open")
# =============================================================================
VARIANT = "shared"            # "shared" (case C1) or "speed" (arcs per speed class)
PAD_LAT, PAD_LON = 52.0, 4.0  # landing pad
H_ARC = (100.0, 90.0)         # m: arc height for even / odd gates; the gate lies at the same height
H_MERGE = 40.0                # m: merge point, straight above the pad
GATE_BEARINGS = (0.0, 90.0, 180.0, 270.0)
ARC_DEG = 150.0               # arc extent (deg)
ARC_DIR = 1                   # 1 = clockwise seen from above
T_TARGET = 40.0               # s: target separation at arc entry and merge point
SEP_H, SEP_V = 32.0, 7.62     # m: separation minima (Metropolis II)
LC_TYPES = {"AMZN"}           # BlueSky types that are Lift+Cruise (same as ENERGY)
V_ARC = (MC.v_endurance, LC.v_arc)    # m/s waiting speed (MC v_e 5.7, L+C 1.05 Vstall 12.4)
V_ENTRY = (MC.v_range, LC.v_arc)      # m/s from gate to arc (MC v_r 9.0, L+C on the wing): fixed for now,
                                      # the L+C speed window follows with the sequencer (decision 29-10-2026)
VS_ENTRY = 2.0                # m/s: vertical speed for height corrections on the entry leg and arcs
VS_DIRECT_MAX = 2.0           # m/s largest descent rate on the direct leg: no drone descends faster than
                              #    2 m/s anywhere (vortex ring state, decision 29-10-2026; see V_DESC)
GATE_MARGIN = 100.0           # m: level stretch gate -> outer arc: the L+C starts its turn onto the
                              #    arc look_ahead (67 m) before the arc start, after ~2.5 s straight
T_GATE = 10.0                 # s: minimum time between two drones at the same gate (any type): the shortest
                              #    value that works, the kinematic minimum of an L+C behind an MC (9.7 s,
                              #    gate_spacing_min()) rounded up (decision 29-10-2026)
A_DECEL = LC.v_arc ** 2 / (2 * 83.0)  # m/s2: L+C stops in 83 m (Mathur transition); also MC
A_ACC = 2.0                   # m/s2: speeding up from a hold (BlueSky axmax of M200 and AMZN)
V_DESC = (2.0, 0.5)           # m/s final descent above / below H_SLOW (profile P2). 2 m/s is 0.37 (MC) and
                              #    0.32 (L+C) of the induced velocity in hover: below the onset of the vortex
                              #    ring state (about 0.4, Talaeizadeh et al.; Mori et al. saw none at about 0.65)
H_SLOW = 5.0                  # m
T_PAD = 6.0                   # s on the pad when TRAFFIC gives none
L_MIN = 15.0                  # m: smallest look-ahead distance on the arc
T_CLOSE = 10.0                # s: how fast a follower restores its spacing on the arc
GAP_MIN = 2 * SEP_H           # m: smallest gap on an arc, behind a drone that hovers
ARC_SPACING = "distance"      # what a follower keeps in its lane (entry leg + arc): "distance" = GAP_MIN, a
                              #    buffer of 2 x SEP_H above the hard minimum (decision 29-10-2026; the 40 s is
                              #    a wish at the merge point, not a rule in the lane). "time" = max(T_TARGET x
                              #    v_lead, GAP_MIN), the earlier rule, kept for the comparison test
V_GROUND = 4.0                # m/s: below this airspeed on an arc, steer on ground speed (wind)
BANK = 25.0                   # deg: bank angle of the drones in BlueSky (bankdef)
DT = 0.5                      # s: update interval

ENTRY, ARC, HOLD, DIRECT, FINAL, PAD = 1, 2, 3, 4, 5, 6
STATE_NAMES = {0: "-", ENTRY: "entry", ARC: "arc", HOLD: "hold", DIRECT: "direct",
               FINAL: "final", PAD: "pad"}
M_PER_DEG = 111320.0
COSLAT = np.cos(np.radians(PAD_LAT))


# =============================================================================
# GEOMETRY
# =============================================================================
def arc_radius(is_lc):
    """Arc radius (m): two target separations at the waiting speed over 150 deg."""
    v = V_ARC[1] if VARIANT == "shared" else V_ARC[int(is_lc)]
    return 2 * T_TARGET * v / np.radians(ARC_DEG)


def gate_distance():
    """Horizontal distance pad -> gate (m): outer arc + level stretch (decision 29-10-2026)."""
    return arc_radius(True) + GATE_MARGIN


def gate_spacing_min():
    """Smallest time (s) between an MC and a following L+C at the same gate (speed variant). Both fly
    the radial line from the gate inwards, at the same height, and the L+C is faster. Two cases:
      A. the L+C turns onto its arc: it leaves the line look_ahead before the start of the arc;
      B. the L+C is cleared at once and flies direct to the merge point along the same line, at arc
         height until SEP_H + 5 m inside its arc; it is clear of the MC once it is SEP_V lower.
    The L+C must stay SEP_H behind the MC until then. Straight-line kinematics; B is the binding case
    (about 9.7 s). In case A the start of the turn adds about 0.6 s (A then about 5 s)."""
    v_mc, v_lc = V_ENTRY[0], V_ENTRY[1]
    s_a = max(GATE_MARGIN - look_ahead(v_lc) - v_lc * DT, 0.0)           # m from the gate
    d0 = arc_radius(True) - SEP_H - 5.0                                   # descent starts here
    vs = min((min(H_ARC) - H_MERGE) / time_to_merge(d0 - 3.0, v_lc), VS_DIRECT_MAX)  # lowest arc: slowest
    s_b = gate_distance() - d0 + v_lc * SEP_V / vs
    return max((SEP_H + s) / v_mc - s / v_lc for s in (s_a, s_b))


def arc_key(gate, is_lc):
    return gate + 4 * int(is_lc) if VARIANT == "speed" else gate


def to_xy(lat, lon):
    """Metres east and north of the pad."""
    return (lon - PAD_LON) * COSLAT * M_PER_DEG, (lat - PAD_LAT) * M_PER_DEG


def to_latlon(x, y):
    return PAD_LAT + y / M_PER_DEG, PAD_LON + x / (COSLAT * M_PER_DEG)


def polar(bearing, dist):
    """Point at this bearing (deg) and distance (m) from the pad, as x, y."""
    b = np.radians(bearing)
    return dist * np.sin(b), dist * np.cos(b)


def gate(g):
    """Gate g: latitude, longitude, altitude (m) and heading towards the pad."""
    lat, lon = to_latlon(*polar(GATE_BEARINGS[g], gate_distance()))
    return lat, lon, H_ARC[g % 2], (GATE_BEARINGS[g] + 180) % 360


def entry_leg_time(is_lc):
    return (gate_distance() - arc_radius(is_lc)) / V_ENTRY[int(is_lc)]


def look_ahead(gs):
    """Look-ahead distance (m) for following the arc: twice the turn radius at this ground speed,
    so a drone starts its turn onto the arc in time and does not overshoot it."""
    return max(L_MIN, 2 * gs ** 2 / (9.81 * np.tan(np.radians(BANK))))


def time_to_merge(dist, v):
    """Time (s) to fly dist at speed v and stop at the merge point with deceleration A_DECEL."""
    d_stop = v ** 2 / (2 * A_DECEL)
    if dist > d_stop:
        return (dist - d_stop) / v + v / A_DECEL
    return np.sqrt(2 * dist / A_DECEL)


def follow_speed(gap, v_lead, v_own):
    """Speed of a follower at this gap (m) behind a leader at v_lead (m/s): no overtaking. It closes
    or opens the gap to the wanted spacing in about T_CLOSE s, and can always stop GAP_MIN behind."""
    gap_needed = max(T_TARGET * v_lead, GAP_MIN) if ARC_SPACING == "time" else GAP_MIN
    v_stop = v_lead + np.sqrt(2 * A_DECEL * max(gap - GAP_MIN, 0.0))
    return float(np.clip(min(v_lead + (gap - gap_needed) / T_CLOSE, v_stop), 0.0, v_own))


def flightlog():
    for name, mod in sys.modules.items():
        if name.endswith(".flightlog") and hasattr(mod, "FlightLog"):
            return mod.FlightLog.implinstance()
    return None


def init_plugin():
    # Drones never fly at Mach numbers: BlueSky reads speeds between 0.1 and 2 as Mach unless
    # this threshold is lowered, and drones hover and descend at exactly such speeds.
    aero.casmachthr(0.01)
    Vpms()
    return {"plugin_name": "VPMS", "plugin_type": "sim"}


class Vpms(core.Entity):
    def __init__(self):
        super().__init__()
        with self.settrafarrays():
            self.state = np.array([], dtype=int)     # 0 = not managed by VPMS
            self.is_lc = np.array([], dtype=bool)
            self.gate = np.array([], dtype=int)
            self.t_arc_in = np.array([])             # time the drone entered its arc
            self.t_merge = np.array([])              # planned time at the merge point
            self.vs_direct = np.array([])            # descent rate on the direct leg (m/s)
            self.t_touch = np.array([])              # touchdown time
            self.t_pad = np.array([])                # time on the pad (s)
            self.eta_err = np.array([])              # relative error of the predicted time to the merge point
            self.hx = np.array([])                   # hover position (m east, north)
            self.hy = np.array([])
        self.reset_state()

    def reset_state(self):
        self.pending = {}            # acid -> (gate, pad time), filled by TRAFFIC
        self.last_entry = {}         # arc key -> planned arc-entry time of the last drone
        self.last_gate = {}          # gate -> time the last drone passed it
        self.t_last_merge = -np.inf  # planned merge time of the last cleared drone
        self.merges = []             # (time, acid) at the merge point
        self.touchdowns = []         # (time, acid) on the pad

    def reset(self):
        super().reset()
        self.reset_state()

    # ------------------------------------------------------------------ interface
    def entry_time(self, g, is_lc, t_now):
        """Earliest time a drone may pass gate g: it enters its arc at least T_TARGET after the
        previous drone on that arc (first control point of Chang), and passes the gate at least
        T_GATE after the previous drone at that gate. Not used by TRAFFIC since 29-10-2026
        (option B: no waiting before the gate); kept for a metered variant."""
        last = self.last_entry.get(arc_key(g, is_lc), -np.inf)
        return max(t_now, last + T_TARGET - entry_leg_time(is_lc),
                   self.last_gate.get(g, -np.inf) + T_GATE)

    def register(self, acid, g, is_lc, t_now, t_pad=T_PAD, eta_err=0.0):
        """Called by TRAFFIC just before it creates the drone at gate g."""
        self.pending[acid] = (g, t_pad, eta_err)
        self.last_entry[arc_key(g, is_lc)] = t_now + entry_leg_time(is_lc)
        self.last_gate[g] = t_now

    def create(self, n=1):
        super().create(n)
        for k in range(traf.ntraf - n, traf.ntraf):
            info = self.pending.pop(traf.id[k], None)
            if info is None:
                self.state[k] = 0
                continue
            self.state[k] = ENTRY
            self.is_lc[k] = traf.type[k].upper() in LC_TYPES
            self.gate[k], self.t_pad[k], self.eta_err[k] = info
            self.t_arc_in[k] = self.t_merge[k] = self.t_touch[k] = np.inf

    def event(self, i, name, info=""):
        fl = flightlog()
        if fl is not None and fl.files:
            fl.event(i, name, info)

    def departing_limit(self, i, idx, x, y, dist, v):
        """Decision 29-10-2026 (2b): a drone that left this lane for the merge point stays a leader for
        the drones still in the lane until it is SEP_H + 5 m inside its arc: the follower then cannot
        pass the point of departure too closely. Gap = straight-line distance, leader speed = its speed."""
        key = arc_key(self.gate[i], self.is_lc[i])
        trk = np.radians(traf.trk[i])
        for j in idx:
            if j == i or self.state[j] != DIRECT or arc_key(self.gate[j], self.is_lc[j]) != key:
                continue
            if dist[j] < arc_radius(self.is_lc[j]) - SEP_H - 5.0:
                continue
            dx, dy = x[j] - x[i], y[j] - y[i]
            if dx * np.sin(trk) + dy * np.cos(trk) <= 0.0:   # not ahead of the follower
                continue
            v = min(v, follow_speed(np.hypot(dx, dy), traf.tas[j], v))
        return v

    # ------------------------------------------------------------------ commands to BlueSky
    def fly(self, i, track=None, tas=None, alt=None, vs=None):
        """Set track (deg), airspeed (m/s), altitude (m) and vertical speed (m/s) of drone i."""
        if track is not None:
            traf.ap.trk[i] = track % 360
            traf.swlnav[i] = False
        if tas is not None:
            traf.selspd[i] = vtas2cas(max(tas, 0.0), traf.alt[i])
            traf.swvnavspd[i] = False
        if alt is not None:
            traf.selalt[i] = max(alt, 0.0)
            traf.selvs[i] = -abs(vs) if alt < traf.alt[i] else abs(vs)
            traf.swvnav[i] = False

    def wind(self, i):
        """Wind at drone i (m/s, towards north and east)."""
        if traf.wind.winddim == 0:
            return 0.0, 0.0
        vn, ve = traf.wind.getdata(traf.lat[i:i + 1], traf.lon[i:i + 1], traf.alt[i:i + 1])
        return float(np.atleast_1d(vn)[0]), float(np.atleast_1d(ve)[0])

    def fly_ground(self, i, gn, ge):
        """Fly drone i with this ground velocity (m/s north, east), whatever the wind.
        Airspeed = ground velocity - wind; BlueSky corrects the heading for the wind."""
        wn, we = self.wind(i)
        an, ae = gn - wn, ge - we
        tas = float(np.hypot(an, ae))
        if an * gn + ae * ge > 0:            # nose roughly along the ground track
            track = np.degrees(np.arctan2(ge, gn))
        else:                                # slower than the wind: nose into the air flow
            track = np.degrees(np.arctan2(ae, an))
        self.fly(i, track=track, tas=tas)

    def cruise_gs(self, i, un, ue):
        """Ground speed towards (un, ue) when flying at the waiting airspeed in this wind."""
        wn, we = self.wind(i)
        w_along, w_cross = wn * un + we * ue, -wn * ue + we * un
        v_air = V_ARC[int(self.is_lc[i])]
        return np.sqrt(max(v_air ** 2 - w_cross ** 2, 0.0)) + w_along

    def hover(self, i, hx, hy):
        """Hover at (hx, hy): nose into the wind at airspeed = wind speed (so the energy
        model sees the right airspeed); the position is held exactly (station keeping)."""
        wn, we = self.wind(i)
        traf.lat[i], traf.lon[i] = to_latlon(hx, hy)
        self.fly(i, track=np.degrees(np.arctan2(-we, -wn)), tas=float(np.hypot(wn, we)))

    # ------------------------------------------------------------------ main loop
    @core.timed_function(name="vpms", dt=DT)
    def update(self, dt):
        idx = np.where(self.state > 0)[0]
        if len(idx) == 0:
            return
        now = sim.simt
        x, y = to_xy(traf.lat, traf.lon)
        dist = np.hypot(x, y)
        brg = np.degrees(np.arctan2(x, y)) % 360

        # progress along the arc (deg) for drones on an arc
        progress = np.zeros(traf.ntraf)
        for i in idx:
            if self.state[i] in (ARC, HOLD):
                p = ((brg[i] - GATE_BEARINGS[self.gate[i]]) * ARC_DIR) % 360
                progress[i] = 0.0 if p > 300 else p

        # 1. entry leg: to the start of the arc. No overtaking from the gate on (decision 29-10-2026):
        #    entry leg and arc form one lane per arc; a drone on the entry leg keeps its spacing to the
        #    drone ahead in that lane (the last one that entered the arc, or one ahead on the entry leg)
        entry = idx[self.state[idx] == ENTRY]
        d_start = {}
        for i in entry:
            tx, ty = polar(GATE_BEARINGS[self.gate[i]], arc_radius(self.is_lc[i]))
            d_start[i] = np.hypot(tx - x[i], ty - y[i])
        for i in entry:
            lc = int(self.is_lc[i])
            tx, ty = polar(GATE_BEARINGS[self.gate[i]], arc_radius(lc))
            d = d_start[i]
            v = V_ENTRY[lc]
            key = arc_key(self.gate[i], self.is_lc[i])
            ahead_entry = [j for j in entry if j != i and arc_key(self.gate[j], self.is_lc[j]) == key
                           and d_start[j] < d]
            ahead_arc = [j for j in idx if self.state[j] in (ARC, HOLD)
                         and arc_key(self.gate[j], self.is_lc[j]) == key]
            lead, gap = None, np.inf
            if ahead_entry:                                    # nearest drone ahead on the entry leg
                lead = max(ahead_entry, key=lambda j: d_start[j])
                gap = d - d_start[lead]
            elif ahead_arc:                                    # last drone that entered the arc
                lead = min(ahead_arc, key=lambda j: progress[j])
                gap = d + arc_radius(lc) * np.radians(progress[lead])
                if progress[lead] < 1.0:                       # still turning in: straight-line distance
                    gap = np.hypot(x[lead] - x[i], y[lead] - y[i])
            if lead is not None:
                v_lead = 0.0 if self.state[lead] == HOLD else traf.tas[lead]
                v = follow_speed(gap, v_lead, v)
            v = self.departing_limit(i, idx, x, y, dist, v)
            self.fly(i, track=np.degrees(np.arctan2(tx - x[i], ty - y[i])), tas=v,
                     alt=H_ARC[self.gate[i] % 2], vs=VS_ENTRY)
            if d < look_ahead(traf.gs[i]) + traf.gs[i] * DT:          # start the turn onto the arc
                self.state[i] = ARC
                self.t_arc_in[i] = now
                self.event(i, "arc_in", f"gate {self.gate[i]}")

        # 2. arcs: follow the circle; no overtaking on the same arc
        on_arc = idx[np.isin(self.state[idx], (ARC, HOLD))]
        for key in set(arc_key(self.gate[i], self.is_lc[i]) for i in on_arc):
            members = [i for i in on_arc if arc_key(self.gate[i], self.is_lc[i]) == key]
            members.sort(key=lambda i: -progress[i])          # first = furthest along
            for rank, i in enumerate(members):
                lc = int(self.is_lc[i])
                r = arc_radius(lc)
                if self.state[i] == ARC and progress[i] >= ARC_DEG:
                    self.state[i] = HOLD
                    self.hx[i], self.hy[i] = polar(GATE_BEARINGS[self.gate[i]] + ARC_DIR * ARC_DEG, r)
                    self.event(i, "hold")
                if self.state[i] == HOLD:
                    self.hover(i, self.hx[i], self.hy[i])
                    continue
                v = V_ARC[lc]
                if rank > 0:                                   # drone ahead on the same arc
                    lead = members[rank - 1]
                    # straight-line distance (chord), as in the separation check; on these radii it is
                    # at most 0.4 m shorter than the distance along the arc (decision 29-10-2026)
                    gap = 2 * r * np.sin(np.radians(progress[lead] - progress[i]) / 2)
                    v_lead = 0.0 if self.state[lead] == HOLD else traf.tas[lead]
                    v = follow_speed(gap, v_lead, v)
                    if self.state[lead] == ARC:                # the leader will stop at the end of the
                        d_end = r * np.radians(max(ARC_DEG - progress[i], 0.0))   # arc if not cleared:
                        v = min(v, np.sqrt(2 * A_DECEL * max(d_end - GAP_MIN, 0.0)))  # stop GAP_MIN before
                v = self.departing_limit(i, idx, x, y, dist, v)
                alt = H_ARC[self.gate[i] % 2]
                if v >= V_GROUND:                              # normal: airspeed on the arc
                    # aim at a point one look-ahead distance further along the arc (pure pursuit);
                    # placed at r / cos(a), so that the drone flies exactly on the circle of radius r
                    a = look_ahead(traf.gs[i]) / r
                    ax, ay = polar(brg[i] + ARC_DIR * np.degrees(a), r / np.cos(a))
                    track = np.degrees(np.arctan2(ax - x[i], ay - y[i]))
                    self.fly(i, track=track, tas=v, alt=alt, vs=VS_ENTRY)
                else:                                          # slow: hold the arc against the wind
                    tangent = np.radians(brg[i] + ARC_DIR * 90)
                    v_in = -np.clip((dist[i] - r) / T_CLOSE, -2.0, 2.0)          # back to the circle
                    gn = v * np.cos(tangent) + v_in * y[i] / dist[i]
                    ge = v * np.sin(tangent) + v_in * x[i] / dist[i]
                    self.fly_ground(i, gn, ge)
                    self.fly(i, alt=alt, vs=VS_ENTRY)

        # 3. clearance (first-come-first-served, case C1)
        i = self.next_clearance(idx)
        if i is not None:
            gs = self.cruise_gs(i, -y[i] / dist[i], -x[i] / dist[i])
            t = time_to_merge(max(dist[i] - 3.0, 0.0), gs)          # merge point reached at 3 m
            t += max(gs - traf.gs[i], 0.0) / (2 * A_ACC)               # speeding up from a hold
            t_pred = t * (1 + self.eta_err[i])                     # what the sequencer believes
            if now + t_pred >= self.t_last_merge + T_TARGET + DT:  # + DT: update interval
                self.state[i] = DIRECT
                self.t_merge[i] = self.t_last_merge = now + t_pred
                self.vs_direct[i] = max(traf.alt[i] - H_MERGE, 0.0) / t
                self.event(i, "direct_to", f"planned merge {now + t_pred:.1f}")

        # 4. direct leg: to the merge point, arriving in hover. The merge point is not free while
        #    the drone below it is less than SEP_V lower (hard minimum, also when the target fails)
        busy_merge = any(self.state[j] == FINAL and traf.alt[j] > H_MERGE - SEP_V for j in idx)
        for i in idx[self.state[idx] == DIRECT]:
            if dist[i] < 3.0:
                self.state[i] = FINAL
                self.merges.append((now, traf.id[i]))
                self.event(i, "merge")
                self.hover(i, 0.0, 0.0)
                continue
            un, ue = -y[i] / dist[i], -x[i] / dist[i]             # unit vector to the merge point
            stop = 1.0
            if busy_merge or any(self.state[j] == DIRECT and dist[j] < dist[i] for j in idx if j != i):
                stop = SEP_H + 3.0     # hard minimum: wait SEP_H from a merge point that is not free
            v = min(self.cruise_gs(i, un, ue), np.sqrt(2 * A_DECEL * max(dist[i] - stop, 0.0)))
            self.fly_ground(i, v * un, v * ue)
            # keep the arc height until SEP_H + 5 m inside the arc: a drone from an upper arc then
            # passes over the neighbouring lower arc with the full 10 m between them
            if dist[i] > arc_radius(self.is_lc[i]) - SEP_H - 5.0:
                self.fly(i, alt=H_ARC[self.gate[i] % 2], vs=VS_ENTRY)
            else:
                t_left = max(time_to_merge(max(dist[i] - 3.0, 0.0), max(traf.gs[i], 1.0)), 1.0)
                vs = min(max((traf.alt[i] - H_MERGE) / t_left, 0.1), VS_DIRECT_MAX)
                self.fly(i, alt=H_MERGE, vs=vs)

        # 5. final descent: never within SEP_V above the drone below (or on the pad)
        column = idx[np.isin(self.state[idx], (FINAL, PAD))]
        delete = []
        for i in column:
            if self.state[i] == PAD:
                if now >= self.t_touch[i] + self.t_pad[i]:
                    delete.append(traf.id[i])
                    self.t_pad[i] = np.inf
                continue
            below = [traf.alt[j] for j in column if j != i and traf.alt[j] < traf.alt[i]]
            floor = max(below) + SEP_V if below else 0.0
            target = floor if floor < traf.alt[i] else traf.alt[i]
            self.hover(i, 0.0, 0.0)
            if traf.alt[i] > H_SLOW + 0.05:
                self.fly(i, alt=max(target, H_SLOW), vs=V_DESC[0])
            else:
                self.fly(i, alt=target, vs=V_DESC[1])
            if traf.alt[i] <= 0.05:
                self.state[i] = PAD
                self.t_touch[i] = now
                last = self.touchdowns[-1][0] if self.touchdowns else np.nan
                self.touchdowns.append((now, traf.id[i]))
                self.event(i, "touchdown", f"{now - last:.1f} s after previous")
        for acid in delete:
            stack.stack(f"DEL {acid}")

    def next_clearance(self, idx):
        """First-come-first-served: the drone that entered its arc first and is not cleared."""
        waiting = [i for i in idx if self.state[i] in (ARC, HOLD)]
        return min(waiting, key=lambda i: self.t_arc_in[i]) if waiting else None

    # ------------------------------------------------------------------ command
    @stack.command(name="VPMS")
    def command(self, what: str = "SHOW", value: float = 0.0):
        """VPMS [SHOW | SHARED | SPEED | TARGET s | GATE s | TIME | DISTANCE]: draw the structure, choose the arc
        variant, set the target separation (s) or the minimum time between two drones at a gate (s).
        Variant, target and gate time before the traffic starts."""
        global VARIANT, T_TARGET, T_GATE, ARC_SPACING
        what = what.upper()
        if what in ("TIME", "DISTANCE"):
            ARC_SPACING = what.lower()
            return True, f"VPMS spacing in a lane: {ARC_SPACING}"
        if what == "GATE" and value >= 0:
            T_GATE = float(value)
            return True, f"VPMS gate time {T_GATE:.1f} s (kinematic minimum {gate_spacing_min():.1f} s)"
        if what == "TARGET" and value > 0:
            T_TARGET = float(value)
            return True, (f"VPMS target {T_TARGET:.0f} s: radius MC {arc_radius(False):.0f} m, "
                          f"L+C {arc_radius(True):.0f} m, gates at {gate_distance():.0f} m")
        if what in ("SHARED", "SPEED"):
            VARIANT = what.lower()
            return True, (f"VPMS variant {VARIANT}: radius MC {arc_radius(False):.0f} m, "
                          f"L+C {arc_radius(True):.0f} m, gates at {gate_distance():.0f} m")
        for g in range(len(GATE_BEARINGS)):
            for lc in ((True,) if VARIANT == "shared" else (False, True)):
                pts = [to_latlon(*polar(GATE_BEARINGS[g] + ARC_DIR * a, arc_radius(lc)))
                       for a in np.linspace(0, ARC_DEG, 31)]
                gl, gn, _, _ = gate(g)
                pts = [(gl, gn)] + pts
                coords = ",".join(f"{la:.6f},{lo:.6f}" for la, lo in pts)
                stack.stack(f"POLYLINE ARC{g}{'L' if lc else 'M'},{coords}")
        return True, (f"VPMS {VARIANT}: radius MC {arc_radius(False):.0f} m, "
                      f"L+C {arc_radius(True):.0f} m, gates at {gate_distance():.0f} m")
