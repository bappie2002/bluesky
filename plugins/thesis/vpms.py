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
  - entry points (decision 03-11-2026): in the speed variant every gate has two entry points,
    ENTRY_OFFSET (25 m) to either side of the gate's radial line, 50 m apart: the MC on the
    counter-clockwise side, the L+C on the clockwise side, the side to which it turns onto its arc,
    so it never crosses the MC's line. Both fly a line parallel to the radial to their arc, which
    therefore starts a few degrees from the gate bearing (arc_start_bearing: MC -8.3, L+C +3.8 deg).
    50 m and not 32 m: the direct leg of an L+C cleared at its arc start converges on the merge
    point, and must stay SEP_H from the MC's line until it is SEP_V lower (at least 37 m needed).
    The shared variant keeps one entry point per gate.

What a drone does (one state per drone):
  ENTRY   entry point -> start of its arc, level, at the arc height. TRAFFIC hands drones over at the
          transit minimum (handover_time: SEP_H behind the previous drone at the same entry point, more
          for a faster follower, decision 03-11-2026). Drones enter at their drawn time (option B):
          bunches are absorbed in the structure, not before the gate
  ARC     along the arc at its waiting speed. No overtaking (decision 26-10-2026): a drone
          that closes in on the drone ahead on the same arc takes over its speed. For an
          L+C behind a multicopter this means leaving the wing and flying on the rotors.
          Decision 29-10-2026: entry leg and arc form one lane per arc; a follower keeps GAP_MIN
          (64 m = 2 x SEP_H) to the drone ahead, not T_TARGET (the 40 s is a wish at the merge point),
          and a drone that leaves for the merge point stays its leader until SEP_H + 5 m inside the arc.
          Restoring GAP_MIN after a tight hand-over (decision 03-11-2026, a structure rule in all cases):
          a multicopter follower falls back by flying slower; an L+C cannot (it flies just above stall),
          so the L+C ahead of it speeds up, to at most V_LC_MAX, until GAP_MIN is back. If that leader
          has no room ahead, the follower falls back after all (and leaves its wing)
  HOLD    hover at the end of the arc while not cleared (as in Chang et al.). The drone ahead brakes to a
          standstill at the end of its arc (A_DECEL) and then hovers (decision 04-11-2026; before, it
          stopped there at once, which BlueSky could only do with its own, higher acceleration)
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

Flight performance in BlueSky (decision 04-11-2026): the drones are the BlueSky types GMC and GLC, the
generic drones of this thesis, with their own limits in BlueSky's OpenAP type file
(bluesky/resources/performance/OpenAP/rotor/aircraft.json): speed MC 17 m/s, L+C 15.4 m/s; vertical
speed 5 m/s up and down; ceiling 150 m; acceleration and braking A_DECEL (0.93 m/s2) for both.
Before, they were the DJI M200 and the Amazon octocopter of that file, with their limits.

Drones are handed to VPMS by the TRAFFIC plugin (register); other drones are ignored.
Events for FLIGHTLOG: arc_in, hold, direct_to, merge, touchdown, speedup, speedup_end.
Commands: VPMS SHOW (draw the structure), VPMS SHARED / VPMS SPEED (variant, before traffic),
          VPMS TARGET 40 (target separation in s, before traffic; the arc radius follows it),
          VPMS OFFSET 25 (side offset of each entry point in m, speed variant; 0 = one shared entry
          point, the design of 29-10-2026; before traffic),
          VPMS TIME / VPMS DISTANCE (spacing a follower keeps in its lane, ARC_SPACING).
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
T_TARGET = 40.0               # s: target separation at the merge point (arc entry only measured)
SEP_H, SEP_V = 32.0, 7.62     # m: separation minima (Metropolis II)
LC_TYPES = {"GLC"}            # BlueSky types that are Lift+Cruise (same as ENERGY)
V_ARC = (MC.v_endurance, LC.v_arc)    # m/s waiting speed (MC v_e 5.7, L+C 1.05 Vstall 12.4)
V_ENTRY = (MC.v_range, LC.v_arc)      # m/s from gate to arc (MC v_r 9.0, L+C on the wing): fixed for now,
                                      # the L+C speed window follows with the sequencer (decision 29-10-2026)
VS_ENTRY = 2.0                # m/s: vertical speed for height corrections on the entry leg and arcs
VS_DIRECT_MAX = 2.0           # m/s largest descent rate on the direct leg: no drone descends faster than
                              #    2 m/s anywhere (vortex ring state, decision 29-10-2026; see V_DESC)
GATE_MARGIN = 100.0           # m: level stretch gate -> outer arc: the L+C starts its turn onto the
                              #    arc look_ahead (67 m) before the arc start, after ~2.5 s straight
ENTRY_OFFSET = 25.0           # m: speed variant: each entry point lies this far to the side of the gate's
                              #    radial, MC counter-clockwise, L+C clockwise: 50 m apart (decision 03-11-2026).
                              #    Needed: >= SEP_H / (1 + r_clear / r_LC) = 18.5 m per side (see the top);
                              #    0 = one shared entry point (design of 29-10-2026, needs single_entry_time())
V_LC_MAX = 15.4               # m/s: an L+C leader speeds up to at most this to restore GAP_MIN for an L+C
                              #    behind it (decision 03-11-2026). Decision 04-11-2026: 15.4 m/s, the highest
                              #    speed of Metropolis II, so that all speeds lie within the conditions of its
                              #    separation minima (was 16.7 m/s, the calibration point of the wing model).
                              #    Also v_max of GLC in BlueSky
A_DECEL = LC.v_arc ** 2 / (2 * 83.0)  # m/s2: L+C stops in 83 m (Mathur transition); also MC
A_ACC = A_DECEL               # m/s2: speeding up (decision 04-11-2026): BlueSky accelerates and brakes GMC and
                              #    GLC at one rate, ax_max in their type file = A_DECEL (was 2.0, an assumption)
ARRIVE_R, ARRIVE_V = 0.5, 0.5 # m, m/s: a drone has arrived at the end of its arc when it is within ARRIVE_R of
                              #    it and slower than ARRIVE_V: a standstill (decision 04-11-2026)
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


def clear_distance():
    """Distance from the pad (m) at which an L+C on its direct leg is SEP_V below its arc height: it keeps
    the arc height until SEP_H + 5 m inside the arc, then descends at vs (lowest arcs: slowest, worst case)."""
    v_lc = V_ENTRY[1]
    d0 = arc_radius(True) - SEP_H - 5.0                                   # descent starts here
    vs = min((min(H_ARC) - H_MERGE) / time_to_merge(d0 - 3.0, v_lc), VS_DIRECT_MAX)
    return d0 - v_lc * SEP_V / vs                                         # about 278 m


def single_entry_time():
    """Design of 29-10-2026, kept as the reason for two entry points: with ONE entry point per gate in the
    speed variant, an MC and a following L+C fly the same line inwards, at the same height, and the L+C is
    faster. Smallest time (s) between them at the gate so that the L+C stays SEP_H behind until it leaves
    the line:  A. it turns onto its arc (look_ahead before the arc start);  B. it is cleared at once and
    flies direct along the same line until it is SEP_V lower (clear_distance()). B binds: about 9.7 s."""
    v_mc, v_lc = V_ENTRY[0], V_ENTRY[1]
    s_a = max(GATE_MARGIN - look_ahead(v_lc) - v_lc * DT, 0.0)           # m from the gate
    s_b = gate_distance() - clear_distance()
    return max((SEP_H + s) / v_mc - s / v_lc for s in (s_a, s_b))


def entry_offset_min():
    """Smallest side offset (m) of each entry point (speed variant, symmetric). The direct leg of an L+C
    cleared at its arc start points at the merge point, so it comes closer to the MC's line further in;
    at clear_distance() it must still be SEP_H away: h + h * d_clear / r_LC >= SEP_H. About 18.5 m."""
    return SEP_H / (1 + clear_distance() / arc_radius(True))


def arc_key(gate, is_lc):
    """One lane per arc: per gate in the shared variant, per gate and type in the speed variant."""
    return gate + 4 * int(is_lc) if VARIANT == "speed" else gate


def entry_key(gate, is_lc):
    """Entry point of a drone; the same as its lane (decision 03-11-2026)."""
    return arc_key(gate, is_lc)


def lateral(is_lc):
    """Side offset (m) of the entry line from the gate's radial; + = clockwise side (the side an arc turns
    to). Speed variant: the L+C clockwise, the MC counter-clockwise; shared variant: 0."""
    if VARIANT != "speed":
        return 0.0
    return ENTRY_OFFSET * ARC_DIR * (1 if is_lc else -1)


def offset_xy(g, is_lc, along):
    """Point (x, y) on the entry line of this gate and type, `along` m from the pad measured along the radial."""
    b = np.radians(GATE_BEARINGS[g])
    side = lateral(is_lc)
    return along * np.sin(b) + side * np.cos(b), along * np.cos(b) - side * np.sin(b)


def arc_start_along(is_lc):
    """Where the entry line meets the arc, measured along the radial (m)."""
    return np.sqrt(max(arc_radius(is_lc) ** 2 - lateral(is_lc) ** 2, 0.0))


def arc_start_bearing(g, is_lc):
    """Bearing (deg) of the start of the arc: the gate bearing, turned by the side offset."""
    return GATE_BEARINGS[g] + np.degrees(np.arctan2(lateral(is_lc), arc_start_along(is_lc)))


def arc_start(g, is_lc):
    """Start of the arc (x, y): where the entry line meets the circle."""
    return offset_xy(g, is_lc, arc_start_along(is_lc))


def handover_time(lead_lc, foll_lc):
    """Smallest time (s) between two drones at the same entry point (decision 03-11-2026). The transit
    airspace only guarantees SEP_H: when the follower passes the entry point, the leader must be at least
    SEP_H ahead. A faster follower (L+C behind MC, shared variant) also closes in while braking to the
    leader's speed, and reacts one step late, so it needs SEP_H + dv^2 / (2 A_DECEL) + dv DT (about 40 m).
    Plus DT: a drone is created at the first update after its drawn time."""
    v_l, v_f = V_ENTRY[int(lead_lc)], V_ENTRY[int(foll_lc)]
    dv = max(v_f - v_l, 0.0)
    return (SEP_H + dv ** 2 / (2 * A_DECEL) + dv * DT) / v_l + DT


def to_xy(lat, lon):
    """Metres east and north of the pad."""
    return (lon - PAD_LON) * COSLAT * M_PER_DEG, (lat - PAD_LAT) * M_PER_DEG


def to_latlon(x, y):
    return PAD_LAT + y / M_PER_DEG, PAD_LON + x / (COSLAT * M_PER_DEG)


def polar(bearing, dist):
    """Point at this bearing (deg) and distance (m) from the pad, as x, y."""
    b = np.radians(bearing)
    return dist * np.sin(b), dist * np.cos(b)


def gate(g, is_lc=False):
    """Entry point of gate g for this type: latitude, longitude, altitude (m) and heading, parallel to the
    radial towards the pad. Shared variant: one point per gate, the same for both types."""
    lat, lon = to_latlon(*offset_xy(g, is_lc, gate_distance()))
    return lat, lon, H_ARC[g % 2], (GATE_BEARINGS[g] + 180) % 360


def entry_leg_time(is_lc):
    return (gate_distance() - arc_start_along(is_lc)) / V_ENTRY[int(is_lc)]


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
            self.speedup = np.array([], dtype=bool)  # L+C leader speeding up for an L+C behind it
        self.reset_state()

    def reset_state(self):
        self.pending = {}            # acid -> (gate, pad time), filled by TRAFFIC
        self.last_entry = {}         # arc key -> planned arc-entry time of the last drone
        self.last_gate = {}          # entry key -> (time, is_lc) of the last drone that passed it
        self.t_last_merge = -np.inf  # planned merge time of the last cleared drone
        self.merges = []             # (time, acid) at the merge point
        self.touchdowns = []         # (time, acid) on the pad

    def reset(self):
        super().reset()
        self.reset_state()

    # ------------------------------------------------------------------ interface
    def entry_time(self, g, is_lc, t_now):
        """Earliest time a drone may pass gate g: it enters its arc at least T_TARGET after the
        previous drone on that arc (first control point of Chang), and passes its entry point at least
        handover_time() after the previous drone there. Not used by TRAFFIC since 29-10-2026
        (option B: no waiting before the gate); kept for a metered variant."""
        last = self.last_entry.get(arc_key(g, is_lc), -np.inf)
        t_prev, lc_prev = self.last_gate.get(entry_key(g, is_lc), (-np.inf, False))
        return max(t_now, last + T_TARGET - entry_leg_time(is_lc), t_prev + handover_time(lc_prev, is_lc))

    def register(self, acid, g, is_lc, t_now, t_pad=T_PAD, eta_err=0.0):
        """Called by TRAFFIC just before it creates the drone at gate g."""
        self.pending[acid] = (g, t_pad, eta_err)
        self.last_entry[arc_key(g, is_lc)] = t_now + entry_leg_time(is_lc)
        self.last_gate[entry_key(g, is_lc)] = (t_now, is_lc)

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
            self.speedup[k] = False

    def event(self, i, name, info=""):
        fl = flightlog()
        if fl is not None and fl.files:
            fl.event(i, name, info)

    def departing_limit(self, i, idx, x, y, dist, v):
        """Decision 29-10-2026 (2b): a drone that left this lane for the merge point stays a leader for
        the drones still in the lane until it is SEP_H + 5 m inside its arc: the follower then cannot
        pass the point of departure too closely. Gap = straight-line distance, leader speed = its speed.
        Decision 03-11-2026: only while the follower closes in on it. After a tight hand-over the gap can
        be below GAP_MIN when the leader leaves; their paths then diverge (the leader turns in to the merge
        point, the follower goes on along the lane), and slowing down would take an L+C off its wing."""
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
            closing = dx * (traf.gseast[j] - traf.gseast[i]) + dy * (traf.gsnorth[j] - traf.gsnorth[i]) < 0.0
            if not closing:
                continue
            gap = float(np.hypot(dx, dy))
            if self.is_lc[i] and gap > SEP_H + 2.0 and traf.tas[j] >= V_ARC[1] - 0.1:
                v = min(v, float(traf.tas[j]))      # an L+C behind a leader on its wing: do not close in,
            else:                                   # but do not fall back below stall either
                v = min(v, follow_speed(gap, traf.tas[j], v))
        return v

    def lane_speed(self, i, lead, gap, v_lead, v, restore=()):
        """Speed of drone i in its lane behind `lead` at this gap (m). Decision 03-11-2026: an L+C whose
        leader speeds up for it (in `restore` this step, or self.speedup) does not fall back, because that
        would take it below stall and off its wing; it flies no faster than its leader, so the gap cannot
        shrink, and the gap opens from the front. If the leader cannot speed up and slows down, the
        follower slows with it (and leaves its wing): the fall-back. Otherwise the normal rule."""
        if self.is_lc[i] and (lead in restore or self.speedup[lead]) and gap > SEP_H + 2.0:
            return float(min(v, v_lead))
        return follow_speed(gap, v_lead, v)

    def restore_flags(self, idx, x, y, progress, d_start):
        """Decision 03-11-2026: which L+C leaders must speed up. Per lane the drones in order (entry leg:
        minus the distance to the arc start; arc: distance along the arc); an L+C (ENTRY or ARC) whose
        follower is an L+C closer than GAP_MIN (straight line) speeds up for it."""
        lanes = {}
        for i in idx:
            if self.state[i] == ENTRY:
                pos = -d_start[i]
            elif self.state[i] in (ARC, HOLD):
                pos = arc_radius(self.is_lc[i]) * np.radians(progress[i])
            else:
                continue
            lanes.setdefault(arc_key(self.gate[i], self.is_lc[i]), []).append((pos, i))
        restore = {}
        for members in lanes.values():
            members.sort(reverse=True)                          # first = furthest along the lane
            for (_, lead), (_, foll) in zip(members, members[1:]):
                if (self.is_lc[lead] and self.is_lc[foll] and self.state[lead] in (ENTRY, ARC)
                        and self.state[foll] in (ENTRY, ARC)):
                    gap = float(np.hypot(x[lead] - x[foll], y[lead] - y[foll]))
                    if gap < GAP_MIN:
                        restore[lead] = (traf.id[foll], gap)
        return restore

    def set_speedup(self, i, on, info=""):
        """Keep the speedup flag of drone i and log its start and end."""
        if on != bool(self.speedup[i]):
            self.event(i, "speedup" if on else "speedup_end", info)
        self.speedup[i] = on

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

        # progress along the arc (deg) for drones on an arc, from the start of their arc
        progress = np.zeros(traf.ntraf)
        for i in idx:
            if self.state[i] in (ARC, HOLD):
                p = ((brg[i] - arc_start_bearing(self.gate[i], self.is_lc[i])) * ARC_DIR) % 360
                progress[i] = 0.0 if p > 300 else p

        # 1. entry leg: to the start of the arc. No overtaking from the gate on (decision 29-10-2026):
        #    entry leg and arc form one lane per arc; a drone on the entry leg keeps its spacing to the
        #    drone ahead in that lane (the last one that entered the arc, or one ahead on the entry leg)
        entry = idx[self.state[idx] == ENTRY]
        d_start = {}
        for i in entry:
            tx, ty = arc_start(self.gate[i], self.is_lc[i])
            d_start[i] = np.hypot(tx - x[i], ty - y[i])
        # L+C leaders that speed up for an L+C behind them (decision 03-11-2026)
        restore = self.restore_flags(idx, x, y, progress, d_start)
        for i in idx:
            if self.state[i] not in (ENTRY, ARC) and self.speedup[i]:
                self.set_speedup(i, False, "left the lane")
        for i in entry:
            lc = int(self.is_lc[i])
            tx, ty = arc_start(self.gate[i], lc)
            d = d_start[i]
            v = V_LC_MAX if i in restore else V_ENTRY[lc]
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
                v = self.lane_speed(i, lead, gap, v_lead, v, restore)
            v = self.departing_limit(i, idx, x, y, dist, v)
            if i in restore:
                self.set_speedup(i, v > V_ENTRY[lc] + 0.1, "for {}, gap {:.0f} m".format(*restore[i]))
            elif self.speedup[i]:
                self.set_speedup(i, False, "gap restored")
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
                d_own = r * np.radians(max(ARC_DEG - progress[i], 0.0))   # to the end of its own arc
                if self.state[i] == ARC and (progress[i] >= ARC_DEG
                                             or (d_own < ARRIVE_R and traf.gs[i] < ARRIVE_V)):
                    self.state[i] = HOLD
                    self.hx[i], self.hy[i] = polar(arc_start_bearing(self.gate[i], self.is_lc[i])
                                                   + ARC_DIR * ARC_DEG, r)
                    self.event(i, "hold")
                    if self.speedup[i]:
                        self.set_speedup(i, False, "hold")
                if self.state[i] == HOLD:
                    self.hover(i, self.hx[i], self.hy[i])
                    continue
                v = V_LC_MAX if i in restore else V_ARC[lc]
                v = min(v, np.sqrt(2 * A_DECEL * d_own))       # can always stop at the end of its arc
                if rank > 0:                                   # drone ahead on the same arc
                    lead = members[rank - 1]
                    # straight-line distance (chord), as in the separation check; on these radii it is
                    # at most 0.4 m shorter than the distance along the arc (decision 29-10-2026)
                    gap = 2 * r * np.sin(np.radians(progress[lead] - progress[i]) / 2)
                    v_lead = 0.0 if self.state[lead] == HOLD else traf.tas[lead]
                    v = self.lane_speed(i, lead, gap, v_lead, v, restore)
                    if self.state[lead] == ARC:                # the leader will stop at the end of the
                        d_end = r * np.radians(max(ARC_DEG - progress[i], 0.0))   # arc if not cleared:
                        v = min(v, np.sqrt(2 * A_DECEL * max(d_end - GAP_MIN, 0.0)))  # stop GAP_MIN before
                v = self.departing_limit(i, idx, x, y, dist, v)
                if i in restore:
                    self.set_speedup(i, v > V_ARC[lc] + 0.1, "for {}, gap {:.0f} m".format(*restore[i]))
                elif self.speedup[i]:
                    self.set_speedup(i, False, "gap restored")
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
            dv = max(gs - traf.gs[i], 0.0)                         # speeding up (from a hold: from 0):
            t += dv ** 2 / (2 * A_ACC * max(gs, 0.1))              # time lost against flying at gs all the way
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
        """VPMS [SHOW | SHARED | SPEED | TARGET s | OFFSET m | TIME | DISTANCE]: draw the structure, choose the
        arc variant, set the target separation (s) or the side offset of each entry point (m, speed variant).
        Variant, target and offset before the traffic starts."""
        global VARIANT, T_TARGET, ENTRY_OFFSET, ARC_SPACING
        what = what.upper()
        if what in ("TIME", "DISTANCE"):
            ARC_SPACING = what.lower()
            return True, f"VPMS spacing in a lane: {ARC_SPACING}"
        if what == "OFFSET" and value >= 0:
            ENTRY_OFFSET = float(value)
            return True, (f"VPMS entry offset {ENTRY_OFFSET:.1f} m per side ({2 * ENTRY_OFFSET:.0f} m apart; "
                          f"needed {entry_offset_min():.1f} m; 0 = one entry point, then "
                          f"{single_entry_time():.1f} s between an MC and an L+C)")
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
                b0 = arc_start_bearing(g, lc)
                pts = [to_latlon(*polar(b0 + ARC_DIR * a, arc_radius(lc)))
                       for a in np.linspace(0, ARC_DEG, 31)]
                gl, gn, _, _ = gate(g, lc)
                pts = [(gl, gn)] + pts
                coords = ",".join(f"{la:.6f},{lo:.6f}" for la, lo in pts)
                stack.stack(f"POLYLINE ARC{g}{'L' if lc else 'M'},{coords}")
        return True, (f"VPMS {VARIANT}: radius MC {arc_radius(False):.0f} m, "
                      f"L+C {arc_radius(True):.0f} m, gates at {gate_distance():.0f} m")
