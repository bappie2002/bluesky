"""VPMS: the vertiport point-merge structure of Chang et al. (2026), scaled to cargo drones, with
first-come-first-served sequencing (cases C1 and C2).

Structure (all numbers and the geometry in experimenten/config/model.py)
  - Landing pad at PAD_LAT, PAD_LON; merge point H_MERGE (40 m) straight above it.
  - Four gates at 0, 90, 180 and 270 deg, gate_distance() (480 m) from the pad, each at the height of its own arc.
    From a gate a level entry leg of ENTRY_LEG (100 m) leads to an arc of 150 deg around the pad, flown clockwise,
    at 100 m (gates 0, 2) or 88.57 m (gates 1, 3): overlapping arcs lie K_SPACING x SEP_V apart.
  - Arc radius 2 T_DESIGN V_ARC / theta: an arc absorbs two target separations at its waiting speed.
      VARIANT "shared": one arc per gate for both types, radius set by the L+C (380 m).
      VARIANT "speed":  per gate an MC arc (174 m) and an L+C arc (380 m) at the same height, with two entry points
                        entry_offset() (27.4 m) to either side of the radial: MC counter-clockwise, L+C clockwise.
  - Entry leg + arc form one lane per arc (per gate and type in the speed variant).

Flight of a drone (state per drone)
  ENTRY   gate -> fixed turn onto its arc, level, at V_ENTRY.
  ARC     the fixed turn (turn_geom), then along the arc at V_ARC.
  RING    flying on through the 30 deg gap to the arc of the opposite gate (the L+C; with RINGMC ON also the MC).
  HOLD    hover at the end of the arc while not cleared (with RESTART ON a drone that may keep flying flies on from
          there, G2).
  DIRECT  cleared: straight to the merge point at V_DIRECT, along a planned descent (plan_descent), arriving in hover.
  FINAL   vertical descent, 2 m/s to 5 m and 0.5 m/s below, never within K_SPACING x SEP_V above another drone.
  PAD     on the pad for its pad time, then deleted.

Rules of the structure (codes of the methodology and the Keuzes-Excel)
  - Lanes: no overtaking; a follower keeps GAP_MIN (48 m, straight line) to the drone ahead (follow_speed). A drone
    that left the lane for the merge point stays its leader until SEP_H inside the arc (departing_limit).
  - P-17 making room: a leader with an L+C behind it closer than the L+C can tolerate, or with an announced L+C that
    would arrive that close, speeds up to V_MAX (MC 11.18, L+C 15.4 m/s), also on the entry leg, and so does the
    drone ahead of it while within reach (restore_flags). A multicopter follower falls back itself. An L+C behind an
    L+C that makes room flies no faster than it only while that leader flies at its arc speed; behind an L+C that
    slows down or stands still it keeps GAP_MIN (lane_speed, block 31). A drone cleared while making room keeps
    V_MAX on its direct leg while it still leads its lane (keep_of, block 31; part of the prediction and the plan).
  - G3 hand-over protection (block 29): a drone on its entry leg whose follower was just handed over closer than
    SEP_H + one update + G3_BUFFER, or whose next announced drone would be, does not slow down: it keeps the follower's
    speed (or the speed that gives the announced drone that distance), may use the margin to its own leader for it
    (down to SEP_H + one update + G3_BUFFER), and the drone ahead of it makes room (handover_protect, restore_flags).
    Everything stays inside the structure; the transit never waits.
  - Gentle braking: a drone with an L+C behind it in its lane brakes with the deceleration of the L+C (lc_like).
  - P-16 fixed turn: every drone turns from its entry leg onto its arc along one circle per lane, TURN_MARGIN x the
    turn radius at the highest speed of the lane (MC lanes 34.2 m, L+C and shared lanes 64.8 m).
  - P-18 keep flying (RING_ON; M.may_ring): an L+C (with RING_MC also a multicopter, block 27) that is not cleared
    where it would start braking for the end of its arc flies on to the arc of the opposite gate if ring_check allows
    it; otherwise it brakes and hovers.
  - Ring (block 29): a drone on the ring also keeps its gap to the last drone on the arc it is about to join, and that
    drone makes room for an L+C; ring_check counts with the earliest time the drone can be at the junction, because it
    may speed up to make room on the way. Block 31: no L+C joins the ring while a multicopter of the opposite gate is
    slow on its entry leg near the crossing (its queue reaches the L+C circle).
  - G2 restart (RESTART_ON, on since block 27): a drone that may keep flying and hovers at the end of its arc checks
    every update whether it may fly on from there (ring_check, timed from hover) and does so, unless its clearance is
    expected within T_RESTART_MIN.
  - Announcement: VPMS may look H_ANNOUNCE ahead in the list of arrivals from TRAFFIC (announced): computed by
    M.announce_horizon from the ring flights that are on (23 s with ring and restart, 42 s with RINGMC), unless set
    with VPMS HORIZON.
  - Clearance: FCFS in the order of passing the gate, T_TARGET (40 s) apart at the merge point, given when the first
    drone is on its arc (next_clearance). Its arrival is predicted by flying the direct leg ahead (predict_time, with
    the speed it keeps, keep_of).
    A clearance waits where no conflict-free descent exists (no-go zone).
  - Merge point and column: the merge point is not free while the drone below is less than K_SPACING x SEP_V lower;
    a drone waits K_SPACING x SEP_H before it.

TRAFFIC hands drones over (register); other drones are ignored. Events for FLIGHTLOG: arc_in, hold, ring, ring_join,
ring_refused, restart, speedup, speedup_end, nogo, nogo_hold, direct_to, merge, touchdown.
Commands (all before the traffic starts; LEG, RING, RINGMC and RESTART compute the horizon again unless HORIZON s set
it; HORIZON -1 computes it again):
  VPMS SHOW | SHARED | SPEED | TARGET s | OFFSET m | LEG m | RING ON/OFF | RINGMC ON/OFF | RESTART ON/OFF | HORIZON s
This file only steers the drones; every number and formula is in experimenten/config/model.py (imported as M).
Why each choice was made, and what it replaced: experimenten/docs/BESLUITEN.md.
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
import model as M   # noqa: E402  (every number and formula: one place)


_ENERGY = {"cls": None, "n": -1}


def energy():
    """The ENERGY plugin, or None when it is not loaded (looked up again only after new modules were loaded)."""
    if _ENERGY["cls"] is None and _ENERGY["n"] != len(sys.modules):
        _ENERGY["n"] = len(sys.modules)
        for name, mod in list(sys.modules.items()):
            if name.endswith(".energy") and hasattr(mod, "Energy"):
                _ENERGY["cls"] = mod.Energy
                break
    return _ENERGY["cls"].implinstance() if _ENERGY["cls"] is not None else None


def flightlog():
    for name, mod in sys.modules.items():
        if name.endswith(".flightlog") and hasattr(mod, "FlightLog"):
            return mod.FlightLog.implinstance()
    return None


def init_plugin():
    # BlueSky reads speeds between 0.1 and 2 as Mach unless this threshold is lowered; drones hover and descend
    # at exactly such speeds
    aero.casmachthr(0.01)
    Vpms()
    return {"plugin_name": "VPMS", "plugin_type": "sim"}


# =============================================================================
# THE PLUGIN
# =============================================================================
class Vpms(core.Entity):
    def __init__(self):
        super().__init__()
        with self.settrafarrays():
            self.state = np.array([], dtype=int)     # 0 = not managed by VPMS
            self.is_lc = np.array([], dtype=bool)
            self.gate = np.array([], dtype=int)
            self.t_arc_in = np.array([])             # time the drone started its turn onto the arc
            self.t_gate = np.array([])               # time the drone passed its gate (FCFS order)
            self.t_merge = np.array([])              # planned time at the merge point
            self.t_touch = np.array([])              # touchdown time
            self.t_pad = np.array([])                # time on the pad (s)
            self.eta_err = np.array([])              # relative error of the predicted time to the merge point
            self.hx = np.array([])                   # hover position (m east, north)
            self.hy = np.array([])
            self.speedup = np.array([], dtype=bool)  # making room for an L+C behind it (P-17)
            self.ring_go = np.array([], dtype=int)   # keep flying at the end of this arc: 0 open, 1 yes, -1 no (P-18)
            self.t_ring = np.array([])               # time it first started to keep flying
            self.turning = np.array([], dtype=bool)  # on its fixed turn onto the arc (P-16)
            self.protected = np.array([], dtype=bool)  # kept its speed for a drone handed over behind it (G3)
        self.reset_state()

    def reset_state(self):
        self.pending = {}            # acid -> (gate, pad time, eta_err, time at the gate), filled by TRAFFIC
        self.t_last_merge = -np.inf  # planned merge time of the last cleared drone
        self.merges = []             # (time, acid) at the merge point
        self.touchdowns = []         # (time, acid) on the pad
        self.descent = {}            # acid -> (clearance time, planned height per update) on the direct leg
        self.nogo_since = {}         # acid -> first time its clearance waited for a no-go zone
        self.keep_v = {}             # acid -> speed it keeps on the direct leg: cleared while making room (block 31)
        self.nogo_delays = []        # (acid, s) clearance delayed by a no-go zone
        self.nogo_hold = []          # drones in a hold without a conflict-free descent (expected none)
        self.close_entries = []      # (time, acid, other, dh) created within SEP_H of another (TRAFFIC; expected none)
        self.ring_log = []           # (time, acid, event, info): ring, ring_join, ring_refused (P-18), restart (G2)
        self.speedups = []           # (time, acid, type, follower): every start of making room (P-17)
        self.sched_t = np.array([])  # list of arrivals from TRAFFIC: time at the gate, gate, type, call sign
        self.sched_g = np.array([], dtype=int)
        self.sched_lc = np.array([], dtype=bool)
        self.sched_id = np.array([], dtype=object)
        self.registered = set()      # call signs created by TRAFFIC (no longer announced)
        self.gentle = set()          # indices that brake as an L+C this update (lc_like)
        self.protect = {}            # index -> lowest speed this update (G3 hand-over protection)

    def reset(self):
        super().reset()
        self.reset_state()

    # ------------------------------------------------------------------ interface with TRAFFIC
    def set_schedule(self, t_abs, gates, is_lc, acids):
        """The list of arrivals of this run: simulation time at which each drone passes its gate, its gate, its type
        and its call sign. VPMS may look H_ANNOUNCE ahead in it (announced)."""
        order = np.argsort(np.asarray(t_abs, dtype=float), kind="stable")
        self.sched_t = np.asarray(t_abs, dtype=float)[order]
        self.sched_g = np.asarray(gates, dtype=int)[order]
        self.sched_lc = np.asarray(is_lc, dtype=bool)[order]
        self.sched_id = np.asarray(acids, dtype=object)[order]

    def announced(self, now, gate=None, key=None):
        """Drones that will pass a gate within H_ANNOUNCE s and are not created yet, as (time, is_lc), in time order;
        for one gate, or for one lane (key)."""
        if len(self.sched_t) == 0:
            return []
        lo = int(np.searchsorted(self.sched_t, now - M.DT, side="right"))
        hi = int(np.searchsorted(self.sched_t, now + M.H_ANNOUNCE, side="right"))
        out = []
        for k in range(lo, hi):
            g, lc = int(self.sched_g[k]), bool(self.sched_lc[k])
            if self.sched_id[k] in self.registered:
                continue
            if (gate is None or g == gate) and (key is None or M.arc_key(g, lc) == key):
                out.append((float(self.sched_t[k]), lc))
        return out

    def register(self, acid, g, is_lc, t_now, t_pad=M.T_PAD, eta_err=0.0, t_gate=None):
        """Called by TRAFFIC just before it creates the drone at gate g. t_gate: the exact time it passed its gate
        (default now)."""
        self.pending[acid] = (g, t_pad, eta_err, t_now if t_gate is None else t_gate)
        self.registered.add(acid)

    def create(self, n=1):
        super().create(n)
        for k in range(traf.ntraf - n, traf.ntraf):
            info = self.pending.pop(traf.id[k], None)
            if info is None:
                self.state[k] = 0
                continue
            self.state[k] = M.ENTRY
            self.is_lc[k] = traf.type[k].upper() in M.LC_TYPES
            self.gate[k], self.t_pad[k], self.eta_err[k], self.t_gate[k] = info
            self.t_arc_in[k] = self.t_merge[k] = self.t_touch[k] = self.t_ring[k] = np.inf
            self.speedup[k] = False
            self.ring_go[k] = 0
            self.turning[k] = False
            self.protected[k] = False

    def event(self, i, name, info=""):
        fl = flightlog()
        if fl is not None and fl.files:
            fl.event(i, name, info)

    # ------------------------------------------------------------------ lane rules
    def departing_limit(self, i, idx, x, y, dist, v):
        """A drone that left this lane for the merge point stays a leader for the drones still in the lane until it is
        SEP_H inside its arc, but only while the follower closes in on it. Its speed is taken along the follower's
        track: a leader that turns in moves away sideways, so the gap closes almost at the follower's own speed."""
        key = M.arc_key(self.gate[i], self.is_lc[i])
        trk = np.radians(traf.trk[i])
        for j in idx:
            if j == i or self.state[j] != M.DIRECT or M.arc_key(self.gate[j], self.is_lc[j]) != key:
                continue
            if dist[j] < M.arc_radius(self.is_lc[j]) - M.SEP_H:
                continue
            dx, dy = x[j] - x[i], y[j] - y[i]
            if dx * np.sin(trk) + dy * np.cos(trk) <= 0.0:   # not ahead of the follower
                continue
            closing = dx * (traf.gseast[j] - traf.gseast[i]) + dy * (traf.gsnorth[j] - traf.gsnorth[i]) < 0.0
            if not closing:
                continue
            gap = float(np.hypot(dx, dy))
            v_along = max(float(traf.gseast[j] * np.sin(trk) + traf.gsnorth[j] * np.cos(trk)), 0.0)
            if self.is_lc[i] and gap > M.SEP_H + 2.0 and v_along >= M.V_ARC[1] - 0.1:
                v = min(v, v_along)                 # an L+C behind a leader on its wing: do not close in,
            else:                                   # but do not fall back below stall either
                v = min(v, M.follow_speed(gap, v_along, v, self.lc_like(i)))
        return v

    def lane_order(self, idx, progress, d_start):
        """Per lane the drones on the entry leg, arc, hold and ring as (position along the lane, index)."""
        lanes = {}
        for i in idx:
            if self.state[i] == M.ENTRY:
                pos = -d_start[i]
            elif self.state[i] in (M.ARC, M.HOLD, M.RING):
                pos = M.arc_radius(self.is_lc[i]) * np.radians(progress[i])
            else:
                continue
            lanes.setdefault(M.arc_key(self.gate[i], self.is_lc[i]), []).append((pos, i))
        return lanes

    def lc_behind(self, idx, progress, d_start):
        """The multicopters that have an L+C somewhere behind them in their lane (they brake gently, lc_like)."""
        out = set()
        for members in self.lane_order(idx, progress, d_start).values():
            members.sort()                                      # from the back of the lane
            seen_lc = False
            for _, i in members:
                if seen_lc and not self.is_lc[i]:
                    out.add(i)
                seen_lc = seen_lc or bool(self.is_lc[i])
        return out

    def lc_like(self, i):
        """True for an L+C and for a drone with an L+C behind it (self.gentle): it plans its braking with the
        deceleration of the L+C, because the L+C behind it cannot brake harder."""
        return bool(self.is_lc[i]) or i in self.gentle

    def a_dec(self, i):
        return M.A_DECEL[1] if self.lc_like(i) else M.A_DECEL[0]

    def lane_speed(self, i, lead, gap, v_lead, v, restore=()):
        """Speed of drone i in its lane behind `lead` at this gap (m). An L+C whose L+C leader makes room for it does
        not fall back (that would take it off its wing): it flies no faster than its leader. Otherwise the normal rule
        (follow_speed), also behind a multicopter that makes room. Block 31: only while that leader still flies at its
        arc speed; behind a leader that slows down or stands still (the end of a queue) the L+C leaves its wing anyway,
        and with "no faster than the leader" it stopped about 115 m behind instead of GAP_MIN (L+C lanes then held about
        12 instead of about 20 drones)."""
        if (self.is_lc[i] and self.is_lc[lead] and (lead in restore or self.speedup[lead]) and gap > M.SEP_H + 2.0
                and v_lead >= M.V_ARC[1] - 0.5):     # block 31: only while the leader is on its wing and can make room;
            return float(min(v, v_lead))         # a leader that stands still cannot, so the normal rule (48 m) applies
        return M.follow_speed(gap, v_lead, v, self.lc_like(i))

    def along_lane(self, i, progress, d_start):
        """Distance (m) of drone i from its entry point along its lane (entry leg, then arc or ring)."""
        lc = bool(self.is_lc[i])
        leg = M.gate_distance() - M.arc_start_along(lc)
        if self.state[i] == M.ENTRY:
            return leg - d_start[i]
        return leg + M.arc_radius(lc) * np.radians(progress[i])

    def restore_flags(self, idx, x, y, progress, d_start, now=None, seeds=None):
        """Making room (P-17): the leaders that speed up, as {index: (for whom, gap)}. Per lane, a leader whose follower
        is an L+C closer than GAP_MIN + T_CLOSE x (V_ARC of the L+C - leader speed) speeds up; so does the last drone of a
        lane when an announced L+C would pass the gate that close behind it; and so does the drone ahead of a leader that
        speeds up while it is within reach (a chain). A multicopter follower does not make its leader speed up."""
        restore, moving = {}, (M.ENTRY, M.ARC, M.RING)
        for i in idx:                       # block 29: the last drone on an arc makes room for an L+C ring flyer that is
            if self.state[i] != M.RING or not self.is_lc[i]:                # about to join that arc behind it
                continue
            g2 = (int(self.gate[i]) + 2) % len(M.GATE_BEARINGS)
            ahead2 = [j for j in idx if int(self.gate[j]) == g2 and self.state[j] == M.ARC
                      and M.arc_key(g2, self.is_lc[j]) == M.arc_key(g2, True)]
            if ahead2:
                lead = min(ahead2, key=lambda j: progress[j])
                gap = float(np.hypot(x[lead] - x[i], y[lead] - y[i]))
                if gap < M.GAP_MIN + M.T_CLOSE * max(M.V_ARC[1] - float(traf.tas[lead]), 0.0):
                    restore.setdefault(lead, (traf.id[i], gap))
        for key, members in self.lane_order(idx, progress, d_start).items():
            members.sort(reverse=True)                          # first = furthest along the lane
            pairs = []
            for (_, lead), (_, foll) in zip(members, members[1:]):
                if self.state[lead] in moving and self.state[foll] in moving:
                    gap = float(np.hypot(x[lead] - x[foll], y[lead] - y[foll]))
                    pairs.append((lead, foll, gap))
                    reach_l = M.GAP_MIN + M.T_CLOSE * max(M.V_ARC[1] - float(traf.tas[lead]), 0.0)
                    if self.is_lc[foll] and gap < reach_l:
                        restore[lead] = (traf.id[foll], gap)
            last = members[-1][1]                               # an announced L+C behind the last drone of the lane
            if now is not None and self.state[last] in moving and last not in restore:
                s_last = self.along_lane(last, progress, d_start)
                for t_a, lc_a in self.announced(now, key=key):
                    gap_pred = s_last + float(traf.tas[last]) * max(t_a - now, 0.0)
                    if lc_a and gap_pred < M.GAP_MIN + M.T_CLOSE * max(M.V_ARC[1] - float(traf.tas[last]), 0.0):
                        restore[last] = (f"an L+C announced in {t_a - now:.1f} s", gap_pred)
                        break
            for _, j in members:                                # G3: drones ahead of a protected drone
                if seeds and j in seeds and j not in restore and self.state[j] in moving:
                    restore[j] = seeds[j]
            for lead, foll, gap in reversed(pairs):             # the chain
                if foll in restore and lead not in restore:
                    reach = M.GAP_MIN + M.T_CLOSE * max(M.V_MAX[int(self.is_lc[foll])] - M.V_ARC[int(self.is_lc[lead])], 0.0)
                    if gap < reach:
                        restore[lead] = (traf.id[foll], gap)
        return restore

    def handover_protect(self, idx, x, y, progress, d_start, now):
        """G3 hand-over protection (block 29): {index: lowest speed} for drones on the entry leg that must not slow down,
        because the drone behind them in their lane was just handed over at the minimum, or the next drone announced
        for their lane would otherwise be handed over closer than SEP_H plus one update and G3_BUFFER. The transit
        delivers a drone SEP_H behind the previous one at its entry speed; when that drone slows down right after the
        gate (its lane is full), the new one appears too close. The protected drone keeps the follower's speed, or the
        speed that gives the announced drone its distance, and may use the margin to its own leader for it (fly_entry_legs)."""
        prot, room = {}, {}
        for key, members in self.lane_order(idx, progress, d_start).items():
            members.sort(reverse=True)                          # first = furthest along the lane
            for (_, lead), (_, foll) in zip(members, members[1:]):
                if self.state[lead] != M.ENTRY or self.state[foll] != M.ENTRY:
                    continue
                v_f = float(traf.tas[foll])
                gap = float(np.hypot(x[lead] - x[foll], y[lead] - y[foll]))
                if gap < M.SEP_H + v_f * M.DT + M.G3_BUFFER:
                    prot[lead] = max(prot.get(lead, 0.0), v_f)
            last = members[-1][1]
            if self.state[last] != M.ENTRY:
                continue
            s_last = self.along_lane(last, progress, d_start)
            for t_a, lc_a in self.announced(now, key=key):      # only the next drone of this lane
                v0 = M.V_ENTRY[int(lc_a)]
                need = M.SEP_H + v0 * M.DT + M.G3_BUFFER
                dt_a = max(t_a - now, M.DT)
                if s_last + float(traf.tas[last]) * dt_a < need:
                    v_req = min(max(v0, (need - s_last) / dt_a), M.V_MAX[int(self.is_lc[last])])
                    prot[last] = max(prot.get(last, 0.0), v_req)
                break
            for (_, lead), (_, foll) in zip(members, members[1:]):   # the drone ahead of a protected drone makes room
                if foll in prot and self.state[lead] in (M.ENTRY, M.ARC, M.RING):
                    gap = float(np.hypot(x[lead] - x[foll], y[lead] - y[foll]))
                    if gap < M.GAP_MIN + M.T_CLOSE * max(prot[foll] - float(traf.tas[lead]), 0.0):
                        room[lead] = (f"hand-over behind {traf.id[foll]}", gap)
        return prot, room

    def ring_check(self, i, x, y, progress, idx, now, d_own, v0=None):
        """Keep flying (P-18): may drone i, which would start braking for the end of its arc now, fly on to the arc of the
        gate opposite (True, '') or not (False, reason)? Not with a hold in that lane, nor for an L+C with a multicopter
        in it; not when another drone (on its entry leg, on that arc or announced) can be at the junction within GAP_MIN
        at the waiting speed of drone i plus T_RING_MARGIN, between its fastest and slowest time; for an L+C in the speed
        variant also not on the MC entry line it crosses. A multicopter (RING_MC, block 27) uses its own arc and speed;
        its ring gap crosses no entry line (speed variant: inside the L+C arcs). v0: its speed now when it starts from
        (nearly) standing still (G2 restart): its times then include speeding up at A_ACC; None: at its arc speed."""
        g = int(self.gate[i])
        g2 = (g + 2) % len(M.GATE_BEARINGS)
        lc_i = bool(self.is_lc[i])
        r, v_r = M.arc_radius(lc_i), M.V_ARC[int(lc_i)]
        key2 = M.arc_key(g2, lc_i)
        def ahead(d):                                                       # s to fly d m from here
            return d / v_r if v0 is None else M.travel_time(d, v0, v_r, M.A_ACC[int(lc_i)])
        t_j = now + ahead(d_own + r * np.radians(180.0 - M.ARC_DEG))         # at the start of the opposite arc
        v_fast = M.V_MAX[int(lc_i)]                                         # block 29: it may make room on the way,
        t_jf = now + M.travel_time(d_own + r * np.radians(180.0 - M.ARC_DEG),  # so it can be there this early
                                   v_r if v0 is None else v0, v_fast, M.A_ACC[int(lc_i)])
        w = M.GAP_MIN / v_r + M.T_RING_MARGIN
        for j in idx:
            if j == i or self.state[j] not in M.WAITING or M.arc_key(self.gate[j], self.is_lc[j]) != key2:
                continue
            if lc_i and not self.is_lc[j]:
                return False, f"multicopter {traf.id[j]} in the lane of gate {g2}"
            if self.state[j] == M.HOLD:
                return False, f"{traf.id[j]} holds in the lane of gate {g2}"
            if self.state[j] == M.ENTRY:
                tx, ty = M.arc_start(g2, lc_i)
                d = float(np.hypot(tx - x[j], ty - y[j]))
                t_lo = now + d / M.V_MAX[int(self.is_lc[j])]
                t_hi = now + d / max(float(traf.tas[j]), 1.0)
                if t_lo - w < t_j and t_jf < t_hi + w:
                    return False, f"{traf.id[j]} reaches the junction in {t_lo - now:.0f}-{t_hi - now:.0f} s, ring {t_jf - now:.0f}-{t_j - now:.0f} s"
            elif self.state[j] == M.ARC:
                s_k = r * np.radians(progress[j]) + float(traf.tas[j]) * (t_jf - now)
                if s_k < M.GAP_MIN + v_r * M.T_RING_MARGIN:
                    return False, f"{traf.id[j]} only {s_k:.0f} m into the arc of gate {g2} at the junction"
        leg = {lc: M.gate_distance() - M.arc_start_along(lc) for lc in (False, True)}
        for t_a, lc_a in self.announced(now, gate=g2):
            if M.VARIANT == "speed" and lc_a != lc_i:
                continue                                           # another lane (L+C: the MC line below)
            if lc_i and not lc_a:                                  # shared: an MC would be ahead of it or too close
                if t_a + leg[False] / M.V_MAX[0] < t_j + w:          # (t_j: the latest it can be there)
                    return False, f"multicopter announced at gate {g2} in {t_a - now:.0f} s"
                continue
            t_lo, t_hi = t_a + leg[lc_a] / M.V_MAX[int(lc_a)], t_a + leg[lc_a] / M.V_ENTRY[int(lc_a)]
            if t_lo - w < t_j and t_jf < t_hi + w:
                return False, f"{'L+C' if lc_a else 'multicopter'} announced at gate {g2} in {t_a - now:.0f} s"
        if M.VARIANT == "speed" and lc_i:                            # it crosses the MC entry line of gate g2
            along_c = np.sqrt(max(r ** 2 - M.lateral(False) ** 2, 0.0))
            bx, by = M.offset_xy(g2, False, along_c)
            end = M.arc_start_bearing(g, True) + M.ARC_DIR * M.ARC_DEG
            dth = ((np.degrees(np.arctan2(bx, by)) - end) * M.ARC_DIR) % 360
            t_x = now + ahead(d_own + r * np.radians(dth))
            w_x = M.GAP_MIN * np.hypot(v_r, M.V_ENTRY[0]) / (v_r * M.V_ENTRY[0]) + M.T_RING_MARGIN
            b = np.radians(M.GATE_BEARINGS[g2])
            key_mc = M.arc_key(g2, False)
            for j in idx:
                if self.state[j] != M.ENTRY or M.arc_key(self.gate[j], self.is_lc[j]) != key_mc:
                    continue
                rem = float(x[j] * np.sin(b) + y[j] * np.cos(b)) - along_c    # m to the crossing (< 0: passed)
                if rem < -2 * M.GAP_MIN:
                    continue
                if rem < M.GAP_MIN and float(traf.tas[j]) < M.V_ENTRY[0] - 1.0:   # block 31: an MC that slows down
                    return False, f"multicopter {traf.id[j]} slow near the crossing of gate {g2}"   # near the crossing
                if rem < -M.GAP_MIN:                                             # (its queue reaches the L+C circle)
                    continue
                v_j = max(float(traf.tas[j]), 1.0)
                t_lo = now + (rem / M.V_MAX[0] if rem > 0 else rem / v_j)
                t_hi = now + rem / v_j
                if t_lo - w_x < t_x < t_hi + w_x:
                    return False, f"multicopter {traf.id[j]} on the entry line of gate {g2}"
            leg_c = M.gate_distance() - along_c
            for t_a, lc_a in self.announced(now, gate=g2):
                if lc_a:
                    continue
                t_lo, t_hi = t_a + leg_c / M.V_MAX[0], t_a + leg_c / M.V_ENTRY[0]
                if t_lo - w_x < t_x < t_hi + w_x:
                    return False, f"multicopter announced at gate {g2} crosses in {t_lo - now:.0f}-{t_hi - now:.0f} s"
        return True, ""

    def restart_check(self, i, x, y, dist, progress, idx, now):
        """G2 (block 24): may drone i, hovering at the end of its arc, fly on to the arc opposite now (True, '') or not
        (False, reason)? Not when its clearance is expected within T_RESTART_MIN: its slot at the merge point is at
        least (its place in the FCFS order + 1) x T_TARGET after the last one, minus its predicted direct leg from here
        (as clear_next); otherwise ring_check with the times of speeding up from hover."""
        waiting = sorted((j for j in idx if self.state[j] in M.WAITING),
                         key=lambda j: (self.t_gate[j], M.call_number(traf.id[j])))
        rank = waiting.index(i)
        t_pred = self.predicted_direct(i, x, y, dist)
        wait = self.t_last_merge + (rank + 1) * M.T_TARGET - (now + t_pred)
        if wait < M.T_RESTART_MIN:
            return False, f"clearance expected in {max(wait, 0.0):.0f} s"
        return self.ring_check(i, x, y, progress, idx, now, 0.0, v0=float(traf.tas[i]))

    def predicted_direct(self, i, x, y, dist):
        """What the sequencer believes the direct leg of drone i takes from here (s): flown ahead (predict_time), with
        its prediction error."""
        gs = self.cruise_gs(i, -y[i] / dist[i], -x[i] / dist[i])
        turn = abs((traf.hdg[i] - np.degrees(np.arctan2(-x[i], -y[i])) + 180) % 360 - 180)
        t = M.predict_time(dist[i], gs, traf.tas[i], turn, self.is_lc[i],   # until standstill on the merge point
                           self.keep_of(i, dist))
        return t * (1 + self.eta_err[i])

    def keep_of(self, i, dist):
        """Block 31: a drone that is cleared while it makes room for a drone behind it in its lane (speedup) keeps that
        speed (V_MAX) on its direct leg while it is still a leader of that lane (departing_limit: until SEP_H inside
        its arc). Before block 31 it slowed down to its direct-leg speed at once, and an L+C handed over close behind
        it could not brake hard enough (31-32 m, concentration runs, 19-11-2026). Part of the prediction and of the
        descent plan, so the drone still arrives at the merge point when planned. Returns (v_keep, d_keep) or None."""
        d_keep = float(M.arc_radius(self.is_lc[i])) - M.SEP_H
        if not self.speedup[i] or dist[i] < d_keep:
            return None
        return float(M.V_MAX[int(self.is_lc[i])]), d_keep

    def set_speedup(self, i, on, info=""):
        """Keep the speedup flag of drone i and log its start and end."""
        if on != bool(self.speedup[i]):
            self.event(i, "speedup" if on else "speedup_end", info)
            if on:
                self.speedups.append((sim.simt, traf.id[i], "LC" if self.is_lc[i] else "MC", info))
        self.speedup[i] = on

    # ------------------------------------------------------------------ commands to BlueSky
    def fly(self, i, track=None, tas=None, alt=None, vs=None):
        """Set track (deg), airspeed (m/s), altitude (m) and vertical speed (m/s) of drone i."""
        if track is not None:
            traf.ap.trk[i] = track % 360
            traf.swlnav[i] = False
        if tas is not None:
            lc = int(self.is_lc[i])
            en = energy()
            if en is not None:                  # ENERGY starts a rotor -> wing transition from the speed asked for
                en.tell(i, tas)
            if M.A_ACC[lc] < M.A_DECEL[lc] and tas > traf.tas[i]:          # BlueSky has one ax_max per type (= A_DECEL):
                tas = min(tas, float(traf.tas[i]) + M.A_ACC[lc] * M.DT)    # speed up at A_ACC
            if not lc and i in self.gentle and tas < traf.tas[i]:       # a multicopter with an L+C behind it brakes no
                tas = max(tas, float(traf.tas[i]) - M.A_DECEL[1] * M.DT)   # harder than that L+C can
            traf.selspd[i] = vtas2cas(max(tas, 0.0), traf.alt[i])
            traf.swvnavspd[i] = False
        if alt is not None:
            traf.selalt[i] = max(alt, 0.0)
            traf.selvs[i] = -abs(vs) if alt < traf.alt[i] else abs(vs)
            traf.swvnav[i] = False

    def wind(self, i):
        """Wind at drone i (m/s, towards north and east); none in this study."""
        if traf.wind.winddim == 0:
            return 0.0, 0.0
        vn, ve = traf.wind.getdata(traf.lat[i:i + 1], traf.lon[i:i + 1], traf.alt[i:i + 1])
        return float(np.atleast_1d(vn)[0]), float(np.atleast_1d(ve)[0])

    def fly_ground(self, i, gn, ge):
        """Fly drone i with this ground velocity (m/s north, east), whatever the wind."""
        wn, we = self.wind(i)
        an, ae = gn - wn, ge - we
        tas = float(np.hypot(an, ae))
        if an * gn + ae * ge > 0:            # nose roughly along the ground track
            track = np.degrees(np.arctan2(ge, gn))
        else:                                # slower than the wind: nose into the air flow
            track = np.degrees(np.arctan2(ae, an))
        self.fly(i, track=track, tas=tas)

    def cruise_gs(self, i, un, ue):
        """Ground speed towards (un, ue) when flying at the direct-leg airspeed V_DIRECT in this wind."""
        wn, we = self.wind(i)
        w_along, w_cross = wn * un + we * ue, -wn * ue + we * un
        v_air = M.V_DIRECT[int(self.is_lc[i])]
        return np.sqrt(max(v_air ** 2 - w_cross ** 2, 0.0)) + w_along

    def hover(self, i, hx, hy):
        """Hover at (hx, hy): nose into the wind at airspeed = wind speed; the position is held exactly."""
        wn, we = self.wind(i)
        traf.lat[i], traf.lon[i] = M.to_latlon(hx, hy)
        self.fly(i, track=np.degrees(np.arctan2(-we, -wn)), tas=float(np.hypot(wn, we)))

    # ------------------------------------------------------------------ main loop
    @core.timed_function(name="vpms", dt=M.DT)
    def update(self, dt):
        idx = np.where(self.state > 0)[0]
        if len(idx) == 0:
            return
        now = sim.simt
        x, y = M.to_xy(traf.lat, traf.lon)
        dist = np.hypot(x, y)
        brg = np.degrees(np.arctan2(x, y)) % 360

        progress = np.zeros(traf.ntraf)                 # deg along the arc, from its start
        for i in idx:
            if self.state[i] in (M.ARC, M.HOLD, M.RING):
                p = ((brg[i] - M.arc_start_bearing(self.gate[i], self.is_lc[i])) * M.ARC_DIR) % 360
                progress[i] = 0.0 if p > 300 else p

        # the lanes: who makes room (P-17) and who brakes gently
        entry = idx[self.state[idx] == M.ENTRY]
        d_start = {}
        for i in entry:
            tx, ty = M.arc_start(self.gate[i], self.is_lc[i])
            d_start[i] = np.hypot(tx - x[i], ty - y[i])
        self.protect, room = self.handover_protect(idx, x, y, progress, d_start, now)  # G3 hand-over protection
        restore = self.restore_flags(idx, x, y, progress, d_start, now, room)    # making room (P-17, G3)
        self.gentle = set(restore) | self.lc_behind(idx, progress, d_start)     # gentle braking

        for i in idx:
            if self.state[i] not in (M.ENTRY, M.ARC, M.RING) and self.speedup[i]:
                self.set_speedup(i, False, "left the lane")
        self.fly_entry_legs(entry, idx, now, x, y, dist, progress, d_start, restore)
        self.fly_arcs(idx, now, x, y, dist, brg, progress, restore)
        self.clear_next(idx, now, x, y, dist)
        self.fly_direct(idx, now, x, y, dist)
        self.fly_final(idx, now)

    def fly_entry_legs(self, entry, idx, now, x, y, dist, progress, d_start, restore):
        """1. Entry leg: towards the start of the arc, behind the drone ahead in the lane; the fixed turn starts at T1."""
        for i in entry:
            lc = int(self.is_lc[i])
            tx, ty = M.arc_start(self.gate[i], lc)
            d = d_start[i]
            v = M.V_MAX[lc] if i in restore else M.V_ENTRY[lc]
            key = M.arc_key(self.gate[i], self.is_lc[i])
            ahead_entry = [j for j in entry if j != i and M.arc_key(self.gate[j], self.is_lc[j]) == key
                           and d_start[j] < d]
            ahead_arc = [j for j in idx if self.state[j] in (M.ARC, M.HOLD, M.RING)
                         and M.arc_key(self.gate[j], self.is_lc[j]) == key]
            lead, gap = None, np.inf
            if ahead_entry:                                    # nearest drone ahead on the entry leg
                lead = max(ahead_entry, key=lambda j: d_start[j])
            elif ahead_arc:                                    # last drone that entered the arc
                lead = min(ahead_arc, key=lambda j: progress[j])
            if lead is not None:
                gap = np.hypot(x[lead] - x[i], y[lead] - y[i])  # straight line, as the separation check
                v_lead = 0.0 if self.state[lead] == M.HOLD else traf.tas[lead]
                v = self.lane_speed(i, lead, gap, v_lead, v, restore)
            if i in self.protect:                              # G3: do not slow down right after a hand-over; use the
                v_safe = M.V_MAX[lc]                           # margin to the drone ahead (down to SEP_H + one update)
                if lead is not None:
                    gap_eff = gap + M.GAP_MIN - (M.SEP_H + M.V_MAX[lc] * M.DT + M.G3_BUFFER)
                    v_safe = M.follow_speed(gap_eff, v_lead, M.V_MAX[lc], self.lc_like(i))
                v_prot = min(self.protect[i], v_safe)
                if v_prot > v + 0.05:
                    if not self.protected[i]:
                        self.event(i, "handover_protect", f"keeps {v_prot:.1f} m/s instead of {v:.1f} m/s")
                    self.protected[i] = True
                v = max(v, v_prot)
            v = self.departing_limit(i, idx, x, y, dist, v)
            if i in restore:
                self.set_speedup(i, v > M.V_ENTRY[lc] + 0.1, "for {}, gap {:.0f} m".format(*restore[i]))
            elif self.speedup[i]:
                self.set_speedup(i, False, "gap restored")
            self.fly(i, track=np.degrees(np.arctan2(tx - x[i], ty - y[i])), tas=v,
                     alt=M.H_ARC[self.gate[i] % 2], vs=M.VS_ENTRY)
            t1x, t1y = M.turn_geom(self.gate[i], self.is_lc[i])["T1"]
            if (np.hypot(t1x - x[i], t1y - y[i]) < traf.gs[i] * M.DT + 0.5         # at T1: start the fixed turn
                    or d < M.turn_lookahead(self.is_lc[i])):
                self.state[i] = M.ARC
                self.turning[i] = True
                self.t_arc_in[i] = now
                self.event(i, "arc_in", f"gate {self.gate[i]}")

    def fly_arcs(self, idx, now, x, y, dist, brg, progress, restore):
        """2. Arcs: the fixed turn, then along the circle; no overtaking. An L+C that is not cleared decides where it would
        start braking for the end of its arc whether it flies on (RING, P-18) or brakes and hovers (HOLD); with RING_MC a
        multicopter too (block 27)."""
        on_arc = idx[np.isin(self.state[idx], (M.ARC, M.HOLD, M.RING))]
        lanes_now = {}                                         # lanes fixed at the start of the update: a drone that
        for i in on_arc:                                       # joins the opposite lane now belongs there next update
            lanes_now.setdefault(M.arc_key(self.gate[i], self.is_lc[i]), []).append(i)
        for key, members in lanes_now.items():
            members.sort(key=lambda i: -progress[i])          # first = furthest along
            for rank, i in enumerate(members):
                lc = int(self.is_lc[i])
                r = M.arc_radius(lc)
                d_own = r * np.radians(max(M.ARC_DEG - progress[i], 0.0))   # to the end of its own arc
                if self.state[i] == M.RING and progress[i] >= 180.0 - 1e-6:   # at the start of the opposite arc
                    g_old = int(self.gate[i])
                    self.gate[i] = (g_old + 2) % len(M.GATE_BEARINGS)
                    self.state[i], self.ring_go[i] = M.ARC, 0
                    self.ring_log.append((now, traf.id[i], "ring_join", f"gate {g_old} -> {self.gate[i]}"))
                    self.event(i, "ring_join", f"gate {g_old} -> {self.gate[i]}")
                    continue
                if (M.may_ring(lc) and self.state[i] == M.ARC and self.ring_go[i] == 0
                        and d_own <= traf.tas[i] ** 2 / (2 * self.a_dec(i)) + traf.tas[i] * M.DT + 2.0):
                    lead_holds = rank > 0 and (self.state[members[rank - 1]] == M.HOLD
                                               or self.ring_go[members[rank - 1]] == -1)
                    if lead_holds:
                        ok, why = False, f"{traf.id[members[rank - 1]]} ahead holds"
                    elif traf.tas[i] < M.V_ARC[lc] - 0.5:
                        ok, why = False, f"already braking ({traf.tas[i]:.1f} m/s)"
                    else:
                        ok, why = self.ring_check(i, x, y, progress, idx, now, d_own)
                    self.ring_go[i] = 1 if ok else -1
                    if not ok:
                        self.ring_log.append((now, traf.id[i], "ring_refused", why))
                        self.event(i, "ring_refused", why)
                if self.state[i] == M.ARC and (progress[i] >= M.ARC_DEG
                                             or (d_own < M.ARRIVE_R and traf.gs[i] < M.ARRIVE_V)):
                    if self.ring_go[i] == 1:                   # keep flying (P-18)
                        self.state[i] = M.RING
                        self.t_ring[i] = min(self.t_ring[i], now)
                        self.ring_log.append((now, traf.id[i], "ring", f"gate {self.gate[i]}"))
                        self.event(i, "ring", f"gate {self.gate[i]} -> {(self.gate[i] + 2) % len(M.GATE_BEARINGS)}")
                    else:
                        self.state[i] = M.HOLD
                        self.hx[i], self.hy[i] = M.polar(M.arc_start_bearing(self.gate[i], self.is_lc[i])
                                                       + M.ARC_DIR * M.ARC_DEG, r)
                        self.event(i, "hold")
                        if self.speedup[i]:
                            self.set_speedup(i, False, "hold")
                if self.state[i] == M.HOLD and M.RESTART_ON and M.may_ring(lc):      # G2 restart
                    ok, _ = self.restart_check(i, x, y, dist, progress, idx, now)
                    if ok:
                        self.state[i], self.ring_go[i] = M.RING, 1
                        self.t_ring[i] = min(self.t_ring[i], now)
                        self.ring_log.append((now, traf.id[i], "restart", f"gate {self.gate[i]}"))
                        self.event(i, "restart", f"gate {self.gate[i]} -> {(self.gate[i] + 2) % len(M.GATE_BEARINGS)}")
                if self.state[i] == M.HOLD:
                    self.hover(i, self.hx[i], self.hy[i])
                    continue
                v = M.V_MAX[lc] if i in restore else M.V_ARC[lc]
                if self.state[i] == M.ARC and self.ring_go[i] != 1:
                    v = min(v, np.sqrt(2 * self.a_dec(i) * d_own))  # can always stop at the end of its arc
                if rank > 0:                                   # drone ahead on the same arc
                    lead = members[rank - 1]
                    gap = np.hypot(x[lead] - x[i], y[lead] - y[i])
                    v_lead = 0.0 if self.state[lead] == M.HOLD else traf.tas[lead]
                    v = self.lane_speed(i, lead, gap, v_lead, v, restore)
                    if self.state[lead] == M.ARC and self.ring_go[lead] != 1 and self.state[i] == M.ARC:
                        d_end = r * np.radians(max(M.ARC_DEG - progress[i], 0.0))   # the leader may stop at the end
                        v = min(v, np.sqrt(2 * self.a_dec(i) * max(d_end - M.GAP_MIN, 0.0)))  # of the arc: stop GAP_MIN before
                if self.state[i] == M.RING:                    # block 29: also behind the last drone on the arc it
                    g2 = (int(self.gate[i]) + 2) % len(M.GATE_BEARINGS)       # is about to join (across the junction)
                    key2 = M.arc_key(g2, self.is_lc[i])
                    ahead2 = [j for j in on_arc if j != i and int(self.gate[j]) == g2
                              and M.arc_key(g2, self.is_lc[j]) == key2 and self.state[j] in (M.ARC, M.HOLD)]
                    if ahead2:
                        lead2 = min(ahead2, key=lambda j: progress[j])
                        gap2 = np.hypot(x[lead2] - x[i], y[lead2] - y[i])
                        v_lead2 = 0.0 if self.state[lead2] == M.HOLD else traf.tas[lead2]
                        v = min(v, self.lane_speed(i, lead2, gap2, v_lead2, v, restore))
                v = self.departing_limit(i, idx, x, y, dist, v)
                if i in restore:
                    self.set_speedup(i, v > M.V_ARC[lc] + 0.1, "for {}, gap {:.0f} m".format(*restore[i]))
                elif self.speedup[i]:
                    self.set_speedup(i, False, "gap restored")
                alt = M.H_ARC[self.gate[i] % 2]
                if self.turning[i]:                            # the fixed turn (P-16): along its circle to T2
                    tg = M.turn_geom(self.gate[i], self.is_lc[i])
                    if self.state[i] != M.ARC or progress[i] >= tg["p2"] - 0.2:
                        self.turning[i] = False
                if self.turning[i]:
                    cx, cy = tg["C"]
                    rt = tg["R"]
                    bc = np.degrees(np.arctan2(x[i] - cx, y[i] - cy))
                    a = max(M.L_MIN / 3.0, float(traf.gs[i]) * 1.0) / rt          # aim about 1 s ahead on the circle,
                    ax, ay = M.polar(bc + tg["sense"] * np.degrees(a), rt / np.cos(a))   # placed so that it flies on it
                    track = np.degrees(np.arctan2(cx + ax - x[i], cy + ay - y[i]))
                    if v >= M.V_GROUND:
                        self.fly(i, track=track, tas=v, alt=alt, vs=M.VS_ENTRY)
                    else:
                        tr = np.radians(track)
                        self.fly_ground(i, v * np.cos(tr), v * np.sin(tr))
                        self.fly(i, alt=alt, vs=M.VS_ENTRY)
                elif v >= M.V_GROUND:                            # on the arc: pure pursuit of a point one look-ahead
                    a = M.look_ahead(traf.gs[i]) / r             # further, at r / cos(a) so that the drone stays on r
                    ax, ay = M.polar(brg[i] + M.ARC_DIR * np.degrees(a), r / np.cos(a))
                    track = np.degrees(np.arctan2(ax - x[i], ay - y[i]))
                    self.fly(i, track=track, tas=v, alt=alt, vs=M.VS_ENTRY)
                else:                                          # slow: hold the arc against the wind
                    tangent = np.radians(brg[i] + M.ARC_DIR * 90)
                    v_in = -np.clip((dist[i] - r) / M.T_CLOSE, -2.0, 2.0)          # back to the circle
                    gn = v * np.cos(tangent) + v_in * y[i] / dist[i]
                    ge = v * np.sin(tangent) + v_in * x[i] / dist[i]
                    self.fly_ground(i, gn, ge)
                    self.fly(i, alt=alt, vs=M.VS_ENTRY)

    def clear_next(self, idx, now, x, y, dist):
        """3. Clearance: first-come-first-served, T_TARGET apart at the merge point, along a conflict-free descent."""
        i = self.next_clearance(idx)
        if i is not None:
            gs = self.cruise_gs(i, -y[i] / dist[i], -x[i] / dist[i])
            t_pred = self.predicted_direct(i, x, y, dist)          # what the sequencer believes
            if now + t_pred >= self.t_last_merge + M.T_TARGET - 1e-6:
                acid = traf.id[i]
                keep = self.keep_of(i, dist)                      # block 31
                plan = M.plan_descent(self.gate[i], self.is_lc[i], float(x[i]), float(y[i]), float(traf.tas[i]),
                                    float(traf.hdg[i]), float(traf.alt[i]), gs, keep=keep)
                if plan is None and self.state[i] == M.HOLD:     # never expected: clear it without avoiding
                    self.nogo_hold.append(acid)
                    self.event(i, "nogo_hold", "no conflict-free descent from the hold")
                    plan = M.plan_descent(self.gate[i], self.is_lc[i], float(x[i]), float(y[i]), float(traf.tas[i]),
                                        float(traf.hdg[i]), float(traf.alt[i]), gs, avoid=False, keep=keep)
                if plan is None:                               # no-go zone: the clearance waits
                    if acid not in self.nogo_since:
                        self.nogo_since[acid] = now
                        self.event(i, "nogo", "no conflict-free descent from here; clearance waits")
                else:
                    t_wait = self.nogo_since.pop(acid, None)
                    if t_wait is not None:
                        self.nogo_delays.append((acid, now - t_wait))
                    self.state[i] = M.DIRECT
                    if keep is not None:                          # block 31: keeps its speed for a while
                        self.keep_v[acid] = keep
                    self.t_merge[i] = self.t_last_merge = now + t_pred
                    self.descent[acid] = (now, plan)
                    self.event(i, "direct_to", f"planned merge {now + t_pred:.1f}"
                               + (f", after {now - t_wait:.1f} s in a no-go zone" if t_wait is not None else ""))

    def fly_direct(self, idx, now, x, y, dist):
        """4. Direct leg: to the merge point, arriving in hover. The merge point is not free while the drone below it is
        less than K_SPACING x SEP_V lower; a drone then waits K_SPACING x SEP_H before it."""
        busy_merge = any(self.state[j] == M.FINAL and traf.alt[j] > M.H_MERGE - M.K_SPACING * M.SEP_V for j in idx)
        for i in idx[self.state[idx] == M.DIRECT]:
            if dist[i] < M.ARRIVE_R and traf.gs[i] < M.ARRIVE_V:          # standstill on the merge point
                self.state[i] = M.FINAL
                self.descent.pop(traf.id[i], None)
                self.keep_v.pop(traf.id[i], None)
                self.merges.append((now, traf.id[i]))
                self.event(i, "merge")
                self.hover(i, 0.0, 0.0)
                continue
            d = max(dist[i], 1e-6)
            un, ue = -y[i] / d, -x[i] / d                         # unit vector to the merge point
            stop = 0.0
            if busy_merge or any(self.state[j] == M.DIRECT and dist[j] < dist[i] for j in idx if j != i):
                stop = M.K_SPACING * M.SEP_H
            v_cruise = M.keep_cruise(self.cruise_gs(i, un, ue), dist[i],    # block 31: cleared while making room:
                                     self.keep_v.get(traf.id[i]))           # keeps that speed for a while (keep_of)
            v = min(v_cruise,                                     # brake one update early: no overshoot
                    np.sqrt(2 * M.A_DECEL[int(self.is_lc[i])] * max(dist[i] - stop - traf.gs[i] * M.DT, 0.0)))
            self.fly_ground(i, v * un, v * ue)
            t0, plan = self.descent.get(traf.id[i], (now, None))  # height: the plan, one update ahead
            if plan is None:
                self.fly(i, alt=M.H_MERGE, vs=M.VS_DIRECT_MAX)
            else:
                h = float(plan[min(int(round((now - t0) / M.DT)) + 1, len(plan) - 1)])
                self.fly(i, alt=h, vs=max(abs(float(traf.alt[i]) - h) / M.DT, 0.01))

    def fly_final(self, idx, now):
        """5. Final descent: never within K_SPACING x SEP_V above the drone below (or on the pad), then the pad."""
        column = idx[np.isin(self.state[idx], (M.FINAL, M.PAD))]
        delete = []
        for i in column:
            if self.state[i] == M.PAD:
                if now >= self.t_touch[i] + self.t_pad[i]:
                    delete.append(traf.id[i])
                    self.t_pad[i] = np.inf
                continue
            below = [traf.alt[j] for j in column if j != i and traf.alt[j] < traf.alt[i]]
            floor = max(below) + M.K_SPACING * M.SEP_V if below else 0.0
            target = floor if floor < traf.alt[i] else traf.alt[i]
            self.hover(i, 0.0, 0.0)
            if traf.alt[i] > M.H_SLOW + 0.05:
                self.fly(i, alt=max(target, M.H_SLOW), vs=M.V_DESC[0])
            else:
                self.fly(i, alt=target, vs=M.V_DESC[1])
            if traf.alt[i] <= 0.05:
                self.state[i] = M.PAD
                self.t_touch[i] = now
                last = self.touchdowns[-1][0] if self.touchdowns else np.nan
                self.touchdowns.append((now, traf.id[i]))
                self.event(i, "touchdown", f"{now - last:.1f} s after previous")
        for acid in delete:
            stack.stack(f"DEL {acid}")

    def next_clearance(self, idx):
        """First-come-first-served, the start line at the gate: of the drones without a clearance, the one that passed
        its gate first gets the next slot, once it is on its arc (while it is on its entry leg nobody is cleared).
        Ties: the earlier drawn arrival (the number in the call sign). A SEQUENCER plugin replaces this function."""
        waiting = [i for i in idx if self.state[i] in M.WAITING]
        if not waiting:
            return None
        first = min(waiting, key=lambda i: (self.t_gate[i], M.call_number(traf.id[i])))
        return first if self.state[first] in (M.ARC, M.HOLD, M.RING) else None

    # ------------------------------------------------------------------ command
    @stack.command(name="VPMS")
    def command(self, what: str = "SHOW", value: str = "0"):
        """VPMS [SHOW | SHARED | SPEED | TARGET s | OFFSET m | LEG m | RING ON/OFF | RINGMC ON/OFF | RESTART ON/OFF |
        HORIZON s]: draw the structure, choose the arc variant, the target separation (s), the side offset of the entry
        points (m, speed variant; -1 = computed), the length of the entry leg (m), keep flying on or off (every type),
        keep flying for the multicopter too, the restart from hover (G2) on or off, or the announcement horizon (s;
        -1 = computed). LEG, RING, RINGMC and RESTART compute the horizon again (M.announce_horizon) unless HORIZON s
        set it. All before the traffic starts."""
        what = what.upper()
        def horizon():
            if not M.H_SET:
                M.H_ANNOUNCE = M.announce_horizon()
            return f"; announcement horizon {M.H_ANNOUNCE:g} s{' (set)' if M.H_SET else ''}"
        def on(v):
            return str(v).upper() not in ("OFF", "0", "FALSE", "NO")
        if what == "RING":
            M.RING_ON = on(value)
            return True, f"VPMS keep flying (ring): {'on' if M.RING_ON else 'off'}" + horizon()
        if what == "RINGMC":
            M.RING_MC = on(value)
            return True, f"VPMS keep flying for the multicopter too: {'on' if M.RING_MC else 'off'}" + horizon()
        if what == "RESTART":
            M.RESTART_ON = on(value)
            return True, f"VPMS restart from hover (G2): {'on' if M.RESTART_ON else 'off'}" + horizon()
        try:
            value = float(value)
        except ValueError:
            return False, f"VPMS {what}: number expected, got {value}"
        if what == "HORIZON":
            M.H_SET = value >= 0
            M.H_ANNOUNCE = float(value) if M.H_SET else M.announce_horizon()
            return True, f"VPMS announcement horizon {M.H_ANNOUNCE:g} s ({'set' if M.H_SET else 'computed'})"
        if what == "LEG" and value > 0:
            M.ENTRY_LEG = float(value)
            return True, f"VPMS entry leg {M.ENTRY_LEG:g} m: gates at {M.gate_distance():.0f} m" + horizon()
        if what == "OFFSET":
            M.ENTRY_OFFSET = None if value < 0 else float(value)
            return True, (f"VPMS entry offset {M.entry_offset():.1f} m per side ({2 * M.entry_offset():.0f} m apart; "
                          f"{'computed' if M.ENTRY_OFFSET is None else 'set'}; computed {M.entry_offset_min():.1f} m)")
        if what == "TARGET" and value > 0:
            M.T_TARGET = float(value)
            return True, (f"VPMS target {M.T_TARGET:g} s at the merge point; structure fixed, designed for "
                          f"{M.T_DESIGN:g} s: radius MC {M.arc_radius(False):.0f} m, L+C {M.arc_radius(True):.0f} m, "
                          f"gates at {M.gate_distance():.0f} m")
        if what in ("SHARED", "SPEED"):
            M.VARIANT = what.lower()
            return True, (f"VPMS variant {M.VARIANT}: radius MC {M.arc_radius(False):.0f} m, "
                          f"L+C {M.arc_radius(True):.0f} m, gates at {M.gate_distance():.0f} m")
        for g in range(len(M.GATE_BEARINGS)):
            for lc in ((True,) if M.VARIANT == "shared" else (False, True)):
                b0 = M.arc_start_bearing(g, lc)
                pts = [M.to_latlon(*M.polar(b0 + M.ARC_DIR * a, M.arc_radius(lc)))
                       for a in np.linspace(0, M.ARC_DEG, 31)]
                gl, gn, _, _ = M.gate(g, lc)
                pts = [(gl, gn)] + pts
                coords = ",".join(f"{la:.6f},{lo:.6f}" for la, lo in pts)
                stack.stack(f"POLYLINE ARC{g}{'L' if lc else 'M'},{coords}")
        return True, (f"VPMS {M.VARIANT}: radius MC {M.arc_radius(False):.0f} m, "
                      f"L+C {M.arc_radius(True):.0f} m, gates at {M.gate_distance():.0f} m")
