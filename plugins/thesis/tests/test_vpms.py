""" VPMS + TRAFFIC (+ ENERGY, FLIGHTLOG): a short run through the structure, without GUI.
Run from the bluesky folder:  python plugins/thesis/tests/test_vpms.py [shared|speed] [eta_err_pct] [demand] [target_s]
                                                                        [warmup_s] [measure_s]
(target_s: VPMS TARGET, default 40; the structure must stay the one for 40 s, decision 05-11-2026;
warmup_s and measure_s: default 300 and 600 s, a short test; 3600 3600 is a full run as in the experiments)
Checks: every drone lands; merge and touchdown intervals; losses of separation (32 m / 7.62 m);
L+C drones that had to leave their wing behind a multicopter; the descent on the direct leg (fastest rate,
no faster than 2 m/s at low speed, exactly 40 m at the merge point) and the no-go zone (decision 05-11-2026);
an odd number of transitions for every L+C created on its wing (decision 09-11-2026). """
import os, sys
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, ROOT); os.chdir(ROOT)
import numpy as np
import bluesky as bs

variant = sys.argv[1] if len(sys.argv) > 1 else "shared"
eta = sys.argv[2] if len(sys.argv) > 2 else "0"
demand = sys.argv[3] if len(sys.argv) > 3 else "60"
target = sys.argv[4] if len(sys.argv) > 4 else "40"
warmup = sys.argv[5] if len(sys.argv) > 5 else "300"
measure = sys.argv[6] if len(sys.argv) > 6 else "600"
bs.init(mode="sim", detached=True)
for cmd in ["DT 0.5", "PLUGINS LOAD ENERGY", "PLUGINS LOAD FLIGHTLOG", "PLUGINS LOAD VPMS",
            "PLUGINS LOAD TRAFFIC", f"VPMS {variant}", f"VPMS TARGET {target}", f"FLIGHTLOG vpms_{variant}",
            f"TRAFFIC {demand} 1 {eta} BASE {warmup} {measure}"]:
    bs.stack.stack(cmd)
bs.sim.step()
def plugin_module(name, cls):
    return next(m for k, m in sys.modules.items() if k.endswith("." + name) and hasattr(m, cls))
mods = {"vpms": plugin_module("vpms", "Vpms"), "thesis_traffic": plugin_module("thesis_traffic", "Traffic"),
        "energy": plugin_module("energy", "Energy")}
vp, sc, en = mods["vpms"].Vpms.implinstance(), mods["thesis_traffic"].Traffic.implinstance(), mods["energy"].Energy.implinstance()
V = mods["vpms"]
print(f"Variant {variant}, prediction error {eta} %, {demand} drones/h, target {V.T_TARGET:g} s "
      f"(structure designed for {V.T_DESIGN:g} s): radius MC {V.arc_radius(False):.0f} m, "
      f"L+C {V.arc_radius(True):.0f} m, gates at {V.gate_distance():.0f} m")

los_pairs, overshoot, max_radial, trans_on_arc, spawned = set(), 0.0, 0.0, set(), set()
sped_up, v_max_seen = set(), 0.0       # L+C leaders that sped up for an L+C behind them (decision 03-11-2026)
braking_to_end = set()                 # L+C below stall because it brakes to the end of its arc (decision 04-11-2026)
perf_seen = {}                         # BlueSky limits per type, as BlueSky uses them (decision 04-11-2026)
prev_alt, vs_fast, vrs_steps, merge_dev, seen_final = {}, 0.0, 0, 0.0, set()   # descent checks (05-11-2026)
print(f"Entry offset {V.entry_offset():.1f} m per side (speed variant; computed {V.entry_offset_min():.1f} m); "
      f"lane gap {V.GAP_MIN:.0f} m; arcs at {V.H_ARC[0]:.2f} / {V.H_ARC[1]:.2f} m; hand-over MC-MC "
      f"{V.handover_time(False, False):.1f} s, L+C-L+C {V.handover_time(True, True):.1f} s"
      + (f", L+C behind MC {V.handover_time(False, True):.1f} s" if V.VARIANT == "shared" else
         " (MC and L+C have their own entry points)"))
cleared_at = {}                        # acid -> time of its go (FCFS order check, decision 06-11-2026)
prev_tas, acc_seen, dec_seen = {}, [0.0, 0.0], [0.0, 0.0]   # observed acceleration per type (09-11-2026)
while bs.sim.simt < float(warmup) + float(measure) + 3000 and not sc.done:
    bs.sim.step()
    n = bs.traf.ntraf
    spawned.update(bs.traf.id)
    for i, acid in enumerate(bs.traf.id):                                  # acceleration per type (09-11-2026)
        if acid in prev_tas and vp.state[i] > 0:
            a_obs = (float(bs.traf.tas[i]) - prev_tas[acid]) / V.DT
            k = int(vp.is_lc[i])
            acc_seen[k], dec_seen[k] = max(acc_seen[k], a_obs), max(dec_seen[k], -a_obs)
        prev_tas[acid] = float(bs.traf.tas[i])
    for i, acid in enumerate(bs.traf.id):
        if vp.state[i] == V.DIRECT and acid not in cleared_at:
            cleared_at[acid] = bs.sim.simt
    for i, acid in enumerate(bs.traf.id):                                  # descent checks (05-11-2026)
        if vp.state[i] == V.DIRECT:
            if acid in prev_alt:
                rate = (prev_alt[acid] - float(bs.traf.alt[i])) / V.DT          # descent rate in this step
                vs_fast = max(vs_fast, rate)
                if rate > V.VS_DIRECT_MAX + 0.01 and bs.traf.gs[i] < V.K_VRS * V.V_IH[int(vp.is_lc[i])]:
                    vrs_steps += 1
            prev_alt[acid] = float(bs.traf.alt[i])
        elif vp.state[i] == V.FINAL and acid not in seen_final:
            seen_final.add(acid)                                          # just arrived at the merge point
            merge_dev = max(merge_dev, abs(float(bs.traf.alt[i]) - V.H_MERGE))
    if n < 2:
        continue
    x, y = V.to_xy(bs.traf.lat, bs.traf.lon)
    dh = np.hypot(x[:, None] - x[None, :], y[:, None] - y[None, :])
    dv = np.abs(bs.traf.alt[:, None] - bs.traf.alt[None, :])
    on_pad = bs.traf.alt < 0.1
    los = (dh < V.SEP_H) & (dv < V.SEP_V) & ~np.eye(n, dtype=bool) & ~(on_pad[:, None] & on_pad[None, :])
    for a, b in zip(*np.where(np.triu(los))):
        pair = tuple(sorted((bs.traf.id[a], bs.traf.id[b])))
        if pair not in los_pairs:
            S = V.STATE_NAMES
            print(f"   LoS t={bs.sim.simt:.0f}: {bs.traf.id[a]} {S[vp.state[a]]} r={np.hypot(x[a], y[a]):.0f} m "
                  f"alt={bs.traf.alt[a]:.0f} | {bs.traf.id[b]} {S[vp.state[b]]} r={np.hypot(x[b], y[b]):.0f} m "
                  f"alt={bs.traf.alt[b]:.0f} | dh={dh[a, b]:.0f} m dv={dv[a, b]:.1f} m")
        los_pairs.add(pair)
    prog_all = {}
    for i in range(n):
        if vp.state[i] in (V.ARC, V.HOLD):
            p_ = (np.degrees(np.arctan2(x[i], y[i])) - V.arc_start_bearing(vp.gate[i], vp.is_lc[i])) % 360
            prog_all[i] = 0.0 if p_ > 300 else p_
        try:
            P = bs.traf.perf
            perf_seen[bs.traf.type[i]] = (P.vmax[i], P.vsmin[i], P.vsmax[i], P.axmax[i], P.hmax[i])
        except (AttributeError, IndexError):
            pass
    for i in range(n):
        if vp.state[i] == V.ARC:
            err = np.hypot(x[i], y[i]) - V.arc_radius(vp.is_lc[i])        # + outside, - inside
            prog = (np.degrees(np.arctan2(x[i], y[i])) - V.arc_start_bearing(vp.gate[i], vp.is_lc[i])) % 360
            prog = 0.0 if prog > 300 else prog
            if prog > 1.0:
                overshoot = min(overshoot, err)                        # inside the circle: overshoot
            if prog > 20.0:
                max_radial = max(max_radial, abs(err))                 # once on the arc
            if vp.is_lc[i] and bs.traf.tas[i] < 11.8:
                key = V.arc_key(vp.gate[i], vp.is_lc[i])
                ahead = any(j != i and V.arc_key(vp.gate[j], vp.is_lc[j]) == key and prog_all[j] > prog_all[i]
                            for j in prog_all)
                (trans_on_arc if ahead else braking_to_end).add(bs.traf.id[i])
        if vp.speedup[i]:
            sped_up.add(bs.traf.id[i])
            v_max_seen = max(v_max_seen, float(bs.traf.tas[i]))


merge = np.diff([t for t, _ in vp.merges]); touch = np.diff([t for t, _ in vp.touchdowns])
print(f"\nSimulated {bs.sim.simt:.0f} s; {len(spawned)} drones created, "
      f"{len(vp.touchdowns)} landed, {bs.traf.ntraf} still flying (expected 0)")
print(f"Merge-point intervals:  min {merge.min():.1f} s, mean {merge.mean():.1f} s  (target {V.T_TARGET:.0f} s)")
t_min = V.T_PAD + (V.K_SPACING * V.SEP_V - V.H_SLOW) / V.V_DESC[0] + V.H_SLOW / V.V_DESC[1]   # column rule
print(f"Touchdown intervals:    min {touch.min():.1f} s  (never below {t_min:.1f} s, the column rule; "
      f"one drone at a time in the final descent when the target is at least 33.5 s)")
print(f"Arcs: overshoot to the inside {max(-overshoot, 0.0):.1f} m; largest deviation after the first 20 deg {max_radial:.1f} m")
print(f"Losses of separation (32 m and 7.62 m): {len(los_pairs)} pairs {sorted(los_pairs)[:6]}")
print(f"Observed acceleration: MC up to {acc_seen[0]:.2f}, braking {dec_seen[0]:.2f} m/s2; L+C up to {acc_seen[1]:.2f}, "
      f"braking {dec_seen[1]:.2f} m/s2 (at most MC {V.A_ACC[0]:.2f} / {V.A_DECEL[0]:.2f}, L+C {V.A_ACC[1]:.2f} / "
      f"{V.A_DECEL[1]:.2f}; decision 09-11-2026)")
print(f"Slow entries (created slower than the entry speed, safety net of 06-11-2026): {len(vp.slow_entries)} "
      f"(expected 0 below capacity; entry leg {V.GATE_MARGIN:g} m)")
for t_se, acid_se, v_se, note_se in vp.slow_entries[:6]:
    print(f"   t={t_se:.0f} {acid_se}: {note_se}")
print(f"L+C below stall on an arc (behind a slower or holding drone): {len(trans_on_arc)} {sorted(trans_on_arc)[:6]}")
print(f"L+C below stall while braking to the end of its own arc (no drone ahead): {len(braking_to_end)}")
print(f"Direct leg: fastest descent {vs_fast:.2f} m/s (at most {V.VS_FAST:.0f}); faster than {V.VS_DIRECT_MAX:.0f} m/s "
      f"below {V.K_VRS} v_ih: {vrs_steps} steps (expected 0); height at the merge point off 40 m by at most {merge_dev:.2f} m")
order = [V.call_number(a) for a in sorted(cleared_at, key=cleared_at.get)]   # call number = drawn arrival order
out_of_order = sum(1 for a, b in zip(order, order[1:]) if b < a)
print(f"FCFS: {len(order)} clearances, {out_of_order} out of gate order (expected 0: the start line is the gate)")
nd = [d for _, d in vp.nogo_delays]
print(f"No-go zone: {len(nd)} clearances delayed" + (f", longest {max(nd):.1f} s, mean {np.mean(nd):.1f} s" if nd else "")
      + f"; holds without a conflict-free descent: {len(vp.nogo_hold)} (expected 0)")
print(f"L+C leaders that sped up to restore {V.GAP_MIN:.0f} m: {len(sped_up)} {sorted(sped_up)[:6]}; fastest {v_max_seen:.1f} m/s "
      f"(at most {V.V_LC_MAX} m/s)")

for typ, (vmax, vsmin, vsmax, axmax, hmax) in sorted(perf_seen.items()):
    print(f"BlueSky limits {typ}: speed max {vmax:.1f} m/s, vertical {vsmin:.1f} to {vsmax:.1f} m/s, "
          f"acceleration {axmax:.2f} m/s2, ceiling {hmax:.0f} m (expected for GMC/GLC: 17.0 / 15.4, -5 to 5, "
          f"MC {V.A_DECEL[0]:.2f} / L+C {V.A_DECEL[1]:.2f}, 150)")
holds = sum(1 for t, _ in vp.merges)  # placeholder to keep names short
import csv
rows = list(csv.DictReader(open(f"output/vpms_{variant}/flights.csv")))
for cfg in ("MC", "LC"):
    r = [x for x in rows if x["config"] == cfg]
    if r:
        m = lambda c: np.mean([float(x[c]) for x in r])
        print(f"{cfg}: {len(r)} drones, {m('flight_time'):.0f} s in the structure, {m('energy_wh'):.1f} Wh, "
              f"hover {m('t_hover'):.0f} s, transitions {m('n_trans'):.1f}")
# transitions per L+C (decision 09-11-2026): off and on the wing alternate and every L+C lands on its rotors, so an
# L+C created on its wing lands with an odd number (1, 3, 5, ...); an even number means a transition was missed
# or counted twice (block 11 missed every rotor -> wing transition)
v_stall = mods["energy"].LC.v_stall
below = {a for _, a, v, _ in vp.slow_entries if v < v_stall}          # created below stall: not on its wing
lc_rows = [x for x in rows if x["config"] == "LC"]
n_tr = {}
for x in lc_rows:
    n_tr[int(x["n_trans"])] = n_tr.get(int(x["n_trans"]), 0) + 1
even = sorted(x["id"] for x in lc_rows if int(x["n_trans"]) % 2 == 0 and x["id"] not in below)
print(f"L+C transitions per drone: {dict(sorted(n_tr.items()))}; even number (created on the wing): {len(even)} "
      f"{even[:6]} (expected 0)")
