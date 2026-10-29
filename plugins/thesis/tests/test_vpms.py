""" VPMS + TRAFFIC (+ ENERGY, FLIGHTLOG): a short run through the structure, without GUI.
Run from the bluesky folder:  python plugins/thesis/tests/test_vpms.py [shared|speed] [eta_err_pct] [demand]
Checks: every drone lands; merge and touchdown intervals; losses of separation (32 m / 7.62 m);
L+C drones that had to leave their wing behind a multicopter. """
import os, sys
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, ROOT); os.chdir(ROOT)
import numpy as np
import bluesky as bs

variant = sys.argv[1] if len(sys.argv) > 1 else "shared"
eta = sys.argv[2] if len(sys.argv) > 2 else "0"
demand = sys.argv[3] if len(sys.argv) > 3 else "60"
bs.init(mode="sim", detached=True)
for cmd in ["DT 0.5", "PLUGINS LOAD ENERGY", "PLUGINS LOAD FLIGHTLOG", "PLUGINS LOAD VPMS",
            "PLUGINS LOAD TRAFFIC", f"VPMS {variant}", f"FLIGHTLOG vpms_{variant}",
            f"TRAFFIC {demand} 1 {eta} BASE 300 600"]:
    bs.stack.stack(cmd)
bs.sim.step()
def plugin_module(name, cls):
    return next(m for k, m in sys.modules.items() if k.endswith("." + name) and hasattr(m, cls))
mods = {"vpms": plugin_module("vpms", "Vpms"), "thesis_traffic": plugin_module("thesis_traffic", "Traffic"),
        "energy": plugin_module("energy", "Energy")}
vp, sc, en = mods["vpms"].Vpms.implinstance(), mods["thesis_traffic"].Traffic.implinstance(), mods["energy"].Energy.implinstance()
V = mods["vpms"]
print(f"Variant {variant}, prediction error {eta} %, {demand} drones/h: radius MC {V.arc_radius(False):.0f} m, "
      f"L+C {V.arc_radius(True):.0f} m, gates at {V.gate_distance():.0f} m")

los_pairs, overshoot, max_radial, trans_on_arc, spawned = set(), 0.0, 0.0, set(), set()
while bs.sim.simt < 4000 and not sc.done:
    bs.sim.step()
    n = bs.traf.ntraf
    spawned.update(bs.traf.id)
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
    for i in range(n):
        if vp.state[i] == V.ARC:
            err = np.hypot(x[i], y[i]) - V.arc_radius(vp.is_lc[i])        # + outside, - inside
            prog = (np.degrees(np.arctan2(x[i], y[i])) - V.GATE_BEARINGS[vp.gate[i]]) % 360
            prog = 0.0 if prog > 300 else prog
            if prog > 1.0:
                overshoot = min(overshoot, err)                        # inside the circle: overshoot
            if prog > 20.0:
                max_radial = max(max_radial, abs(err))                 # once on the arc
            if vp.is_lc[i] and bs.traf.tas[i] < 11.8:
                trans_on_arc.add(bs.traf.id[i])

merge = np.diff([t for t, _ in vp.merges]); touch = np.diff([t for t, _ in vp.touchdowns])
print(f"\nSimulated {bs.sim.simt:.0f} s; {len(spawned)} drones created, "
      f"{len(vp.touchdowns)} landed, {bs.traf.ntraf} still flying (expected 0)")
print(f"Merge-point intervals:  min {merge.min():.1f} s, mean {merge.mean():.1f} s  (target {V.T_TARGET:.0f} s)")
print(f"Touchdown intervals:    min {touch.min():.1f} s  (one at a time: >= 33.5 s at 40 s target)")
print(f"Arcs: overshoot to the inside {max(-overshoot, 0.0):.1f} m; largest deviation after the first 20 deg {max_radial:.1f} m")
print(f"Losses of separation (32 m and 7.62 m): {len(los_pairs)} pairs {sorted(los_pairs)[:6]}")
print(f"L+C below stall on an arc (behind a slower or holding drone): {len(trans_on_arc)} {sorted(trans_on_arc)[:6]}")

holds = sum(1 for t, _ in vp.merges)  # placeholder to keep names short
import csv
rows = list(csv.DictReader(open(f"output/vpms_{variant}/flights.csv")))
for cfg in ("MC", "LC"):
    r = [x for x in rows if x["config"] == cfg]
    if r:
        m = lambda c: np.mean([float(x[c]) for x in r])
        print(f"{cfg}: {len(r)} drones, {m('flight_time'):.0f} s in the structure, {m('energy_wh'):.1f} Wh, "
              f"hover {m('t_hover'):.0f} s, transitions {m('n_trans'):.1f}")
