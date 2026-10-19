""" Test of the FLIGHTLOG plugin, without GUI.
Run from the bluesky folder:  python plugins/thesis/tests/test_flightlog.py """
import os, sys, csv
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, ROOT); os.chdir(ROOT)
import bluesky as bs

bs.init(mode="sim", detached=True)


def run_until(t):
    while bs.sim.simt < t:
        bs.sim.step()


for cmd in ["PLUGINS LOAD ENERGY", "PLUGINS LOAD FLIGHTLOG", "FLIGHTLOG test TRACE",
            "CRE MC1 M200 52.0 4.0 90 300 0",        # hovers
            "CRE MC2 M200 52.0 4.0002 90 300 0",     # hovers 13.7 m east of MC1: loss of separation
            "SOC MC2 0.5",                           # as the ARRIVALS plugin does
            "CRE LC1 AMZN 52.0 4.05 90 300 30"]:     # flies on its wing, far away
    bs.stack.stack(cmd)
bs.sim.step()
energy = next(m for k, m in sys.modules.items() if k.endswith(".energy"))
run_until(60)
bs.stack.stack("MC2 SPD 16")      # MC2 flies away east at 16 kts (8.2 m/s): hover -> rotor, end of LoS
bs.stack.stack("LC1 SPD 0")       # LC1 slows down: wing -> rotor (transition) -> hover
run_until(120)
for acid in ["MC1", "MC2", "LC1"]:
    bs.stack.stack(f"DEL {acid}")
bs.sim.step()


def table(path, columns):
    rows = list(csv.DictReader(open(path)))
    print(f"\n{path}: {len(rows)} lines")
    print("  " + "".join(f"{c:>12}" for c in columns))
    for r in rows:
        print("  " + "".join(f"{r[c]:>12}" for c in columns))
    return rows


table("output/test/flights.csv", ["id", "config", "flight_time", "energy_wh", "wh_hover", "wh_rotor",
                                  "wh_wing", "wh_trans", "n_trans", "t_hover", "t_rotor", "t_wing"])
table("output/test/flights.csv", ["id", "soc_start", "soc_end", "min_sep_m", "los_time"])
table("output/test/events.csv", ["t", "id", "event", "info", "mode", "tas", "soc", "energy_wh"])
trace = list(csv.DictReader(open("output/test/trace.csv")))
print(f"\noutput/test/trace.csv: {len(trace)} lines (3 drones x 120 s, so about 360)")

print("\nExpected:")
print("  MC1: 8.78 Wh, all hover (t_hover 120 s), no events except spawn and end")
print("  MC2: soc_start 0.5; hover until 60 s, then rotor; min_sep 13.7 m; los_end soon after 60 s")
print(f"  LC1: wing until 60 s, then rotor and hover; n_trans 1, wh_trans {energy.LC['E_tr'] / 3600:.2f} Wh")
print("  events: los_start for MC1 and MC2 at the start, los_end for both after 60 s")
print("  All energy_wh = wh_hover + wh_rotor + wh_wing + wh_trans")
