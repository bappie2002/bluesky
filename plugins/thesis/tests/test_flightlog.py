""" Test of the FLIGHTLOG plugin, without GUI.
Run from the bluesky folder:  python plugins/thesis/tests/test_flightlog.py """
import os, sys
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, ROOT); os.chdir(ROOT)
import bluesky as bs

bs.init(mode="sim", detached=True)
for cmd in ["PLUGINS LOAD ENERGY", "PLUGINS LOAD FLIGHTLOG", "FLIGHTLOG test",
            "CRE MC1 M200 52.0 4.0 90 300 0",        # hovers
            "CRE MC2 M200 52.0 4.0002 90 300 0",     # hovers 13.7 m east of MC1: loss of separation
            "SOC MC2 0.5",                           # as the ARRIVALS plugin does
            "CRE LC1 AMZN 52.0 4.05 90 300 30"]:     # flies on its wing, far away
    bs.stack.stack(cmd)
bs.sim.step()
while bs.sim.simt < 120:
    bs.sim.step()
for acid in ["MC1", "MC2", "LC1"]:
    bs.stack.stack(f"DEL {acid}")
bs.sim.step()

print(open("output/flights_test.csv").read())
print("Expected: flight_time 120 s; MC1 energy 8.78 Wh and hover_time 120 s; MC2 soc_start 0.5;")
print("          MC1 and MC2 min_sep 13.7 m and los_time 120 s; LC1 hover_time 0.")
