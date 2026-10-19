""" ENERGY + FLIGHTLOG + ARRIVALS together, without GUI.
Run from the bluesky folder:  python plugins/thesis/tests/test_together.py """
import os, sys, csv
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, ROOT); os.chdir(ROOT)
import bluesky as bs

bs.init(mode="sim", detached=True)
for cmd in ["DT 0.5", "PLUGINS LOAD ENERGY", "PLUGINS LOAD FLIGHTLOG", "PLUGINS LOAD ARRIVALS",
            "FLIGHTLOG together", "ARRIVALS 120 300 1"]:
    bs.stack.stack(cmd)
bs.sim.step()
while bs.sim.simt < 400:
    bs.sim.step()
n = bs.traf.ntraf
for acid in list(bs.traf.id):        # until the VPMS plugin lands them: delete all
    bs.stack.stack(f"DEL {acid}")
bs.sim.step()

rows = list(csv.DictReader(open("output/flights_together.csv")))
print(f"{n} drones flown, {len(rows)} lines in the log (must be equal)")
for r in rows[:5]:
    print(f"  {r['id']}: {float(r['flight_time']):5.0f} s, {float(r['energy_wh']):5.2f} Wh, "
          f"SoC {float(r['soc_start']):.3f} -> {float(r['soc_end']):.3f}")
