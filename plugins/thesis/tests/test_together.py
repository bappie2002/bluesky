""" ENERGY + FLIGHTLOG + ARRIVALS together, without GUI.
Run from the bluesky folder:  python plugins/thesis/tests/test_together.py """
import os, sys, csv
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, ROOT); os.chdir(ROOT)
import bluesky as bs

bs.init(mode="sim", detached=True)
for cmd in ["DT 0.5", "PLUGINS LOAD ENERGY", "PLUGINS LOAD FLIGHTLOG", "PLUGINS LOAD ARRIVALS",
            "FLIGHTLOG together TRACE", "ARRIVALS 120 300 1"]:
    bs.stack.stack(cmd)
bs.sim.step()
while bs.sim.simt < 400:
    bs.sim.step()
n = bs.traf.ntraf
for acid in list(bs.traf.id):        # until the VPMS plugin lands them: delete all
    bs.stack.stack(f"DEL {acid}")
bs.sim.step()

folder = "output/together"
data = {k: list(csv.DictReader(open(f"{folder}/{k}.csv"))) for k in ["flights", "events", "trace"]}
print(f"{n} drones flown, {len(data['flights'])} lines in flights.csv (must be equal)")
for k, rows in data.items():
    print(f"  {k}.csv: {len(rows):5d} lines, {os.path.getsize(f'{folder}/{k}.csv') / 1e3:6.1f} kB")

print("\nPer configuration (mean per drone)")
print(f"  {'':<4}{'n':>3}{'time s':>8}{'Wh':>7}{'Wh/min':>8}{'hover%':>8}{'SoC end':>9}")
for cfg in ["MC", "LC"]:
    rows = [r for r in data["flights"] if r["config"] == cfg]
    mean = lambda c: sum(float(r[c]) for r in rows) / len(rows)
    print(f"  {cfg:<4}{len(rows):3d}{mean('flight_time'):8.0f}{mean('energy_wh'):7.2f}"
          f"{60 * mean('energy_wh') / mean('flight_time'):8.2f}"
          f"{100 * mean('t_hover') / mean('flight_time'):8.0f}{mean('soc_end'):9.3f}")
print("\nExpected: 7 = 7; MC uses more Wh per minute than LC (rotors vs wing); hover 0% (no holding yet).")
print(f"For figures: python analysis/run_report.py <path to bluesky>/{folder}  (from the experimenten folder)")
