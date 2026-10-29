""" ENERGY + FLIGHTLOG + VPMS + TRAFFIC together, without GUI.
Run from the bluesky folder:  python plugins/thesis/tests/test_together.py """
import os, sys, csv
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, ROOT); os.chdir(ROOT)
import bluesky as bs

bs.init(mode="sim", detached=True)
for cmd in ["DT 0.5", "PLUGINS LOAD ENERGY", "PLUGINS LOAD FLIGHTLOG", "PLUGINS LOAD VPMS",
            "PLUGINS LOAD TRAFFIC", "FLIGHTLOG together TRACE", "TRAFFIC 120 1 0 BASE 0 300"]:
    bs.stack.stack(cmd)
bs.sim.step()
traffic = next(m for k, m in sys.modules.items()
               if k.endswith(".thesis_traffic") and hasattr(m, "Traffic")).Traffic.implinstance()
created = set()
while bs.sim.simt < 3000 and not traffic.done:
    bs.sim.step()
    created.update(bs.traf.id)
bs.sim.step()

folder = "output/together"
data = {k: list(csv.DictReader(open(f"{folder}/{k}.csv"))) for k in ["flights", "events", "trace"]}
print(f"{len(created)} drones flown and landed by VPMS, {len(data['flights'])} lines in flights.csv (must be equal)")
for k, rows in data.items():
    print(f"  {k}.csv: {len(rows):5d} lines, {os.path.getsize(f'{folder}/{k}.csv') / 1e3:6.1f} kB")
events = [r["event"] for r in data["events"]]
print("  events: " + ", ".join(f"{e} {events.count(e)}" for e in
                               ["spawn", "arc_in", "direct_to", "merge", "touchdown", "end", "los_start"]))

print("\nPer configuration (mean per drone)")
print(f"  {'':<4}{'n':>3}{'time s':>8}{'Wh':>7}{'Wh/min':>8}{'hover%':>8}{'trans':>7}{'SoC end':>9}")
for cfg in ["MC", "LC"]:
    rows = [r for r in data["flights"] if r["config"] == cfg]
    mean = lambda c: sum(float(r[c]) for r in rows) / len(rows)
    print(f"  {cfg:<4}{len(rows):3d}{mean('flight_time'):8.0f}{mean('energy_wh'):7.2f}"
          f"{60 * mean('energy_wh') / mean('flight_time'):8.2f}"
          f"{100 * mean('t_hover') / mean('flight_time'):8.0f}{mean('n_trans'):7.1f}{mean('soc_end'):9.3f}")
print("\nExpected: equal counts; spawn = arc_in = merge = touchdown = end; los_start 0;")
print("hover > 0 (final descent); LC at least 1 transition (landing). In these short flights the LC uses")
print("more Wh per minute than the MC: a third of its time is the final descent at hover power (690 vs 298 W).")
print(f"For figures: python analysis/run_report.py <path to bluesky>/{folder}  (from the experimenten folder)")
