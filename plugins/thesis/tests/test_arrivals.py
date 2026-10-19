""" Test of the ARRIVALS plugin, without GUI.
Run from the bluesky folder:  python plugins/thesis/tests/test_arrivals.py """
import os, sys
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, ROOT); os.chdir(ROOT)
import numpy as np
import bluesky as bs

bs.init(mode="sim", detached=True)
for cmd in ["DT 0.5", "PLUGINS LOAD ENERGY", "PLUGINS LOAD ARRIVALS", "ARRIVALS 120 600 1"]:
    bs.stack.stack(cmd)
bs.sim.step()
arr = next(m for k, m in sys.modules.items() if k.endswith(".arrivals"))
energy = next(m for k, m in sys.modules.items() if k.endswith(".energy")).Energy.implinstance()

# 1. The arrival list itself
a, b = arr.arrival_list(120, 3600, 1), arr.arrival_list(120, 3600, 1)
c = arr.arrival_list(120, 3600, 2)
print("1. Arrival list for 120 drones/h, 1 h")
print(f"   count {len(a['time'])} (expected about 120 +- 11)")
print(f"   share L+C {np.mean(a['config'] == 'LC'):.2f} (expected about 0.5)")
print(f"   SoC from {a['soc'].min():.2f} to {a['soc'].max():.2f} (range {arr.SOC_RANGE})")
print(f"   same seed gives same list: {np.array_equal(a['time'], b['time'])}, "
      f"other seed differs: {not np.array_equal(a['time'][:5], c['time'][:5])}")

# 2. Creation in the simulation
while bs.sim.simt < 600:
    bs.sim.step()
plan = arr.Arrivals.implinstance().plan
print(f"\n2. After 600 s: {bs.traf.ntraf} drones created, {len(plan['time'])} planned")
for i in range(min(4, bs.traf.ntraf)):
    print(f"   {bs.traf.id[i]}: type {bs.traf.type[i]}, alt {bs.traf.alt[i]:.0f} m, "
          f"tas {bs.traf.tas[i]:.1f} m/s, SoC {energy.soc[i]:.3f}")
