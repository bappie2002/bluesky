""" Test of the ENERGY plugin, without GUI.
Run from the bluesky folder:  python plugins/thesis/tests/test_energy.py """
import os, sys
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, ROOT); os.chdir(ROOT)          # run as if started in the bluesky folder
import bluesky as bs

bs.init(mode="sim", detached=True)
for cmd in ["PLUGINS LOAD ENERGY",
            "CRE MC1 M200 52.0 4.0 90 300 0",     # multicopter, hovering
            "CRE LC1 AMZN 52.0 4.05 90 300 30"]:  # Lift+Cruise, 30 kts on its wing
    bs.stack.stack(cmd)
bs.sim.step()                                      # processes the commands above
energy = next(m for k, m in sys.modules.items() if k.endswith(".energy"))
plugin = energy.Energy.implinstance()


def run(seconds):
    t_end = bs.sim.simt + seconds
    while bs.sim.simt < t_end:
        bs.sim.step()


def show(label):
    print(f"\n{label} (t = {bs.sim.simt:.0f} s)")
    for i, acid in enumerate(bs.traf.id):
        print(f"  {acid}: tas {bs.traf.tas[i]:5.1f} m/s, on wing {plugin.on_wing[i]!s:5}, "
              f"used {plugin.used_wh[i]:6.2f} Wh, SoC {plugin.soc[i]:.4f}")


run(60)
show("After 60 s")
bs.stack.stack("LC1 SPD 0")       # Lift+Cruise slows down to hover
run(60)
show("After 120 s")
print(f"\nExpected MC1 after 120 s hover: {energy.MC['P_h'] * 120 / 3600:.2f} Wh")
print(f"Transition of LC1 (included): {energy.LC['E_tr'] / 3600:.2f} Wh")
