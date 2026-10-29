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
    print(f"  {'id':<5}{'tas m/s':>8}{'mode':>7}{'power W':>9}{'used Wh':>9}{'SoC':>8}"
          f"{'hover':>8}{'rotor':>8}{'wing':>8}{'trans':>8}  (Wh per mode)")
    for i, acid in enumerate(bs.traf.id):
        print(f"  {acid:<5}{bs.traf.tas[i]:8.1f}{energy.MODES[plugin.mode[i]]:>7}{plugin.power[i]:9.1f}"
              f"{plugin.used_wh[i]:9.2f}{plugin.soc[i]:8.4f}{plugin.wh_hover[i]:8.2f}{plugin.wh_rotor[i]:8.2f}"
              f"{plugin.wh_wing[i]:8.2f}{plugin.wh_trans[i]:8.2f}")


run(60)
show("After 60 s")
bs.stack.stack("LC1 SPD 0")       # Lift+Cruise slows down to hover
run(60)
show("After 120 s")
bs.stack.stack("LC1 SPD 30")      # Lift+Cruise speeds up again and goes back on its wing
run(60)
show("After 180 s")
print("\nExpected:")
print(f"  MC1 hover power {energy.MC.p_hover:.1f} W; after 120 s {energy.MC.p_hover * 120 / 3600:.2f} Wh, all in 'hover'")
print(f"  LC1 hover power {energy.LC.p_hover:.1f} W; wing power at 15.5 m/s {energy.LC.wing_power(15.5):.1f} W")
print(f"  LC1 after 120 s: one transition (wing -> rotor) {energy.E_TRANS_J / 3600:.2f} Wh in 'trans'")
print(f"  LC1 after 180 s: plus rotor -> wing {energy.E_TRANS_WING_J / 3600:.2f} Wh, "
      f"total {(energy.E_TRANS_J + energy.E_TRANS_WING_J) / 3600:.2f} Wh in 'trans', n_trans 2: "
      f"{int(plugin.n_trans[1])}")
print("  used = hover + rotor + wing + trans for both")
