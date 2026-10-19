"""Generate 2D cost map: x=fuel, y=detumble+slew time, color=cost."""
import sys, os, json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import sad_py
from SAD_Project.core import config as C
from SAD_Project.missions.run_monte_carlo import run_one

def main():
    grid_Kd = [0.005, 0.01, 0.03, 0.067, 0.1]
    grid_Ks = [0.1, 0.2, 0.34, 0.7, 1.5]
    N = 3
    fuels = []; times = []; costs = []; params_list = []
    rng = np.random.default_rng(7)
    for kd in grid_Kd:
        for ks in grid_Ks:
            C.ACS_K_DETUMBLE = kd
            C.ACS_K_SLEW = ks
            # keep P proportional-ish
            C.ACS_P_DETUMBLE = kd * 2.5
            C.ACS_P_SLEW = ks * 2.0
            for _ in range(N):
                r = run_one(rng)
                fuel = r['fuel_used']
                # total time detumble+slew
                ds = r['detumble_settle']
                ss = r['slew_settle']
                if ss is not None:
                    ttime = ss
                elif ds is not None:
                    ttime = ds
                else:
                    ttime = 120.0
                fr = r['final_rate']
                if fr is None or (isinstance(fr, float) and (np.isnan(fr) or np.isinf(fr))):
                    cost = 50.0
                else:
                    cost = float(fr) + (0.0 if r['reached_track'] else 10.0)
                fuels.append(fuel)
                times.append(ttime)
                costs.append(cost)
                params_list.append((kd, ks))
                print(f"kd={kd:.3f} ks={ks:.3f} fuel={fuel:.4f} time={ttime:.1f} cost={cost:.2f}")
    fuels = np.array(fuels); times = np.array(times); costs = np.array(costs)
    # save data
    out_dir = os.path.join(os.path.dirname(__file__), '..', 'results_cost_map')
    os.makedirs(out_dir, exist_ok=True)
    np.savez(os.path.join(out_dir, 'cost_map.npz'),
             fuel=fuels, time=times, cost=costs, params=np.array(params_list))
    # plot
    fig, ax = plt.subplots(figsize=(8,6))
    sc = ax.scatter(fuels, times, c=costs, cmap='viridis_r', s=60, edgecolors='k', linewidths=0.5)
    plt.colorbar(sc, ax=ax, label='Cost')
    ax.set_xlabel('Fuel consumption (kg)')
    ax.set_ylabel('Total time detumble+slew (s)')
    ax.set_title('Cost surface: fuel vs detumble+slew time')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'cost_map.png'), dpi=150)
    print('saved', os.path.join(out_dir, 'cost_map.png'))

if __name__ == '__main__':
    main()
