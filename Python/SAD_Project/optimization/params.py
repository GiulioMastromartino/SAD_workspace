"""Control parameter space for SAD_Project optimization."""
import numpy as np

PARAM_NAMES = [
    'detumble_K', 'detumble_P', 'detumble_Ki',
    'slew_K', 'slew_P', 'slew_Ki',
    'track_K', 'track_P', 'track_Ki',
    'TS',  # time budget for detumble+slew (seconds)
]
BOUNDS = [
    (0.001, 10.0),  # detumble_K
    (0.01, 50.0),   # detumble_P
    (1e-6, 5.0),    # detumble_Ki
    (0.001, 10.0),  # slew_K
    (0.01, 50.0),   # slew_P
    (1e-6, 5.0),    # slew_Ki
    (0.01, 20.0),   # track_K
    (0.05, 50.0),   # track_P
    (1e-6, 10.0),   # track_Ki
    (60.0, 600.0),  # TS: detumble+slew time budget (1-10 min)
]
DEFAULTS = [5.339150251994063, 0.8828143280025442, 0.027321708852311766,
           1.286717625102853, 0.4457756848994122, 0.0031478207674284194,
           4.409346198161378, 49.85938825119385, 0.00010264766213601192,
           278.2200065567522]

def theta_to_params(theta):
    return dict(zip(PARAM_NAMES, [float(x) for x in theta]))

def params_to_theta(p):
    return np.array([p[n] for n in PARAM_NAMES], dtype=float)
