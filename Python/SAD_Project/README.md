# SAD_Project

6U CubeSat attitude dynamics. Modular C++ library (first-class Simulink blocks) + Python reference + parity tests.

## Build

```bash
cd Python/SAD_Project
rm -rf build && mkdir build && cd build
cmake .. && make -j8
```

Produces `python/sad_py*.so` (pybind module) for Python parity tests.

The mission batch has a fixed-step RK4 backend (default) and an optional
Heyoka Taylor backend.  CMake enables Heyoka automatically when it finds it in
`Python/env`; disable with `-DSAD_WITH_HEYOKA=OFF`.  In Python, set
`BatchParams.use_heyoka = True` (and optionally `heyoka_tol`).  Benchmark both:

```bash
cd Python
PYTHONPATH=.:SAD_Project/python env/bin/python \
    SAD_Project/diagnostics/bench_integrators.py --N 300 --repeats 3
```

See `diagnostics/README.md` and `results_bench/README.md` for numbers.

## Run mission

```bash
cd Python
PYTHONPATH=. env/bin/python SAD_Project/missions/run_detumble_slew.py
```

## Tests

```bash
cd Python
PYTHONPATH=. env/bin/python -m pytest SAD_Project/tests/ -q
```

## Simulink

Each module ships a Level-2 C MEX S-function in `cpp/simulink/`. Build with MATLAB's `mex` against the module headers. See `SAD_Project_PLAN.html` for the full module inventory and roadmap.

## Structure

- `cpp/` - C++ modules (core, dynamics, actuators, sensors, environment, control, mission, python_bindings, simulink)
- `python/` - pybind module output
- `SAD_Project/` - Python reference package (config, core, control, missions, tests)
- `tests/` - pytest suite



cd /Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python && nohup env PYTHONPATH=/Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python:/Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python/SAD_Project/python /Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python/env/bin/python -c "
from SAD_Project.optimization.optimizer import main
main(generations=50, popsize=15, sigma0=0.6, seed=42, N=50, workers=8)
" > /tmp/opt18.log 2>&1 &
echo "started"
