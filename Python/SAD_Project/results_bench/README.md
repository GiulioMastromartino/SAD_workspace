# Integrator benchmark

RK4 vs the optional Heyoka Taylor backend, same C++ mission kernel, same seeds
and dispersions (N=300, seed 42, three repeats, warm-up excluded).

Command:

```bash
env/bin/python SAD_Project/diagnostics/bench_integrators.py \
    --N 300 --repeats 3 --seed 42 --tol 1e-12
```

| backend | wall time (s) | ms/run | relative |
|---|---:|---:|---:|
| RK4 | 2.710 | 9.03 | 1.00x |
| Heyoka, tol 1e-12 | 5.082 | 16.94 | 1.88x slower |
| Heyoka, tol 1e-9 | 4.007 | 13.36 | 1.52x slower |
| Heyoka, tol 1e-12, compact | 12.495 | 41.65 | 4.75x slower |

Both backends: 0/300 failures, 300/300 reached track.  Pointwise KPI
differences at tol 1e-12: final rate `8.9e-12 rad/s`, fuel `1.1e-10 kg`,
`box_max` `4.6e-10 km`, track max `1.8e-6 deg`.

Files:

* `bench_heyoka_N300_tol1e-12.{json,log}` — default benchmark
* `bench_heyoka_N300_tol1e-09.{json,log}` — looser tolerance
* `bench_heyoka_N300_tol1e-12_compact.{json,log}` — Heyoka compact mode

The fixed-step RK4 is faster at the mission's 0.1 s / 1.0 s control steps;
Heyoka is a high-accuracy cross-check rather than a throughput win.
