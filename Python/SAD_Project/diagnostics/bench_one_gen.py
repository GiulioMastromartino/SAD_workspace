"""End-to-end benchmark: N generations of CMA-ES at pop 15 / N=50 / 8 workers.

The first generation includes process-pool startup (cold); the second is the
steady-state number to compare against.

Run from Python/:
    PYTHONPATH=$PWD:$PWD/SAD_Project/python env/bin/python \
        -m SAD_Project.diagnostics.bench_one_gen
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, '..', '..'))   # Python/
sys.path.insert(0, os.path.join(_ROOT, 'SAD_Project', 'python'))
sys.path.insert(0, _ROOT)

from SAD_Project.optimization.optimizer import main

if __name__ == '__main__':
    main(generations=2, popsize=15, sigma0=0.6, seed=42, N=50,
         workers=8, timing=False)
