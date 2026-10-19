#!/usr/bin/env bash
# Regenerates this results folder from a clean checkout.
# Run from the Python/ directory.
set -euo pipefail

./env/bin/python -m cubesat_12u_sim.tuning.run_mc_optimize --generations 50 --popsize 50 --runs 50 --vehicles 25 --workers 4 --seed 20261007 --checkpoint ckpt_mc.json

./env/bin/python -m cubesat_12u_sim.tuning.export_results
