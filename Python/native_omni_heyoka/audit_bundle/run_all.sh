#!/usr/bin/env bash
# One-command audit-bundle runner (macOS host).
# Usage:  bash audit_bundle/run_all.sh
# Idempotent: mkdir -p results; re-running overwrites JSON, appends logs.
set -euo pipefail
shopt -s nullglob

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUNDLE_DIR="${SCRIPT_DIR}"
PROBES_DIR="${BUNDLE_DIR}/probes"
RESULTS_DIR="${BUNDLE_DIR}/results"
mkdir -p "${RESULTS_DIR}"

# Probes import via `from probes.common import ...` (package) with fallback to
# `from common import ...` (flat). Export both parents so either works no
# matter the host cwd. Also export the repo Python/ dir so `cubesat_12u_sim`
# (sibling of native_omni_heyoka) and `omniheyoka.presets` resolve — wp0's
# layout check needs it.
export PYTHONPATH="${BUNDLE_DIR}:${PROBES_DIR}:${BUNDLE_DIR}/../..:${PYTHONPATH:-}"

# Prefer the repo venv two levels up (repo_root/env/bin/python), else python3.
PY="python3"
if [[ -x "${BUNDLE_DIR}/../../env/bin/python" ]]; then
  PY="${BUNDLE_DIR}/../../env/bin/python"
elif [[ -x "${BUNDLE_DIR}/../env/bin/python" ]]; then
  PY="${BUNDLE_DIR}/../env/bin/python"
fi
echo "[run_all] python: ${PY}"
"${PY}" --version || true

# Run every probes/wp*.py sequentially; each probe writes results/<name>.json.
# Guard with `||` so one failing probe never stops the bundle (set -e safe).
for f in "${PROBES_DIR}"/wp*.py; do
  name="$(basename "${f}" .py)"
  echo "[run_all] === ${name} ==="
  # Fresh log per run; probe appends via log_text().
  : > "${RESULTS_DIR}/${name}.log"
  "${PY}" "${f}" 2>&1 | tee -a "${RESULTS_DIR}/${name}.log" || echo "[run_all] ${name} FAILED (continuing)"
done

# Aggregate into bundle_summary.json (never raises).
"${PY}" "${PROBES_DIR}/collect.py" 2>&1 | tee -a "${RESULTS_DIR}/collect.log" || echo "[run_all] collect FAILED"

echo "[run_all] done: ${RESULTS_DIR}/bundle_summary.json"
