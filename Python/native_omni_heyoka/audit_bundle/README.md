# Audit bundle — one-command runner (macOS host)

Run ONE command from the repo root (`native_omni_heyoka/`):

```bash
bash audit_bundle/run_all.sh
```

What it does: picks `../../env/bin/python` if present else `python3`,
runs every `probes/wp*.py` sequentially (`tee` to `results/<name>.log`),
then `probes/collect.py` → `results/bundle_summary.json`. Idempotent.

Outputs (all under `audit_bundle/results/`, never just stdout):

- `wp0_guards.json` — layout/trap/A3-purity audit (this skeleton's probe)
- `wp<N>_*.json` — later probes drop files here; collector picks up `wp*.json`
- `wp<N>_*.log` — per-probe tee'd logs (+ `.exception.log` on crash)
- `bundle_summary.json` — per-probe status + pass/fail counts + `env_info`
- `collect.log` — aggregator log

Probes ONLY write to `audit_bundle/results/` (JSON + log). Basilisk/heyoka
imports are lazy in `try/except` and recorded in each JSON's `env` block.

To iterate: paste back `results/bundle_summary.json` (and any failing
`results/wp*.json`) into the chat. Re-run after each fix — no cleanup needed.

Env notes: host is macOS (Linux container can't run Basilisk binaries, so no
execution here). Python 3.11 compatible. WP0 is static checks only (<5s).
Paths are pathlib-relative to file location, so any cwd works.
