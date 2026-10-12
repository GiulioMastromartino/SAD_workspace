"""Aggregate results/wp*.json -> results/bundle_summary.json.  Never raises."""
from __future__ import annotations

import glob
import json
import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from common import RESULTS_DIR, env_info, log_text, save_result


def _load_one(path: Path) -> dict:
    try:
        doc = json.loads(path.read_text(encoding="utf-8"))
        payload = doc.get("payload", {}) if isinstance(doc, dict) else {}
        status = str(payload.get("status", doc.get("status", "unknown")))
        passed = payload.get("passed", None)
        checks = payload.get("checks", payload.get("findings", {}))
        if isinstance(checks, dict):
            n_pass = sum(1 for v in checks.values()
                         if isinstance(v, dict) and v.get("ok") is True)
            n_fail = sum(1 for v in checks.values()
                         if isinstance(v, dict) and v.get("ok") is False)
        else:
            n_pass, n_fail = 0, 0
        return {"file": path.name, "probe": doc.get("name", path.stem),
                "status": status, "passed": passed,
                "n_pass": n_pass, "n_fail": n_fail, "ok": True}
    except Exception as exc:
        return {"file": path.name, "probe": path.stem, "status": "unreadable",
                "error": f"{type(exc).__name__}: {exc}", "ok": False}


def main() -> dict:
    files = sorted(Path(RESULTS_DIR).glob("wp*.json"))
    # Fallback: literal glob (same dir) in case RESULTS_DIR differs.
    if not files:
        files = sorted(Path(glob.escape(str(RESULTS_DIR))).glob("wp*.json"))
    per_probe = [_load_one(p) for p in files]
    summary = {
        "status": "ok" if per_probe else "empty",
        "n_probes": len(per_probe),
        "n_ok": sum(1 for r in per_probe if r.get("ok")),
        "n_fail_files": sum(1 for r in per_probe if not r.get("ok")),
        "n_checks_pass": sum(int(r.get("n_pass", 0)) for r in per_probe),
        "n_checks_fail": sum(int(r.get("n_fail", 0)) for r in per_probe),
        "probes": per_probe,
        "env": env_info(),
    }
    out = save_result("bundle_summary", summary)
    try:
        log_text("collect", f"aggregated {len(per_probe)} probe(s) -> {out}")
    except Exception:
        pass
    print(out)
    return summary


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # last-resort: never raise out of collect.py
        try:
            save_result("bundle_summary",
                        {"status": "error",
                         "error": f"{type(exc).__name__}: {exc}",
                         "traceback": traceback.format_exc()[-4000:]})
        except Exception:
            pass
        print(traceback.format_exc())
