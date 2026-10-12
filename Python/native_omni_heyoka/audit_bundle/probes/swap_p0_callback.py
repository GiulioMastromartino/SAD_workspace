"""Phase-0 probe 1 (swap): heyoka numeric-callback RHS — confirm absent.

Static include-probe against the heyoka headers used by the host build
(heyoka 7.12.1). Checks:
  (a) every taylor_adaptive constructor takes a symbolic system
      (vector<pair<expression,expression>> or var_ode_sys) — no
      numeric-callable RHS overload;
  (b) heyoka::func (UDF) requires taylor_decompose LLVM rules or
      gradient() — a stock numeric EOM cannot satisfy it without
      hand-written Taylor arithmetic;
  (c) heyoka::callable / step_callback serve events + per-step
      observation, never the RHS.
Verdict closes the shortcut door; mirror+compose stays mandatory
(and is already validated by Gates F/G/t8/A).

Run: ../../env/bin/python audit_bundle/probes/swap_p0_callback.py
Writes: audit_bundle/results/swap_p0_callback.json
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import env_info, log_text, save_result  # noqa: E402


def _find_heyoka_include() -> Path | None:
    cands = []
    try:
        import heyoka  # noqa: F401
        import sysconfig
        cands.append(Path(sysconfig.get_paths()["include"]) / "heyoka")
    except Exception:
        pass
    for raw in ("/Users/giuliomastromartino/Documents/Polimi/MAGISTRALE/SAD/Project/Python/env/include/heyoka",
                "/opt/homebrew/include/heyoka"):
        cands.append(Path(raw))
    for c in cands:
        if (c / "taylor.hpp").exists():
            return c
    return None


def main() -> int:
    inc = _find_heyoka_include()
    payload: dict = {"include_dir": str(inc) if inc else None}
    if inc is None:
        payload.update({"status": "error", "verdict": "unknown (headers not found)"})
        save_result("swap_p0_callback", payload)
        return 2

    taylor = (inc / "taylor.hpp").read_text()
    ctors = re.findall(r"^\s*(?:explicit\s+)?taylor_adaptive\(([^;]*?)\);",
                       taylor, re.M)
    ctor_sigs = ["taylor_adaptive(%s)" % re.sub(r"\s+", " ", c).strip()
                 for c in ctors]
    symbolic_only = all(
        ("expression" in c or "var_ode_sys" in c or "llvm_state" in c
         or c.strip() in ("", "private_ctor_t", "const taylor_adaptive &",
                          "taylor_adaptive &&") or "KwArgs" in c)
        and "unction" not in c and "allable" not in c and "ambda" not in c
        for c in ctors)
    payload["taylor_adaptive_ctors"] = ctor_sigs
    payload["ctors_symbolic_only"] = symbolic_only

    func = (inc / "func.hpp").read_text()
    payload["func_requires_taylor_decompose"] = (
        "taylor_decompose" in func)
    payload["func_requires_gradient_or_decompose"] = (
        "func_has_gradient" in func and "func_has_taylor_decompose" in func)

    events = (inc / "events.hpp").read_text()
    payload["callable_used_for"] = (
        "event callbacks" if "callback_t = callable<" in events
        else "unknown")
    step_cb = (inc / "step_callback.hpp").read_text() if (inc / "step_callback.hpp").exists() else ""
    payload["step_callback_is_observation"] = (
        "propagate" in step_cb or "taylor_adaptive" in step_cb)

    numeric_rhs_overload = any(
        ("unction" in c or "allable" in c) and "expression" not in c
        for c in ctors)
    verdict = ("absent (mirror+compose mandatory)"
               if (symbolic_only and not numeric_rhs_overload)
               else "REVIEW (unexpected ctor shape)")
    payload.update({"numeric_rhs_overload_found": numeric_rhs_overload,
                    "verdict": verdict,
                    "status": "ok" if verdict.startswith("absent") else "review"})
    save_result("swap_p0_callback", payload)
    log_text("swap_p0_callback",
             f"[{env_info()['os_tag']}] ctors={len(ctor_sigs)} "
             f"symbolic_only={symbolic_only} verdict={verdict}")
    print(f"probe1 verdict: {verdict} (ctypes={len(ctor_sigs)} checked)")
    return 0 if payload["status"] == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
