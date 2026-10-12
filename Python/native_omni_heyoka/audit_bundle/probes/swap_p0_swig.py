"""Phase-0 probe 3 (swap): SWIG swap-surface check.

Runtime facts about DynamicObject::setIntegrator and the integrator slot:
  (a) stock Spacecraft ships with a default integrator (upstream ctor =
      svIntegratorRK4) and exposes it as a StateVecIntegrator proxy;
  (b) setIntegrator accepts StateVecIntegrator* (stock RK4 ok) and
      rejects wrong types with TypeError;
  (c) Python-level subclassing of StateVecIntegrator is blocked
      (abstract, no directors) -> the swap MUST be implemented in C++;
  (d) syncDynamicsIntegration exists (multi-spacecraft path);
  (e) bsk_sdk ships stateVecIntegrator.h (SWIG %import target).

Run: ../../env/bin/python audit_bundle/probes/swap_p0_swig.py
Writes: audit_bundle/results/swap_p0_swig.json
Host-only (needs Basilisk macOS build); writes status=skipped otherwise.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import log_text, save_result, safe_import_basilisk  # noqa: E402


def main() -> int:
    if not safe_import_basilisk():
        save_result("swap_p0_swig",
                    {"status": "skipped", "reason": "Basilisk not importable"})
        return 0
    payload: dict = {"status": "ok"}
    from Basilisk.simulation import spacecraft, svIntegrators

    sc = spacecraft.Spacecraft()
    payload["default_integrator_proxy"] = type(sc.integrator).__name__

    rk = svIntegrators.svIntegratorRK4(sc)
    sc.setIntegrator(rk)
    payload["setIntegrator_stock_ok"] = True

    try:
        sc.setIntegrator(42)
        payload["wrong_type_rejected"] = False
    except TypeError as exc:
        payload["wrong_type_rejected"] = True
        payload["wrong_type_error"] = str(exc)[:160]

    try:
        class PyInt(svIntegrators.StateVecIntegrator):
            pass
        PyInt(sc)  # definition succeeds; instantiation must fail (abstract)
        payload["python_subclass_blocked"] = False
    except (AttributeError, TypeError) as exc:
        payload["python_subclass_blocked"] = True
        payload["python_subclass_error"] = str(exc)[:160]

    payload["syncDynamicsIntegration_present"] = hasattr(
        sc, "syncDynamicsIntegration")
    payload["isDynamicsSynced_present"] = hasattr(sc, "isDynamicsSynced")

    try:
        import importlib.util
        spec = importlib.util.find_spec("bsk_sdk")
        hdr = (Path(spec.origin).resolve().parent / "include" / "Basilisk"
               / "simulation" / "dynamics" / "_GeneralModuleFiles"
               / "stateVecIntegrator.h")
        payload["bsk_sdk_stateVecIntegrator_h"] = hdr.exists()
        payload["bsk_sdk_header"] = str(hdr)
    except Exception as exc:
        payload["bsk_sdk_stateVecIntegrator_h"] = False
        payload["bsk_sdk_error"] = f"{type(exc).__name__}: {exc}"

    ok = (payload["setIntegrator_stock_ok"]
          and payload["wrong_type_rejected"]
          and payload["python_subclass_blocked"]
          and payload["syncDynamicsIntegration_present"]
          and payload["bsk_sdk_stateVecIntegrator_h"])
    payload["verdict"] = ("C++-only subclass, SWIG pointer check, "
                          "header present" if ok else "REVIEW")
    if not ok:
        payload["status"] = "fail"
    save_result("swap_p0_swig", payload)
    log_text("swap_p0_swig", f"verdict={payload['verdict']}")
    print(f"probe3 verdict: {payload['verdict']}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
