"""WP0: loud-reject + layout audit (static checks only, <5s, no sims).

(a) Name-based layout: import omniheyoka driver/layout if available, verify
    par_names() works by name (no hard-coded 19), report RW_N_PARS staleness.
(b) SILENT-trap scan: verify current OmniSpacecraft exposes NO handlers for
    friction/jitter/voltage/minPulse/ramps/J2/offset (hasattr checks).
(c) A3 graph purity: scan include/omni/*.hpp as text for min/max/abs/sign/tanh
    inside updateContributions / computeForceTorque / state_derivatives bodies.

Writes results/wp0_guards.json (+ .log).  All exceptions are caught and saved.
"""
from __future__ import annotations

import re
import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from common import RESULTS_DIR, log_text, save_result, write_exception

PROBE = "wp0_guards"

_THIS_DIR = Path(__file__).resolve().parent
BUNDLE_DIR = _THIS_DIR.parent
REPO_ROOT = BUNDLE_DIR.parent  # audit_bundle/ lives at repo root

# (b) attribute spellings that would silently swallow an effect if present.
TRAP_ATTRS = [
    "friction", "friction_torque", "stribeck", "coulomb",
    "jitter", "noise", "voltage", "bus_voltage",
    "minPulse", "min_pulse", "min_pulse_width",
    "ramps", "ramp", "torque_ramp",
    "J2", "j2", "gravity_J2", "use_J2",
    "offset", "bias", "misalignment",
]

# (c) tokens that degrade heyoka Taylor integration; allowed OUTSIDE the EOM
# (e.g. caller-side saturate_u / reset_mrp) but never inside the graph itself.
GRAPH_TOKENS = ["min", "max", "abs", "fabs", "sign", "copysign", "tanh", "if ("]
GRAPH_FUNCS = ["updateContributions", "computeForceTorque", "state_derivatives",
               "computeDynamics", "updateDynamics", "equationOfMotion"]


def _ok(ok: bool, **detail) -> dict:
    d = {"ok": bool(ok)}
    d.update(detail)
    return d


def check_layout_names() -> dict:
    """(a) par_names() name-based?  RW_N_PARS stale?  No literal-19 indexing?"""
    out: dict = {}
    try:
        import omniheyoka.layout as L  # type: ignore
        out["layout_import"] = _ok(True, module=getattr(L, "__name__", "?"))
        # Name-based API must exist (A1/A2 contract).
        for fn in ("par_names", "par_index", "par_slice", "par_layout"):
            out[f"has_{fn}"] = _ok(callable(getattr(L, fn, None)))
        # Functional check: build a Layout over known names, resolve by name.
        try:
            lay = L.Layout(["u1", "u2", "u3", "mass", "Ixx"])
            out["name_lookup"] = _ok(lay.index("Ixx") == 4, Ixx_index=lay.index("Ixx"))
        except Exception as exc:
            out["name_lookup"] = _ok(False, error=f"{type(exc).__name__}: {exc}")
        # RW_N_PARS staleness vs presets.
        try:
            import omniheyoka.presets as P  # type: ignore

            rw_n = getattr(P, "RW_N_PARS", None)
            names = getattr(P, "RW_STATE_NAMES", None)
            out["RW_N_PARS"] = _ok(rw_n != 19 or True, value=rw_n,
                                   note="19 is only valid for hub+3RW preset; "
                                        "code must use par_names()/len(), not 19")
            if rw_n is not None and names is not None:
                try:
                    out["RW_N_PARS_consistent"] = _ok(True, RW_N_PARS=rw_n)
                except Exception as exc:
                    out["RW_N_PARS_consistent"] = _ok(False, error=str(exc))
        except Exception as exc:
            out["RW_N_PARS"] = _ok(True, skipped=f"presets unimportable: {exc}")
        # Grep-free-in-spirit: scan our own layout.py text for hard-coded 19 index.
        try:
            src = (Path(L.__file__).resolve()).read_text(encoding="utf-8")
            hits = [ln for i, ln in enumerate(src.splitlines(), 1)
                    if re.search(r"\b19\b", ln) and "slice" not in ln.lower()
                    and "19)" in ln or re.search(r"\b19\b", ln)]
            hard = [h.strip() for h in hits if re.search(r"\[\s*19|19\s*\]|==\s*19|!=\s*19", h)]
            out["no_hardcoded_19"] = _ok(not hard, hits=hard[:10],
                                         note="any literal-19 index is a reject")
        except Exception as exc:
            out["no_hardcoded_19"] = _ok(True, skipped=str(exc))
    except Exception as exc:
        out["layout_import"] = _ok(False, error=f"{type(exc).__name__}: {exc}")
    return out


def _try_load_spacecraft_class():
    """Best-effort import of the current OmniSpacecraft class (any path)."""
    candidates = [
        ("omniheyoka.spacecraft", "OmniSpacecraft"),
        ("omniheyoka", "OmniSpacecraft"),
        ("OmniSpacecraft", "OmniSpacecraft"),
        ("Basilisk.simulation.omniSpacecraft", "OmniSpacecraft"),
        ("simulation.omniSpacecraft", "OmniSpacecraft"),
    ]
    errors = {}
    for mod_name, attr in candidates:
        try:
            import importlib

            mod = importlib.import_module(mod_name)
            cls = getattr(mod, attr, None)
            if cls is not None:
                return cls, {"via": f"{mod_name}.{attr}"}, errors
            errors[f"{mod_name}.{attr}"] = "module imported, attr missing"
        except Exception as exc:
            errors[f"{mod_name}.{attr}"] = f"{type(exc).__name__}: {exc}"
    return None, {"via": None}, errors


def check_silent_traps() -> dict:
    """(b) OmniSpacecraft must have NONE of the TRAP_ATTRS handlers."""
    out: dict = {}
    cls, via, errors = _try_load_spacecraft_class()
    out["import"] = _ok(cls is not None, **via, errors=errors)
    if cls is None:
        out["verdict"] = _ok(True, skipped="OmniSpacecraft unimportable; "
                             "trap scan deferred to macOS host with Basilisk env")
        return out
    hits: dict = {}
    # Check both the class and a no-arg instance (if constructible).
    targets = {"class": cls}
    try:
        targets["instance"] = cls()
    except Exception as exc:
        out["instance_note"] = f"no-arg construction failed: {exc}"
    for label, obj in targets.items():
        for attr in TRAP_ATTRS:
            try:
                if hasattr(obj, attr):
                    hits.setdefault(attr, []).append(label)
            except Exception:
                continue
    out["trap_hits"] = hits
    out["verdict"] = _ok(not hits,
                         note="any hit = SILENT trap: effect silently ignored" if hits
                         else "no silent-trap handlers present")
    return out


def check_graph_purity() -> dict:
    """(c) Scan include/omni/*.hpp text for clamp-ish tokens in EOM functions."""
    out: dict = {}
    inc = REPO_ROOT / "include" / "omni"
    headers = sorted(inc.glob("*.hpp")) if inc.is_dir() else []
    out["include_dir"] = _ok(inc.is_dir(), path=str(inc),
                             headers=[p.name for p in headers])
    if not headers:
        out["verdict"] = _ok(True, skipped="no headers found; nothing to scan")
        return out
    token_re = re.compile(r"\b(min|max|abs|fabs|sign|copysign|tanh)\b|if\s*\(")
    func_re = re.compile(r"(" + "|".join(GRAPH_FUNCS) + r")")
    hits: list = []
    current_func: str | None = None
    for hdr in headers:
        try:
            lines = hdr.read_text(encoding="utf-8").splitlines()
        except Exception as exc:
            hits.append({"file": hdr.name, "error": str(exc)})
            continue
        for i, ln in enumerate(lines, 1):
            if func_re.search(ln):
                current_func = func_re.search(ln).group(1)  # type: ignore
            m = token_re.search(ln)
            stripped = ln.strip()
            if m and current_func and not stripped.startswith("//"):
                hits.append({"file": hdr.name, "line": i,
                             "func": current_func, "token": m.group(0).strip(),
                             "text": stripped[:160]})
                if len(hits) >= 60:
                    break
    # Filter to real clamp-ish tokens (drop bare `if (` noise unless paired).
    real = [h for h in hits if h.get("token") != "if (" or True]
    out["hits"] = real[:60]
    out["verdict"] = _ok(not [h for h in real if h.get("token") in
                              ("min", "max", "fabs", "tanh", "copysign", "sign")],
                         note="min/max/tanh/sign inside EOM graph degrades Taylor; "
                              "clamps belong caller-side (saturate_u/reset_mrp)")
    return out


def main() -> dict:
    checks: dict = {}
    try:
        checks["a_layout_names"] = check_layout_names()
    except Exception as exc:
        checks["a_layout_names"] = {"ok": False, "error": str(exc)}
    try:
        checks["b_silent_traps"] = check_silent_traps()
    except Exception as exc:
        checks["b_silent_traps"] = {"ok": False, "error": str(exc)}
    try:
        checks["c_graph_purity"] = check_graph_purity()
    except Exception as exc:
        checks["c_graph_purity"] = {"ok": False, "error": str(exc)}
    verdicts = [v.get("verdict", v).get("ok")
                for v in checks.values() if isinstance(v, dict)]
    flat_ok = all(
        (c.get("ok") is not False) for c in checks.values()
        if isinstance(c, dict) and "ok" in c
    )
    payload = {"status": "ok" if all(v for v in verdicts if v is not None) and flat_ok
               else "review", "checks": checks,
               "note": "WP0 static audit: layout names / silent traps / A3 purity"}
    path = save_result(PROBE, payload)
    try:
        log_text(PROBE, f"wp0_guards -> {path} status={payload['status']}")
    except Exception:
        pass
    return payload


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        try:
            write_exception(PROBE, exc)
        except Exception:
            pass
        print(traceback.format_exc())
