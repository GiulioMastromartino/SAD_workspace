#!/usr/bin/env bash
# scripts/ci_envelope_checks.sh -- WP0/WP7 envelope CI grep gates (WP25-C3 extended).
#
# Five checks (PLAN_ENVELOPE.md Ground rules 1/2/7/8, Rev 1 + WP25-C3):
#   (a) graph purity (A3): no min(/max(/std::min/std::max/std::abs/abs(/
#       tanh(/sign inside the symbolic graph paths of
#       include/omni/contributors.hpp, include/omni/system.hpp,
#       include/omni/environment.hpp, include/omni/force_torque.hpp,
#       include/omni/backsub_t.hpp and include/omni/linear3.hpp, i.e.
#       inside updateContributions / computeForceTorque / state_derivatives /
#       field bodies. Allowed: `if constexpr (...is_expression_v<S>)`
#       idle-skips, and the step-time double-side converters fillInputs /
#       quantize / onTime / thruster_on_times (never symbolic). The env wrench
#       helpers (dragWrench/srpFacet) are double-only free functions
#       (module-side, never symbolic) -- outside the tracked bodies by
#       construction, same as the converters.
#       AUTHORITATIVE GATE: the wp0_guards.py probe purity scan is a non-gate
#       mirror (naive token regex over wider GRAPH_FUNCS, verdict vacuous on
#       `if (` noise) -- this scoped awk is what binds.
#   (b) layout hygiene (A2): no hard-coded parameter counts
#       (`nPars() == 19`-style, `RW_N_PARS = 19`, `len(pars) - 16`) in
#       omniheyoka/*.py, tests/, scratch*. Full-line `#` comments are exempt
#       (historical notes); code must use par_names()/layout helpers.
#       Bare-`19` width args (`parse_system(..., names, 19)`) are check (e).
#   (c) layout hygiene (A1/A2): no hard-coded numeric state subscripts
#       (`st[12 + i]`-style) in src/ -- use state_offset-based members or
#       layout helpers. Full-line `//` comments are exempt.
#   (d) layout hygiene (A1): no UNJUSTIFIED numeric `.state[N]` subscripts in
#       omniheyoka/ctypes_driver.py (legacy RW-only ABI). Lines carrying a
#       trailing `LAYOUT-OK:` justification (hub block fixed 12 by A1, revive
#       via layout helpers) are exempt -- the driver itself is untouched.
#   (e) layout hygiene (A2): no bare-`19` par-width args
#       (`names, 19)`-shape, the historical reference-layout width) in
#       omniheyoka/, tests/, scratch*/ or the BUILD_STATUS.md Gate-G repro.
#       Lines carrying `CI-EXEMPT(bare-19)` (reference-layout repros pinned to
#       heyoka_rw.pack_rw_pars, never live counts) are exempt.
#
# Prints PASS/FAIL per check; exits 0 iff all pass, 1 otherwise.
# Pure grep/awk -- runs on Linux CI and the macOS host alike (no build).

set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)"
cd "$ROOT" || exit 2

PASS=0
FAIL=0
pass() { PASS=$((PASS + 1)); echo "PASS: $1"; }
fail() { FAIL=$((FAIL + 1)); echo "FAIL: $1"; }

# ------------------------------------------------------------------ (a) -----
# awk scope machine: tracks whether we are inside one of the four symbolic
# bodies (checked) or one of the four step-time converters (skipped) via
# definition-site detection + brace balance. Anything else is unscanned.
AWK_PURITY='
BEGIN {
  nT = split("updateContributions computeForceTorque state_derivatives field", TG, " ");
  nS = split("fillInputs quantize onTime thruster_on_times", SK, " ");
  mode = ""; depth = 0; pend = ""; pskip = 0; cur = ""; fails = 0;
}
{
  line = $0;
  sub(/\/\/.*$/, "", line);
  gsub(/\/\*.*\*\//, "", line);
  if (mode == "") {
    if (pend != "") {
      if (line ~ /\{/) { mode = (pskip ? "skip" : "check"); cur = pend; pend = ""; }
      else if (line ~ /;|\}/) { pend = ""; }
      if (pend != "" || mode == "") next;
    } else {
      f = ""; sk = 0;
      for (i = 1; i <= nT; i++) {
        if (match(line, "(^|[^.>:A-Za-z0-9_])" TG[i] "[ \t]*\\(")) { f = TG[i]; break; }
      }
      if (f == "") {
        for (i = 1; i <= nS; i++) {
          if (match(line, "(^|[^.>:A-Za-z0-9_])" SK[i] "[ \t]*\\(")) { f = SK[i]; sk = 1; break; }
        }
      }
      if (f == "") next;
      if (line ~ /\{/) { mode = (sk ? "skip" : "check"); cur = f; }
      else if (line ~ /;/) next;
      else { pend = f; pskip = sk; next; }
    }
  }
  if (mode == "check") {
    if (line !~ /if[ \t]+constexpr/) {
      if (line ~ /(std::min|std::max|std::abs)|([^A-Za-z0-9_:]|^)(min|max|abs|tanh|sign)[ \t]*\(/) {
        printf "%s:%d: [%s] %s\n", FILENAME, FNR, cur, $0;
        fails++;
      }
    }
  }
  a = line; nopen = gsub(/\{/, "X", a);
  b = line; nclose = gsub(/\}/, "X", b);
  depth += nopen - nclose;
  if (depth <= 0) { mode = ""; cur = ""; depth = 0; }
}
END { exit (fails > 0 ? 1 : 0); }
'

check_graph_purity() {
  local f out rc bad_all
  bad_all=""
  for f in include/omni/contributors.hpp include/omni/system.hpp \
           include/omni/environment.hpp include/omni/force_torque.hpp \
           include/omni/backsub_t.hpp include/omni/linear3.hpp; do
    if [ ! -f "$f" ]; then
      fail "graph-purity: missing file $f"
      return
    fi
    out="$(awk "$AWK_PURITY" "$f")"
    rc=$?
    if [ "$rc" -ne 0 ] || [ -n "$out" ]; then
      bad_all="${bad_all}${out}"$'\n'
    fi
  done
  if [ -z "$(printf '%s' "$bad_all" | tr -d '[:space:]')" ]; then
    pass "graph-purity: no min/max/abs/tanh/sign in symbolic paths"
  else
    printf '%s' "$bad_all"
    fail "graph-purity: clamp/branch tokens inside updateContributions/computeForceTorque/state_derivatives/field"
  fi
}

# ------------------------------------------------------------------ (b) -----
# SCOPE NOTE (L12, 2026-10-14, comment-only): audit_bundle/probes/
# closed_loop_fuel.py + swap_multi.py are UNSCANNED by decision -- outside
# the grep scope below (omniheyoka tests scratch*). Grep-verified clean for
# the (b) patterns on 2026-10-14; if they ever enter scope, re-grep first.
check_hardcoded_counts() {
  local hits
  hits="$(grep -rnE \
    -e 'nPars\(\)[[:space:]]*[=!]=[[:space:]]*19' \
    -e '(^|[^A-Za-z0-9_.])n_pars[[:space:]]*[=!]=[[:space:]]*19' \
    -e 'RW_N_PARS[[:space:]]*=[[:space:]]*19' \
    -e 'len\(pars\)[[:space:]]*-[[:space:]]*16' \
    omniheyoka tests scratch scratch_ws1 scratch_ws2 2>/dev/null \
    | grep -vE ':[0-9]+:[[:space:]]*#' || true)"
  if [ -z "$hits" ]; then
    pass "layout-counts: no hard-coded nPars==19 / RW_N_PARS=19 / len(pars)-16"
  else
    printf '%s\n' "$hits"
    fail "layout-counts: hard-coded parameter counts (use par_names()/layout helpers)"
  fi
}

# ------------------------------------------------------------------ (c) -----
check_state_indices() {
  local hits
  hits="$(grep -rnE -e '(^|[^A-Za-z0-9_])(st|y)\[[[:space:]]*[0-9]' src/ 2>/dev/null \
    | grep -vE ':[0-9]+:[[:space:]]*//' || true)"
  if [ -z "$hits" ]; then
    pass "layout-indices: no hard-coded st[N]/y[N] state subscripts in src/"
  else
    printf '%s\n' "$hits"
    fail "layout-indices: hard-coded state indices in src/ (use state_offset-based members)"
  fi
}

# ------------------------------------------------------------------ (d) -----
check_ctypes_state_indices() {
  local hits
  hits="$(grep -rnE -e 'state\[[[:space:]]*[0-9]' omniheyoka/ctypes_driver.py 2>/dev/null \
    | grep -v 'LAYOUT-OK:' || true)"
  if [ -z "$hits" ]; then
    pass "layout-ctypes: no unjustified .state[N] subscripts in ctypes_driver.py"
  else
    printf '%s\n' "$hits"
    fail "layout-ctypes: numeric .state[N] in ctypes_driver.py without LAYOUT-OK: justification"
  fi
}

# ------------------------------------------------------------------ (e) -----
# SCOPE NOTE (L12, 2026-10-14, comment-only): same exclusion as (b) --
# audit_bundle/probes/closed_loop_fuel.py + swap_multi.py unscanned by
# decision (outside grep scope); grep-verified clean for bare-19 widths.
check_bare19_widths() {
  local hits
  hits="$(grep -rnE -e 'names,[[:space:]]*19[[:space:]]*\)' \
    omniheyoka tests scratch scratch_ws1 scratch_ws2 BUILD_STATUS.md 2>/dev/null \
    | grep -v -e __pycache__ -e 'CI-EXEMPT(bare-19)' || true)"
  if [ -z "$hits" ]; then
    pass "layout-bare19: no unjustified bare-19 par-width args"
  else
    printf '%s\n' "$hits"
    fail "layout-bare19: bare '19' par width (use par_names()-derived counts or tag CI-EXEMPT(bare-19) with justification)"
  fi
}

check_graph_purity
check_hardcoded_counts
check_state_indices
check_ctypes_state_indices
check_bare19_widths

echo "ci_envelope_checks: PASS=${PASS} FAIL=${FAIL}"
if [ "$FAIL" -ne 0 ]; then echo "RESULT: FAIL"; exit 1; fi
echo "RESULT: PASS"
