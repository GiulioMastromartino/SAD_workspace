#!/usr/bin/env bash
# scripts/ci_envelope_checks.sh -- WP0/WP7 envelope CI grep gates.
#
# Three checks (PLAN_ENVELOPE.md Ground rules 1/2/7, Rev 1):
#   (a) graph purity (A3): no min(/max(/std::min/std::max/std::abs/abs(/
#       tanh(/sign inside the symbolic graph paths of
#       include/omni/contributors.hpp and include/omni/system.hpp, i.e.
#       inside updateContributions / computeForceTorque / state_derivatives /
#       field bodies. Allowed: `if constexpr (...is_expression_v<S>)`
#       idle-skips, and the step-time double-side converters fillInputs /
#       quantize / onTime / thruster_on_times (never symbolic).
#   (b) layout hygiene (A2): no hard-coded parameter counts
#       (`nPars() == 19`-style, `RW_N_PARS = 19`, `len(pars) - 16`) in
#       omniheyoka/*.py, tests/, scratch*. Full-line `#` comments are exempt
#       (historical notes); code must use par_names()/layout helpers.
#   (c) layout hygiene (A1/A2): no hard-coded numeric state subscripts
#       (`st[12 + i]`-style) in src/ -- use state_offset-based members or
#       layout helpers. Full-line `//` comments are exempt.
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
  for f in include/omni/contributors.hpp include/omni/system.hpp; do
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

check_graph_purity
check_hardcoded_counts
check_state_indices

echo "ci_envelope_checks: PASS=${PASS} FAIL=${FAIL}"
if [ "$FAIL" -ne 0 ]; then echo "RESULT: FAIL"; exit 1; fi
echo "RESULT: PASS"
