"""Parser for the infix RHS strings emitted by the omni C++ core.

WHY THIS EXISTS
---------------
heyoka's Python entry point ``hy.expression("x + y")`` does NOT evaluate an
arithmetic string -- it silently builds a single *variable* whose name is the
literal string ``"x + y"``.  (Verified: ``str(hy.expression("x + y")) ==
"x + y"`` and the object is a variable, so any system built from it fails with
"the variable 'x + y' appears in the right-hand side but not in the
left-hand side".)  heyoka exposes no public C++ expression-tree walker either,
so the C++ core cannot emit a structural form.

The grammar the C++ core emits is small and fully under our control -- the
entire EOM is built from ``omni::linear3`` / ``omni::mrp_t`` /
``omni::backsub_t``, which use only ``+ - * /`` plus ``sqrt``, printed
parenthesised with plain decimal literals and identifiers that are either a
state name (``rx``, ``omx``, ``W1``, ...) or a parameter reference (``p0`` ..
``p18``).  So a ~60-line recursive-descent parser is sufficient, and exact.

KEY GOTCHA: heyoka's C++ formatter prints ``sqrt(a)`` as ````a ** 0.5````.
That is heyoka's own canonical form (``str(hy.sqrt(x*x+1))`` gives
``(1 + x*x)**0.5``), and Python's expression supports ``**`` -- so the parser
must handle the power operator.
"""
from __future__ import annotations

import re
from typing import List

import heyoka as hy

__all__ = ["parse_rhs", "parse_system", "build_taylor_adaptive"]

_TOKEN_RE = re.compile(
    r"\s*(?:"
    r"(?P<num>\d+\.\d*(?:[eE][+-]?\d+)?|\d+[eE][+-]?\d+|\d+)"
    r"|(?P<pow>\*\*)"
    r"|(?P<id>[A-Za-z_][A-Za-z_0-9]*)"
    r"|(?P<op>[-+*/()])"
    r")"
)


def _tokenize(s: str):
    pos = 0
    out: List[tuple] = []
    while pos < len(s):
        if s[pos].isspace():
            pos += 1
            continue
        m = _TOKEN_RE.match(s, pos)
        if not m:
            raise ValueError(
                f"omni parser: bad token near {s[pos:pos+30]!r} in {s[:60]!r}")
        pos = m.end()
        if m.group("num") is not None:
            out.append(("num", float(m.group("num"))))
        elif m.group("pow") is not None:
            out.append(("pow", "**"))
        elif m.group("id") is not None:
            out.append(("id", m.group("id")))
        else:
            out.append(("op", m.group("op")))
    out.append(("end", None))
    return out


class _Parser:
    """expr   := term (('+'|'-') term)*
    term   := power (('*'|'/') power)*
    power  := unary ('**' unary)?          # right-associative
    unary  := '-' unary | primary
    primary:= NUMBER | IDENT | '(' expr ')'
    """

    def __init__(self, toks, var_resolver, n_pars):
        self.t = toks
        self.i = 0
        self.var = var_resolver
        self.n_pars = n_pars

    def _peek(self):
        return self.t[self.i]

    def _eat(self, kind=None, val=None):
        k, v = self.t[self.i]
        if kind and k != kind:
            raise ValueError(f"omni parser: expected {kind}, got {k}")
        if val is not None and v != val:
            raise ValueError(f"omni parser: expected {v!r}, got {v!r}")
        self.i += 1
        return v

    def parse(self):
        e = self.expr()
        if self._peek()[0] != "end":
            raise ValueError(f"omni parser: trailing input at {self.t[self.i]}")
        return e

    def expr(self):
        left = self.term()
        while self._peek() in (("op", "+"), ("op", "-")):
            op = self._eat("op")
            right = self.term()
            left = left + right if op == "+" else left - right
        return left

    def term(self):
        left = self.power()
        while self._peek() in (("op", "*"), ("op", "/")):
            op = self._eat("op")
            right = self.power()
            left = left * right if op == "*" else left / right
        return left

    def power(self):
        base = self.unary()
        if self._peek()[0] == "pow":
            self._eat("pow")
            return base ** self.power()  # right-associative
        return base

    def unary(self):
        if self._peek() == ("op", "-"):
            self._eat("op")
            return -self.unary()
        return self.primary()

    def primary(self):
        k, v = self._peek()
        if k == "op" and v == "(":
            self._eat("op")
            e = self.expr()
            self._eat("op", ")")
            return e
        if k == "num":
            self._eat("num")
            return hy.expression(v)
        if k == "id":
            self._eat("id")
            # heyoka writes parameter references as p<idx>; state names are
            # anything else.
            if v[0] == "p" and v[1:].isdigit():
                idx = int(v[1:])
                if idx >= self.n_pars:
                    raise ValueError(
                        f"omni parser: par {idx} out of range (n={self.n_pars})")
                return hy.par[idx]
            return self.var(v)
        raise ValueError(f"omni parser: unexpected token {v!r}")


def parse_rhs(s: str, variables, n_pars: int) -> hy.expression:
    """Parse one C++-printed RHS string into a heyoka expression.

    ``variables`` maps a state name to the heyoka expression (the variable).
    """

    def resolve(name: str) -> hy.expression:
        if name not in variables:
            raise ValueError(f"omni parser: unknown variable {name!r} "
                             f"(known: {sorted(variables)})")
        return variables[name]

    return _Parser(_tokenize(s), resolve, n_pars).parse()


def parse_system(rhs_strings: List[str], state_names: List[str], n_pars: int):
    """Return [(var_expr, rhs_expr), ...] ready for hy.taylor_adaptive.

    The variable objects used in the RHS are the SAME objects as the
    left-hand sides, so heyoka's "variable not on the LHS" check passes.
    """
    if len(rhs_strings) != len(state_names):
        raise ValueError(f"omni parser: {len(rhs_strings)} RHS strings for "
                         f"{len(state_names)} states")
    variables = {n: hy.expression(n) for n in state_names}
    out = []
    for name, rhs in zip(state_names, rhs_strings):
        out.append((variables[name], parse_rhs(rhs, variables, n_pars)))
    return out


def build_taylor_adaptive(rhs_strings: List[str], state_names: List[str],
                          state, pars, n_pars: int, **kw):
    """Build a real heyoka taylor_adaptive from C++-emitted RHS strings."""
    sysrows = parse_system(rhs_strings, state_names, n_pars)
    return hy.taylor_adaptive(sysrows, list(map(float, state)),
                             pars=list(map(float, pars)), **kw)
