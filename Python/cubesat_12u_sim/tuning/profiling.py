"""Hierarchical timing for the control/tuning pipeline.

Why this module exists
----------------------
Before it, the only timing that reached disk was two scalars in
``res_optimization.json`` (``wall_seconds`` and ``stage_wall_seconds``). Every
detailed number in the pipeline write-up (README §10: the 530x / 18x / 8x backend
speedups, the ~7 s cold versus 0.17 s warm JIT, the parity figures) was measured
by hand and pasted into prose, so none of it could be regenerated or regressed
on. This module makes timing a first-class, hierarchical output of the pipeline.

Three ideas, in order of importance:

1. **Hierarchy, not a flat list.** Sections nest (``stage.nmpc`` >
   ``horizon.H8`` > ``restart.3`` > ``generation.17`` > ``eval`` >
   ``sim.accel``). A flat stopwatch list cannot answer "how much of the sweep is
   the H=12 horizon?", which is the question a structural sweep exists to ask.

2. **Averages, not per-call numbers.** Nothing here measures a single RK4 step
   or a single SLSQP iteration and reports that as a result. Timed regions are
   deliberately coarse enough to be well above timer resolution, and the
   interesting quantities are ``total / calls`` accumulated over an entire
   sweep. A per-step number that lands under a millisecond is dominated by clock
   overhead and by whatever else the machine was doing; its mean over a sweep is
   the measurement. See :func:`timing.summarize` for the repeated-run
   statistics that back this up.

3. **Cross-process aggregation.** Candidate evaluation happens in a
   ``ProcessPoolExecutor``, so a worker's timings die with the worker unless they
   are shipped back. ``isolated()`` snapshots a subtree, and ``merge_dict()``
   folds a serialised subtree from another process into the parent — which means
   the parent can report *CPU* time (sum over workers, larger than wall time when
   parallel) next to wall time, and the two disagreeing is itself the signal that
   the pool is doing its job.

Overhead: one ``perf_counter`` pair per timed region. For a 60 s-step RK4
integration step (tens of microseconds) that is well under a percent; for the
regions actually instrumented (a solve, an evaluation, a generation) it is
unmeasurable. ``set_enabled(False)`` makes every entry point a single boolean
test.

Used from both layers: ``nmpc.py``/``hierarchical.py`` (the control path) and
``optimization/*`` (the tuning harness). It lives at package level rather than
under ``optimization/`` so that ``nmpc.py`` can use it without importing its own
consumer — the dependency would otherwise be circular.
"""
from __future__ import annotations

import time
from contextlib import contextmanager

__all__ = ["Node", "Profiler", "profiler", "set_enabled", "is_enabled",
           "reset", "elapsed", "format_report", "flatten", "summarize",
           "CLOCK"]

# perf_counter, not time.time: monotonic, and the highest resolution the
# platform offers. Wall-clock jumps (NTP steps, DST) would otherwise be able to
# produce negative section durations.
CLOCK = time.perf_counter


class Node:
    """One timed region in the tree.

    ``total`` is *inclusive* — it spans child regions too, because a parent
    section's clock keeps running while its children are timed. ``exclusive()``
    subtracts them, which is what you want when asking "how much did this level
    cost that was not already attributed to a subsection".
    """

    __slots__ = ("name", "total", "calls", "counts", "children")

    def __init__(self, name: str = "") -> None:
        self.name = name
        self.total = 0.0
        self.calls = 0
        self.counts: dict[str, float] = {}
        self.children: dict[str, "Node"] = {}

    def child(self, name: str) -> "Node":
        node = self.children.get(name)
        if node is None:
            node = self.children[name] = Node(name)
        return node

    def exclusive(self) -> float:
        return self.total - sum(c.total for c in self.children.values())

    def walk(self, depth: int = 0):
        yield depth, self
        for name in sorted(self.children):
            yield from self.children[name].walk(depth + 1)

    def to_dict(self) -> dict:
        d: dict = {"total_s": self.total, "calls": self.calls,
                   "exclusive_s": self.exclusive()}
        if self.calls:
            d["mean_ms"] = self.total / self.calls * 1e3
        if self.counts:
            d["counts"] = {k: float(v) for k, v in sorted(self.counts.items())}
        if self.children:
            d["children"] = {k: v.to_dict()
                             for k, v in sorted(self.children.items())}
        return d

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return (f"Node({self.name!r}, total={self.total:.4f}s, "
                f"calls={self.calls}, children={sorted(self.children)})")


def _merge_node(dst: Node, src: Node) -> None:
    """Fold ``src`` into ``dst`` (additive: totals, calls, counts, children)."""
    dst.total += src.total
    dst.calls += src.calls
    for k, v in src.counts.items():
        dst.counts[k] = dst.counts.get(k, 0.0) + v
    for name, child in src.children.items():
        _merge_node(dst.child(name), child)


def _merge_dict(dst: Node, d: dict) -> None:
    """Fold a serialised subtree (``Node.to_dict``) into ``dst``.

    This is the cross-process path: a worker serialises its subtree, the parent
    deserialises it and folds it in. ``mean_ms``/``exclusive_s`` are derived
    quantities and are deliberately ignored here — they are recomputed from the
    merged totals by :meth:`Node.to_dict`, so folding them in would double-count.
    """
    dst.total += float(d.get("total_s", 0.0))
    dst.calls += int(d.get("calls", 0))
    for k, v in (d.get("counts") or {}).items():
        dst.counts[k] = dst.counts.get(k, 0.0) + float(v)
    for name, cd in (d.get("children") or {}).items():
        _merge_dict(dst.child(name), cd)


class Profiler:
    """A nestable section timer.

    Sections are keyed by name within their parent, so re-entering ``"eval"``
    accumulates into one node with a call count rather than creating a new node
    per call. That is what makes ``total / calls`` meaningful at the end.
    """

    def __init__(self, enabled: bool = True, root: str = "pipeline") -> None:
        self.enabled = bool(enabled)
        self.root = Node(root)
        self._stack: list[Node] = [self.root]
        # parallel stack of (node, start_time) for begin()/end()
        self._marks: list[tuple[Node, float]] = []

    # -- internals ---------------------------------------------------------

    @property
    def _current(self) -> Node:
        return self._stack[-1]

    # -- recording ---------------------------------------------------------

    @contextmanager
    def section(self, name: str):
        """Time a block as a child of the enclosing section.

        Yields the :class:`Node`, so a caller may attach counts to it. Never
        suppresses exceptions: the elapsed time is still recorded on the way out,
        because a section that died halfway did consume time.
        """
        if not self.enabled:
            yield None
            return
        node = self._current.child(name)
        t0 = CLOCK()
        self._stack.append(node)
        try:
            yield node
        finally:
            self._stack.pop()
            node.total += CLOCK() - t0
            node.calls += 1

    def begin(self, name: str) -> None:
        """Open a section manually (pair with :meth:`end`).

        Needed where the region is a loop body that must be timed with a clock
        pair anyway. ``section()`` is a generator-based context manager, and
        instantiating one costs about a microsecond — enough to distort a
        region that costs ten. ``begin``/``end`` are plain attribute updates,
        so a hot loop can attribute its work to a real node without the
        generator cost, and any section opened *inside* the loop nests under it
        correctly rather than appearing as a sibling.
        """
        if not self.enabled:
            return
        node = self._current.child(name)
        self._stack.append(node)
        self._marks.append((node, CLOCK()))

    def end(self) -> None:
        """Close the innermost :meth:`begin` and record its elapsed time."""
        if not self.enabled:
            return
        node, t0 = self._marks.pop()
        self._stack.pop()
        node.total += CLOCK() - t0
        node.calls += 1

    def add(self, name: str, dt: float) -> None:
        """Add one already-measured duration as a single call.

        Used where the measurement is taken inline but no nesting is wanted —
        for example reporting a loop's two halves side by side. Prefer
        :meth:`begin`/:meth:`end` when the loop also opens its own sections,
        so those nest inside the region instead of overlapping it.
        """
        if not self.enabled:
            return
        node = self._current.child(name)
        node.total += dt
        node.calls += 1

    def add_bulk(self, name: str, dt: float, n: int = 1) -> None:
        """Add ``dt`` seconds accumulated over ``n`` calls, in one node update.

        Lets a hot loop accumulate into locals and report once per iteration
        instead of dict-walking per step.
        """
        if not self.enabled or n <= 0:
            return
        node = self._current.child(name)
        node.total += dt
        node.calls += n

    def count(self, name: str, n: int = 1) -> None:
        """Record a dimensionless count (cache hits, fallbacks, steps)."""
        if not self.enabled:
            return
        c = self._current.counts
        c[name] = c.get(name, 0.0) + n

    # -- scoping -----------------------------------------------------------

    @contextmanager
    def isolated(self, name: str = "run", merge: bool = True):
        """Run a block against a fresh subtree, optionally folding it back in.

        With ``merge=True`` (the default) this is how an in-process worker batch
        appears in the parent tree as one ``name`` subtree. With ``merge=False``
        the subtree is yielded and left unmerged, which is what a repeated-run
        benchmark wants: each repetition keeps its own numbers so they can be
        summarised into a mean and a spread instead of one undifferentiated sum.

        Safe to nest and safe against exceptions.
        """
        if not self.enabled:
            yield Node(name)
            return
        saved_root, saved_stack, saved_marks = self.root, self._stack, self._marks
        sub = Node(name)
        self.root = sub
        self._stack = [sub]
        self._marks = []
        try:
            yield sub
        finally:
            self.root, self._stack, self._marks = saved_root, saved_stack, saved_marks
            if merge:
                _merge_node(self._current.child(name), sub)

    # -- output ------------------------------------------------------------

    def reset(self) -> None:
        self.root = Node(self.root.name)
        self._stack = [self.root]
        self._marks = []

    def total_s(self) -> float:
        """Sum of the top-level sections (the tree's inclusive wall cost)."""
        return sum(c.total for c in self.root.children.values())

    def to_dict(self) -> dict:
        return {name: c.to_dict()
                for name, c in sorted(self.root.children.items())}

    def merge_dict(self, name: str, d: dict) -> None:
        """Fold one serialised *node* (``Node.to_dict``) into child ``name``."""
        if not self.enabled or not d:
            return
        _merge_dict(self._current.child(name), d)

    def node(self, name: str) -> "Node | None":
        """The node for a section reached by a dotted path from the root.

        ``node("stage_nmpc.horizons.horizonH8")``. Sections nest, so a section
        worth lifting out on its own is rarely a direct child of the root;
        taking a path (rather than a bare name) also disambiguates sections that
        share a name at different depths.

        Returns ``None`` if any component is missing, so callers do not have to
        distinguish "not recorded" from "recorded as zero".
        """
        cur: Node = self.root
        for part in name.split("."):
            cur = cur.children.get(part)
            if cur is None:
                return None
        return cur

    def subtree(self, name: str) -> dict:
        """Serialised form of the section at a dotted path, for results."""
        n = self.node(name)
        return n.to_dict() if n is not None else {}

    def merge_all(self, snapshot: dict) -> None:
        """Fold a whole serialised *tree* (as returned by ``to_dict``) in.

        This is the cross-process entry point: a worker hands back
        ``profiler().to_dict()``, whose top level is a set of named nodes, and
        each of those is merged into the identically-named node here. Passing
        that whole mapping to :meth:`merge_dict` would silently drop everything,
        because it expects a single node rather than a set of them.
        """
        if not self.enabled or not snapshot:
            return
        for name, node in snapshot.items():
            if isinstance(node, dict):
                self.merge_dict(name, node)

    def report(self, title: str = "pipeline") -> str:
        return format_report(title, self.root, self.total_s())


# ---------------------------------------------------------------------------
# module-level profiler (the one the pipeline actually records into)
# ---------------------------------------------------------------------------

_GLOBAL = Profiler()


def profiler() -> Profiler:
    return _GLOBAL


def set_enabled(value: bool) -> None:
    _GLOBAL.enabled = bool(value)


def is_enabled() -> bool:
    return _GLOBAL.enabled


def reset() -> None:
    _GLOBAL.reset()


@contextmanager
def elapsed(name: str):
    """Convenience wrapper: ``with timing.elapsed("stage.pid"): ...``."""
    with _GLOBAL.section(name):
        yield


def _format_s(v: float) -> str:
    if v >= 100.0:
        return f"{v:8.2f}s"
    if v >= 1.0:
        return f"{v:8.4f}s"
    return f"{v * 1e3:8.3f}ms"


def format_report(title: str, root: Node, total: float | None = None) -> str:
    """Render the tree as an indented table.

    ``exclusive`` is shown because a parent's inclusive time is the sum of itself
    and its children, and reporting only inclusive numbers would make every level
    look equally expensive.
    """
    if total is None:
        total = sum(c.total for c in root.children.values())
    lines = [f"=== timing: {title} ===",
             f"{'total':<44}{'total':>11}{'calls':>9}{'mean':>11}"
             f"{'excl':>11}{'share':>8}"]
    if total <= 0.0:
        lines.append("  (no timed sections)")
        return "\n".join(lines)
    for depth, node in root.walk():
        if node is root:
            continue
        share = 100.0 * node.total / total
        mean = node.total / node.calls if node.calls else 0.0
        lines.append(f"  {'  ' * depth}{node.name:<{42 - 2 * depth}}"
                     f"{_format_s(node.total):>11}{node.calls:>9}"
                     f"{_format_s(mean):>11}"
                     f"{_format_s(node.exclusive()):>11}{share:>7.1f}%")
        for cname, cval in sorted(node.counts.items()):
            lines.append(f"  {'  ' * (depth + 1)}#{cname} = {cval:g}")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# repeated-run statistics
# ---------------------------------------------------------------------------

def flatten(node: Node, prefix: str = "") -> dict[str, float]:
    """``{dotted.path: inclusive seconds}`` for every node in the tree."""
    out: dict[str, float] = {}
    for name, child in node.children.items():
        path = f"{prefix}{name}"
        out[path] = child.total
        out.update(flatten(child, f"{path}."))
    return out


def summarize(snapshots: list[dict[str, float]]) -> dict[str, dict]:
    """Mean / std / min / max per section path across repeated runs.

    This is the answer to "what does one of these cost". A single sweep gives
    ``total / calls``; repeating the *whole* fast simulation N times and
    summarising gives a spread, which is the only way to tell a real per-solve
    cost apart from scheduler noise. Sections missing from a repetition count as
    zero, so a path that only appears on some repetitions does not silently shift
    the mean.

    ``std_s`` uses the sample (n-1) denominator.
    """
    if not snapshots:
        return {}
    paths = sorted({p for s in snapshots for p in s})
    out: dict[str, dict] = {}
    for p in paths:
        vals = [float(s.get(p, 0.0)) for s in snapshots]
        n = len(vals)
        mean = sum(vals) / n
        if n > 1:
            var = sum((v - mean) ** 2 for v in vals) / (n - 1)
        else:
            var = 0.0
        out[p] = {"n": n, "mean_s": mean, "std_s": var ** 0.5,
                  "min_s": min(vals), "max_s": max(vals)}
    return out


def format_summary(summary: dict[str, dict], total_s: float | None = None) -> str:
    """Table form of :func:`summarize`, optionally with a share column."""
    if not summary:
        return "=== timing: (no repetitions) ==="
    lines = [f"{'section':<44}{'mean':>11}{'std':>11}{'min':>11}"
             f"{'max':>11}{'n':>5}"]
    if total_s is None:
        total_s = max((v["mean_s"] for k, v in summary.items()
                       if "." not in k), default=0.0)
    for p, v in summary.items():
        share = f"{100.0 * v['mean_s'] / total_s:>7.1f}%" if total_s > 0 else "      -"
        lines.append(f"  {p:<42}{_format_s(v['mean_s']):>11}"
                     f"{_format_s(v['std_s']):>11}{_format_s(v['min_s']):>11}"
                     f"{_format_s(v['max_s']):>11}{v['n']:>5}  {share}")
    return "\n".join(lines)