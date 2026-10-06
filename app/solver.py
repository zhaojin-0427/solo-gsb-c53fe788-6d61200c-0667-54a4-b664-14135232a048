"""CP-SAT model for the multi-stop truck loading problem.

Coordinate system
-----------------
- x: longitudinal axis, x = 0 is the rear (unloading end) of the truck.
- y: lateral axis, z: vertical axis. All coordinates are integers.
- Boxes are axis-aligned and may only be rotated horizontally (length/width swap).

Modelled rules
--------------
1. Boxes lie inside the cargo compartment and do not overlap.
2. Full support: a box bottom is either on the floor (z = 0) or its whole
   footprint is covered by tops of boxes whose top is exactly at that height.
3. Unloading at a station pulls boxes straight towards the rear (-x). While a
   box slides out it must not collide with any other remaining box, i.e. no
   remaining box may sit in its swept corridor.
4. Before any unloading at a station and after every single pull, the
   remaining boxes must still be fully supported and (when non-empty) the
   doubled longitudinal COG must lie inside that station's interval.

Rules 3 and 4 are enforced jointly with the placement, so the returned
packing and the unloading sequence are consistent by construction.
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass

from ortools.sat.python import cp_model

DEFAULT_TIME_LIMIT = float(os.environ.get("SOLVER_TIME_LIMIT", "20"))
DEFAULT_WORKERS = int(os.environ.get("SOLVER_WORKERS", "8"))
# After the first feasible plan is found, keep improving it for at most this
# many seconds, then return the best plan seen. Keeps latency predictable.
DEFAULT_GRACE = float(os.environ.get("SOLVER_GRACE_SECONDS", "2"))


@dataclass(frozen=True)
class BoxInput:
    id: str
    length: int
    width: int
    height: int
    weight: int
    dest: int  # index into the ordered station list


@dataclass(frozen=True)
class StationInput:
    id: str
    cog2_min: int
    cog2_max: int


@dataclass
class SolveOutput:
    status: str  # "feasible" | "infeasible" | "unknown"
    placements: list[dict] | None
    unload_plan: list[dict] | None
    cog_checks: list[dict]
    message: str | None = None


def _fits_at_all(length: int, width: int, height: int, L: int, W: int, H: int) -> bool:
    return height <= H and ((length <= L and width <= W) or (width <= L and length <= W))


def solve_loading(
    truck_length: int,
    truck_width: int,
    truck_height: int,
    stations: list[StationInput],
    boxes: list[BoxInput],
    time_limit: float | None = None,
    workers: int | None = None,
) -> SolveOutput:
    """Solve the joint placement + unloading-sequence problem."""
    L, W, H = truck_length, truck_width, truck_height
    n = len(boxes)
    limit = DEFAULT_TIME_LIMIT if time_limit is None else float(time_limit)
    num_workers = DEFAULT_WORKERS if workers is None else int(workers)

    # Quick impossibility check: a box that can never fit inside the truck.
    for b in boxes:
        if not _fits_at_all(b.length, b.width, b.height, L, W, H):
            return SolveOutput(
                status="infeasible",
                placements=None,
                unload_plan=None,
                cog_checks=[],
                message=f"box '{b.id}' cannot fit inside the truck in any horizontal orientation",
            )

    m = cp_model.CpModel()
    zero = m.new_constant(0)

    # ------------------------------------------------------------------
    # Placement variables
    # ------------------------------------------------------------------
    rot, dx, dy, x, y, z = [], [], [], [], [], []
    for i, b in enumerate(boxes):
        o = m.new_bool_var(f"rot_{i}")
        dxi = m.new_int_var(min(b.length, b.width), max(b.length, b.width), f"dx_{i}")
        dyi = m.new_int_var(min(b.length, b.width), max(b.length, b.width), f"dy_{i}")
        m.add(dxi == b.length + (b.width - b.length) * o)
        m.add(dyi == b.width + (b.length - b.width) * o)
        xi = m.new_int_var(0, L - min(b.length, b.width), f"x_{i}")
        yi = m.new_int_var(0, W - min(b.length, b.width), f"y_{i}")
        zi = m.new_int_var(0, H - b.height, f"z_{i}")
        m.add(xi + dxi <= L)
        m.add(yi + dyi <= W)
        rot.append(o)
        dx.append(dxi)
        dy.append(dyi)
        x.append(xi)
        y.append(yi)
        z.append(zi)

    # ------------------------------------------------------------------
    # Non-overlap (pairwise disjunctive in 3D)
    # ------------------------------------------------------------------
    for i in range(n):
        for j in range(i + 1, n):
            c = [m.new_bool_var(f"nov_{i}_{j}_{k}") for k in range(6)]
            m.add(x[i] + dx[i] <= x[j]).only_enforce_if(c[0])
            m.add(x[j] + dx[j] <= x[i]).only_enforce_if(c[1])
            m.add(y[i] + dy[i] <= y[j]).only_enforce_if(c[2])
            m.add(y[j] + dy[j] <= y[i]).only_enforce_if(c[3])
            m.add(z[i] + boxes[i].height <= z[j]).only_enforce_if(c[4])
            m.add(z[j] + boxes[j].height <= z[i]).only_enforce_if(c[5])
            m.add_bool_or(c)

    # ------------------------------------------------------------------
    # Per-station pull-sequence variables (needed by precedence constraints)
    # ------------------------------------------------------------------
    by_station: list[list[int]] = [[] for _ in stations]
    for i, b in enumerate(boxes):
        by_station[b.dest].append(i)

    pos: dict[int, cp_model.IntVar] = {}
    gt: dict[tuple[int, int], cp_model.IntVar] = {}
    for k, members in enumerate(by_station):
        if not members:
            continue
        nk = len(members)
        for i in members:
            pos[i] = m.new_int_var(0, nk - 1, f"pos_{i}")
        m.add_all_different([pos[i] for i in members])
        for i in members:
            for j in members:
                if i == j:
                    continue
                g = m.new_bool_var(f"gt_{j}_{i}")  # true iff j is pulled after i
                m.add(pos[j] >= pos[i] + 1).only_enforce_if(g)
                m.add(pos[j] <= pos[i]).only_enforce_if(g.negated())
                gt[(j, i)] = g

    # ------------------------------------------------------------------
    # Pairwise geometric relations: overlaps, support, blocking
    # ------------------------------------------------------------------
    def overlap1d(lo_a, hi_a, lo_b, hi_b, coord_cap: int, ov_cap: int, tag: str):
        lo = m.new_int_var(0, coord_cap, f"lo_{tag}")
        m.add_max_equality(lo, [lo_a, lo_b])
        hi = m.new_int_var(0, coord_cap, f"hi_{tag}")
        m.add_min_equality(hi, [hi_a, hi_b])
        ov = m.new_int_var(0, ov_cap, f"ov_{tag}")
        m.add_max_equality(ov, [hi - lo, zero])
        return ov

    def positive(ov, tag: str):
        b = m.new_bool_var(f"pos_{tag}")
        m.add(ov >= 1).only_enforce_if(b)
        m.add(ov <= 0).only_enforce_if(b.negated())
        return b

    support_area: list[list[cp_model.LinearExpr]] = [[] for _ in range(n)]

    for i in range(n):
        for j in range(i + 1, n):
            tag = f"{i}_{j}"
            bi, bj = boxes[i], boxes[j]
            xov = overlap1d(
                x[i], x[i] + dx[i], x[j], x[j] + dx[j], L,
                min(max(bi.length, bi.width), max(bj.length, bj.width)), f"x_{tag}",
            )
            yov = overlap1d(
                y[i], y[i] + dy[i], y[j], y[j] + dy[j], W,
                min(max(bi.length, bi.width), max(bj.length, bj.width)), f"y_{tag}",
            )
            zov = overlap1d(
                z[i], z[i] + bi.height, z[j], z[j] + bj.height, H,
                min(bi.height, bj.height), f"z_{tag}",
            )
            bx = positive(xov, f"x_{tag}")
            by = positive(yov, f"y_{tag}")
            bz = positive(zov, f"z_{tag}")

            area_cap = min(
                max(bi.length, bi.width) ** 2, max(bj.length, bj.width) ** 2
            )
            area = m.new_int_var(0, area_cap, f"area_{tag}")
            m.add_multiplication_equality(area, [xov, yov])

            # --- support, both directions (supporter below, supported above) ---
            for (up, down) in ((i, j), (j, i)):
                aligned = m.new_bool_var(f"al_{down}_{up}")
                m.add(z[down] + boxes[down].height == z[up]).only_enforce_if(aligned)
                m.add(z[down] + boxes[down].height != z[up]).only_enforce_if(aligned.negated())

                d_up, d_down = boxes[up].dest, boxes[down].dest
                if d_down < d_up:
                    # The supporter would leave earlier: the upper box would
                    # be left floating at the supporter's station. Forbidden.
                    m.add_bool_or([aligned.negated(), bx.negated(), by.negated()])
                    continue

                active = m.new_bool_var(f"sup_{down}_{up}")
                m.add_bool_and([aligned, bx, by]).only_enforce_if(active)
                m.add_bool_or([aligned.negated(), bx.negated(), by.negated()]).only_enforce_if(
                    active.negated()
                )
                gated = m.new_int_var(0, area_cap, f"gsup_{down}_{up}")
                m.add(gated == area).only_enforce_if(active)
                m.add(gated == 0).only_enforce_if(active.negated())
                support_area[up].append(gated)

                if d_down == d_up:
                    # Same station: the upper box must be pulled out first.
                    m.add(pos[up] < pos[down]).only_enforce_if(active)

            # --- rear-pull blocking, both directions ---
            for (front, rear) in ((i, j), (j, i)):
                # `rear` blocks `front` iff their y/z intervals overlap and
                # `rear` lies (weakly) behind `front` along x.
                behind = m.new_bool_var(f"beh_{rear}_{front}")
                m.add(x[rear] + dx[rear] <= x[front]).only_enforce_if(behind)
                m.add(x[rear] + dx[rear] >= x[front] + 1).only_enforce_if(behind.negated())

                d_front, d_rear = boxes[front].dest, boxes[rear].dest
                if d_rear > d_front:
                    # A box bound for a later stop may never sit in the pull
                    # corridor of a box unloaded earlier.
                    m.add_bool_or([by.negated(), bz.negated(), behind.negated()])
                elif d_rear == d_front:
                    # Same station: the rearmost box must be pulled first.
                    blocks = m.new_bool_var(f"blk_{rear}_{front}")
                    m.add_bool_and([by, bz, behind]).only_enforce_if(blocks)
                    m.add_bool_or([by.negated(), bz.negated(), behind.negated()]).only_enforce_if(
                        blocks.negated()
                    )
                    m.add(pos[rear] < pos[front]).only_enforce_if(blocks)
                # d_rear < d_front: the blocker is gone before `front`'s stop.

    # ------------------------------------------------------------------
    # Full support of every box bottom
    # ------------------------------------------------------------------
    for i, b in enumerate(boxes):
        on_floor = m.new_bool_var(f"floor_{i}")
        m.add(z[i] == 0).only_enforce_if(on_floor)
        m.add(z[i] >= 1).only_enforce_if(on_floor.negated())
        footprint = m.new_int_var(1, max(b.length, b.width) ** 2, f"fp_{i}")
        m.add_multiplication_equality(footprint, [dx[i], dy[i]])
        m.add(sum(support_area[i]) == footprint).only_enforce_if(on_floor.negated())

    # ------------------------------------------------------------------
    # COG constraints (doubled coordinates keep everything integral)
    # ------------------------------------------------------------------
    # s_i = w_i * (2*x_i + dx_i); doubled COG of a set S is sum(s_i)/sum(w_i).
    s = [b.weight * (2 * x[i] + dx[i]) for i, b in enumerate(boxes)]

    # Arrival (== before unloading) at each station.
    for k, st in enumerate(stations):
        remaining = [i for i, b in enumerate(boxes) if b.dest >= k]
        if not remaining:
            continue
        s_sum = sum(s[i] for i in remaining)
        w_sum = sum(boxes[i].weight for i in remaining)
        m.add(s_sum >= st.cog2_min * w_sum)
        m.add(s_sum <= st.cog2_max * w_sum)

    # After every single pull at station k the remaining load must satisfy
    # station k's interval. Remaining = boxes for later stations plus the
    # same-station boxes not pulled yet.
    for k, members in enumerate(by_station):
        if not members:
            continue
        st = stations[k]
        later = [i for i, b in enumerate(boxes) if b.dest > k]
        s_later = sum(s[i] for i in later)
        w_later = sum(boxes[i].weight for i in later)
        for i in members:
            s_terms = [s_later]
            w_terms = [w_later]
            for j in members:
                if j == i:
                    continue
                g = gt[(j, i)]
                cap = 2 * boxes[j].weight * L  # upper bound of s_j
                p = m.new_int_var(0, cap, f"cog_{j}_{i}")
                m.add(p <= s[j])
                m.add(p <= cap * g)
                m.add(p >= s[j] - cap * (1 - g))
                s_terms.append(p)
                w_terms.append(boxes[j].weight * g)
            # If the truck is empty both sides are 0 and the constraint holds
            # trivially, matching the "only when non-empty" rule.
            m.add(sum(s_terms) >= st.cog2_min * sum(w_terms))
            m.add(sum(s_terms) <= st.cog2_max * sum(w_terms))

    # ------------------------------------------------------------------
    # Mild objective: prefer low stacks, then compact towards the rear.
    # ------------------------------------------------------------------
    m.minimize(sum(z[i] * (L + W) + x[i] + y[i] for i in range(n)))

    # Guide the search towards a compact floor-first packing: this is almost
    # always the shape of a good solution and speeds up finding the first one.
    m.add_decision_strategy(
        [z[i] for i in range(n)],
        cp_model.CHOOSE_FIRST,
        cp_model.SELECT_MIN_VALUE,
    )
    m.add_decision_strategy(
        [v for i in range(n) for v in (x[i], y[i])],
        cp_model.CHOOSE_FIRST,
        cp_model.SELECT_MIN_VALUE,
    )
    for i in range(n):
        m.add_hint(z[i], 0)
        m.add_hint(rot[i], 0)

    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = limit
    try:
        solver.parameters.num_search_workers = num_workers
    except AttributeError:  # older OR-Tools
        solver.parameters.num_workers = num_workers

    class _StopAfterGrace(cp_model.CpSolverSolutionCallback):
        def __init__(self) -> None:
            super().__init__()
            self._first_solution_at: float | None = None

        def on_solution_callback(self) -> None:
            if self._first_solution_at is None:
                self._first_solution_at = time.monotonic()
            elif time.monotonic() - self._first_solution_at >= DEFAULT_GRACE:
                self.stop_search()

    status = solver.solve(m, _StopAfterGrace())

    if status in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        return _build_output(solver, stations, boxes, by_station, pos, rot, dx, dy, x, y, z)
    if status == cp_model.INFEASIBLE:
        return SolveOutput(
            status="infeasible",
            placements=None,
            unload_plan=None,
            cog_checks=[],
            message="no loading plan satisfies all support, pull-corridor and COG constraints",
        )
    return SolveOutput(
        status="unknown",
        placements=None,
        unload_plan=None,
        cog_checks=[],
        message=f"solver could not decide within {limit:g}s (try a larger time limit)",
    )


def _build_output(solver, stations, boxes, by_station, pos, rot, dx, dy, x, y, z) -> SolveOutput:
    v = solver.value
    placements = []
    for i, b in enumerate(boxes):
        placements.append(
            {
                "box_id": b.id,
                "x": v(x[i]),
                "y": v(y[i]),
                "z": v(z[i]),
                "dx": v(dx[i]),
                "dy": v(dy[i]),
                "dz": b.height,
                "rotated": bool(v(rot[i])),
            }
        )

    unload_plan = []
    for k, members in enumerate(by_station):
        ordered = sorted(members, key=lambda i: v(pos[i]))
        unload_plan.append({"station": stations[k].id, "order": [boxes[i].id for i in ordered]})

    # COG trace: arrival at each station, then after every pull.
    cog_checks = []
    remaining = set(range(len(boxes)))

    def cog2_of(ids) -> float | None:
        w_sum = sum(boxes[i].weight for i in ids)
        if w_sum == 0:
            return None
        s_sum = sum(boxes[i].weight * (2 * v(x[i]) + v(dx[i])) for i in ids)
        return s_sum / w_sum

    for k, members in enumerate(by_station):
        st = stations[k]
        c = cog2_of(remaining)
        cog_checks.append(
            {
                "station": st.id,
                "stage": "arrival",
                "box_id": None,
                "cog2": None if c is None else round(c, 4),
                "cog2_min": st.cog2_min,
                "cog2_max": st.cog2_max,
                "ok": c is None or st.cog2_min <= c <= st.cog2_max,
            }
        )
        for i in sorted(members, key=lambda t: v(pos[t])):
            remaining.discard(i)
            c = cog2_of(remaining)
            cog_checks.append(
                {
                    "station": st.id,
                    "stage": "after_pull",
                    "box_id": boxes[i].id,
                    "cog2": None if c is None else round(c, 4),
                    "cog2_min": st.cog2_min,
                    "cog2_max": st.cog2_max,
                    "ok": c is None or st.cog2_min <= c <= st.cog2_max,
                }
            )

    return SolveOutput(
        status="feasible",
        placements=placements,
        unload_plan=unload_plan,
        cog_checks=cog_checks,
    )
