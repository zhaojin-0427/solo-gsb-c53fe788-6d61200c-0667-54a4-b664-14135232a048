"""多站货车装载规划的 OR-Tools CP-SAT 模型。

坐标系（整数格点，车尾为 x=0，x 朝车头方向增大）：
  x：车长方向；y：车宽方向；z：竖直方向。
箱 i 占据格点闭区间
  [xi, xi+xsi-1] × [yi, yi+ysi-1] × [zi, zi+hi-1]。

联合建模的约束：
  1. 水平旋转（仅交换长宽）、车厢边界、3D 不重叠；
  2. 箱底由地板或“若干等高箱顶”完整支撑（支撑面投影面积必须覆盖箱底面积）；
  3. 抽出通道：先卸箱向车尾直线拖出时，任何后卸箱不得在 x/y/z 上同时挡道；
  4. 支撑与抽箱顺序耦合：任一箱被抽走后，仍在车的箱底仍须被完整支撑，
     因此支撑箱的卸货次序必须严格晚于被支撑箱；
  5. 各站卸货前、每抽走一箱后（剩余非空时）两倍纵向重心落在该站闭区间内。
"""

from __future__ import annotations

from ortools.sat.python import cp_model

from .schemas import PlanRequest


def _overlap_1d(model, starts, sizes, i, j, coord_max, tag):
    """两个整数区间 [s, s+a) 、[t, t+b) 的严格相交工具。

    返回 (ov, before_ij, before_ji, positive)：
      ov            实际相交格点数（分离/相切时恒为 0，不会为负）；
      before_ij     i 整体位于 j 远离原点一侧（s_i+a_i <= s_j）；
      before_ji     j 整体位于 i 远离原点一侧；
      positive      ov >= 1（严格重叠）。
    """
    ov = model.NewIntVar(0, coord_max, f"ov_{tag}_{i}_{j}")
    before_ij = model.NewBoolVar(f"before_{tag}_{i}_{j}")
    before_ji = model.NewBoolVar(f"before_{tag}_{j}_{i}")
    positive = model.NewBoolVar(f"ovpos_{tag}_{i}_{j}")

    end_i = starts[i] + sizes[i]
    end_j = starts[j] + sizes[j]

    # before_ij: end_i <= start_j；before_ji 对称
    model.Add(end_i <= starts[j]).OnlyEnforceIf(before_ij)
    model.Add(end_i > starts[j]).OnlyEnforceIf(before_ij.Not())
    model.Add(end_j <= starts[i]).OnlyEnforceIf(before_ji)
    model.Add(end_j > starts[i]).OnlyEnforceIf(before_ji.Not())

    # ov = max(0, min(end_i,end_j) - max(start_i,start_j))
    raw = model.NewIntVar(-coord_max, coord_max, f"raw_{tag}_{i}_{j}")
    min_end = model.NewIntVar(0, coord_max, f"minend_{tag}_{i}_{j}")
    max_start = model.NewIntVar(0, coord_max, f"maxstart_{tag}_{i}_{j}")
    model.AddMinEquality(min_end, [end_i, end_j])
    model.AddMaxEquality(max_start, [starts[i], starts[j]])
    model.Add(raw == min_end - max_start)
    model.Add(ov == 0).OnlyEnforceIf(before_ij)
    model.Add(ov == 0).OnlyEnforceIf(before_ji)
    model.Add(ov == raw).OnlyEnforceIf([before_ij.Not(), before_ji.Not()])

    model.Add(ov >= 1).OnlyEnforceIf(positive)
    model.Add(ov == 0).OnlyEnforceIf(positive.Not())
    return ov, before_ij, before_ji, positive


def solve_plan(req: PlanRequest, time_limit: float = 20.0) -> dict:
    L, W, H = req.carriage_length, req.carriage_width, req.carriage_height
    boxes = req.boxes
    stops = req.stops
    n = len(boxes)
    S = len(stops)
    stop_index = {s: k for k, s in enumerate(stops)}

    # 每站箱数与全局卸货序号区间边界：stop k 的箱 rank ∈ [base_k, base_{k+1})
    counts = [0] * S
    for b in boxes:
        counts[stop_index[b.destination]] += 1
    base = [0] * (S + 1)
    for k in range(S):
        base[k + 1] = base[k] + counts[k]

    m = cp_model.CpModel()

    # ---------------------------------------------------------------- 变量
    xs, ys = [], []          # 旋转后的长 / 宽
    x0, y0, z0 = [], [], []  # 各轴最小角坐标
    rotated = []
    on_floor = []
    footprint = []           # 箱底格点面积
    rank = []                # 全局抽箱次序 0..n-1

    for i, b in enumerate(boxes):
        r = m.NewBoolVar(f"rot_{i}")
        rotated.append(r)
        sx = m.NewIntVar(min(b.length, b.width), max(b.length, b.width), f"xs_{i}")
        sy = m.NewIntVar(min(b.length, b.width), max(b.length, b.width), f"ys_{i}")
        # 未旋转：x 长=length, y 宽=width；旋转后交换
        m.Add(sx == b.length).OnlyEnforceIf(r.Not())
        m.Add(sx == b.width).OnlyEnforceIf(r)
        m.Add(sy == b.width).OnlyEnforceIf(r.Not())
        m.Add(sy == b.length).OnlyEnforceIf(r)
        xs.append(sx)
        ys.append(sy)

        xi = m.NewIntVar(0, L - 1, f"x_{i}")
        yi = m.NewIntVar(0, W - 1, f"y_{i}")
        zi = m.NewIntVar(0, H - 1, f"z_{i}")
        m.Add(xi + sx <= L)
        m.Add(yi + sy <= W)
        m.Add(zi + b.height <= H)
        x0.append(xi)
        y0.append(yi)
        z0.append(zi)

        fl = m.NewBoolVar(f"floor_{i}")
        m.Add(zi == 0).OnlyEnforceIf(fl)
        m.Add(zi >= 1).OnlyEnforceIf(fl.Not())
        on_floor.append(fl)

        area = m.NewIntVar(1, L * W, f"area_{i}")
        m.AddMultiplicationEquality(area, [sx, sy])
        footprint.append(area)

        k = stop_index[b.destination]
        rk = m.NewIntVar(base[k], base[k + 1] - 1, f"rank_{i}")
        rank.append(rk)

    m.AddAllDifferent(rank)

    # 3D 不重叠：OR-Tools Python 接口无 3D no-overlap，按对建模——
    # 任意两箱至少在 x / y / z 一个轴上整体分离（含相切）。
    # 方位布尔量在下方成对几何段创建，析取子句在那里统一添加。

    # ------------------------------------------- 成对几何 / 次序关系
    # 对每对 i<j：
    #   ox/oy/oz：三轴严格相交格点数；pos_x/pos_y/pos_z：是否严格相交；
    #   bx_ij/bx_ji、by_ij/by_ji：沿 x / y 谁更靠近某一端；
    #   earlier_p：rank_i < rank_j（i 先于 j 被抽走）；
    #   support_ij：j 的箱顶与 i 的箱底等高（j 可能在下方支撑 i），反向同理；
    #   支撑面积贡献 = 等高 × 顶投影交面积 × 支撑箱后卸。
    pair_data = {}
    z_sizes = [b.height for b in boxes]
    for p in range(n):
        for q in range(p + 1, n):
            ox, bx_pq, bx_qp, pos_x = _overlap_1d(m, x0, xs, p, q, L, f"x{p}_{q}")
            oy, by_pq, by_qp, pos_y = _overlap_1d(m, y0, ys, p, q, W, f"y{p}_{q}")
            oz, bz_pq, bz_qp, pos_z = _overlap_1d(m, z0, z_sizes, p, q, H, f"z{p}_{q}")

            # 3D 不重叠：至少一轴整体分离
            m.AddBoolOr([bx_pq, bx_qp, by_pq, by_qp, bz_pq, bz_qp])

            earlier = m.NewBoolVar(f"earlier_{p}_{q}")  # rank_p < rank_q
            m.Add(rank[p] < rank[q]).OnlyEnforceIf(earlier)
            m.Add(rank[p] > rank[q]).OnlyEnforceIf(earlier.Not())

            # 等高支撑：箱顶恰好等于另一箱箱底
            supp_p_by_q = m.NewBoolVar(f"supp_{p}_by_{q}")  # z_q+h_q == z_p
            supp_q_by_p = m.NewBoolVar(f"supp_{q}_by_{p}")
            m.Add(z0[q] + boxes[q].height == z0[p]).OnlyEnforceIf(supp_p_by_q)
            m.Add(z0[q] + boxes[q].height != z0[p]).OnlyEnforceIf(supp_p_by_q.Not())
            m.Add(z0[p] + boxes[p].height == z0[q]).OnlyEnforceIf(supp_q_by_p)
            m.Add(z0[p] + boxes[p].height != z0[q]).OnlyEnforceIf(supp_q_by_p.Not())

            # 顶/底投影相交面积（x、y 都严格相交才非零）
            proj_area = m.NewIntVar(0, L * W, f"proj_{p}_{q}")
            m.AddMultiplicationEquality(proj_area, [ox, oy])

            # p 由 q 有效支撑：等高 + q 后卸（rank_p < rank_q）+ 投影相交
            gate_p = m.NewBoolVar(f"gate_{p}_by_{q}")
            gate_q = m.NewBoolVar(f"gate_{q}_by_{p}")
            m.Add(supp_p_by_q == 1).OnlyEnforceIf(gate_p)
            m.Add(earlier == 1).OnlyEnforceIf(gate_p)
            m.AddBoolOr([supp_p_by_q.Not(), earlier.Not()]).OnlyEnforceIf(gate_p.Not())
            m.Add(supp_q_by_p == 1).OnlyEnforceIf(gate_q)
            m.Add(earlier == 0).OnlyEnforceIf(gate_q)
            m.AddBoolOr([supp_q_by_p.Not(), earlier]).OnlyEnforceIf(gate_q.Not())

            eff_p = m.NewIntVar(0, L * W, f"eff_{p}_by_{q}")
            eff_q = m.NewIntVar(0, L * W, f"eff_{q}_by_{p}")
            m.AddMultiplicationEquality(eff_p, [gate_p, proj_area])
            m.AddMultiplicationEquality(eff_q, [gate_q, proj_area])

            pair_data[(p, q)] = dict(
                ox=ox, oy=oy, oz=oz, pos_x=pos_x, pos_y=pos_y, pos_z=pos_z,
                bx_pq=bx_pq, bx_qp=bx_qp, by_pq=by_pq, by_qp=by_qp,
                earlier=earlier, eff_p=eff_p, eff_q=eff_q,
            )

    # ------------------------------------------------- 支撑约束
    # 每个中间状态都成立 ⇔ 有效支撑面积（支撑箱后卸）覆盖整个箱底，
    # 或箱底在地板上。同层箱互不重叠 ⇒ 各支撑面积不重复计数。
    for i in range(n):
        terms = []
        floor_term = m.NewIntVar(0, L * W, f"floorarea_{i}")
        m.AddMultiplicationEquality(floor_term, [on_floor[i], footprint[i]])
        for (p, q), d in pair_data.items():
            if i == p:
                terms.append(d["eff_p"])      # i=p 由 q 支撑
            elif i == q:
                terms.append(d["eff_q"])      # i=q 由 p 支撑
        m.Add(sum(terms, 0) + floor_term >= footprint[i])

    # ------------------------------------------------- 抽出通道
    # i 先抽、j 仍在车时，j 若 (a) 在 x 上更靠车尾且封死退路、
    # (b) y 严格相交、(c) z 严格相交，则 i 无法直线拖出，禁止。
    # 由 3D 不重叠，y/z 均相交时 x 必分离，故“封死退路”用相切也算挡
    # （j 的靠车头端 <= i 的靠车尾端，即 bx 对应字面量）。
    for (p, q), d in pair_data.items():
        not_y = d["pos_y"].Not()
        not_z = d["pos_z"].Not()
        # p 先抽：q 挡道 ⇔ q 位于 p 车尾侧（bx_qp）
        m.AddBoolOr([d["earlier"].Not(), d["bx_qp"].Not(), not_y, not_z])
        # q 先抽：p 挡道 ⇔ p 位于 q 车尾侧（bx_pq）
        m.AddBoolOr([d["earlier"], d["bx_pq"].Not(), not_y, not_z])

    # ------------------------------------------------- 重心约束
    # 状态 t（已抽走 rank < t 的箱，t=0 为到站初始装载）：
    #   M2(t) = Σ present_i w_i (2 x_i + xs_i)
    # 要求 lo2_s * W(t) <= M2(t) <= hi2_s * W(t)。
    # 站 k 覆盖闭状态区间 t ∈ [base_k, base_{k+1}]（到站 + 本站每抽一箱后；
    # 相邻两站共享边界状态，故该状态同时施加两站区间）。t=n 为空车，不检查；
    # 空站（base_k == base_{k+1}）映射到其唯一到站状态，也施加本站区间。
    total_w = sum(b.weight for b in boxes)
    max_moment = 2 * L * total_w
    state_of_t = {}
    for t in range(n):
        for k in range(S):
            if base[k] <= t <= base[k + 1]:
                state_of_t.setdefault(t, []).append(k)

    for t, ks in state_of_t.items():
        msum = m.NewIntVar(0, max_moment, f"m2_{t}")
        wsum = m.NewIntVar(0, total_w, f"w_{t}")
        mterms, wterms = [], []
        for i, b in enumerate(boxes):
            present = m.NewBoolVar(f"pres_{i}_{t}")
            if t == 0:
                m.Add(present == 1)
            else:
                m.Add(rank[i] >= t).OnlyEnforceIf(present)
                m.Add(rank[i] <= t - 1).OnlyEnforceIf(present.Not())
            mt = m.NewIntVar(0, 2 * L * b.weight, f"mt_{i}_{t}")
            wt = m.NewIntVar(0, b.weight, f"wt_{i}_{t}")
            m.Add(mt == b.weight * (2 * x0[i] + xs[i])).OnlyEnforceIf(present)
            m.Add(mt == 0).OnlyEnforceIf(present.Not())
            m.Add(wt == b.weight).OnlyEnforceIf(present)
            m.Add(wt == 0).OnlyEnforceIf(present.Not())
            mterms.append(mt)
            wterms.append(wt)
        m.Add(msum == sum(mterms, 0))
        m.Add(wsum == sum(wterms, 0))
        for k in ks:
            cg = req.cg_limits[stops[k]]
            m.Add(msum >= cg.lo * wsum)
            m.Add(msum <= cg.hi * wsum)

    # ---------------------------------------------------------------- 求解
    solver = cp_model.CpSolver()
    solver.parameters.max_deterministic_time = time_limit
    solver.parameters.num_search_workers = 8
    status = solver.Solve(m)
    elapsed = round(solver.WallTime(), 4)

    if status not in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        return {
            "feasible": False,
            "status": "infeasible" if status == cp_model.INFEASIBLE else "timeout",
            "carriage": {
                "length": L, "width": W, "height": H,
            },
            "stops": stops,
            "placements": [],
            "unloading": [{"stop": s, "order": []} for s in stops],
            "removal_order": [],
            "solver_time_seconds": elapsed,
        }

    placements = []
    for i, b in enumerate(boxes):
        placements.append(
            {
                "box_id": b.box_id,
                "rotated": bool(solver.Value(rotated[i])),
                "x": [solver.Value(x0[i]), solver.Value(x0[i]) + solver.Value(xs[i]) - 1],
                "y": [solver.Value(y0[i]), solver.Value(y0[i]) + solver.Value(ys[i]) - 1],
                "z": [solver.Value(z0[i]), solver.Value(z0[i]) + b.height - 1],
            }
        )

    order_ids = [""] * n
    for i, b in enumerate(boxes):
        order_ids[solver.Value(rank[i])] = b.box_id
    unloading = []
    for k, s in enumerate(stops):
        unloading.append({"stop": s, "order": order_ids[base[k]:base[k + 1]]})

    return {
        "feasible": True,
        "status": "optimal" if status == cp_model.OPTIMAL else "feasible",
        "carriage": {"length": L, "width": W, "height": H},
        "stops": stops,
        "placements": sorted(placements, key=lambda p: p["box_id"]),
        "unloading": unloading,
        "removal_order": order_ids,
        "solver_time_seconds": elapsed,
    }
