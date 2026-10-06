"""独立于求解器的结果校验器：枚举每个卸货中间状态检查全部物理约束。

用于测试，也可作为约束语义的可执行说明。
"""

from __future__ import annotations


def _cells(p, axis):
    lo, hi = p[axis]
    return set(range(lo, hi + 1))


def verify(req, result):
    """对 solve_plan 的成功结果做完整校验，返回问题列表（空列表表示通过）。"""
    assert result["feasible"], result
    L, W, H = req.carriage_length, req.carriage_width, req.carriage_height
    boxes = {b.box_id: b for b in req.boxes}
    stops = req.stops
    stop_of = {bid: b.destination for bid, b in boxes.items()}
    pl = {p["box_id"]: p for p in result["placements"]}
    problems = []

    # ---- 静态：尺寸/朝向/边界/不重叠/地板支撑
    occupied = {}
    for bid, b in boxes.items():
        p = pl[bid]
        xs, ys, zs = _cells(p, "x"), _cells(p, "y"), _cells(p, "z")
        dx, dy = len(xs), len(ys)
        if p["rotated"]:
            expect = (b.width, b.length)
        else:
            expect = (b.length, b.width)
        if (dx, dy) != expect:
            problems.append(f"{bid}: 尺寸/朝向不一致 {(dx,dy)} != {expect}")
        if len(zs) != b.height:
            problems.append(f"{bid}: 高度 {len(zs)} != {b.height}")
        if min(xs) < 0 or max(xs) >= L or min(ys) < 0 or max(ys) >= W or min(zs) < 0 or max(zs) >= H:
            problems.append(f"{bid}: 超出车厢边界")
        for x in xs:
            for y in ys:
                for z in zs:
                    if (x, y, z) in occupied:
                        problems.append(f"{bid} 与 {occupied[(x,y,z)]} 在格点 ({x},{y},{z}) 重叠")
                    occupied[(x, y, z)] = bid

    def support_ok(alive):
        """每个存活箱的每个底格点都由地板或同高箱顶覆盖。"""
        bad = []
        for bid in alive:
            b, p = boxes[bid], pl[bid]
            zbase = p["z"][0]
            for x in _cells(p, "x"):
                for y in _cells(p, "y"):
                    if zbase == 0:
                        continue
                    supporter = occupied.get((x, y, zbase - 1))
                    if supporter is None or supporter not in alive:
                        bad.append(f"{bid} 底格点 ({x},{y},{zbase}) 无支撑")
                        continue
                    sp = pl[supporter]
                    if sp["z"][1] != zbase - 1:
                        bad.append(f"{bid} 底格点支撑 {supporter} 不等高")
        return bad

    alive = set(boxes)
    problems += support_ok(alive)

    # ---- 卸货顺序与站点一致
    flat = result["removal_order"]
    if sorted(flat) != sorted(boxes):
        problems.append("removal_order 与箱集合不一致")
    prev_stop_idx = -1
    for bid in flat:
        idx = stops.index(stop_of[bid])
        if idx < prev_stop_idx:
            problems.append(f"卸货次序中站点倒退: {bid}")
        prev_stop_idx = idx
    for u in result["unloading"]:
        members = [bid for bid in flat if stop_of[bid] == u["stop"]]
        if members != u["order"]:
            problems.append(f"站 {u['stop']} 的顺序与全局顺序不一致")

    # ---- 逐状态：抽出通道 + 支撑 + 重心
    # 状态 t：rank<t 的箱已抽走。到站 t=base_k 与本站最后一次抽完后的
    # 状态 t=base_{k+1} 都施加站 k 区间（共享状态同时受相邻两站约束）。
    counts = {s: 0 for s in stops}
    for bid in flat:
        counts[stop_of[bid]] += 1
    base = [0]
    for s in stops:
        base.append(base[-1] + counts[s])
    stops_for_state = {}
    for t in range(len(boxes) + 1):
        for k in range(len(stops)):
            if base[k] <= t <= base[k + 1]:
                stops_for_state.setdefault(t, []).append(stops[k])

    def corridor_ok(removing, alive_before):
        # 向车尾 (x 减小方向) 直线拖出，扫过 [0, x_hi] × y交 × z交
        p = pl[removing]
        for j in alive_before:
            if j == removing:
                continue
            q = pl[j]
            if _cells(p, "y") & _cells(q, "y") and _cells(p, "z") & _cells(q, "z"):
                if q["x"][1] <= p["x"][0]:
                    return f"{removing} 抽出时被 {j} 挡住"
        return None

    def cg_ok(alive, stop):
        num = den = 0
        for bid in alive:
            b = boxes[bid]
            p = pl[bid]
            num += b.weight * (p["x"][0] + p["x"][1] + 1)  # 2*center
            den += b.weight
        lo = req.cg_limits[stop].lo
        hi = req.cg_limits[stop].hi
        if not (lo * den <= num <= hi * den):
            return f"站 {stop}: 两倍重心 {num}/{den} 超出 [{lo},{hi}]"
        return None

    alive = set(boxes)
    for t, bid in enumerate(flat):
        # t 状态（抽本箱之前）：通道 + 各适用站点的重心
        msg = corridor_ok(bid, alive)
        if msg:
            problems.append(msg)
        for s in stops_for_state[t]:
            msg = cg_ok(alive, s)
            if msg:
                problems.append(f"状态 t={t}（{bid} 抽出前）: {msg}")
        alive = alive - {bid}
        problems += [f"抽 {bid} 后: {e}" for e in support_ok(alive)]
        # t+1 状态（抽本箱之后，空车不检查重心）
        if alive:
            for s in stops_for_state[t + 1]:
                msg = cg_ok(alive, s)
                if msg:
                    problems.append(f"状态 t={t + 1}（{bid} 抽出后）: {msg}")

    return problems
