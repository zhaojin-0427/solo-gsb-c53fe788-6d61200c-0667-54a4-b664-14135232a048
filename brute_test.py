"""极小实例的暴力金标准：枚举位置/朝向/卸货顺序，与 CP-SAT 结果对比。"""

import itertools
import random

from app.schemas import BoxIn, CgInterval, PlanRequest
from app.solver import solve_plan
from verify import verify


def brute_feasible(req):
    L, W, H = req.carriage_length, req.carriage_width, req.carriage_height
    boxes = req.boxes
    n = len(boxes)
    stops = req.stops
    stop_idx = {s: k for k, s in enumerate(stops)}

    dims = []
    for b in boxes:
        opts = [(b.length, b.width)]
        if (b.width, b.length) not in opts:
            opts.append((b.width, b.length))
        dims.append(opts)

    positions = []
    for b, opts in zip(boxes, dims):
        plist = []
        for sx, sy in opts:
            for x in range(L - sx + 1):
                for y in range(W - sy + 1):
                    for z in range(H - b.height + 1):
                        plist.append((x, y, z, sx, sy))
        positions.append(plist)

    def occ_set(k, pi):
        x, y, z, sx, sy = pi
        b = boxes[k]
        return set((x + dx, y + dy, z + dz)
                   for dx in range(sx) for dy in range(sy) for dz in range(b.height))

    # 本站约束的卸货顺序：同站内任意排列
    by_stop = {s: [] for s in stops}
    for i, b in enumerate(boxes):
        by_stop[b.destination].append(i)
    stop_perms = [list(itertools.permutations(by_stop[s])) for s in stops]

    def check(placement):
        occ = [None] * n
        allc = {}
        for k in range(n):
            cs = occ_set(k, placement[k])
            for c in cs:
                if c in allc:
                    return False
                allc[c] = k
            occ[k] = cs

        def support(alive):
            for i in alive:
                x, y, z, sx, sy = placement[i]
                if z == 0:
                    continue
                for dx in range(sx):
                    for dy in range(sy):
                        j = allc.get((x + dx, y + dy, z - 1))
                        if j is None or j not in alive:
                            return False
            return True

        if not support(set(range(n))):
            return False

        def cg_ok(alive, s):
            num = den = 0
            for i in alive:
                b = boxes[i]
                x, y, z, sx, sy = placement[i]
                num += b.weight * (2 * x + sx)
                den += b.weight
            lim = req.cg_limits[s]
            return lim.lo * den <= num <= lim.hi * den

        def corridor(i, alive):
            x, y, z, sx, sy = placement[i]
            b = boxes[i]
            for j in alive:
                if j == i:
                    continue
                x2, y2, z2, sx2, sy2 = placement[j]
                yov = not (y + sy <= y2 or y2 + sy2 <= y)
                zov = not (z + b.height <= z2 or z2 + boxes[j].height <= z)
                if yov and zov and x2 + sx2 <= x:
                    return False
            return True

        for sp in itertools.product(*stop_perms):
            alive = set(range(n))
            ok = True
            for si, perm in enumerate(sp):
                if alive and not cg_ok(alive, stops[si]):
                    ok = False
                    break
                for i in perm:
                    if not corridor(i, alive):
                        ok = False
                        break
                    alive.remove(i)
                    if not support(alive):
                        ok = False
                        break
                    if alive and not cg_ok(alive, stops[si]):
                        ok = False
                        break
                if not ok:
                    break
            if ok:
                return True
        return False

    for placement in itertools.product(*positions):
        if check(placement):
            return True
    return False


def main(seed=1, iters=200):
    rng = random.Random(seed)
    checked = 0
    for it in range(iters):
        L, W, H = (rng.randint(2, 3), rng.randint(1, 2), rng.randint(1, 2))
        n = rng.randint(1, 3)
        nstops = rng.randint(1, 2)
        stops = [f"S{k}" for k in range(nstops)]
        boxes = []
        used_vol = 0
        for i in range(n):
            lx = rng.randint(1, 2)
            wy = rng.randint(1, min(2, W))
            hz = rng.randint(1, H)
            used_vol += lx * wy * hz
            boxes.append(BoxIn(
                box_id=f"b{i}", length=lx, width=wy, height=hz,
                weight=rng.randint(1, 3), destination=rng.choice(stops),
            ))
        if used_vol > L * W * H:
            continue
        cg_lo = rng.choice([0, 2])
        cg_hi = rng.choice([2 * L, 2 * L + 1])
        if cg_lo > cg_hi:
            cg_lo, cg_hi = cg_hi, cg_lo
        try:
            req = PlanRequest(
                carriage_length=L, carriage_width=W, carriage_height=H,
                stops=stops,
                cg_limits={s: CgInterval(lo=cg_lo, hi=cg_hi) for s in stops},
                boxes=boxes,
            )
        except Exception:
            continue
        r = solve_plan(req, time_limit=5)
        if r["status"] == "timeout":
            continue
        bf = brute_feasible(req)
        if r["feasible"]:
            probs = verify(req, r)
            assert not probs, probs[:3]
        checked += 1
        if r["feasible"] != bf:
            print(f"MISMATCH iter {it}: cpsat={r['feasible']} brute={bf}")
            print(req.model_dump_json(indent=1))
            return 1
    print(f"{checked} 个极小实例：CP-SAT 与暴力枚举完全一致")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
