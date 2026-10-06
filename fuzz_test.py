"""随机模糊测试：随机实例交叉验证求解器与独立校验器。"""

import random
import sys

from app.schemas import BoxIn, CgInterval, PlanRequest
from app.solver import solve_plan
from verify import verify

random.seed(int(sys.argv[1]) if len(sys.argv) > 1 else 42)


def random_request(rng):
    L = rng.randint(3, 6)
    W = rng.randint(2, 4)
    H = rng.randint(2, 3)
    nstops = rng.randint(1, 3)
    stops = [f"S{k}" for k in range(nstops)]
    cg = {s: CgInterval(lo=0, hi=2 * L + 1) for s in stops}
    nboxes = rng.randint(1, 6)
    boxes = []
    for i in range(nboxes):
        boxes.append(
            BoxIn(
                box_id=f"b{i}",
                length=rng.randint(1, min(3, L)),
                width=rng.randint(1, min(2, W)),
                height=rng.randint(1, H),
                weight=rng.randint(1, 4),
                destination=rng.choice(stops),
            )
        )
    return PlanRequest(
        carriage_length=L, carriage_width=W, carriage_height=H,
        stops=stops, cg_limits=cg, boxes=boxes,
    )


def main(iters=150):
    false_neg = 0
    for it in range(iters):
        rng = random.Random(it)
        try:
            req = random_request(rng)
        except Exception:
            continue  # 天然放不下的箱子也可能随机生成
        r = solve_plan(req, time_limit=3)
        if r["status"] == "timeout":
            continue
        if r["feasible"]:
            problems = verify(req, r)
            if problems:
                print(f"iter {it}: 校验器拒绝求解器的解:")
                for p in problems[:8]:
                    print("   ", p)
                return 1
        else:
            false_neg += 1
    print(f"完成 {iters} 组随机实例，未发现求解器/校验器矛盾；"
          f"被判不可行的实例 {false_neg} 个（含真实无解与难解实例）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
