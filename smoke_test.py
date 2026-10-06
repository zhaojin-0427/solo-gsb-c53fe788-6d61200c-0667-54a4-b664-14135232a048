"""冒烟测试：求解 + 独立校验器交叉验证。"""

from app.schemas import BoxIn, CgInterval, PlanRequest
from app.solver import solve_plan
from verify import verify

FAIL = []


def check(title, req, expect_feasible, extra=None):
    r = solve_plan(req, time_limit=10)
    ok = r["feasible"] == expect_feasible
    detail = ""
    if expect_feasible:
        if r["feasible"]:
            problems = verify(req, r)
            if problems:
                ok = False
                detail = "; ".join(problems[:5])
    mark = "PASS" if ok else "FAIL"
    if not ok:
        FAIL.append(title)
    print(f"[{mark}] {title} (status={r['status']}, t={r['solver_time_seconds']}s) {detail}")
    if r["feasible"]:
        for p in r["placements"]:
            print("   ", p)
        for u in r["unloading"]:
            print("   ", u)
    if extra and r["feasible"]:
        extra(req, r)
    return r


CG2 = {"S1": CgInterval(lo=0, hi=8), "S2": CgInterval(lo=0, hi=8)}

# 1) README 旋转示例
check(
    "rotation 3-box example",
    PlanRequest(
        carriage_length=3, carriage_width=2, carriage_height=2,
        stops=["S1", "S2"], cg_limits=CG2,
        boxes=[
            BoxIn(box_id="a", length=2, width=1, height=1, weight=1, destination="S1"),
            BoxIn(box_id="b", length=2, width=1, height=1, weight=1, destination="S2"),
            BoxIn(box_id="c", length=1, width=1, height=1, weight=1, destination="S2"),
        ],
    ),
    True,
)

# 2) 重心不可能：站 1 只剩 a 时 2*cg=1，区间要求 >=6
check(
    "CG infeasible",
    PlanRequest(
        carriage_length=3, carriage_width=2, carriage_height=2,
        stops=["S1", "S2"],
        cg_limits={"S1": CgInterval(lo=6, hi=8), "S2": CgInterval(lo=0, hi=8)},
        boxes=[
            BoxIn(box_id="a", length=2, width=1, height=1, weight=1, destination="S1"),
            BoxIn(box_id="b", length=2, width=1, height=1, weight=1, destination="S2"),
            BoxIn(box_id="c", length=1, width=1, height=1, weight=1, destination="S2"),
        ],
    ),
    False,
)

# 3) 抽出通道被满高柱封死：L=3,W=1,H=2。w1、w2 为 S2 的 2 格高柱，
#    a（S1）长 2、高 1。两柱必占两个不同 x 格（各 z=0..1）：
#      - 柱占 x=0,1：a 只能放 x=1..2 或 x=0..1，其车尾侧格点必有满高柱；
#      - 柱占 x=0,2：a 只能放 x=0..1，x=0 与柱同 z；
#      - 柱占 x=1,2：a 只能放 x=0..1，退路在 x=1 被柱封；
#    a 无法抬到 z 层（柱满高）。⇒ 无解（体积充足，纯通道/站序耦合）。
check(
    "full-height columns block wide early box",
    PlanRequest(
        carriage_length=3, carriage_width=1, carriage_height=2,
        stops=["S1", "S2"], cg_limits=CG2,
        boxes=[
            BoxIn(box_id="w1", length=1, width=1, height=2, weight=1, destination="S2"),
            BoxIn(box_id="w2", length=1, width=1, height=2, weight=1, destination="S2"),
            BoxIn(box_id="a", length=2, width=1, height=1, weight=1, destination="S1"),
        ],
    ),
    False,
)

# 4) 同站满底叠放：上层必须先抽
def top_first(req, r):
    order = r["unloading"][0]["order"]
    pos = {p["box_id"]: p for p in r["placements"]}
    if order[0] != "up" or pos["up"]["z"][0] != 1:
        FAIL.append("top_first ordering")
        print("    [FAIL] top must go first")


check(
    "same-stop stack top-first",
    PlanRequest(
        carriage_length=2, carriage_width=2, carriage_height=2,
        stops=["S1"],
        cg_limits={"S1": CgInterval(lo=0, hi=8)},
        boxes=[
            BoxIn(box_id="low", length=2, width=2, height=1, weight=1, destination="S1"),
            BoxIn(box_id="up", length=2, width=2, height=1, weight=1, destination="S1"),
        ],
    ),
    True,
    top_first,
)

# 5) 多箱平铺 + 三层站点
check(
    "flat 6 boxes 3 stops",
    PlanRequest(
        carriage_length=6, carriage_width=2, carriage_height=2,
        stops=["A", "B", "C"],
        cg_limits={
            "A": CgInterval(lo=0, hi=24),
            "B": CgInterval(lo=0, hi=24),
            "C": CgInterval(lo=0, hi=24),
        },
        boxes=[
            BoxIn(box_id=f"box{i}", length=1, width=1, height=1, weight=2,
                  destination=d)
            for i, d in enumerate(["A", "A", "B", "B", "C", "C"])
        ],
    ),
    True,
)

# 6) 非满底支撑：上层小箱骑在两个下层箱拼缝上（同站，可卸）
check(
    "partial support over two boxes same stop",
    PlanRequest(
        carriage_length=2, carriage_width=1, carriage_height=2,
        stops=["S1"],
        cg_limits={"S1": CgInterval(lo=0, hi=8)},
        boxes=[
            BoxIn(box_id="l1", length=1, width=1, height=1, weight=1, destination="S1"),
            BoxIn(box_id="l2", length=1, width=1, height=1, weight=1, destination="S1"),
            BoxIn(box_id="up", length=2, width=1, height=1, weight=1, destination="S1"),
        ],
    ),
    True,
)

# 7) 真正的跨站支撑耦合不可行（非体积原因）：
#    L=2,W=1,H=2。deck(S2) 2×1×1 占满地板；early(S1) 为 1×1×1。
#    early 放地板：无空位（deck 占满）；放 z=1：必须由 deck 支撑，
#    放在 x=0 或 x=1。抽 early 时其 z 区间 [1,1]，deck z=[0,0] 不相交，
#    通道畅通 ⇒ 实际可行。
#    加 post(S2,1×1×2) 后体积超容；改为把 deck 设为 2×1×1 且
#    early 与 deck 的 y 相同（W=1 必然），再把 early 设成
#    “被抽走后 deck 无影响”—— 仍可行。因此真正不可行的最简形态：
#    early(2×1×1) 只能放在 deck(2×1×1) 顶（z=1），y 全覆盖，
#    二者同 x 投影；抽 early 时不与 deck 相交（z 分离）⇒ 仍可抽。
#    结论：纯叠放跨站不构成不可行；必须叠加通道封死。见 case 3
#    的严格版：下面用一堵满高后墙封死所有退路。
check(
    "stacked early over later is feasible (slides over the top)",
    PlanRequest(
        carriage_length=2, carriage_width=1, carriage_height=2,
        stops=["S1", "S2"], cg_limits=CG2,
        boxes=[
            BoxIn(box_id="deck", length=2, width=1, height=1, weight=1, destination="S2"),
            BoxIn(box_id="early", length=2, width=1, height=1, weight=1, destination="S1"),
        ],
    ),
    True,
)

print()

# 8) 空中间站：S2 无箱，其到站状态 = S1 抽完后的状态，须满足 S2 重心区间。
#    S2 区间 [0,2] 迫使仅剩的 S3 箱 b 位于 x=0；整体可行。
check(
    "empty intermediate stop applies its CG bound",
    PlanRequest(
        carriage_length=3, carriage_width=2, carriage_height=2,
        stops=["S1", "S2", "S3"],
        cg_limits={
            "S1": CgInterval(lo=0, hi=8),
            "S2": CgInterval(lo=0, hi=2),
            "S3": CgInterval(lo=0, hi=8),
        },
        boxes=[
            BoxIn(box_id="a", length=1, width=1, height=1, weight=1, destination="S1"),
            BoxIn(box_id="b", length=1, width=1, height=1, weight=1, destination="S3"),
        ],
    ),
    True,
    lambda req, r: (
        FAIL.append("empty stop CG: b must be at x=0")
        or print("    [FAIL] b 不在 x=0")
        if next(p for p in r["placements"] if p["box_id"] == "b")["x"][0] != 0
        else None
    ),
)

# 9) 空站区间不可满足：要求剩余箱两倍重心 <=0，任何 1 格箱都不满足。
check(
    "empty stop CG impossible",
    PlanRequest(
        carriage_length=3, carriage_width=2, carriage_height=2,
        stops=["S1", "S2"],
        cg_limits={
            "S1": CgInterval(lo=0, hi=8),
            "S2": CgInterval(lo=0, hi=0),
        },
        boxes=[
            BoxIn(box_id="a", length=1, width=1, height=1, weight=1, destination="S1"),
            BoxIn(box_id="b", length=1, width=1, height=1, weight=1, destination="S2"),
        ],
    ),
    False,
)

print()
if FAIL:
    print("FAILURES:", FAIL)
    raise SystemExit(1)
print("ALL TESTS PASSED")
