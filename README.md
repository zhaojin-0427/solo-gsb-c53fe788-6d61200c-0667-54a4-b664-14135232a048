# 多站货车装载规划 HTTP API

基于 **Python 3.11 + FastAPI + OR-Tools CP-SAT** 的多站点货车装箱/卸货顺序联合规划服务。

给定整数格车厢、按顺序访问的站点、每个箱子的整数长宽高/正整数重量/目的站，
以及各站允许的纵向重心闭区间（以**两倍坐标**的整数给出），服务一次性求解：

- 每个箱子的格点位置与水平朝向（可 90° 水平旋转，仅交换长、宽）；
- 每个站点的逐箱卸货顺序。

装载排布与卸货顺序**联合**满足以下全部约束：

1. **不重叠、不出界**：箱体占据整数格点闭区间，可水平旋转。
2. **完整支撑**：箱底的每一个格点都由地板，或箱顶恰好与箱底等高的箱体覆盖。
3. **抽出通道**：到站后只能把本站箱沿车长方向**直线拉向车尾**（车尾为 `x=0`）；
   扫掠路径上若有后卸箱在 y、z 两个方向同时占据通道，则该箱无法抽出。
4. **过程支撑**：卸货前以及每抽走一箱之后，剩余每个箱体仍必须被完整支撑
   （因此支撑箱的卸货次序一定晚于被支撑箱）。
5. **逐状态重心**：卸货前及每抽走一箱后，只要车上还有箱，剩余货物的两倍纵向
   重心都必须落在该站给定的整数闭区间内。

无可行装载时返回 `feasible: false`（HTTP 200）；非法尺寸、重复箱 ID、
未知目的站等输入错误返回 **HTTP 422**。

## 坐标与数据约定

- `x`：车长方向，**车尾为 0**，向车头递增；`y`：车宽；`z`：竖直向上，地板为 0。
- 箱 `i` 占据格点闭区间
  `x ∈ [xi, xi+xsi−1]`、`y ∈ [yi, yi+ysi−1]`、`z ∈ [zi, zi+hi−1]`。
- 重心：箱重心取几何中心，其两倍 x 坐标为 `2·xi + xsi`。对车上箱集合 `R` 要求

  ```
  lo_s · Σw  ≤  Σ w_i·(2·xi + xsi)  ≤  hi_s · Σw
  ```

  其中 `[lo_s, hi_s]` 是站点 `s` 的 `cg_limits`，全程无除法、无浮点。
- 状态覆盖：到站初始装载、本站每抽走一箱之后的剩余状态（即也是下一站的到站
  状态）都施加该站区间；空车不检查。

## 一键启动（Docker Compose）

```bash
docker compose up --build
```

服务监听 `http://localhost:8000`，可用环境变量 `SOLVER_TIME_LIMIT`（秒，
默认 20）调整 CP-SAT 确定性求解时限。超时且未找到解时返回
`status: "timeout"`；模型被证明无解时返回 `status: "infeasible"`。

健康检查：

```bash
curl http://localhost:8000/health
# {"status":"ok"}
```

交互式文档（Swagger UI）：<http://localhost:8000/docs>。

## API：`POST /plans`

### 请求字段

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `carriage_length/width/height` | 正整数 | 车厢格点尺寸（x/y/z） |
| `stops` | 字符串数组 | 站点访问顺序，不允许重复 |
| `cg_limits` | 对象 | 每个站点一个 `{lo, hi}` 整数闭区间，站点必须与 `stops` 一一对应 |
| `boxes[].box_id` | 字符串 | 全请求唯一 |
| `boxes[].length/width/height` | 正整数 | 原始朝向的长宽高（格） |
| `boxes[].weight` | 正整数 | 重量 |
| `boxes[].destination` | 字符串 | 目的站，必须出现在 `stops` 中 |

箱体即使旋转（交换长、宽）也放不进车厢，返回 422。

### 请求示例（实际可用）

```bash
curl -s -X POST http://localhost:8000/plans \
  -H 'Content-Type: application/json' \
  -d '{
    "carriage_length": 3,
    "carriage_width": 2,
    "carriage_height": 2,
    "stops": ["S1", "S2"],
    "cg_limits": {
      "S1": {"lo": 0, "hi": 8},
      "S2": {"lo": 0, "hi": 8}
    },
    "boxes": [
      {"box_id": "a", "length": 2, "width": 1, "height": 1, "weight": 1, "destination": "S1"},
      {"box_id": "b", "length": 2, "width": 1, "height": 1, "weight": 1, "destination": "S2"},
      {"box_id": "c", "length": 1, "width": 1, "height": 1, "weight": 1, "destination": "S2"}
    ]
  }'
```

### 响应示例（上述请求的真实返回）

```json
{
    "feasible": true,
    "status": "optimal",
    "carriage": {"length": 3, "width": 2, "height": 2},
    "stops": ["S1", "S2"],
    "placements": [
        {"box_id": "a", "rotated": true,  "x": [0, 0], "y": [0, 1], "z": [1, 1]},
        {"box_id": "b", "rotated": false, "x": [0, 1], "y": [0, 0], "z": [0, 0]},
        {"box_id": "c", "rotated": false, "x": [0, 0], "y": [1, 1], "z": [0, 0]}
    ],
    "unloading": [
        {"stop": "S1", "order": ["a"]},
        {"stop": "S2", "order": ["b", "c"]}
    ],
    "removal_order": ["a", "b", "c"],
    "solver_time_seconds": 0.0171
}
```

读法：

- `b`（2×1×1，S2）横放在地板左列 `x=0..1,y=0`；
- `c`（1×1×1，S2）立在车尾右列 `x=0,y=1` 地板上；
- `a`（原始 2×1，S1）**旋转 90°** 后变成 1×1，放在 `x=0,y=1,z=1`，
  箱底恰好由 `c` 的箱顶完整支撑；
- S1 到站：`a` 沿车尾方向直线抽出。它与 `b` 在 y 方向分列两列
  （`a` 占 y=1，`b` 占 y=0），与 `c` 虽在 y=1 重叠但高度分层
  （`a` 在 z=1，`c` 在 z=0）——没有任何后卸箱在 y、z 两方向同时
  挡住扫掠路径，通道畅通；
- S2 到站：按 `b → c` 抽出（顺序由重心与通道约束共同决定）。
- `unloading` 给出逐站序列，`removal_order` 是全局抽箱次序（按时间拼接）。
- `x/y/z` 均为**整数闭区间** `[起, 止]`（格点坐标，含端点）。

### 无解示例

把 S1 重心区间收紧到 `[6,8]`：S1 抽箱前的两倍重心不满足该区间，
服务返回 HTTP 200：

```json
{
    "feasible": false,
    "status": "infeasible",
    "carriage": {"length": 3, "width": 2, "height": 2},
    "stops": ["S1", "S2"],
    "placements": [],
    "unloading": [
        {"stop": "S1", "order": []},
        {"stop": "S2", "order": []}
    ],
    "removal_order": [],
    "solver_time_seconds": 0.0015
}
```

### 输入校验示例（HTTP 422）

重复箱 ID、目的站未知、尺寸非正、箱体旋转后仍超车厢、站点重复、
某站缺少 `cg_limits` 等均返回 422，例如：

```bash
curl -i -X POST http://localhost:8000/plans \
  -H 'Content-Type: application/json' \
  -d '{"carriage_length":3,"carriage_width":2,"carriage_height":2,
       "stops":["S1"],"cg_limits":{"S1":{"lo":0,"hi":8}},
       "boxes":[
         {"box_id":"a","length":1,"width":1,"height":1,"weight":1,"destination":"S1"},
         {"box_id":"a","length":1,"width":1,"height":1,"weight":1,"destination":"S1"}]}'
# HTTP/1.1 422 Unprocessable Entity
# {"detail":[{"type":"value_error", ... ,
#   "msg":"Value error, box_id 重复: ['a']", ...}]}
```

## 本地不用 Docker 运行

```bash
pip install -r requirements.txt
uvicorn app.main:app --host 0.0.0.0 --port 8000
```

## 项目结构

```
app/
  main.py      FastAPI 应用与 /plans、/health 路由
  schemas.py   Pydantic 请求/响应模型与 422 校验
  solver.py    OR-Tools CP-SAT 联合模型（布局 + 卸货次序）
verify.py      独立的结果校验器（逐中间状态枚举支撑/通道/重心）
smoke_test.py  典型可行/不可行用例（求解器 × 校验器交叉验证）
brute_test.py  极小实例暴力枚举金标准对比
fuzz_test.py   随机实例模糊测试
```

### CP-SAT 建模要点

- 每箱一个旋转布尔量，条件化长/宽尺寸；位置 `(x,y,z)` 为整数变量，
  成对在三轴上析取“整体分离”实现 3D 不重叠。
- 对每对箱建立三轴严格相交长度（门控为非负）、等高（甲顶=乙底）布尔量，
  用“等高 × 支撑箱后卸 × xy 投影交面积”之和覆盖整个箱底面积，
  一次性表达所有中间状态的完整支撑。
- 每对箱一条布尔子句：若 i 先卸，则任何在其车尾侧、y/z 同时相交的箱
  都不允许存在——即直线抽出通道约束（同高相切也算挡道）。
- 全局 `rank` 变量按目的站分段，`AllDifferent` 给出各站卸货顺序。
- 对“到站 + 每次抽箱后”的每个非空状态建立在场布尔量和重心矩条件，
  施加该站的两倍重心闭区间。

模型仅用整数/布尔变量，区间端点全部整数，求解结果可被 `verify.py`
逐格点复核。
