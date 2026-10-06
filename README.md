# 多站货车装载规划服务

基于 **Python + FastAPI + OR-Tools (CP-SAT)** 的多站货车装载规划 HTTP API。
给定整数格车厢、站点顺序和一批带目的站的箱子，服务会**联合求解装载排布与逐站卸货顺序**，
使以下约束同时成立：

- **装箱**：箱体整数格放置，可水平旋转（长宽互换），不得相互重叠，不得超出车厢；
- **支撑**：箱底必须由地板（`z=0`）或与箱底等高的箱顶**完整**支撑（不允许悬挑）；
- **卸货通道**：到达某站后，该站箱子只能沿车长方向**直线拉向车尾**（`x=0` 为车尾），
  拉动过程中不得碰撞任何仍在车上的箱子；
- **重心**：到达每站卸货前、以及每抽走一箱之后，剩余箱体必须仍受完整支撑，
  且非空时纵向重心必须落在**该站**给定的区间内（区间以两倍坐标整数闭区间给出）；
- **联合性**：装载排布与卸货顺序在同一个 CP-SAT 模型中联合求解，
  因此返回的方案天然同时满足支撑、抽出通道与重心约束；
  返回前还会用一个独立的校验器（`app/verifier.py`）对方案做全规则复检。

## 快速开始（Docker Compose 一键启动）

```bash
docker compose up --build
```

服务监听 <http://localhost:8000>：

- 交互式 API 文档（Swagger UI）：<http://localhost:8000/docs>
- 健康检查：`GET /health`
- 装载规划：`POST /plans`

可选环境变量（见 `docker-compose.yml`）：

| 变量 | 默认值 | 含义 |
| --- | --- | --- |
| `SOLVER_TIME_LIMIT` | `20` | 单次请求 CP-SAT 求解硬时限（秒） |
| `SOLVER_WORKERS` | `8` | 并行搜索线程数 |
| `SOLVER_GRACE_SECONDS` | `2` | 找到首个可行方案后继续改进的秒数 |

## 本地开发

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt -r requirements-dev.txt
uvicorn app.main:app --reload --port 8000
pytest                 # 运行测试（含对方案的独立全规则校验）
```

## 坐标系与数据约定

- `x` 为纵向（车长方向），**车尾为 `x=0`**，越靠近车头 `x` 越大；`y` 为横向，`z` 为高度；均为整数格。
- 箱子只绕竖直轴旋转：`rotated=false` 时 `dx=length, dy=width`；`rotated=true` 时 `dx=width, dy=length`；`dz=height` 恒定。
- 每站的允许纵向重心区间 `[cog2_min, cog2_max]` 使用**两倍坐标**：
  剩余货物重心 `c = Σ wᵢ·(xᵢ+dxᵢ/2) / Σ wᵢ` 合法当且仅当 `cog2_min ≤ 2·c ≤ cog2_max`。
  用两倍坐标表示后，所有约束都是整数运算，无浮点误差。
- 空车（`Σ wᵢ = 0`）时不检查重心。

## `POST /plans`

### 请求体

| 字段 | 说明 |
| --- | --- |
| `truck.length / width / height` | 车厢整数尺寸（正整数） |
| `stations[]` | 站点，**按到达顺序**给出；`id` 唯一，`cog2_min ≤ cog2_max` 为该站两倍坐标重心区间 |
| `boxes[]` | 箱子；`id` 唯一，`length/width/height/weight` 为正整数，`destination` 必须是已知站点 id |
| `solver_time_limit_seconds` | 可选，覆盖本次请求的求解时限（默认取 `SOLVER_TIME_LIMIT`） |

### 响应

`status` 取值：

- `feasible`：返回 `placements`（每个箱子的格点位置、占格尺寸与朝向）和
  `unload_plan`（逐站卸货顺序），另附 `cog_checks`（到站/每抽一箱后的两倍重心实测值，便于核对）；
- `infeasible`：无可行装载方案（报无解）；
- `unknown`：时限内未能判定（可加大 `solver_time_limit_seconds` 重试）。

非法输入返回 **422**：非正整数/非整数尺寸、重量非正、箱子 id 重复、目的站未知、
站点 id 重复、重心区间上下界颠倒、站点列表为空等。

### 示例 1：可行方案（真实请求与响应）

请求：

```bash
curl -s -X POST http://localhost:8000/plans \
  -H "Content-Type: application/json" \
  -d '{
    "truck": {"length": 12, "width": 6, "height": 5},
    "stations": [
      {"id": "A", "cog2_min": 6, "cog2_max": 16},
      {"id": "B", "cog2_min": 4, "cog2_max": 18}
    ],
    "boxes": [
      {"id": "b1", "length": 3, "width": 2, "height": 2, "weight": 5, "destination": "B"},
      {"id": "b2", "length": 2, "width": 2, "height": 2, "weight": 4, "destination": "A"},
      {"id": "b3", "length": 4, "width": 2, "height": 1, "weight": 6, "destination": "B"},
      {"id": "b4", "length": 2, "width": 3, "height": 2, "weight": 3, "destination": "A"},
      {"id": "b5", "length": 2, "width": 2, "height": 3, "weight": 2, "destination": "B"}
    ]
  }'
```

响应（`200 OK`，以下为服务实际输出；同一请求再次求解可能返回另一个同样合法的计划）：

```json
{
  "status": "feasible",
  "placements": [
    {"box_id": "b1", "x": 2, "y": 0, "z": 0, "dx": 2, "dy": 3, "dz": 2, "rotated": true},
    {"box_id": "b2", "x": 0, "y": 0, "z": 0, "dx": 2, "dy": 2, "dz": 2, "rotated": false},
    {"box_id": "b3", "x": 4, "y": 0, "z": 0, "dx": 4, "dy": 2, "dz": 1, "rotated": false},
    {"box_id": "b4", "x": 0, "y": 4, "z": 0, "dx": 3, "dy": 2, "dz": 2, "rotated": true},
    {"box_id": "b5", "x": 0, "y": 2, "z": 0, "dx": 2, "dy": 2, "dz": 3, "rotated": false}
  ],
  "unload_plan": [
    {"station": "A", "order": ["b2", "b4"]},
    {"station": "B", "order": ["b5", "b1", "b3"]}
  ],
  "cog_checks": [
    {"station": "A", "stage": "arrival", "box_id": null, "cog2": 6.15, "cog2_min": 6, "cog2_max": 16, "ok": true},
    {"station": "A", "stage": "after_pull", "box_id": "b2", "cog2": 7.1875, "cog2_min": 6, "cog2_max": 16, "ok": true},
    {"station": "A", "stage": "after_pull", "box_id": "b4", "cog2": 8.1538, "cog2_min": 6, "cog2_max": 16, "ok": true},
    {"station": "B", "stage": "arrival", "box_id": null, "cog2": 8.1538, "cog2_min": 4, "cog2_max": 18, "ok": true},
    {"station": "B", "stage": "after_pull", "box_id": "b5", "cog2": 9.2727, "cog2_min": 4, "cog2_max": 18, "ok": true},
    {"station": "B", "stage": "after_pull", "box_id": "b1", "cog2": 12.0, "cog2_min": 4, "cog2_max": 18, "ok": true},
    {"station": "B", "stage": "after_pull", "box_id": "b3", "cog2": null, "cog2_min": 4, "cog2_max": 18, "ok": true}
  ],
  "message": null
}
```

解读：A 站先抽 `b2`（`x=0`，最后方）再抽 `b4`；A 站货物卸完后，B 站按
`b5 → b1 → b3` 顺序抽卸（`b5` 在 `x=0` 挡住 `b1` 的抽出通道，必须先卸）。
每个检查点的两倍重心 `cog2` 都落在该站区间内（最后一项 `cog2=null` 表示车已卸空，不检查）。

### 示例 2：无可行方案（报无解）

9 个 `3×3×3` 的箱子放不进 `6×6×6` 的车厢（体积超限）：

```bash
curl -s -X POST http://localhost:8000/plans \
  -H "Content-Type: application/json" \
  -d '{
    "truck": {"length": 6, "width": 6, "height": 6},
    "stations": [{"id": "A", "cog2_min": 0, "cog2_max": 100}],
    "boxes": [
      {"id": "b1", "length": 3, "width": 3, "height": 3, "weight": 2, "destination": "A"},
      {"id": "b2", "length": 3, "width": 3, "height": 3, "weight": 2, "destination": "A"},
      {"id": "b3", "length": 3, "width": 3, "height": 3, "weight": 2, "destination": "A"},
      {"id": "b4", "length": 3, "width": 3, "height": 3, "weight": 2, "destination": "A"},
      {"id": "b5", "length": 3, "width": 3, "height": 3, "weight": 2, "destination": "A"},
      {"id": "b6", "length": 3, "width": 3, "height": 3, "weight": 2, "destination": "A"},
      {"id": "b7", "length": 3, "width": 3, "height": 3, "weight": 2, "destination": "A"},
      {"id": "b8", "length": 3, "width": 3, "height": 3, "weight": 2, "destination": "A"},
      {"id": "b9", "length": 3, "width": 3, "height": 3, "weight": 2, "destination": "A"}
    ]
  }'
```

响应（`200 OK`）：

```json
{
  "status": "infeasible",
  "placements": null,
  "unload_plan": null,
  "cog_checks": [],
  "message": "no loading plan satisfies all support, pull-corridor and COG constraints"
}
```

### 示例 3：非法输入（422）

```bash
# 箱子 id 重复
curl -s -X POST http://localhost:8000/plans -H "Content-Type: application/json" -d '{
  "truck": {"length": 12, "width": 6, "height": 5},
  "stations": [{"id": "A", "cog2_min": 0, "cog2_max": 24}],
  "boxes": [
    {"id": "b1", "length": 2, "width": 2, "height": 2, "weight": 3, "destination": "A"},
    {"id": "b1", "length": 2, "width": 2, "height": 2, "weight": 3, "destination": "A"}
  ]
}'
# => 422 {"detail":"duplicate box id"}

# 目的站未知
curl -s -X POST http://localhost:8000/plans -H "Content-Type: application/json" -d '{
  "truck": {"length": 12, "width": 6, "height": 5},
  "stations": [{"id": "A", "cog2_min": 0, "cog2_max": 24}],
  "boxes": [{"id": "b1", "length": 2, "width": 2, "height": 2, "weight": 3, "destination": "ZZZ"}]
}'
# => 422 {"detail":"box 'b1': unknown destination station 'ZZZ'"}

# 尺寸非法（非正整数）
curl -s -X POST http://localhost:8000/plans -H "Content-Type: application/json" -d '{
  "truck": {"length": 12, "width": 6, "height": 5},
  "stations": [{"id": "A", "cog2_min": 0, "cog2_max": 24}],
  "boxes": [{"id": "b1", "length": 0, "width": 2, "height": 2, "weight": 3, "destination": "A"}]
}'
# => 422 {"detail":[{"type":"greater_than","loc":["body","boxes",0,"length"], ... }]}
```

## 实现要点

```
app/
├── main.py       # FastAPI 入口：请求校验（422）、调用求解器、返回前独立复检
├── models.py     # Pydantic 请求/响应模型
├── solver.py     # OR-Tools CP-SAT 联合模型（排布 + 卸货顺序 + 全部约束）
└── verifier.py   # 独立校验器：对方案逐条重查所有物理规则
tests/
└── test_api.py   # 端到端测试（可行/无解/422 及方案全规则校验）
```

CP-SAT 模型中的关键约束：

- **不重叠**：每对箱子在 x/y/z 三个方向上至少一个方向分离（析取约束）；
- **完整支撑**：对每个箱子，所有"箱顶与其箱底等高"的箱子在其底面上的投影面积之和
  必须等于底面积（`z=0` 的箱子由地板支撑）；
- **支撑先后**：若箱子 j 支撑箱子 i，则 j 的目的站不得早于 i；同站时 i 必须先被抽走
  （否则抽走 j 后 i 会失去支撑）；
- **抽出通道**：若箱子 j 与 i 在 y/z 上投影相交且 j 在 i 的后方，则 j 挡住 i 的直线抽出通道——
  j 的目的站更晚时该排布被禁止，同站时约束 j 先被抽走；
- **重心**：到站时及每抽走一箱后，剩余集合的两倍重心落在这站区间内
  （抽走顺序用每站内部的排序变量建模，与排布联合求解）。

## 测试

```bash
pip install -r requirements-dev.txt
pytest -q
```

测试覆盖：可行方案的独立全规则复检、需要堆叠的场景、体积/重心导致的无解、
以及全部 422 校验分支（重复 id、未知站点、非法尺寸、非法重量、区间颠倒等）。
