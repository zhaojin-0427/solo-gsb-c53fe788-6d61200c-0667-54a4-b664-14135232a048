"""FastAPI 入口：多站货车装载规划服务。"""

from __future__ import annotations

import os

from fastapi import FastAPI

from .schemas import PlanRequest, PlanResponse
from .solver import solve_plan

app = FastAPI(
    title="多站货车装载规划 API",
    version="1.0.0",
    description=(
        "给定整数格车厢、站点顺序、箱子尺寸/重量/目的站及各站允许的两倍"
        "纵向重心闭区间，联合求解装载排布与逐站卸货顺序。"
    ),
)

TIME_LIMIT = float(os.getenv("SOLVER_TIME_LIMIT", "20"))


@app.get("/health")
def health() -> dict:
    return {"status": "ok"}


@app.post("/plans", response_model=PlanResponse)
def create_plan(req: PlanRequest) -> dict:
    return solve_plan(req, time_limit=TIME_LIMIT)
