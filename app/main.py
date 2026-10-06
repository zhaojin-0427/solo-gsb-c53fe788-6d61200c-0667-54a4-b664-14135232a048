"""HTTP API for the multi-stop truck loading planner."""
from __future__ import annotations

import logging

from fastapi import FastAPI, HTTPException

from .models import PlanRequest, PlanResponse
from .solver import BoxInput, StationInput, solve_loading
from .verifier import verify_plan

logger = logging.getLogger("loading-planner")

app = FastAPI(
    title="Multi-stop truck loading planner",
    version="1.0.0",
    description=(
        "Plans box placements (integer grid, horizontal rotation allowed) and a "
        "per-station unloading sequence so that support, rear pull-out corridors "
        "and per-station center-of-gravity intervals all hold simultaneously."
    ),
)


@app.get("/health")
def health() -> dict:
    return {"status": "ok"}


@app.post("/plans", response_model=PlanResponse)
def create_plan(req: PlanRequest) -> PlanResponse:
    # ---- semantic validation (syntactic validation is handled by pydantic) ----
    station_ids = [s.id for s in req.stations]
    if len(set(station_ids)) != len(station_ids):
        raise HTTPException(status_code=422, detail="duplicate station id")
    for s in req.stations:
        if s.cog2_min > s.cog2_max:
            raise HTTPException(
                status_code=422,
                detail=f"station '{s.id}': cog2_min must not exceed cog2_max",
            )
    box_ids = [b.id for b in req.boxes]
    if len(set(box_ids)) != len(box_ids):
        raise HTTPException(status_code=422, detail="duplicate box id")
    known = set(station_ids)
    for b in req.boxes:
        if b.destination not in known:
            raise HTTPException(
                status_code=422,
                detail=f"box '{b.id}': unknown destination station '{b.destination}'",
            )

    index_of = {sid: k for k, sid in enumerate(station_ids)}
    stations = [StationInput(id=s.id, cog2_min=s.cog2_min, cog2_max=s.cog2_max) for s in req.stations]
    boxes = [
        BoxInput(
            id=b.id,
            length=b.length,
            width=b.width,
            height=b.height,
            weight=b.weight,
            dest=index_of[b.destination],
        )
        for b in req.boxes
    ]

    result = solve_loading(
        truck_length=req.truck.length,
        truck_width=req.truck.width,
        truck_height=req.truck.height,
        stations=stations,
        boxes=boxes,
        time_limit=req.solver_time_limit_seconds,
    )

    if result.status != "feasible":
        return PlanResponse(status=result.status, message=result.message)

    # Safety net: independently re-verify the plan before returning it.
    violations = verify_plan(
        req.truck.length,
        req.truck.width,
        req.truck.height,
        stations,
        boxes,
        result.placements or [],
        result.unload_plan or [],
    )
    if violations:
        logger.error("solver produced an invalid plan: %s", violations)
        raise HTTPException(
            status_code=500,
            detail={"message": "internal error: produced plan failed verification",
                    "violations": violations},
        )

    return PlanResponse(
        status="feasible",
        placements=result.placements,
        unload_plan=result.unload_plan,
        cog_checks=result.cog_checks,
    )
