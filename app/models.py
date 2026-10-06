"""Pydantic schemas for the multi-stop truck loading planner API."""
from __future__ import annotations

from typing import Literal, Optional

from pydantic import BaseModel, Field


class TruckSpec(BaseModel):
    """Integer-grid cargo compartment dimensions.

    x is the longitudinal axis; x = 0 is the rear (unloading end) of the truck.
    """

    length: int = Field(gt=0, description="Cargo compartment length along x (rear=0)")
    width: int = Field(gt=0, description="Cargo compartment width along y")
    height: int = Field(gt=0, description="Cargo compartment height along z")


class StationSpec(BaseModel):
    """One stop, in visit order.

    The allowed longitudinal center of gravity while the truck is at this
    station is given as an integer closed interval in *doubled* coordinates:
    a COG of c is allowed iff cog2_min <= 2*c <= cog2_max.
    """

    id: str = Field(min_length=1)
    cog2_min: int = Field(description="Lower bound of 2*(longitudinal COG), inclusive")
    cog2_max: int = Field(description="Upper bound of 2*(longitudinal COG), inclusive")


class BoxSpec(BaseModel):
    id: str = Field(min_length=1)
    length: int = Field(gt=0, description="Box size along x when not rotated")
    width: int = Field(gt=0, description="Box size along y when not rotated")
    height: int = Field(gt=0, description="Box size along z (never rotated)")
    weight: int = Field(gt=0)
    destination: str = Field(description="Id of the station where this box is unloaded")


class PlanRequest(BaseModel):
    truck: TruckSpec
    stations: list[StationSpec] = Field(min_length=1, description="Stops in visit order")
    boxes: list[BoxSpec] = Field(min_length=1)
    solver_time_limit_seconds: Optional[float] = Field(
        default=None,
        gt=0,
        le=300,
        description="Optional override of the CP-SAT time limit for this request",
    )


class Placement(BaseModel):
    box_id: str
    x: int = Field(description="Longitudinal coordinate of the box's rear face (rear of truck = 0)")
    y: int
    z: int
    dx: int = Field(description="Occupied length along x (after rotation)")
    dy: int = Field(description="Occupied width along y (after rotation)")
    dz: int = Field(description="Occupied height along z")
    rotated: bool = Field(
        description="False: length along x, width along y. True: swapped (90-degree horizontal rotation)"
    )


class StationUnload(BaseModel):
    station: str
    order: list[str] = Field(description="Box ids in pull-out order at this station")


class CogCheck(BaseModel):
    station: str
    stage: Literal["arrival", "after_pull"]
    box_id: Optional[str] = Field(default=None, description="Box just pulled (after_pull only)")
    cog2: Optional[float] = Field(
        default=None,
        description="2*(longitudinal COG) of the remaining load; null when the truck is empty",
    )
    cog2_min: int
    cog2_max: int
    ok: bool


class PlanResponse(BaseModel):
    status: Literal["feasible", "infeasible", "unknown"]
    placements: Optional[list[Placement]] = None
    unload_plan: Optional[list[StationUnload]] = None
    cog_checks: list[CogCheck] = []
    message: Optional[str] = None
