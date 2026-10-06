"""POST /plans 的请求/响应模型与输入校验。"""

from __future__ import annotations

from typing import Dict, List

from pydantic import BaseModel, Field, model_validator


class CgInterval(BaseModel):
    """单站允许的纵向重心区间，以“两倍坐标”给出的整数闭区间 [lo, hi]。

    若装载内各箱重心的两倍坐标为 2x_i+xsi_i，则要求
        lo * W_total <= Σ w_i(2x_i+xsi_i) <= hi * W_total。
    """

    lo: int = Field(..., description="允许的两倍重心下界（含）")
    hi: int = Field(..., description="允许的两倍重心上界（含）")

    @model_validator(mode="after")
    def _check_order(self):
        if self.lo > self.hi:
            raise ValueError("cg 区间必须满足 lo <= hi")
        return self


class BoxIn(BaseModel):
    box_id: str = Field(..., min_length=1, description="全请求内唯一的箱 ID")
    length: int = Field(..., gt=0, description="沿车长 x 方向的原始边长（格）")
    width: int = Field(..., gt=0, description="沿车宽 y 方向的原始边长（格）")
    height: int = Field(..., gt=0, description="沿竖直 z 方向的边长（格）")
    weight: int = Field(..., gt=0, description="重量（正整数）")
    destination: str = Field(..., min_length=1, description="目的站，须出现在 stops 中")


class PlanRequest(BaseModel):
    carriage_length: int = Field(..., gt=0, description="车厢长（x 方向格数）")
    carriage_width: int = Field(..., gt=0, description="车厢宽（y 方向格数）")
    carriage_height: int = Field(..., gt=0, description="车厢高（z 方向格数）")
    stops: List[str] = Field(..., min_length=1, description="站点顺序，车尾卸货的先后次序")
    cg_limits: Dict[str, CgInterval] = Field(
        ..., description="各站允许的两倍纵向重心闭区间，键为站点名"
    )
    boxes: List[BoxIn] = Field(..., min_length=1, description="待装箱列表")

    @model_validator(mode="after")
    def _validate_request(self):
        if not self.stops:
            raise ValueError("stops 不能为空")
        if not self.boxes:
            raise ValueError("boxes 不能为空")
        # 站点重复
        if len(set(self.stops)) != len(self.stops):
            raise ValueError("stops 中存在重复站点")
        known = set(self.stops)

        # cg_limits 与 stops 一一对应
        if set(self.cg_limits.keys()) != known:
            missing = sorted(known - set(self.cg_limits.keys()))
            extra = sorted(set(self.cg_limits.keys()) - known)
            details = []
            if missing:
                details.append(f"缺少站点的 cg_limits: {missing}")
            if extra:
                details.append(f"cg_limits 含未知站点: {extra}")
            raise ValueError("；".join(details))

        # 箱 ID 重复
        ids = [b.box_id for b in self.boxes]
        if len(set(ids)) != len(ids):
            dup = sorted({x for x in ids if ids.count(x) > 1})
            raise ValueError(f"box_id 重复: {dup}")

        # 未知目的站
        bad_dest = sorted({b.destination for b in self.boxes if b.destination not in known})
        if bad_dest:
            raise ValueError(f"箱子目的站未知: {bad_dest}")

        # 箱体任何旋转都放不下
        fits_plain = (
            lambda b: b.length <= self.carriage_length
            and b.width <= self.carriage_width
        )
        fits_rotated = (
            lambda b: b.width <= self.carriage_length
            and b.length <= self.carriage_width
        )
        too_big = [
            b.box_id
            for b in self.boxes
            if b.height > self.carriage_height
            or not (fits_plain(b) or fits_rotated(b))
        ]
        if too_big:
            raise ValueError(f"箱体即使旋转也超出车厢尺寸: {too_big}")
        return self


class Placement(BaseModel):
    box_id: str
    rotated: bool = Field(..., description="是否相对原始长宽做了水平 90° 旋转")
    x: List[int] = Field(..., min_length=2, max_length=2, description="x 方向整数闭区间 [起, 止]，车尾为 0")
    y: List[int] = Field(..., min_length=2, max_length=2, description="y 方向整数闭区间 [起, 止]")
    z: List[int] = Field(..., min_length=2, max_length=2, description="z 方向整数闭区间 [起, 止]，地板为 0")


class StopUnload(BaseModel):
    stop: str
    order: List[str] = Field(..., description="本站抽箱顺序（先 → 后），均沿车长方向直线拉向车尾")


class PlanResponse(BaseModel):
    feasible: bool
    status: str = Field(..., description="optimal / feasible / infeasible / timeout")
    carriage: Dict[str, int]
    stops: List[str]
    placements: List[Placement]
    unloading: List[StopUnload]
    removal_order: List[str]
    solver_time_seconds: float
