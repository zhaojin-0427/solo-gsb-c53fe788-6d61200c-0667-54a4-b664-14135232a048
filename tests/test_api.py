"""End-to-end tests for the multi-stop truck loading planner API."""
from __future__ import annotations

import pytest
from fastapi.testclient import TestClient

from app.main import app
from app.solver import BoxInput, StationInput
from app.verifier import verify_plan

client = TestClient(app)


def _request(**overrides):
    req = {
        "truck": {"length": 12, "width": 6, "height": 5},
        "stations": [
            {"id": "A", "cog2_min": 6, "cog2_max": 16},
            {"id": "B", "cog2_min": 4, "cog2_max": 18},
        ],
        "boxes": [
            {"id": "b1", "length": 3, "width": 2, "height": 2, "weight": 5, "destination": "B"},
            {"id": "b2", "length": 2, "width": 2, "height": 2, "weight": 4, "destination": "A"},
            {"id": "b3", "length": 4, "width": 2, "height": 1, "weight": 6, "destination": "B"},
            {"id": "b4", "length": 2, "width": 3, "height": 2, "weight": 3, "destination": "A"},
            {"id": "b5", "length": 2, "width": 2, "height": 3, "weight": 2, "destination": "B"},
        ],
    }
    req.update(overrides)
    return req


def test_health():
    r = client.get("/health")
    assert r.status_code == 200
    assert r.json()["status"] == "ok"


def test_feasible_plan_is_returned_and_verifies():
    r = client.post("/plans", json=_request())
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["status"] == "feasible"

    # every box placed exactly once, with consistent orientation
    assert sorted(p["box_id"] for p in body["placements"]) == ["b1", "b2", "b3", "b4", "b5"]
    for p in body["placements"]:
        assert p["dz"] > 0 and p["dx"] > 0 and p["dy"] > 0

    # unload plan covers all stations in order and every box exactly once
    assert [u["station"] for u in body["unload_plan"]] == ["A", "B"]
    pulled = [bid for u in body["unload_plan"] for bid in u["order"]]
    assert sorted(pulled) == ["b1", "b2", "b3", "b4", "b5"]

    # COG trace is consistent
    assert all(c["ok"] for c in body["cog_checks"])

    # independent re-verification of all physical rules
    req = _request()
    stations = [StationInput(s["id"], s["cog2_min"], s["cog2_max"]) for s in req["stations"]]
    dest = {s.id: k for k, s in enumerate(stations)}
    boxes = [
        BoxInput(b["id"], b["length"], b["width"], b["height"], b["weight"], dest[b["destination"]])
        for b in req["boxes"]
    ]
    errors = verify_plan(12, 6, 5, stations, boxes, body["placements"], body["unload_plan"])
    assert errors == []


def test_stacking_and_support_rules():
    # 5 boxes of 2x2x2 in a 4x4x4 truck: one must be stacked on another.
    req = {
        "truck": {"length": 4, "width": 4, "height": 4},
        "stations": [
            {"id": "A", "cog2_min": 4, "cog2_max": 20},
            {"id": "B", "cog2_min": 2, "cog2_max": 22},
        ],
        "boxes": [
            {"id": "a1", "length": 2, "width": 2, "height": 2, "weight": 4, "destination": "A"},
            {"id": "a2", "length": 2, "width": 2, "height": 2, "weight": 3, "destination": "A"},
            {"id": "b1", "length": 2, "width": 2, "height": 2, "weight": 5, "destination": "B"},
            {"id": "b2", "length": 2, "width": 2, "height": 2, "weight": 2, "destination": "B"},
            {"id": "b3", "length": 2, "width": 2, "height": 2, "weight": 3, "destination": "B"},
        ],
    }
    r = client.post("/plans", json=req)
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["status"] == "feasible"
    assert any(p["z"] > 0 for p in body["placements"])  # stacking happened
    stations = [StationInput(s["id"], s["cog2_min"], s["cog2_max"]) for s in req["stations"]]
    dest = {s.id: k for k, s in enumerate(stations)}
    boxes = [
        BoxInput(b["id"], b["length"], b["width"], b["height"], b["weight"], dest[b["destination"]])
        for b in req["boxes"]
    ]
    assert verify_plan(4, 4, 4, stations, boxes, body["placements"], body["unload_plan"]) == []


def test_infeasible_box_too_large():
    req = _request()
    req["boxes"] = [
        {"id": "huge", "length": 99, "width": 2, "height": 2, "weight": 1, "destination": "A"}
    ]
    r = client.post("/plans", json=req)
    assert r.status_code == 200
    assert r.json()["status"] == "infeasible"


def test_infeasible_cog_interval():
    # A single heavy box in a long truck can never bring the COG far enough
    # towards the front: max doubled COG is 2*(dx/2)=2 at x=0 ... 2*L-? bounded
    # by the box sitting at the very front: 2*(18)+2 = 38 < 40.
    req = {
        "truck": {"length": 20, "width": 4, "height": 4},
        "stations": [{"id": "A", "cog2_min": 40, "cog2_max": 60}],
        "boxes": [
            {"id": "h", "length": 2, "width": 2, "height": 2, "weight": 10, "destination": "A"}
        ],
    }
    r = client.post("/plans", json=req)
    assert r.status_code == 200
    assert r.json()["status"] == "infeasible"


def test_infeasible_volume():
    req = {
        "truck": {"length": 6, "width": 6, "height": 6},
        "stations": [{"id": "A", "cog2_min": 0, "cog2_max": 100}],
        "boxes": [
            {"id": f"b{i}", "length": 3, "width": 3, "height": 3, "weight": 2, "destination": "A"}
            for i in range(9)
        ],
    }
    r = client.post("/plans", json=req)
    assert r.status_code == 200
    assert r.json()["status"] == "infeasible"


# ------------------------- validation (HTTP 422) -------------------------


def test_422_on_duplicate_box_id():
    req = _request()
    req["boxes"][1]["id"] = "b1"
    r = client.post("/plans", json=req)
    assert r.status_code == 422
    assert "duplicate box id" in r.text


def test_422_on_unknown_destination():
    req = _request()
    req["boxes"][0]["destination"] = "NOWHERE"
    r = client.post("/plans", json=req)
    assert r.status_code == 422
    assert "unknown destination" in r.text


def test_422_on_duplicate_station_id():
    req = _request()
    req["stations"][1]["id"] = "A"
    r = client.post("/plans", json=req)
    assert r.status_code == 422
    assert "duplicate station id" in r.text


@pytest.mark.parametrize(
    "field,value",
    [("length", 0), ("length", -3), ("width", 0), ("height", -1)],
)
def test_422_on_illegal_box_dimensions(field, value):
    req = _request()
    req["boxes"][0][field] = value
    r = client.post("/plans", json=req)
    assert r.status_code == 422


@pytest.mark.parametrize("field", ["length", "width", "height"])
def test_422_on_illegal_truck_dimensions(field):
    req = _request()
    req["truck"][field] = 0
    r = client.post("/plans", json=req)
    assert r.status_code == 422


def test_422_on_non_integer_dimension():
    req = _request()
    req["boxes"][0]["length"] = 2.5
    r = client.post("/plans", json=req)
    assert r.status_code == 422


def test_422_on_non_positive_weight():
    req = _request()
    req["boxes"][0]["weight"] = 0
    r = client.post("/plans", json=req)
    assert r.status_code == 422


def test_422_on_inverted_cog_interval():
    req = _request()
    req["stations"][0]["cog2_min"] = 20
    req["stations"][0]["cog2_max"] = 10
    r = client.post("/plans", json=req)
    assert r.status_code == 422
    assert "cog2_min" in r.text


def test_422_on_empty_stations():
    req = _request()
    req["stations"] = []
    r = client.post("/plans", json=req)
    assert r.status_code == 422
