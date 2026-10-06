"""Independent plan verifier.

Re-checks a produced plan against every rule of the problem statement. Used
as a safety net inside the API and by the test-suite. Pure Python, no solver
involved, so a modelling bug in the CP-SAT layer cannot silently pass.
"""
from __future__ import annotations

from .solver import BoxInput, StationInput


def _overlap1d(a_lo: int, a_hi: int, b_lo: int, b_hi: int) -> int:
    return max(0, min(a_hi, b_hi) - max(a_lo, b_lo))


def verify_plan(
    truck_length: int,
    truck_width: int,
    truck_height: int,
    stations: list[StationInput],
    boxes: list[BoxInput],
    placements: list[dict],
    unload_plan: list[dict],
) -> list[str]:
    """Return a list of human-readable violations; empty means fully valid."""
    errors: list[str] = []
    L, W, H = truck_length, truck_width, truck_height
    box_by_id = {b.id: b for b in boxes}

    # ---- placements cover every box exactly once -------------------------
    seen = set()
    pos = {}
    for p in placements:
        bid = p["box_id"]
        if bid not in box_by_id:
            errors.append(f"placement for unknown box '{bid}'")
            continue
        if bid in seen:
            errors.append(f"box '{bid}' placed more than once")
            continue
        seen.add(bid)
        b = box_by_id[bid]
        dims = (p["dx"], p["dy"], p["dz"])
        if dims not in ((b.length, b.width, b.height), (b.width, b.length, b.height)):
            errors.append(f"box '{bid}': placed dims {dims} inconsistent with spec "
                          f"({b.length}x{b.width}x{b.height})")
            continue
        if b.length != b.width:
            # rotated flag must reflect the horizontal swap when it matters
            if p["rotated"] != (dims[0] == b.width):
                errors.append(f"box '{bid}': rotated flag inconsistent with dims")
        if not (0 <= p["x"] and p["x"] + p["dx"] <= L):
            errors.append(f"box '{bid}': x-range out of truck")
        if not (0 <= p["y"] and p["y"] + p["dy"] <= W):
            errors.append(f"box '{bid}': y-range out of truck")
        if not (0 <= p["z"] and p["z"] + p["dz"] <= H):
            errors.append(f"box '{bid}': z-range out of truck")
        pos[bid] = p
    missing = set(box_by_id) - seen
    if missing:
        errors.append(f"boxes not placed: {sorted(missing)}")
        return errors  # further checks need a full placement

    # ---- pairwise non-overlap --------------------------------------------
    ids = sorted(pos)
    for a_i in range(len(ids)):
        for b_i in range(a_i + 1, len(ids)):
            pa, pb = pos[ids[a_i]], pos[ids[b_i]]
            if (
                _overlap1d(pa["x"], pa["x"] + pa["dx"], pb["x"], pb["x"] + pb["dx"]) > 0
                and _overlap1d(pa["y"], pa["y"] + pa["dy"], pb["y"], pb["y"] + pb["dy"]) > 0
                and _overlap1d(pa["z"], pa["z"] + pa["dz"], pb["z"], pb["z"] + pb["dz"]) > 0
            ):
                errors.append(f"boxes '{ids[a_i]}' and '{ids[b_i]}' overlap")

    def support_errors(present: set[str]) -> list[str]:
        errs = []
        for bid in present:
            p = pos[bid]
            if p["z"] == 0:
                continue
            covered = 0
            for other in present:
                if other == bid:
                    continue
                q = pos[other]
                if q["z"] + q["dz"] != p["z"]:
                    continue
                covered += _overlap1d(p["x"], p["x"] + p["dx"], q["x"], q["x"] + q["dx"]) * \
                    _overlap1d(p["y"], p["y"] + p["dy"], q["y"], q["y"] + q["dy"])
            if covered != p["dx"] * p["dy"]:
                errs.append(f"box '{bid}' is not fully supported "
                            f"({covered}/{p['dx'] * p['dy']} of its bottom covered)")
        return errs

    errors.extend(support_errors(set(ids)))

    # ---- unload plan structure -------------------------------------------
    if [e["station"] for e in unload_plan] != [s.id for s in stations]:
        errors.append("unload_plan stations do not match the requested station order")
        return errors
    station_of = {b.id: b.dest for b in boxes}
    pulled: list[str] = []
    for k, entry in enumerate(unload_plan):
        for bid in entry["order"]:
            if bid not in box_by_id:
                errors.append(f"unload plan references unknown box '{bid}'")
            elif station_of[bid] != k:
                errors.append(f"box '{bid}' unloaded at wrong station '{entry['station']}'")
            pulled.append(bid)
    if sorted(pulled) != sorted(box_by_id):
        errors.append("unload plan does not unload every box exactly once")
        return errors

    # ---- simulate the unloading ------------------------------------------
    def cog2_ok(present: set[str], k: int) -> bool:
        w_sum = sum(box_by_id[i].weight for i in present)
        if w_sum == 0:
            return True  # empty truck: no constraint
        s_sum = sum(box_by_id[i].weight * (2 * pos[i]["x"] + pos[i]["dx"]) for i in present)
        return stations[k].cog2_min * w_sum <= s_sum <= stations[k].cog2_max * w_sum

    present = set(ids)
    for k, entry in enumerate(unload_plan):
        if not cog2_ok(present, k):
            errors.append(f"station '{entry['station']}': COG out of interval before unloading")
        for bid in entry["order"]:
            if bid not in present:
                errors.append(f"box '{bid}' pulled twice or not present")
                continue
            p = pos[bid]
            # swept corridor towards the rear must be clear
            for other in present:
                if other == bid:
                    continue
                q = pos[other]
                if (
                    q["x"] < p["x"] + p["dx"]
                    and _overlap1d(p["y"], p["y"] + p["dy"], q["y"], q["y"] + q["dy"]) > 0
                    and _overlap1d(p["z"], p["z"] + p["dz"], q["z"], q["z"] + q["dz"]) > 0
                ):
                    errors.append(f"pulling '{bid}' at station '{entry['station']}' "
                                  f"collides with '{other}'")
            present.discard(bid)
            errors.extend(support_errors(present))
            if not cog2_ok(present, k):
                errors.append(f"station '{entry['station']}': COG out of interval "
                              f"after pulling '{bid}'")
    if present:
        errors.append(f"boxes left in truck after last station: {sorted(present)}")

    return errors
