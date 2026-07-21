from __future__ import annotations

import math
from collections.abc import Iterable


def validate_flat_footprint(values: Iterable[float]) -> tuple[tuple[float, float], ...]:
    """Validate a flat x/y list and return immutable footprint points."""

    flat = tuple(float(value) for value in values)
    if len(flat) < 6 or len(flat) % 2 != 0:
        raise ValueError(
            "behavior footprint must contain at least three x/y pairs, "
            f"got {len(flat)} values"
        )
    if not all(math.isfinite(value) for value in flat):
        raise ValueError("behavior footprint values must all be finite")
    return tuple(zip(flat[0::2], flat[1::2], strict=True))
