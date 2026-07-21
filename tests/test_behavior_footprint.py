from __future__ import annotations

import pytest

from ros_ws.src.b2arx_nav2_bringup.b2arx_nav2_bringup.behavior_footprint_core import (
    validate_flat_footprint,
)


def test_behavior_footprint_accepts_static_b2_rectangle() -> None:
    points = validate_flat_footprint(
        [0.47, 0.31, 0.47, -0.31, -0.47, -0.31, -0.47, 0.31]
    )
    assert points == (
        (0.47, 0.31),
        (0.47, -0.31),
        (-0.47, -0.31),
        (-0.47, 0.31),
    )


@pytest.mark.parametrize(
    "values",
    [
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0, 0.0, 1.0],
        [0.0, 0.0, 1.0, 0.0, float("nan"), 1.0],
    ],
)
def test_behavior_footprint_rejects_invalid_geometry(values) -> None:
    with pytest.raises(ValueError):
        validate_flat_footprint(values)
