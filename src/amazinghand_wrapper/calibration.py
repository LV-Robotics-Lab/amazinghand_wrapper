import ast
import json
import math
import os
import tempfile
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

AMAZING_HAND_MOTORS = (
    "index_1",
    "index_2",
    "middle_1",
    "middle_2",
    "ring_1",
    "ring_2",
    "thumb_1",
    "thumb_2",
)

# Pollen's reference AmazingHand demo defines a calibrated middle position for every
# SCS0009 and applies these deltas for its OpenHand/CloseHand poses.
POLLEN_OPEN_DELTAS_DEGREES = (-35.0, 35.0) * 4
POLLEN_CLOSED_DELTAS_DEGREES = (90.0, -90.0) * 4


def pollen_scs0009_degrees_to_raw(degrees: float) -> int:
    """Match rustypot's SCS0009 AnglePosition::to_raw conversion."""
    degrees = float(degrees)
    if not math.isfinite(degrees):
        raise ValueError("Pollen calibration angles must be finite")
    raw = int(1024.0 * degrees / 300.0 + 511.0)
    if not 0 <= raw <= 1023:
        raise ValueError(f"Pollen calibration angle {degrees} degrees maps outside [0, 1023]")
    return raw


def load_pollen_middle_positions(path: Path, variable: str) -> tuple[float, ...]:
    """Read a MiddlePos list from a Pollen Python demo without executing the file."""
    path = Path(path)
    tree = ast.parse(path.read_text(), filename=str(path))
    value: Any | None = None
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == variable for target in node.targets
        ):
            value = ast.literal_eval(node.value)
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            if node.target.id == variable and node.value is not None:
                value = ast.literal_eval(node.value)
    if value is None:
        raise ValueError(f"{variable!r} was not found in {path}")
    if not isinstance(value, (list, tuple)):
        raise ValueError(f"{variable!r} must be a list or tuple")
    return _validate_middle_positions(value)


def _validate_middle_positions(values: Sequence[float]) -> tuple[float, ...]:
    if len(values) != len(AMAZING_HAND_MOTORS):
        raise ValueError("Pollen MiddlePos must contain exactly eight values")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError("Pollen MiddlePos values must be finite")
    return result


@dataclass(frozen=True)
class HandCalibration:
    open_raw: dict[str, int]
    closed_raw: dict[str, int]
    schema: str = "lv_robotics.amazinghand_calibration.v1"
    provenance: dict[str, Any] | None = None

    def __post_init__(self) -> None:
        expected = set(AMAZING_HAND_MOTORS)
        if set(self.open_raw) != expected or set(self.closed_raw) != expected:
            raise ValueError(f"calibration must define exactly these motors: {AMAZING_HAND_MOTORS}")
        for name in AMAZING_HAND_MOTORS:
            open_value = self.open_raw[name]
            closed_value = self.closed_raw[name]
            if not 0 <= open_value <= 1023 or not 0 <= closed_value <= 1023:
                raise ValueError(f"raw calibration for {name} must be in [0, 1023]")
            if open_value == closed_value:
                raise ValueError(f"open and closed calibration are identical for {name}")

    @classmethod
    def from_motor_ids(
        cls,
        motor_ids: tuple[int, ...],
        open_by_id: dict[int, int],
        closed_by_id: dict[int, int],
    ) -> "HandCalibration":
        if set(open_by_id) != set(motor_ids) or set(closed_by_id) != set(motor_ids):
            raise ValueError("calibration readings do not match configured motor IDs")
        return cls(
            open_raw=dict(zip(AMAZING_HAND_MOTORS, (open_by_id[value] for value in motor_ids), strict=True)),
            closed_raw=dict(
                zip(AMAZING_HAND_MOTORS, (closed_by_id[value] for value in motor_ids), strict=True)
            ),
        )

    @classmethod
    def from_pollen_middle_positions(
        cls,
        middle_degrees: Sequence[float],
        *,
        source: str | None = None,
        variable: str | None = None,
    ) -> "HandCalibration":
        """Convert Pollen's calibrated MiddlePos values into wrapper endpoints."""
        middle = _validate_middle_positions(middle_degrees)
        open_raw = {
            name: pollen_scs0009_degrees_to_raw(center + delta)
            for name, center, delta in zip(
                AMAZING_HAND_MOTORS, middle, POLLEN_OPEN_DELTAS_DEGREES, strict=True
            )
        }
        closed_raw = {
            name: pollen_scs0009_degrees_to_raw(center + delta)
            for name, center, delta in zip(
                AMAZING_HAND_MOTORS, middle, POLLEN_CLOSED_DELTAS_DEGREES, strict=True
            )
        }
        provenance: dict[str, Any] = {
            "source_format": "pollen_robotics.amazinghand_middle_positions.v1",
            "middle_degrees": list(middle),
            "open_deltas_degrees": list(POLLEN_OPEN_DELTAS_DEGREES),
            "closed_deltas_degrees": list(POLLEN_CLOSED_DELTAS_DEGREES),
            "conversion": "int(1024 * degrees / 300 + 511)",
        }
        if source is not None:
            provenance["source"] = source
        if variable is not None:
            provenance["variable"] = variable
        return cls(open_raw=open_raw, closed_raw=closed_raw, provenance=provenance)

    @classmethod
    def load(cls, path: Path) -> "HandCalibration":
        payload = json.loads(Path(path).read_text())
        return cls(
            open_raw={key: int(value) for key, value in payload["open_raw"].items()},
            closed_raw={key: int(value) for key, value in payload["closed_raw"].items()},
            schema=payload.get("schema", "lv_robotics.amazinghand_calibration.v1"),
            provenance=payload.get("provenance"),
        )

    def save(self, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema": self.schema,
            "open_raw": self.open_raw,
            "closed_raw": self.closed_raw,
        }
        if self.provenance is not None:
            payload["provenance"] = self.provenance
        with tempfile.NamedTemporaryFile("w", dir=path.parent, delete=False) as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
            temporary = Path(handle.name)
        os.replace(temporary, path)
