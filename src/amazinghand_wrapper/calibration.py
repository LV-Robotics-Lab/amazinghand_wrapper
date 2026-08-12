import json
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path

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


@dataclass(frozen=True)
class HandCalibration:
    open_raw: dict[str, int]
    closed_raw: dict[str, int]
    schema: str = "lv_robotics.amazinghand_calibration.v1"

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
    def load(cls, path: Path) -> "HandCalibration":
        payload = json.loads(Path(path).read_text())
        return cls(
            open_raw={key: int(value) for key, value in payload["open_raw"].items()},
            closed_raw={key: int(value) for key, value in payload["closed_raw"].items()},
            schema=payload.get("schema", "lv_robotics.amazinghand_calibration.v1"),
        )

    def save(self, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema": self.schema,
            "open_raw": self.open_raw,
            "closed_raw": self.closed_raw,
        }
        with tempfile.NamedTemporaryFile("w", dir=path.parent, delete=False) as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
            temporary = Path(handle.name)
        os.replace(temporary, path)

