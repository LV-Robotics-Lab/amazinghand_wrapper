from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class AmazingHandConfig:
    port: str
    calibration_file: Path
    baudrates: tuple[int, ...] = (1_000_000, 250_000)
    expected_model_numbers: tuple[int, ...] = (1280, 1284)
    motor_ids: tuple[int, ...] = tuple(range(1, 9))
    leader_open_value: float = 100.0
    leader_closed_value: float = 0.0
    max_raw_velocity: float = 240.0
    command_timeout_s: float = 0.25
    max_temperature_c: float | None = 55.0
    max_abs_load: float | None = 900.0
    disable_torque_on_disconnect: bool = True

    def __post_init__(self) -> None:
        object.__setattr__(self, "calibration_file", Path(self.calibration_file))
        if not self.port:
            raise ValueError("port must not be empty")
        if not self.baudrates or any(value <= 0 for value in self.baudrates):
            raise ValueError("baudrates must contain positive values")
        if not self.expected_model_numbers or any(value <= 0 for value in self.expected_model_numbers):
            raise ValueError("expected_model_numbers must contain positive values")
        if len(self.motor_ids) != 8 or len(set(self.motor_ids)) != 8:
            raise ValueError("motor_ids must contain exactly eight unique IDs")
        if any(not 0 <= value <= 253 for value in self.motor_ids):
            raise ValueError("motor IDs must be in [0, 253]")
        if self.leader_open_value == self.leader_closed_value:
            raise ValueError("leader_open_value and leader_closed_value must differ")
        if self.max_raw_velocity <= 0:
            raise ValueError("max_raw_velocity must be positive")
        if self.command_timeout_s <= 0:
            raise ValueError("command_timeout_s must be positive")
        if self.max_temperature_c is not None and self.max_temperature_c <= 0:
            raise ValueError("max_temperature_c must be positive when set")
        if self.max_abs_load is not None and self.max_abs_load <= 0:
            raise ValueError("max_abs_load must be positive when set")

