from dataclasses import dataclass

from .calibration import AMAZING_HAND_MOTORS, HandCalibration


@dataclass(frozen=True)
class GripperSynergyMapper:
    calibration: HandCalibration
    leader_open_value: float = 100.0
    leader_closed_value: float = 0.0

    def __post_init__(self) -> None:
        if self.leader_open_value == self.leader_closed_value:
            raise ValueError("leader_open_value and leader_closed_value must differ")

    def closure_from_gripper(self, gripper_position: float) -> float:
        span = self.leader_closed_value - self.leader_open_value
        closure = (float(gripper_position) - self.leader_open_value) / span
        return min(1.0, max(0.0, closure))

    def targets(self, gripper_position: float) -> dict[str, int]:
        closure = self.closure_from_gripper(gripper_position)
        return {
            motor: round(
                self.calibration.open_raw[motor]
                + closure * (self.calibration.closed_raw[motor] - self.calibration.open_raw[motor])
            )
            for motor in AMAZING_HAND_MOTORS
        }

    def motor_closure(self, motor: str, raw_position: int) -> float:
        span = self.calibration.closed_raw[motor] - self.calibration.open_raw[motor]
        closure = (raw_position - self.calibration.open_raw[motor]) / span
        return min(100.0, max(0.0, closure * 100.0))

