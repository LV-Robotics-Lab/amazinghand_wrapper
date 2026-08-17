from typing import Any

from .calibration import AMAZING_HAND_MOTORS


class LeRobotFeetechBackend:
    """LeRobot Feetech adapter imported lazily to keep the core package backend-neutral."""

    def __init__(self, motor_ids: tuple[int, ...] = tuple(range(1, 9))) -> None:
        if len(motor_ids) != len(AMAZING_HAND_MOTORS):
            raise ValueError("AmazingHand requires eight motor IDs")
        self.motor_ids = motor_ids
        self._names_by_id = dict(zip(motor_ids, AMAZING_HAND_MOTORS, strict=True))
        self.bus: Any | None = None

    def connect(self, port: str, baudrate: int) -> None:
        from lerobot.motors import Motor, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus

        self.bus = FeetechMotorsBus(
            port=port,
            motors={
                name: Motor(motor_id, "scs0009", MotorNormMode.RANGE_0_100)
                for motor_id, name in self._names_by_id.items()
            },
            protocol_version=1,
        )
        self.bus.connect(handshake=False)
        self.bus.set_baudrate(baudrate)

    def _require_bus(self) -> Any:
        if self.bus is None:
            raise RuntimeError("backend is not connected")
        return self.bus

    def disconnect(self) -> None:
        if self.bus is not None:
            self.bus.disconnect(disable_torque=False)
            self.bus = None

    def ping(self, motor_id: int) -> int | None:
        return self._require_bus().ping(motor_id)

    def set_torque(self, enabled: bool) -> None:
        bus = self._require_bus()
        if enabled:
            bus.enable_torque()
        else:
            bus.disable_torque()

    def read_positions(self) -> dict[int, int]:
        bus = self._require_bus()
        return {
            motor_id: int(bus.read("Present_Position", name, normalize=False))
            for motor_id, name in self._names_by_id.items()
        }

    def write_positions(self, positions: dict[int, int]) -> None:
        bus = self._require_bus()
        bus.sync_write(
            "Goal_Position",
            {self._names_by_id[motor_id]: value for motor_id, value in positions.items()},
            normalize=False,
        )

    def latch_current_position(self) -> dict[int, int]:
        current = self.read_positions()
        if set(current) != set(self.motor_ids):
            raise RuntimeError("position read does not match configured AmazingHand motor IDs")
        self.write_positions(current)
        return dict(current)

    def _read_optional(self, register: str) -> dict[int, float] | None:
        bus = self._require_bus()
        try:
            return {
                motor_id: float(bus.read(register, name, normalize=False))
                for motor_id, name in self._names_by_id.items()
            }
        except (KeyError, NotImplementedError):
            return None

    def read_temperatures(self) -> dict[int, float] | None:
        return self._read_optional("Present_Temperature")

    def read_loads(self) -> dict[int, float] | None:
        return self._read_optional("Present_Load")
