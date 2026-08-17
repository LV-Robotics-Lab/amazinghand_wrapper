import time
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum

from .backend import AmazingHandBackend
from .calibration import AMAZING_HAND_MOTORS, HandCalibration
from .config import AmazingHandConfig
from .synergy import GripperSynergyMapper


class HandState(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    ACTIVE = "active"
    FAULT = "fault"


@dataclass(frozen=True)
class HandObservation:
    raw_positions: dict[str, int]
    motor_closure: dict[str, float]
    grasp_closure: float
    temperatures: dict[str, float] | None
    loads: dict[str, float] | None


class AmazingHandController:
    def __init__(
        self,
        config: AmazingHandConfig,
        backend: AmazingHandBackend,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = config
        self.backend = backend
        self.clock = clock
        self.state = HandState.DISCONNECTED
        self.selected_baudrate: int | None = None
        self.detected_models: dict[int, int] = {}
        self.fault_reason: str | None = None
        self.calibration: HandCalibration | None = None
        self.mapper: GripperSynergyMapper | None = None
        self._last_raw: dict[int, int] | None = None
        self._last_command_time: float | None = None
        self._load_calibration_if_present()

    @property
    def is_connected(self) -> bool:
        return self.state in {HandState.CONNECTED, HandState.ACTIVE}

    @property
    def is_calibrated(self) -> bool:
        return self.mapper is not None

    @property
    def is_active(self) -> bool:
        return self.state is HandState.ACTIVE

    def _load_calibration_if_present(self) -> None:
        if self.config.calibration_file.is_file():
            self.set_calibration(HandCalibration.load(self.config.calibration_file), persist=False)

    def set_calibration(self, calibration: HandCalibration, *, persist: bool = True) -> None:
        self.calibration = calibration
        self.mapper = GripperSynergyMapper(
            calibration,
            leader_open_value=self.config.leader_open_value,
            leader_closed_value=self.config.leader_closed_value,
        )
        if persist:
            calibration.save(self.config.calibration_file)

    def connect(self) -> int:
        if self.state is not HandState.DISCONNECTED:
            raise RuntimeError(f"cannot connect from state {self.state.value}")
        errors: list[str] = []
        for baudrate in self.config.baudrates:
            try:
                self.backend.connect(self.config.port, baudrate)
                detected: dict[int, int] = {}
                for motor_id in self.config.motor_ids:
                    model = self.backend.ping(motor_id)
                    if model is None:
                        raise ConnectionError(f"motor ID {motor_id} did not respond")
                    if model not in self.config.expected_model_numbers:
                        raise ConnectionError(
                            f"motor ID {motor_id} reported model {model}; expected one of "
                            f"{self.config.expected_model_numbers}"
                        )
                    detected[motor_id] = model
                self.backend.set_torque(False)
                self.detected_models = detected
                self.selected_baudrate = baudrate
                self.state = HandState.CONNECTED
                return baudrate
            except Exception as error:
                errors.append(f"{baudrate}: {error}")
                try:
                    self.backend.set_torque(False)
                except Exception:
                    pass
                try:
                    self.backend.disconnect()
                except Exception:
                    pass
        raise ConnectionError("AmazingHand probe failed at every baudrate: " + "; ".join(errors))

    def record_calibration(self, open_by_id: dict[int, int], closed_by_id: dict[int, int]) -> None:
        if not self.is_connected:
            raise RuntimeError("connect before recording calibration")
        self.backend.set_torque(False)
        calibration = HandCalibration.from_motor_ids(self.config.motor_ids, open_by_id, closed_by_id)
        self.set_calibration(calibration)

    def calibrate_interactive(self) -> None:
        if not self.is_connected:
            raise RuntimeError("connect before calibration")
        self.backend.set_torque(False)
        input("Place the AmazingHand in its safe fully OPEN pose, then press ENTER...")
        open_by_id = self.backend.read_positions()
        input("Place the AmazingHand in its safe fully CLOSED pose, then press ENTER...")
        closed_by_id = self.backend.read_positions()
        self.record_calibration(open_by_id, closed_by_id)

    def activate(self) -> None:
        if self.state is not HandState.CONNECTED:
            raise RuntimeError(f"cannot activate from state {self.state.value}")
        if self.mapper is None:
            raise RuntimeError("AmazingHand must be calibrated before activation")
        # Refresh the torque-off goal immediately before every enable. A prior
        # composite-level latch proves ordering across devices, but the hand may
        # still have been moved manually in the intervening interval.
        self.latch_current_position()
        self._check_health()
        self.backend.set_torque(True)
        self._last_command_time = self.clock()
        self.state = HandState.ACTIVE

    def latch_current_position(self) -> dict[int, int]:
        """Write the measured pose back as the goal while torque remains off.

        Composite robots call this public operation before enabling any actuator.
        It uses only the :class:`AmazingHandBackend` protocol, verifies all eight
        configured motor IDs, and fails closed on any read or write error.
        """

        if self.state is not HandState.CONNECTED:
            raise RuntimeError(
                "latch_current_position requires a connected, torque-off AmazingHand"
            )
        try:
            current = self.backend.latch_current_position()
            self._assert_complete_ids(current)
            self._last_raw = dict(current)
            return dict(current)
        except Exception as error:
            self.emergency_stop(f"goal latch failure: {error}")
            raise

    def command_grasp(
        self, gripper_position: float, *, command_timestamp: float | None = None
    ) -> dict[int, int]:
        if self.state is not HandState.ACTIVE or self.mapper is None or self._last_raw is None:
            raise RuntimeError("AmazingHand is not active and calibrated")
        now = self.clock()
        if command_timestamp is not None and now - command_timestamp > self.config.command_timeout_s:
            self.emergency_stop("stale command")
            raise RuntimeError("rejected stale AmazingHand command")
        try:
            self._check_health()
            desired_by_name = self.mapper.targets(gripper_position)
            previous_time = self._last_command_time if self._last_command_time is not None else now
            elapsed = max(0.001, now - previous_time)
            max_step = max(1, round(self.config.max_raw_velocity * elapsed))
            desired_by_id = dict(
                zip(
                    self.config.motor_ids,
                    (desired_by_name[name] for name in AMAZING_HAND_MOTORS),
                    strict=True,
                )
            )
            limited = {
                motor_id: min(
                    self._last_raw[motor_id] + max_step,
                    max(self._last_raw[motor_id] - max_step, target),
                )
                for motor_id, target in desired_by_id.items()
            }
            self.backend.write_positions(limited)
            self._last_raw = limited
            self._last_command_time = now
            return limited
        except Exception as error:
            if self.state is not HandState.FAULT:
                self.emergency_stop(f"command failure: {error}")
            raise

    def observe(self) -> HandObservation:
        if not self.is_connected or self.mapper is None:
            raise RuntimeError("AmazingHand is not connected and calibrated")
        try:
            raw_by_id = self.backend.read_positions()
            self._assert_complete_ids(raw_by_id)
            temperatures_by_id, loads_by_id = self._check_health()
        except Exception as error:
            self.emergency_stop(f"observation failure: {error}")
            raise
        raw_by_name = dict(
            zip(AMAZING_HAND_MOTORS, (raw_by_id[value] for value in self.config.motor_ids), strict=True)
        )
        closure = {
            name: self.mapper.motor_closure(name, raw_by_name[name]) for name in AMAZING_HAND_MOTORS
        }
        return HandObservation(
            raw_positions=raw_by_name,
            motor_closure=closure,
            grasp_closure=sum(closure.values()) / len(closure),
            temperatures=self._values_by_name(temperatures_by_id),
            loads=self._values_by_name(loads_by_id),
        )

    def _check_health(self) -> tuple[dict[int, float] | None, dict[int, float] | None]:
        temperatures = self.backend.read_temperatures()
        loads = self.backend.read_loads()
        if self.config.max_temperature_c is not None and temperatures:
            hot = {
                motor_id: value
                for motor_id, value in temperatures.items()
                if value > self.config.max_temperature_c
            }
            if hot:
                raise RuntimeError(f"temperature limit exceeded: {hot}")
        if self.config.max_abs_load is not None and loads:
            overloaded = {
                motor_id: value for motor_id, value in loads.items() if abs(value) > self.config.max_abs_load
            }
            if overloaded:
                raise RuntimeError(f"load limit exceeded: {overloaded}")
        return temperatures, loads

    def _assert_complete_ids(self, values: dict[int, object]) -> None:
        if set(values) != set(self.config.motor_ids):
            raise RuntimeError("backend result does not match configured motor IDs")

    def _values_by_name(self, values: dict[int, float] | None) -> dict[str, float] | None:
        if values is None:
            return None
        self._assert_complete_ids(values)
        return dict(zip(AMAZING_HAND_MOTORS, (values[value] for value in self.config.motor_ids), strict=True))

    def emergency_stop(self, reason: str) -> None:
        self.fault_reason = reason
        try:
            self.backend.set_torque(False)
        finally:
            self.state = HandState.FAULT

    def disconnect(self) -> None:
        if self.state is HandState.DISCONNECTED:
            return
        try:
            if self.config.disable_torque_on_disconnect:
                self.backend.set_torque(False)
        finally:
            self.backend.disconnect()
            self.state = HandState.DISCONNECTED
            self._last_raw = None
            self._last_command_time = None
