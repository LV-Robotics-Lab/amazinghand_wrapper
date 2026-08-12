from pathlib import Path

import pytest

from amazinghand_wrapper import (
    AMAZING_HAND_MOTORS,
    AmazingHandConfig,
    AmazingHandController,
    GripperSynergyMapper,
    HandCalibration,
    HandState,
)


class FakeBackend:
    def __init__(self, models_by_baudrate: dict[int, dict[int, int]] | None = None) -> None:
        self.models_by_baudrate = models_by_baudrate or {1_000_000: dict.fromkeys(range(1, 9), 1280)}
        self.connected_baudrate: int | None = None
        self.positions = {motor_id: 100 + motor_id for motor_id in range(1, 9)}
        self.temperatures = dict.fromkeys(range(1, 9), 25.0)
        self.loads = dict.fromkeys(range(1, 9), 10.0)
        self.torque = False
        self.writes: list[dict[int, int]] = []
        self.connect_attempts: list[int] = []

    def connect(self, port: str, baudrate: int) -> None:
        assert port
        self.connected_baudrate = baudrate
        self.connect_attempts.append(baudrate)

    def disconnect(self) -> None:
        self.connected_baudrate = None

    def ping(self, motor_id: int) -> int | None:
        return self.models_by_baudrate.get(self.connected_baudrate or -1, {}).get(motor_id)

    def set_torque(self, enabled: bool) -> None:
        self.torque = enabled

    def read_positions(self) -> dict[int, int]:
        return dict(self.positions)

    def write_positions(self, positions: dict[int, int]) -> None:
        self.positions = dict(positions)
        self.writes.append(dict(positions))

    def read_temperatures(self) -> dict[int, float] | None:
        return dict(self.temperatures)

    def read_loads(self) -> dict[int, float] | None:
        return dict(self.loads)


def calibration() -> HandCalibration:
    return HandCalibration(
        open_raw={name: 100 + index * 10 for index, name in enumerate(AMAZING_HAND_MOTORS)},
        closed_raw={name: 700 - index * 10 for index, name in enumerate(AMAZING_HAND_MOTORS)},
    )


def config(tmp_path: Path, **overrides: object) -> AmazingHandConfig:
    values = {"port": "/dev/fake", "calibration_file": tmp_path / "hand.json"}
    values.update(overrides)
    return AmazingHandConfig(**values)  # type: ignore[arg-type]


def test_synergy_maps_endpoints_and_clips() -> None:
    mapper = GripperSynergyMapper(calibration())
    assert mapper.targets(100.0) == calibration().open_raw
    assert mapper.targets(0.0) == calibration().closed_raw
    assert mapper.targets(200.0) == calibration().open_raw
    assert mapper.targets(-100.0) == calibration().closed_raw


def test_connect_prefers_official_baud_and_keeps_torque_off(tmp_path: Path) -> None:
    backend = FakeBackend()
    controller = AmazingHandController(config(tmp_path), backend)
    assert controller.connect() == 1_000_000
    assert controller.detected_models == dict.fromkeys(range(1, 9), 1280)
    assert backend.torque is False
    assert controller.state is HandState.CONNECTED


def test_connect_falls_back_to_250k_and_accepts_1284(tmp_path: Path) -> None:
    backend = FakeBackend({250_000: dict.fromkeys(range(1, 9), 1284)})
    controller = AmazingHandController(config(tmp_path), backend)
    assert controller.connect() == 250_000
    assert backend.connect_attempts == [1_000_000, 250_000]


def test_connect_rejects_missing_motor_and_disconnects(tmp_path: Path) -> None:
    models = dict.fromkeys(range(1, 8), 1280)
    backend = FakeBackend({1_000_000: models})
    controller = AmazingHandController(config(tmp_path, baudrates=(1_000_000,)), backend)
    with pytest.raises(ConnectionError, match="motor ID 8"):
        controller.connect()
    assert backend.torque is False
    assert backend.connected_baudrate is None


def test_activation_requires_calibration(tmp_path: Path) -> None:
    backend = FakeBackend()
    controller = AmazingHandController(config(tmp_path), backend)
    controller.connect()
    with pytest.raises(RuntimeError, match="calibrated"):
        controller.activate()
    assert backend.torque is False


def test_calibration_is_atomic_and_reloads(tmp_path: Path) -> None:
    path = tmp_path / "hand.json"
    calibration().save(path)
    assert HandCalibration.load(path) == calibration()
    controller = AmazingHandController(config(tmp_path), FakeBackend())
    assert controller.is_calibrated


def test_velocity_limit_is_per_second(tmp_path: Path) -> None:
    now = [10.0]
    backend = FakeBackend()
    controller = AmazingHandController(
        config(tmp_path, max_raw_velocity=100.0), backend, clock=lambda: now[0]
    )
    controller.set_calibration(calibration())
    controller.connect()
    controller.activate()
    initial = dict(backend.positions)
    now[0] += 0.1
    sent = controller.command_grasp(0.0, command_timestamp=now[0])
    assert all(abs(sent[motor_id] - initial[motor_id]) <= 10 for motor_id in sent)


def test_stale_command_enters_fault_and_disables_torque(tmp_path: Path) -> None:
    now = [10.0]
    backend = FakeBackend()
    controller = AmazingHandController(config(tmp_path), backend, clock=lambda: now[0])
    controller.set_calibration(calibration())
    controller.connect()
    controller.activate()
    with pytest.raises(RuntimeError, match="stale"):
        controller.command_grasp(50.0, command_timestamp=9.0)
    assert controller.state is HandState.FAULT
    assert backend.torque is False


def test_temperature_fault_is_fail_closed(tmp_path: Path) -> None:
    backend = FakeBackend()
    controller = AmazingHandController(config(tmp_path), backend)
    controller.set_calibration(calibration())
    controller.connect()
    controller.activate()
    backend.temperatures[4] = 80.0
    with pytest.raises(RuntimeError, match="temperature"):
        controller.command_grasp(50.0)
    assert controller.state is HandState.FAULT
    assert backend.torque is False


def test_observation_preserves_per_motor_and_scalar_state(tmp_path: Path) -> None:
    backend = FakeBackend()
    controller = AmazingHandController(config(tmp_path), backend)
    controller.set_calibration(calibration())
    controller.connect()
    observation = controller.observe()
    assert set(observation.raw_positions) == set(AMAZING_HAND_MOTORS)
    assert set(observation.motor_closure) == set(AMAZING_HAND_MOTORS)
    assert 0.0 <= observation.grasp_closure <= 100.0
