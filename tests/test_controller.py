from pathlib import Path

import pytest

from amazinghand_wrapper import (
    AMAZING_HAND_MOTORS,
    AmazingHandConfig,
    AmazingHandController,
    GripperSynergyMapper,
    HandCalibration,
    HandState,
    load_pollen_middle_positions,
    pollen_scs0009_degrees_to_raw,
)


class FakeBackend:
    def __init__(
        self,
        models_by_baudrate: dict[int, dict[int, int]] | None = None,
        *,
        fail_read_positions: bool = False,
        fail_write_positions: bool = False,
    ) -> None:
        self.models_by_baudrate = models_by_baudrate or {1_000_000: dict.fromkeys(range(1, 9), 1280)}
        self.fail_read_positions = fail_read_positions
        self.fail_write_positions = fail_write_positions
        self.connected_baudrate: int | None = None
        self.positions = {motor_id: 100 + motor_id for motor_id in range(1, 9)}
        self.temperatures = dict.fromkeys(range(1, 9), 25.0)
        self.loads = dict.fromkeys(range(1, 9), 10.0)
        self.torque = False
        self.writes: list[dict[int, int]] = []
        self.connect_attempts: list[int] = []
        self.read_position_calls = 0
        self.events: list[str] = []

    def connect(self, port: str, baudrate: int) -> None:
        assert port
        self.connected_baudrate = baudrate
        self.connect_attempts.append(baudrate)

    def disconnect(self) -> None:
        self.connected_baudrate = None

    def ping(self, motor_id: int) -> int | None:
        return self.models_by_baudrate.get(self.connected_baudrate or -1, {}).get(motor_id)

    def set_torque(self, enabled: bool) -> None:
        self.events.append(f"torque:{enabled}")
        self.torque = enabled

    def read_positions(self) -> dict[int, int]:
        self.events.append("read_positions")
        self.read_position_calls += 1
        if self.fail_read_positions:
            raise RuntimeError("injected position read failure")
        return dict(self.positions)

    def write_positions(self, positions: dict[int, int]) -> None:
        self.events.append("write_positions")
        if self.fail_write_positions:
            raise RuntimeError("injected goal write failure")
        self.positions = dict(positions)
        self.writes.append(dict(positions))

    def latch_current_position(self) -> dict[int, int]:
        current = self.read_positions()
        if set(current) != set(range(1, 9)):
            raise RuntimeError("incomplete fake position read")
        self.write_positions(current)
        return dict(current)

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


def test_latch_reads_current_pose_and_writes_identical_goal_torque_off(tmp_path: Path) -> None:
    backend = FakeBackend()
    controller = AmazingHandController(config(tmp_path), backend)
    controller.connect()
    measured = dict(backend.positions)

    latched = controller.latch_current_position()

    assert latched == measured
    assert backend.writes == [measured]
    assert backend.events[-2:] == ["read_positions", "write_positions"]
    assert backend.torque is False
    assert controller.state is HandState.CONNECTED


def test_latch_failure_enters_fault_and_disables_torque(tmp_path: Path) -> None:
    backend = FakeBackend(fail_write_positions=True)
    controller = AmazingHandController(config(tmp_path), backend)
    controller.connect()

    with pytest.raises(RuntimeError, match="goal write failure"):
        controller.latch_current_position()

    assert controller.state is HandState.FAULT
    assert controller.fault_reason == "goal latch failure: injected goal write failure"
    assert backend.torque is False


def test_latch_before_connect_never_touches_backend_or_default_device(tmp_path: Path) -> None:
    backend = FakeBackend()
    controller = AmazingHandController(config(tmp_path), backend)

    with pytest.raises(RuntimeError, match="connected, torque-off"):
        controller.latch_current_position()

    assert backend.connect_attempts == []
    assert backend.read_position_calls == 0
    assert backend.writes == []


def test_activate_latches_before_enabling_torque_for_compatibility(tmp_path: Path) -> None:
    backend = FakeBackend()
    controller = AmazingHandController(config(tmp_path), backend)
    controller.set_calibration(calibration())
    controller.connect()
    backend.events.clear()

    controller.activate()

    assert backend.events.index("read_positions") < backend.events.index("write_positions")
    assert backend.events.index("write_positions") < backend.events.index("torque:True")
    assert controller.state is HandState.ACTIVE


def test_calibration_is_atomic_and_reloads(tmp_path: Path) -> None:
    path = tmp_path / "hand.json"
    calibration().save(path)
    assert HandCalibration.load(path) == calibration()
    controller = AmazingHandController(config(tmp_path), FakeBackend())
    assert controller.is_calibrated


def test_pollen_middle_positions_migrate_without_hardware(tmp_path: Path) -> None:
    source = tmp_path / "AmazingHand_Demo_Both.py"
    source.write_text(
        "MiddlePos_1 = [3, 0, -8, -13, 2, -5, -12, -5]\n"
        "MiddlePos_2 = [3, -3, -1, -10, 5, 2, -7, 3]\n"
    )
    middle = load_pollen_middle_positions(source, "MiddlePos_1")
    migrated = HandCalibration.from_pollen_middle_positions(
        middle, source=str(source), variable="MiddlePos_1"
    )
    output = tmp_path / "right.json"
    migrated.save(output)

    assert migrated.open_raw["index_1"] == pollen_scs0009_degrees_to_raw(3 - 35)
    assert migrated.closed_raw["index_1"] == pollen_scs0009_degrees_to_raw(3 + 90)
    assert migrated.open_raw["index_2"] == pollen_scs0009_degrees_to_raw(35)
    assert migrated.closed_raw["index_2"] == pollen_scs0009_degrees_to_raw(-90)
    assert HandCalibration.load(output) == migrated


def test_pollen_import_rejects_missing_or_unsafe_values(tmp_path: Path) -> None:
    source = tmp_path / "demo.py"
    source.write_text("MiddlePos = [1, 2]\n")
    with pytest.raises(ValueError, match="exactly eight"):
        load_pollen_middle_positions(source, "MiddlePos")
    with pytest.raises(ValueError, match="outside"):
        HandCalibration.from_pollen_middle_positions([200] * 8)


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
