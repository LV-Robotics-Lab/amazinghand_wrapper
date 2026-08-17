from amazinghand_wrapper import AMAZING_HAND_MOTORS, LeRobotFeetechBackend


class FakeBus:
    def __init__(self) -> None:
        self.present = {
            name: 100 + index for index, name in enumerate(AMAZING_HAND_MOTORS, start=1)
        }
        self.writes: list[tuple[str, dict[str, int], bool]] = []

    def read(self, register: str, name: str, *, normalize: bool) -> int:
        assert register == "Present_Position"
        assert normalize is False
        return self.present[name]

    def sync_write(self, register: str, values: dict[str, int], *, normalize: bool) -> None:
        self.writes.append((register, dict(values), normalize))


def test_lerobot_backend_public_latch_reads_then_writes_identical_positions() -> None:
    backend = LeRobotFeetechBackend()
    bus = FakeBus()
    backend.bus = bus

    latched = backend.latch_current_position()

    expected = {index: 100 + index for index in range(1, 9)}
    assert latched == expected
    assert bus.writes == [
        (
            "Goal_Position",
            {
                name: 100 + index
                for index, name in enumerate(AMAZING_HAND_MOTORS, start=1)
            },
            False,
        )
    ]
