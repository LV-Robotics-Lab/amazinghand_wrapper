import argparse
import json
from pathlib import Path

from .config import AmazingHandConfig
from .controller import AmazingHandController
from .lerobot_backend import LeRobotFeetechBackend


def _controller(args: argparse.Namespace) -> AmazingHandController:
    motor_ids = tuple(range(1, 9))
    config = AmazingHandConfig(
        port=args.port,
        calibration_file=Path(args.calibration),
        baudrates=tuple(args.baudrate),
        expected_model_numbers=tuple(args.model_number),
        motor_ids=motor_ids,
    )
    return AmazingHandController(config, LeRobotFeetechBackend(motor_ids))


def _probe(args: argparse.Namespace) -> int:
    controller = _controller(args)
    try:
        baudrate = controller.connect()
        print(json.dumps({"baudrate": baudrate, "models": controller.detected_models}, sort_keys=True))
        return 0
    finally:
        controller.disconnect()


def _calibrate(args: argparse.Namespace) -> int:
    controller = _controller(args)
    try:
        controller.connect()
        controller.calibrate_interactive()
        print(f"Calibration saved to {controller.config.calibration_file}")
        return 0
    finally:
        controller.disconnect()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Safety-gated AmazingHand utilities")
    subparsers = parser.add_subparsers(dest="command", required=True)
    for name, handler in (("probe", _probe), ("calibrate", _calibrate)):
        command = subparsers.add_parser(name)
        command.add_argument("--port", required=True)
        command.add_argument("--calibration", default="calibration/amazinghand.json")
        command.add_argument("--baudrate", type=int, action="append", default=[1_000_000, 250_000])
        command.add_argument("--model-number", type=int, action="append", default=[1280, 1284])
        command.set_defaults(handler=handler)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return int(args.handler(args))


if __name__ == "__main__":
    raise SystemExit(main())

