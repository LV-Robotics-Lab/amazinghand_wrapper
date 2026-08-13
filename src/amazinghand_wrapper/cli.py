import argparse
import json
from pathlib import Path

from .calibration import HandCalibration, load_pollen_middle_positions
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


def _migrate_pollen(args: argparse.Namespace) -> int:
    if args.source is not None:
        middle_positions = load_pollen_middle_positions(args.source, args.variable)
        source = str(args.source)
        variable = args.variable
    else:
        middle_positions = tuple(args.middle_pos)
        source = "command-line"
        variable = None
    calibration = HandCalibration.from_pollen_middle_positions(
        middle_positions,
        source=source,
        variable=variable,
    )
    calibration.save(args.output)
    print(
        json.dumps(
            {
                "hardware_touched": False,
                "output": str(args.output),
                "schema": calibration.schema,
            },
            sort_keys=True,
        )
    )
    return 0


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
    migrate = subparsers.add_parser(
        "migrate-pollen",
        help="convert a calibrated Pollen MiddlePos profile without touching hardware",
    )
    source = migrate.add_mutually_exclusive_group(required=True)
    source.add_argument("--source", type=Path, help="Pollen Python demo containing MiddlePos")
    source.add_argument("--middle-pos", type=float, nargs=8, metavar="DEGREES")
    migrate.add_argument("--variable", default="MiddlePos")
    migrate.add_argument("--output", type=Path, required=True)
    migrate.set_defaults(handler=_migrate_pollen)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return int(args.handler(args))


if __name__ == "__main__":
    raise SystemExit(main())
