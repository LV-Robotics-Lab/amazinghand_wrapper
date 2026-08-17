# amazinghand_wrapper

Backend-neutral control and safety boundary for the four-finger Pollen Robotics
AmazingHand (eight Feetech SCS0009 servos).

The package intentionally separates hand lifecycle, calibration, safety checks,
and grasp-synergy mapping from any robot-arm framework. LeRobot integration is
provided by `LeRobotFeetechBackend`, while tests use an in-memory fake backend.

## Status

The software state machine and fake-backend tests are validated. Real hardware
validation is still required before unattended use. The upstream AmazingHand
project warns that the design has not been tested for long, complex prehensile
tasks, so this wrapper fails closed and never enables torque during `connect()`.

## Safety contract

- `connect()` only probes IDs and loads calibration; torque remains disabled.
- `latch_current_position()` reads all eight present positions and writes those
same values as goals while torque is off; any failure enters `FAULT`.
- `activate()` is explicit and requires a complete calibration.
- Commands may carry a monotonic timestamp and stale commands are rejected.
- Raw velocity is limited in units per second, independent of loop frequency.
- Temperature and load thresholds trigger an emergency stop when available.
- Any probe, observation, or write failure disables torque and enters `FAULT`.

Composite robots should call `connect()`, then `latch_current_position()` on
every torque-off component, and only then call `activate()`. For compatibility,
`activate()` refreshes the same public latch immediately before every torque
enable, even when a composite-level latch was already completed. The controller
owns this operation through the public
`AmazingHandBackend.latch_current_position()` protocol; concrete backends
perform the read-and-identical-write operation. Callers must never reach through
`controller.backend`.

## Install

```bash
python -m pip install -e .
# For the LeRobot backend, install the containing LeRobot checkout with its
# Feetech extra as well.
```

## Read-only probe

```bash
amazinghand probe --port /dev/ttyUSB0
```

The default probe order is 1,000,000 baud followed by 250,000 baud. Both model
numbers currently encountered in the Pollen and LeRobot ecosystems (`1280` and
`1284`) are accepted by default and reported; deployments should pin the value
observed on their exact hardware.

## Calibration

```bash
amazinghand calibrate \
  --port /dev/ttyUSB0 \
  --output ~/.cache/amazinghand/right.json
```

Calibration disables torque and records explicit open and closed raw positions.
Do not run it unless the hand can be moved safely by an operator.

### Migrate an existing Pollen calibration

If the hand was already calibrated with Pollen's reference Python demo, migrate its
device-specific `MiddlePos` values offline instead of recalibrating the mechanics:

```bash
amazinghand migrate-pollen \
  --source AmazingHand_Demo_Both.py \
  --variable MiddlePos_1 \
  --output ~/.cache/amazinghand/right.json
```

Run the same command with `MiddlePos_2` for the other hand. The converter matches
Pollen's `OpenHand`/`CloseHand` angles and rustypot's SCS0009 raw-position conversion,
records its assumptions as provenance in the JSON, and never opens a serial port. Do
not use the unmodified example `MiddlePos` values for a different physical hand.

## Tests

```bash
python -m pytest -q
ruff check .
```
