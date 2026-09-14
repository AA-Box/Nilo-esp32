# Nilo-esp32

The device side of Nilo: a custom ESP32-S3 robot board for the upstream ESP-IDF
voice-assistant firmware, with the motion planning and the local safety rules written as
plain C++ so they can be tested on a laptop.

It is a **drop-in board**, not a fork. The firmware lives in its own repository and stays
there; this repository holds `main/boards/nilo-robot/` and a script that installs it.

```bash
python3 scripts/install.py ../firmware     # copy the board in, wire up the build system
cd ../firmware && python scripts/build.py nilo-robot
```

## What is here

```
board/     the board, exactly as it lands in main/boards/nilo-robot/
host/      tests for everything in board/ that is plain C++
patches/   the two build-system edits, as reviewable snippets
scripts/   install.py (idempotent) and its tests
docs/      robot-hardware.md, robot-mcp-tools.md
```

## The design in one diagram

```
                 MCP tools  (robot_mcp_tools.cc)
                     |            reads validated arguments, calls one method
      +--------------+--------------+--------------+
      |              |              |              |
 DriveController  HeadController  LiftController  DisplayExpressionController
      |                                            RobotSensors, BatteryMonitor
 ProfiledDrive  (motion_profile.cc, safety_supervisor.cc)  <- plain C++, host-tested
      |
 MotorDriver    (robot_actuators.cc)  <- the only file that writes a duty cycle
```

Three things follow from that split, and they are the point of it:

* **No tool reaches a pin.** The MCP handlers call interfaces. There is no tool that sets a
  motor, a servo pulse or a duty cycle — the vocabulary is the safety boundary, and a
  backend that could set one would make every safety check decorative. A host test
  enumerates every published schema and fails the build on an actuator word appearing in
  it.
* **The interesting half is testable.** Acceleration, the motion timeout, wheel
  synchronisation, the cliff and obstacle hooks and the watchdog all live in
  `ProfiledDrive`, which is plain C++ over an injected motor driver and an injected clock.
  `host/test_profiled_drive.cc` drives a whole motion, cuts the link half way through, and
  asserts the wheels stopped — on a laptop, in milliseconds.
* **The protections do not depend on the network.** They run on this chip, on a task that
  has no dependency on a socket being open, and the backend cannot relax any of them.

## Safety

| Condition | What happens |
|---|---|
| Cliff detected | stop; forward and rotation refused, reverse still allowed |
| Obstacle inside the clearance | stop; forward refused |
| Picked up | stop, and **coast** rather than brake |
| Link lost mid-motion | stop, past the grace window |
| Command heartbeat expired | stop |
| Motion past its ceiling | stop |
| Control loop missed its tick | stop |

Configuration can only make it more cautious: every field is clamped towards safety, and a
test asserts that a configuration asking for a 1 mm clearance and a ten-minute heartbeat
gets the floor and the ceiling instead.

## Tests

```bash
make -C host                      # motion, safety, drive, schemas
python3 scripts/test_install.py   # the installer, on a fake firmware tree
clang-format --dry-run -Werror board/*.h board/*.cc host/*.h host/*.cc
```

No framework and no package manager: a compiler and a Makefile. Everything under test is
free of ESP-IDF, FreeRTOS and GPIO, which is what makes "the robot stops when the link
dies" a thing that can be asserted rather than hoped for.

The CI workflow is `ci/github-actions.yml`. Install it with:

```bash
mkdir -p .github/workflows && cp ci/github-actions.yml .github/workflows/ci.yml
```

It is not already at that path because the token this repository was created with does not
carry the `workflow` scope, and GitHub refuses a push that adds a workflow file without it.
One `cp` and a commit from an account that has the scope fixes it.

It runs the three commands above on every pull request. It also runs a **real `idf.py build`
of this exact board** in the `espressif/idf:v6.1` container when the repository variable
`FIRMWARE_REPO` is set to the firmware's clone URL (Settings → Secrets and variables →
Actions → Variables). The variable exists so this repository does not hardcode where the
firmware lives.

## Requirements

ESP-IDF **v6.0.1 or later**; v6.1 is what the upstream firmware recommends and what CI
uses. ESP-IDF 5.x is not supported by the firmware this board installs into.

Target: ESP32-S3, 16 MB flash.

## Board identity

`config.json` declares `"type": "nilo-robot"`, and it is unique and stays that way after
release. This is not bookkeeping: OTA updates are keyed on the reported board type, so a
board that reused another's identity would be handed that board's firmware — and a robot
that receives a smart speaker's image stops being a robot at the worst possible moment.

The installer never edits an existing board.

## Before you power the motors

**First powered test is on blocks, wheels off the ground, with a physical power cutoff
within reach.** The bring-up order, the calibration procedure, and an honest list of
everything that has not yet been run on real hardware are in
[docs/robot-hardware.md](docs/robot-hardware.md).

---

[docs/robot-hardware.md](docs/robot-hardware.md) — the board, the layers, calibration, bring-up ·
[docs/robot-mcp-tools.md](docs/robot-mcp-tools.md) — the tools, their schemas, and the notifications
