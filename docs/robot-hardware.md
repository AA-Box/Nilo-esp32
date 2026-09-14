# Robot hardware

The Nilo robot board: what is on it, how the layers fit, how to calibrate it, and what
still has to be proved on a real machine.

* [The layers](#the-layers)
* [Pin map](#pin-map)
* [Local safety](#local-safety)
* [Calibration](#calibration)
* [Device state](#device-state)
* [Bring-up](#bring-up)
* [What still needs hardware](#what-still-needs-hardware)

## The layers

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

Nothing above `MotorDriver` touches a pin. That is what makes the interesting half
testable: `ProfiledDrive` owns acceleration, the motion timeout, wheel synchronisation and
the safety hooks, and it is plain C++ over two injected things — a motor driver and a
clock. `host/test_profiled_drive.cc` drives a whole motion, cuts the link half way through
it, and asserts the wheels stopped, on a laptop.

There is no MCP tool that maps onto `MotorDriver`. A backend-commanded PWM value is not
the normal API and it is not an API at all: the vocabulary is the safety boundary, and a
tool that could set a duty cycle would make every check in `safety_supervisor.h`
decorative.

## Pin map

Every pin is in `board/config.h`. The reference build is an ESP32-S3 with:

| Subsystem | Parts | Notes |
|---|---|---|
| Audio | ES8311 codec on I2S + I2C | the upstream audio path, unchanged |
| Drive | two brushed motors, dual H-bridge | one PWM and one direction pin each, plus a shared enable |
| Head | two servos, pitch and yaw | LEDC at 50 Hz |
| Lift | one servo | LEDC at 50 Hz |
| Sensors | two cliff sensors, a bump switch, a touch pad, one ultrasonic range finder | cliff sensors are debounced asymmetrically |
| Power | battery divider on ADC1 | no charge-detect line on the reference build |
| Display | ST7789 240x240 on SPI | the face |

The H-bridge enable starts **low**. A board that powers up with its motors live is a board
that moves before anything has decided it should.

## Local safety

Five protections, in `board/safety_supervisor.cc`. They run on this chip, on a task that
has no dependency on a socket being open, and **the backend cannot relax any of them**.
There is no method that takes a "disable" flag and no field a command can write.

| Condition | What happens |
|---|---|
| Cliff detected | stop; forward and rotation refused. Reverse is still allowed |
| Obstacle inside the clearance | stop; forward refused. Reverse is still allowed |
| Picked up | stop, and coast rather than brake |
| Link lost mid-motion | stop, once past the grace window |
| Command heartbeat expired | stop |
| Motion past its ceiling | stop |
| Control loop missed its own tick | stop |

Two details that are not obvious and are deliberate:

* **A cliff refuses forward, not reverse.** Backing away from a drop is how a robot
  recovers from finding one. A robot that refuses everything has to be picked up.
* **Being picked up coasts.** A braked wheel in somebody's hand is a locked wheel, and
  locked wheels are how a robot gets dropped.

Configuration can only make it **more** cautious. `SafetyConfig::Clamped()` clamps every
field towards safety and a host test asserts it: a configuration that asks for a 1 mm
clearance and a ten-minute heartbeat gets the floor and the ceiling.

The emergency-stop latch will not clear while the condition that set it is still true. A
latch that clears while the robot is still on the edge of the table is not a latch.

## Calibration

**Every one of these is a knob, not a constant.** A real chassis never matches the paper
one: wheels are moulded a percent small, gearboxes have backlash, one motor is always
slightly faster than the other, and battery voltage moves all of it. The defaults in
`config.h` are the designed values; the measured ones live in NVS under the `nilo`
namespace and override them at boot.

| Key | Default | What it corrects | How to measure it |
|---|---|---|---|
| `wheel_mm` | 65 | wheel diameter | callipers, both wheels, average |
| `track_mm` | 140 | distance between wheel centres | callipers, centre to centre |
| `gear` | 1000 | output turns per motor turn, x1000 | from the gearbox spec |
| `trim` | 0 | one motor being faster than the other | drive 2 m, measure the sideways drift |
| `dist_scale` | 1000 | commanded distance vs measured, x1000 | command 1000 mm, measure, set `1000 * measured / 1000` |
| `turn_scale` | 1000 | commanded angle vs measured, x1000 | command four 90° turns, measure the error |

Order matters. Do them in this order, because each one is measured with the previous ones
already applied:

1. **Trim first.** Command a 2000 mm move on a flat floor and measure how far sideways the
   robot ends up. Adjust `trim` until it drives straight; everything else is measured with
   a robot that curves otherwise.
2. **Distance scale.** Command 1000 mm five times, measure each, take the median.
3. **Turn scale.** Command 90° four times and measure the total error against 360°.
4. **Servo trims.** `CAL_HEAD_PITCH_TRIM_DEG` and `CAL_HEAD_YAW_TRIM_DEG` — command 0 and
   adjust until the head is level and facing forward.
5. **Lift pulse widths.** `CAL_LIFT_MIN_PULSE_US` and `CAL_LIFT_MAX_PULSE_US` — find the
   values that reach the stops *without straining against them*. A servo held against a
   stop draws current until something gives.

Re-measure after any mechanical change. A chassis that has been apart is a new chassis.

## Device state

The firmware already has a device state machine. The robot does **not** get a second one —
`RobotService::FollowDeviceState` reads the existing one and reacts, so there is no pair of
states that can disagree about what the device is doing.

| Device state | The robot |
|---|---|
| `kDeviceStateUpgrading` | emergency stop. A robot that drives during an OTA drives with half an image in flash |
| `kDeviceStateFatalError` | emergency stop |
| `kDeviceStateIdle` / `Listening` / `Speaking` | normal operation |
| anything else | stop; nothing has asked it to move yet |

OTA is untouched. The board adds no upgrade path of its own and overrides nothing in the
upstream one — and because `config.json` declares a unique board type, this robot receives
its own firmware and never another board's.

## Bring-up

**First powered test is on blocks, wheels off the ground, with a physical power cutoff
within reach.** Not because something is likely to go wrong, but because the failure mode
of a motion bug is a robot on the floor and a bug that is now much harder to reproduce.

In order:

1. **Flash and boot with the motor enable disconnected.** Check the log line from
   `PwmMotorDriver`: wheel diameter, track width, trim. If those are wrong, stop here.
2. **Sensors, still with no drive.** Call `robot.sensor.get_cliff` and
   `robot.sensor.get_distance`. Cover each cliff sensor by hand and confirm the flag moves.
   An inverted cliff sensor is the single most dangerous wiring mistake on this board, and
   it is invisible until the robot is on a table.
3. **Head and lift, on blocks.** `robot.head.set_angle` through the full range, slowly.
   Listen for a servo straining at a stop.
4. **Drive, on blocks.** `robot.motion.move` with a small distance. Check both wheels turn
   the right way and stop together.
5. **Safety, on blocks.** Trigger a cliff sensor by hand mid-move and confirm the wheels
   stop. Then pull the network and confirm they stop again.
6. **Only then, on the floor.** Start with the calibration sequence above.

## What still needs hardware

Everything below has been written and reviewed and **not one line of it has been run on a
robot**. This list is the honest boundary; each item needs a machine and a person watching.

| Item | Why a host test cannot cover it |
|---|---|
| Every pin assignment in `config.h` | it matches a schematic that does not exist yet |
| Motor direction and polarity | a reversed direction pin drives backwards and passes every test here |
| Cliff sensor polarity | the code assumes the sensor floats high over a drop; the opposite is silent and fatal |
| Ultrasonic timing | the echo loop is written against a datasheet, not measured |
| ADC divider ratio and battery curve | the percentage is a straight line between two voltages nobody has measured on this pack |
| Servo pulse widths and mechanical stops | `CAL_LIFT_MIN_PULSE_US` and the head limits are design values |
| The IMU | not fitted on the reference build. `robot.sensor.get_imu` reports `available: false` rather than plausible zeros |
| Charge detection | no line on the reference build. `IsCharging()` returns false honestly rather than guessing from a rising voltage, which is indistinguishable from a warming pack |
| Real acceleration limits | `LIMIT_MAX_ACCEL_MMPS2` is a guess until a robot with a payload is measured |
| Brake vs coast behaviour | how quickly this H-bridge actually stops this chassis |
| Tick timing under load | the loop is at priority 6 above the protocol task; whether that is enough with audio running is a measurement |
| Link-loss stop with the backend `SIGKILL`ed | the host test proves the logic; the guarantee is the whole stack |
| Thermal behaviour | motors, H-bridge and regulator under sustained motion |

---

[robot-mcp-tools.md](robot-mcp-tools.md) — the tools this board publishes and what they accept
