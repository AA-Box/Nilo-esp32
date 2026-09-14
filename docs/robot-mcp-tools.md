# Robot MCP tools

What this board publishes to the backend, what each tool accepts, and what it sends back
without being asked.

* [The tools](#the-tools)
* [Schemas](#schemas)
* [Refusals](#refusals)
* [Notifications](#notifications)
* [What is not here](#what-is-not-here)

## The tools

Fourteen, defined as data in `board/robot_tool_schema.cc` and registered in
`board/robot_mcp_tools.cc`.

| Tool | Arguments | Returns |
|---|---|---|
| `robot.get_status` | — | pose, motion, battery, sensors, head, lift, expression, e-stop |
| `robot.motion.move` | `distance_mm`, `speed_mmps` | `action_id`; completion arrives as a notification |
| `robot.motion.turn` | `angle_deg`, `speed_dps` | `action_id`; completion arrives as a notification |
| `robot.motion.stop` | — | `{"stopped": true}`. Never refused |
| `robot.head.set_angle` | `pitch_deg`, `yaw_deg` | the angles actually reached |
| `robot.head.look_at` | `x_pct`, `y_pct` | the angles actually reached |
| `robot.lift.set_position` | `height_pct` | the position actually reached |
| `robot.expression.set` | `emotion`, `intensity_pct` | the expression set |
| `robot.animation.play` | `name`, `duration_ms` | accepted, or an error naming the animation |
| `robot.camera.capture` | `question` | whether it captured, and the answer when asked |
| `robot.sensor.get_distance` | — | the distance readings |
| `robot.sensor.get_imu` | — | the reading, or `available: false` |
| `robot.sensor.get_cliff` | — | left, right, picked up |
| `robot.power.get_battery` | — | percent, charging, millivolts |

Motion tools return an **action id** and report completion by notification. Waiting for a
robot to finish driving inside a tool call would hold the protocol task for seconds, and
the backend correlates on that id.

## Schemas

Every parameter is an integer, a bounded string or a boolean. There is no float and there
never will be: the transport's type system does not carry one, and a parameter that cannot
survive the wire is a bug waiting for a decimal point.

```
robot.motion.move
  distance_mm   integer, required, minimum -1000, maximum 1000
  speed_mmps    integer, optional, minimum 30, maximum <the configured limit>

robot.motion.turn
  angle_deg     integer, required, minimum -180, maximum 180
  speed_dps     integer, optional, minimum 10, maximum <the configured limit>

robot.lift.set_position
  height_pct    integer, required, minimum 0, maximum 100
```

"Maximum: the configured limit" is literal. `BuildToolTable` takes the live `MotionLimits`
and `SafetyConfig`, so the ceiling a backend reads is the ceiling this robot is actually
running — not a number somebody wrote in a header. A host test sets the limit to 120 mm/s
and asserts the published maximum follows.

The bounds are enforced twice, and both are on the device:

1. the schema, so a request past the limit is rejected before it reaches a controller;
2. the safety supervisor, which evaluates the world the command would move through.

Movement calls **reject** unsafe values rather than clamping them. A robot asked to drive
2 metres that quietly drives 1 is a robot whose backend's model of where it is has just
become wrong.

## Refusals

A refusal names a typed reason, not a sentence:

```
refused: cliff
refused: obstacle
refused: picked_up
refused: link_lost
refused: heartbeat_expired
refused: emergency_stop
refused: battery_critical
```

"No" and "no, there is a drop" are different answers to a person standing in the room, and
the backend can only say the second one if it is told which.

## Notifications

Sent without being asked, as JSON-RPC notifications — no id, so nothing is expected back
and a backend that has gone away costs nothing.

| Method | When |
|---|---|
| `notifications/telemetry` | every second: pose, motion, battery, sensors, expression |
| `notifications/pose` | every 200 ms **while moving only** |
| `notifications/battery` | when the percentage changes |
| `notifications/obstacle` | when something enters or leaves the clearance |
| `notifications/cliff` | when a cliff sensor changes |
| `notifications/touch` | when the touch pad changes |
| `notifications/sensor` | when the bump switch changes |
| `notifications/motion_completed` | a motion arrived: action id, distance travelled, pose |
| `notifications/motion_failed` | a motion did not: the same, plus the reason and the sensors |
| `notifications/emergency_stop` | the latch engaged or cleared |

Pose is reported while moving only. A stationary robot reporting its unchanged pose five
times a second is a robot spending its battery on saying nothing.

A motion that is superseded by a new one reports `motion_failed` with `stopped`, rather
than being silently replaced — the backend is correlating on the action id and has to
learn the first one is never completing.

## What is not here

There is no tool that sets a motor, a servo pulse, a PWM duty cycle, a wheel speed or a
register. That is not an omission:

* the vocabulary **is** the safety boundary. A tool that could set a duty cycle would make
  every check in `safety_supervisor.h` decorative, because the backend could simply drive
  around them;
* a host test enumerates every published schema and fails the build on `pwm`, `duty`,
  `servo_us`, `wheel_speed`, `voltage`, `register`, `gpio`, `torque` or `encoder` appearing
  in any tool name, description or parameter.

The lowest layer that knows what a motor is (`MotorDriver`) is reachable from exactly one
place: the drive controller. Nothing routes to it from the network.

---

[robot-hardware.md](robot-hardware.md) — the board, the layers, calibration and bring-up
