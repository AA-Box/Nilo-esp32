// What the robot's body is, as interfaces.
//
// Six of them, and every one exists so that the layer above can be written and tested
// without a soldering iron:
//
//   MotorDriver                  the only thing that sets a wheel speed
//   DriveController              semantic motion: move, turn, stop, emergency stop
//   HeadController               pan and tilt
//   LiftController               vertical travel
//   DisplayExpressionController  the face
//   RobotSensors                 what the robot can feel
//   BatteryMonitor               what is left
//
// The MCP layer talks to these and to nothing else. It never sets a pin, never writes a
// duty cycle, and never knows which servo library is underneath — so swapping a motor
// driver is a new implementation of one interface, not an edit to the tool handlers.
//
// The split is also what makes the interesting half host-testable: DriveController owns
// acceleration, the motion timeout, wheel synchronisation and the safety hooks, and it is
// plain C++ over a MotorDriver. A fake MotorDriver in a test exercises the whole motion
// and safety loop with no hardware and no FreeRTOS.

#pragma once

#include <cstdint>

#include "motion_profile.h"
#include "safety_supervisor.h"

namespace nilo {

// What a motion turned into. Reported to the backend as a notification, and the reason a
// caller can tell "it arrived" from "it stopped because there was a drop".
enum class MotionKind : uint8_t { kMove, kTurn, kFollow };
enum class MotionOutcome : uint8_t { kCompleted, kStopped, kBlocked, kTimedOut, kFailed };

const char* MotionKindName(MotionKind kind);
const char* MotionOutcomeName(MotionOutcome outcome);

struct MotionResult {
    uint32_t action_id = 0;
    MotionKind kind = MotionKind::kMove;
    MotionOutcome outcome = MotionOutcome::kCompleted;
    SafetyReason reason = SafetyReason::kNone;
    int32_t travelled_mm = 0;
    int32_t elapsed_ms = 0;
};

// Where the robot thinks it is. Integrated from the plan, corrected by the calibration
// scales; a chassis with encoders overrides the integration and keeps the same shape.
struct Pose {
    int32_t x_mm = 0;
    int32_t y_mm = 0;
    int32_t heading_mdeg = 0;  // millidegrees, so a hundred small turns do not round away
};

// The lowest layer, and the only one that knows what a motor is. Not reachable from the
// tool layer: there is no MCP tool that maps onto it, by design.
class MotorDriver {
public:
    virtual ~MotorDriver() = default;

    // Signed wheel speeds in mm/s. Positive drives that wheel forwards.
    virtual void SetWheelSpeeds(int32_t left_mmps, int32_t right_mmps) = 0;

    // Stop actively. Used for a normal stop and for an emergency one.
    virtual void Brake() = 0;

    // Cut drive and let the wheels turn freely. What a robot being carried should do.
    virtual void Coast() = 0;
};

// Semantic motion. The vocabulary a backend is allowed to speak.
class DriveController {
public:
    virtual ~DriveController() = default;

    // Start a move. Returns the action id the completion notification will carry, or 0
    // when the request was refused — `last_refusal()` says why.
    virtual uint32_t Move(int32_t distance_mm, int32_t max_speed_mmps) = 0;
    virtual uint32_t Turn(int32_t angle_deg, int32_t max_speed_dps) = 0;

    // Stop what is running. Always accepted: a robot that will not stop because a sensor
    // says it is unsafe to move is the wrong failure.
    virtual void Stop() = 0;

    // Latch. Nothing moves again until the latch is cleared and the condition is gone.
    virtual void EmergencyStop(SafetyReason reason) = 0;
    virtual bool ClearEmergencyStop() = 0;

    // One control tick. Everything time-dependent happens here: the speed ramp, the
    // supervisory checks, the motion timeout and the completion.
    virtual void Tick() = 0;

    virtual bool IsMoving() const = 0;
    virtual Pose GetPose() const = 0;
    virtual SafetyReason last_refusal() const = 0;

    // Pop the oldest finished motion, if there is one. The board turns these into
    // notifications; nothing here knows what a WebSocket is.
    virtual bool PopResult(MotionResult& result) = 0;
};

class HeadController {
public:
    virtual ~HeadController() = default;

    // Absolute angles, clamped to the mechanical stops. Clamping rather than refusing,
    // because the stops are a property of the machine and the caller cannot know them.
    virtual void SetAngle(int32_t pitch_deg, int32_t yaw_deg) = 0;

    // A point in the camera frame, in percent. Converted to angles using the lens's field
    // of view, which is a calibration value rather than a constant.
    virtual void LookAt(int32_t x_pct, int32_t y_pct) = 0;

    virtual int32_t pitch_deg() const = 0;
    virtual int32_t yaw_deg() const = 0;
    virtual void Tick() = 0;
};

class LiftController {
public:
    virtual ~LiftController() = default;

    // Percent of travel, clamped. A lift has two stops and a caller should not have to
    // know where they are in millimetres.
    virtual void SetPosition(int32_t height_pct) = 0;
    virtual int32_t position_pct() const = 0;
    virtual void Tick() = 0;
};

class DisplayExpressionController {
public:
    virtual ~DisplayExpressionController() = default;

    // A name from the closed expression set, not pixels. The backend never sends a frame
    // buffer: what "happy" looks like is a property of this display.
    virtual bool SetExpression(const char* emotion, int32_t intensity_pct) = 0;
    virtual bool PlayAnimation(const char* name, int32_t duration_ms) = 0;
    virtual const char* current_expression() const = 0;
    virtual void Tick() = 0;
};

struct ImuReading {
    int32_t accel_mg[3] = {0, 0, 1000};
    int32_t gyro_mdps[3] = {0, 0, 0};
    bool valid = false;
};

class RobotSensors {
public:
    virtual ~RobotSensors() = default;

    // One consistent snapshot. Separate getters would let the safety supervisor read a
    // cliff flag from one moment and a distance from another.
    virtual SensorSnapshot Read() = 0;
    virtual ImuReading ReadImu() = 0;
    virtual int32_t DistanceMm(uint8_t index) = 0;
    virtual uint8_t DistanceSensorCount() const = 0;
};

class BatteryMonitor {
public:
    virtual ~BatteryMonitor() = default;

    virtual int32_t Percent() = 0;
    virtual int32_t MilliVolts() = 0;
    virtual bool IsCharging() = 0;
};

}  // namespace nilo
