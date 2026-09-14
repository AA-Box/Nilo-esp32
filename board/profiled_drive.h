// The drive controller: semantic motion, acceleration limits, the watchdog, and the
// safety hooks — over any MotorDriver, with no ESP-IDF in sight.
//
// This is where the four things the design note says the controller owns actually live:
//
//   acceleration constraints   the trapezoid from motion_profile.h, per tick
//   motion timeout             a deadline per motion, and a ceiling over all of them
//   wheel synchronisation      one trim value, applied in one place
//   obstacle and cliff hooks   SafetySupervisor, consulted at admission and every tick
//   watchdog                   a tick that has not run recently stops the wheels
//
// It is plain C++ over two injected things — a MotorDriver and a clock — so a test drives
// a whole motion, cuts the link half way through it, and asserts the wheels stopped,
// without a robot, an RTOS or a network.

#pragma once

#include <cstddef>
#include <cstdint>

#include "robot_hardware.h"

namespace nilo {

using ClockFn = int64_t (*)();

// How long between ticks before the controller assumes its own loop has died. The wheels
// stop: a control loop that is not running is not controlling anything.
constexpr int32_t kTickWatchdogMs = 250;

// How many finished motions are held for the reporter before the oldest is dropped. Small
// on purpose: a backlog of completions means the link is down, and the newest is the one
// that matters.
constexpr size_t kResultQueueDepth = 4;

class ProfiledDrive : public DriveController {
public:
    ProfiledDrive(MotorDriver& motors, SafetySupervisor& safety, ClockFn clock,
                  const MotionLimits& limits = MotionLimits(),
                  const ChassisCalibration& chassis = ChassisCalibration());

    // The sensor snapshot the supervisor is evaluated against. Pushed in by the board's
    // poller rather than read here, so this class has nothing to mock.
    void UpdateSensors(const SensorSnapshot& sensors);
    const SensorSnapshot& sensors() const { return sensors_; }

    uint32_t Move(int32_t distance_mm, int32_t max_speed_mmps) override;
    uint32_t Turn(int32_t angle_deg, int32_t max_speed_dps) override;
    void Stop() override;
    void EmergencyStop(SafetyReason reason) override;
    bool ClearEmergencyStop() override;
    void Tick() override;

    bool IsMoving() const override { return active_; }
    Pose GetPose() const override { return pose_; }
    SafetyReason last_refusal() const override { return last_refusal_; }
    bool PopResult(MotionResult& result) override;

    const MotionLimits& limits() const { return limits_; }
    const ChassisCalibration& chassis() const { return chassis_; }
    uint32_t active_action_id() const { return active_ ? action_id_ : 0; }

private:
    uint32_t Start(const MotionPlan& plan, MotionKind kind, Direction direction);
    void Finish(MotionOutcome outcome, SafetyReason reason);
    void Apply(int32_t speed_mmps);
    void IntegratePose(int32_t travelled_mm);
    void PushResult(const MotionResult& result);

    MotorDriver& motors_;
    SafetySupervisor& safety_;
    ClockFn clock_;
    MotionLimits limits_;
    ChassisCalibration chassis_;

    SensorSnapshot sensors_;
    Pose pose_;

    bool active_ = false;
    MotionPlan plan_;
    MotionKind kind_ = MotionKind::kMove;
    Direction direction_ = Direction::kStationary;
    uint32_t action_id_ = 0;
    uint32_t next_action_id_ = 1;
    int64_t started_ms_ = 0;
    int64_t last_tick_ms_ = 0;
    int32_t travelled_mm_ = 0;
    SafetyReason last_refusal_ = SafetyReason::kNone;

    MotionResult results_[kResultQueueDepth];
    size_t result_head_ = 0;
    size_t result_count_ = 0;
};

}  // namespace nilo
