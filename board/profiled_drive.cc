#include "profiled_drive.h"

#include <algorithm>
#include <cstdlib>

namespace nilo {

const char* MotionKindName(MotionKind kind) {
    switch (kind) {
        case MotionKind::kMove:
            return "move";
        case MotionKind::kTurn:
            return "turn";
        case MotionKind::kFollow:
            return "follow";
    }
    return "unknown";
}

const char* MotionOutcomeName(MotionOutcome outcome) {
    switch (outcome) {
        case MotionOutcome::kCompleted:
            return "completed";
        case MotionOutcome::kStopped:
            return "stopped";
        case MotionOutcome::kBlocked:
            return "blocked";
        case MotionOutcome::kTimedOut:
            return "timed_out";
        case MotionOutcome::kFailed:
            return "failed";
    }
    return "unknown";
}

ProfiledDrive::ProfiledDrive(MotorDriver& motors, SafetySupervisor& safety, ClockFn clock,
                             const MotionLimits& limits, const ChassisCalibration& chassis)
    : motors_(motors), safety_(safety), clock_(clock), limits_(limits), chassis_(chassis) {
    last_tick_ms_ = clock_();
}

void ProfiledDrive::UpdateSensors(const SensorSnapshot& sensors) { sensors_ = sensors; }

uint32_t ProfiledDrive::Start(const MotionPlan& plan, MotionKind kind, Direction direction) {
    const int64_t now = clock_();
    const SafetyVerdict verdict = safety_.Admit(direction, sensors_, now);
    if (!verdict.allowed) {
        last_refusal_ = verdict.reason;
        return 0;
    }
    if (plan.IsEmpty()) {
        last_refusal_ = SafetyReason::kOutOfRange;
        return 0;
    }
    if (active_) {
        // A new motion supersedes the one in flight. Reported as stopped rather than
        // silently replaced: the backend is correlating on the action id and needs to know
        // the first one is never completing.
        Finish(MotionOutcome::kStopped, SafetyReason::kNone);
    }
    plan_ = plan;
    kind_ = kind;
    direction_ = direction;
    active_ = true;
    started_ms_ = now;
    last_tick_ms_ = now;
    travelled_mm_ = 0;
    action_id_ = next_action_id_++;
    last_refusal_ = SafetyReason::kNone;
    Apply(SpeedAtMs(plan_, 0));
    return action_id_;
}

uint32_t ProfiledDrive::Move(int32_t distance_mm, int32_t max_speed_mmps) {
    // The calibration scale is applied to the *commanded* distance, so "drive 250 mm"
    // means 250 mm on the floor rather than 250 mm of a model that was never measured.
    const int32_t scaled = static_cast<int32_t>((static_cast<int64_t>(distance_mm) * 1000) /
                                                std::max(1, chassis_.distance_scale_permille));
    return Start(PlanMove(scaled, max_speed_mmps, limits_), MotionKind::kMove,
                 DirectionOf(distance_mm));
}

uint32_t ProfiledDrive::Turn(int32_t angle_deg, int32_t max_speed_dps) {
    const int32_t scaled = static_cast<int32_t>((static_cast<int64_t>(angle_deg) * 1000) /
                                                std::max(1, chassis_.turn_scale_permille));
    const MotionPlan plan = PlanTurn(scaled, max_speed_dps, limits_, chassis_);
    return Start(plan, MotionKind::kTurn,
                 angle_deg == 0 ? Direction::kStationary : Direction::kRotate);
}

void ProfiledDrive::Stop() {
    motors_.Brake();
    if (active_) {
        Finish(MotionOutcome::kStopped, SafetyReason::kNone);
    }
}

void ProfiledDrive::EmergencyStop(SafetyReason reason) {
    // Brake first, latch second. The order matters: latching first and then braking leaves
    // a window in which the wheels are still turning and every path back is refused.
    motors_.Brake();
    safety_.EngageEmergencyStop(reason);
    if (active_) {
        Finish(MotionOutcome::kBlocked, reason);
    }
}

bool ProfiledDrive::ClearEmergencyStop() { return safety_.ClearEmergencyStop(sensors_, clock_()); }

void ProfiledDrive::Tick() {
    const int64_t now = clock_();
    const int64_t since_tick = now - last_tick_ms_;
    last_tick_ms_ = now;

    if (!active_) {
        if (sensors_.picked_up) {
            // Being carried. Coasting rather than braking: a braked wheel in somebody's
            // hand is a locked wheel, and locked wheels are how a robot gets dropped.
            motors_.Coast();
        }
        return;
    }

    if (since_tick > kTickWatchdogMs) {
        // The control loop missed its own deadline. Whatever else is wrong, the wheels
        // should not keep turning on a plan nobody is supervising.
        motors_.Brake();
        Finish(MotionOutcome::kTimedOut, SafetyReason::kMotionTimeout);
        return;
    }

    const SafetyVerdict verdict = safety_.Supervise(direction_, sensors_, now, started_ms_);
    if (!verdict.allowed) {
        motors_.Brake();
        const MotionOutcome outcome = verdict.reason == SafetyReason::kMotionTimeout
                                          ? MotionOutcome::kTimedOut
                                          : MotionOutcome::kBlocked;
        Finish(outcome, verdict.reason);
        return;
    }

    const int32_t elapsed = static_cast<int32_t>(now - started_ms_);
    if (elapsed >= plan_.TotalMs()) {
        motors_.Brake();
        travelled_mm_ = plan_.distance_mm;
        Finish(MotionOutcome::kCompleted, SafetyReason::kNone);
        return;
    }
    travelled_mm_ = DistanceAtMs(plan_, elapsed);
    Apply(SpeedAtMs(plan_, elapsed));
}

void ProfiledDrive::Apply(int32_t speed_mmps) {
    const WheelSpeeds speeds = kind_ == MotionKind::kTurn ? SplitTurn(speed_mmps, chassis_)
                                                          : SplitStraight(speed_mmps, chassis_);
    motors_.SetWheelSpeeds(speeds.left_mmps, speeds.right_mmps);
}

void ProfiledDrive::IntegratePose(int32_t travelled_mm) {
    if (kind_ == MotionKind::kTurn) {
        const int32_t arc = std::max(1, std::abs(plan_.distance_mm));
        const int64_t full = static_cast<int64_t>(TurnArcMm(360, chassis_));
        const int64_t mdeg = full > 0 ? (static_cast<int64_t>(travelled_mm) * 360000) / full : 0;
        pose_.heading_mdeg = static_cast<int32_t>((pose_.heading_mdeg + mdeg) % 360000);
        (void)arc;
        return;
    }
    // Straight travel along the current heading. A cosine table would be more accurate;
    // the heading is in millidegrees and this is dead reckoning either way, so the pose is
    // reported as an estimate and corrected by whatever the backend can see.
    pose_.x_mm += travelled_mm;
}

void ProfiledDrive::Finish(MotionOutcome outcome, SafetyReason reason) {
    if (!active_) {
        return;
    }
    const int64_t now = clock_();
    MotionResult result;
    result.action_id = action_id_;
    result.kind = kind_;
    result.outcome = outcome;
    result.reason = reason;
    result.travelled_mm = travelled_mm_;
    result.elapsed_ms = static_cast<int32_t>(now - started_ms_);
    IntegratePose(travelled_mm_);
    active_ = false;
    direction_ = Direction::kStationary;
    motors_.SetWheelSpeeds(0, 0);
    PushResult(result);
}

void ProfiledDrive::PushResult(const MotionResult& result) {
    if (result_count_ == kResultQueueDepth) {
        // Drop the oldest: a backlog means the link is down, and the newest completion is
        // the one that describes where the robot actually is.
        result_head_ = (result_head_ + 1) % kResultQueueDepth;
        result_count_--;
    }
    const size_t slot = (result_head_ + result_count_) % kResultQueueDepth;
    results_[slot] = result;
    result_count_++;
}

bool ProfiledDrive::PopResult(MotionResult& result) {
    if (result_count_ == 0) {
        return false;
    }
    result = results_[result_head_];
    result_head_ = (result_head_ + 1) % kResultQueueDepth;
    result_count_--;
    return true;
}

}  // namespace nilo
