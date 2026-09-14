#include "safety_supervisor.h"

#include <algorithm>
#include <cstdlib>

namespace nilo {
namespace {

int32_t Clamp(int32_t value, int32_t low, int32_t high) {
    return std::max(low, std::min(high, value));
}

}  // namespace

const char* SafetyReasonName(SafetyReason reason) {
    switch (reason) {
        case SafetyReason::kNone:
            return "none";
        case SafetyReason::kCliff:
            return "cliff";
        case SafetyReason::kObstacle:
            return "obstacle";
        case SafetyReason::kBump:
            return "bump";
        case SafetyReason::kPickedUp:
            return "picked_up";
        case SafetyReason::kLinkLost:
            return "link_lost";
        case SafetyReason::kHeartbeatExpired:
            return "heartbeat_expired";
        case SafetyReason::kEmergencyStop:
            return "emergency_stop";
        case SafetyReason::kBatteryCritical:
            return "battery_critical";
        case SafetyReason::kOutOfRange:
            return "out_of_range";
        case SafetyReason::kMotionTimeout:
            return "motion_timeout";
    }
    return "unknown";
}

SafetyConfig SafetyConfig::Clamped() const {
    SafetyConfig clamped = *this;
    // Each of these clamps in the cautious direction only. A configuration mistake makes
    // the robot more careful; it can never make it less.
    clamped.min_clearance_mm = std::max(kMinClearanceFloorMm, min_clearance_mm);
    clamped.critical_battery_percent = Clamp(critical_battery_percent, 0, 30);
    clamped.heartbeat_timeout_ms = Clamp(heartbeat_timeout_ms, 500, kMaxHeartbeatTimeoutMs);
    clamped.link_grace_ms = Clamp(link_grace_ms, 0, kMaxLinkGraceMs);
    clamped.max_motion_ms = Clamp(max_motion_ms, 500, kMaxMotionMs);
    return clamped;
}

Direction DirectionOf(int32_t signed_amount) {
    if (signed_amount > 0) {
        return Direction::kForward;
    }
    if (signed_amount < 0) {
        return Direction::kBackward;
    }
    return Direction::kStationary;
}

SafetySupervisor::SafetySupervisor(const SafetyConfig& config) : config_(config.Clamped()) {}

SafetyVerdict SafetySupervisor::CheckHazards(Direction direction,
                                             const SensorSnapshot& sensors) const {
    if (sensors.picked_up) {
        // Wheels spinning in somebody's hands. Always wrong, in every direction.
        return SafetyVerdict::Refuse(SafetyReason::kPickedUp);
    }
    if (direction == Direction::kStationary) {
        return SafetyVerdict::Allow();
    }
    const bool towards_hazard = direction == Direction::kForward || direction == Direction::kRotate;
    if (towards_hazard && (sensors.cliff_left || sensors.cliff_right)) {
        // Backing away from a drop is how a robot recovers from finding one, so a cliff
        // refuses forward and rotation but not reverse.
        return SafetyVerdict::Refuse(SafetyReason::kCliff);
    }
    if (direction == Direction::kForward && sensors.bump) {
        return SafetyVerdict::Refuse(SafetyReason::kBump);
    }
    if (direction == Direction::kForward && sensors.front_distance_mm < config_.min_clearance_mm) {
        return SafetyVerdict::Refuse(SafetyReason::kObstacle);
    }
    return SafetyVerdict::Allow();
}

SafetyVerdict SafetySupervisor::Admit(Direction direction, const SensorSnapshot& sensors,
                                      int64_t now_ms) const {
    if (estop_) {
        return SafetyVerdict::Refuse(SafetyReason::kEmergencyStop);
    }
    if (direction != Direction::kStationary && !sensors.charging &&
        sensors.battery_percent <= config_.critical_battery_percent) {
        // Not a hazard to anybody but the robot: a brownout mid-motion leaves it wherever
        // it stopped, with no idea where that is.
        return SafetyVerdict::Refuse(SafetyReason::kBatteryCritical);
    }
    if (direction != Direction::kStationary && !sensors.link_up) {
        return SafetyVerdict::Refuse(SafetyReason::kLinkLost);
    }
    if (direction != Direction::kStationary &&
        now_ms - sensors.last_command_ms > config_.heartbeat_timeout_ms) {
        return SafetyVerdict::Refuse(SafetyReason::kHeartbeatExpired);
    }
    return CheckHazards(direction, sensors);
}

SafetyVerdict SafetySupervisor::Supervise(Direction direction, const SensorSnapshot& sensors,
                                          int64_t now_ms, int64_t motion_started_ms) const {
    if (estop_) {
        return SafetyVerdict::Refuse(SafetyReason::kEmergencyStop);
    }
    const SafetyVerdict hazard = CheckHazards(direction, sensors);
    if (!hazard.allowed) {
        return hazard;
    }
    if (!sensors.link_up && now_ms - sensors.last_command_ms > config_.link_grace_ms) {
        // The socket died while the robot was moving. Nothing is coming to stop it, so it
        // stops itself. This is the case the backend cannot help with by construction.
        return SafetyVerdict::Refuse(SafetyReason::kLinkLost);
    }
    if (now_ms - sensors.last_command_ms > config_.heartbeat_timeout_ms) {
        return SafetyVerdict::Refuse(SafetyReason::kHeartbeatExpired);
    }
    if (now_ms - motion_started_ms > config_.max_motion_ms) {
        // A motion that has run past its ceiling is a stuck wheel, a bad plan, or a lost
        // completion. All three end the same way.
        return SafetyVerdict::Refuse(SafetyReason::kMotionTimeout);
    }
    return SafetyVerdict::Allow();
}

void SafetySupervisor::EngageEmergencyStop(SafetyReason reason) {
    estop_ = true;
    estop_reason_ = reason == SafetyReason::kNone ? SafetyReason::kEmergencyStop : reason;
}

bool SafetySupervisor::ClearEmergencyStop(const SensorSnapshot& sensors, int64_t now_ms) {
    (void)now_ms;
    if (!estop_) {
        return true;
    }
    // A latch that clears while the robot is still on the edge of the table is not a
    // latch. The condition has to be gone before the robot will believe the operator.
    if (sensors.picked_up || sensors.cliff_left || sensors.cliff_right) {
        return false;
    }
    estop_ = false;
    estop_reason_ = SafetyReason::kNone;
    return true;
}

}  // namespace nilo
