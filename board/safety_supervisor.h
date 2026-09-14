// The protections that must work when nothing else does.
//
// Plain C++, like the motion planner, and for a stronger reason: this is the code that has
// to be right when the network is gone, the backend has crashed, and the only thing still
// running is this chip. Code that can only be exercised on hardware is code whose
// behaviour on a bad day is a matter of opinion.
//
// Five rules, and the backend cannot relax any of them. There is no method here that takes
// a "disable" flag and no field a command can write. A protection a remote peer can switch
// off is a protection that will be off at the worst possible moment.
//
//   cliff detected            -> stop, and refuse to drive towards it
//   obstacle closer than the  -> stop, and refuse to drive towards it
//     configured clearance
//   picked up                 -> stop
//   the link died mid-motion  -> stop
//   the command heartbeat     -> stop
//     expired
//
// Configurable numbers exist (clearance, timeouts) but each is clamped to a ceiling this
// file owns, so a bad configuration makes the robot more cautious and never less.

#pragma once

#include <cstdint>

namespace nilo {

// Why motion stopped or was refused. Reported to the backend and logged locally; the
// backend never sees a bare boolean, because "no" and "no, there is a drop" are different
// answers to a person standing in the room.
enum class SafetyReason : uint8_t {
    kNone = 0,
    kCliff,
    kObstacle,
    kBump,
    kPickedUp,
    kLinkLost,
    kHeartbeatExpired,
    kEmergencyStop,
    kBatteryCritical,
    kOutOfRange,
    kMotionTimeout,
};

const char* SafetyReasonName(SafetyReason reason);

// What the sensors say right now. Filled by the board's poller; the supervisor never reads
// a pin itself, which is what makes it testable and what keeps one copy of each rule.
struct SensorSnapshot {
    bool cliff_left = false;
    bool cliff_right = false;
    bool bump = false;
    bool picked_up = false;
    int32_t front_distance_mm = 2000;
    int32_t battery_percent = 100;
    bool charging = false;
    bool link_up = true;
    // Monotonic milliseconds when the backend last said anything. Not wall clock: a device
    // whose clock jumps must not decide its heartbeat expired an hour ago.
    int64_t last_command_ms = 0;
};

// The tunable half. Every field is clamped by the constructor to something this file
// considers safe, so a configuration mistake cannot widen a protection.
struct SafetyConfig {
    int32_t min_clearance_mm = 120;  // closer than this and forward motion is refused
    int32_t critical_battery_percent = 5;
    int32_t heartbeat_timeout_ms = 5000;  // silence from the backend that ends motion
    int32_t link_grace_ms = 1000;         // how long a dropped link may last mid-motion
    int32_t max_motion_ms = 20000;        // no single motion may run longer than this

    // Hard ceilings. A configuration is clamped into these; nothing can raise them.
    static constexpr int32_t kMinClearanceFloorMm = 40;
    static constexpr int32_t kMaxHeartbeatTimeoutMs = 10000;
    static constexpr int32_t kMaxMotionMs = 30000;
    static constexpr int32_t kMaxLinkGraceMs = 3000;

    SafetyConfig Clamped() const;
};

// Whether a thing may happen, and why not when it may not.
struct SafetyVerdict {
    bool allowed = true;
    SafetyReason reason = SafetyReason::kNone;

    static SafetyVerdict Allow() { return SafetyVerdict{true, SafetyReason::kNone}; }
    static SafetyVerdict Refuse(SafetyReason reason) { return SafetyVerdict{false, reason}; }
};

// One direction of travel, because a cliff in front is not a reason to refuse backing away
// from it. This distinction is the difference between a robot that recovers and a robot
// that has to be picked up.
enum class Direction : uint8_t { kForward, kBackward, kRotate, kStationary };

Direction DirectionOf(int32_t signed_amount);

class SafetySupervisor {
public:
    explicit SafetySupervisor(const SafetyConfig& config = SafetyConfig());

    const SafetyConfig& config() const { return config_; }

    // May a new motion start? Evaluated at admission, before anything turns.
    SafetyVerdict Admit(Direction direction, const SensorSnapshot& sensors, int64_t now_ms) const;

    // Must the motion in flight stop *now*? Evaluated every control tick.
    SafetyVerdict Supervise(Direction direction, const SensorSnapshot& sensors, int64_t now_ms,
                            int64_t motion_started_ms) const;

    // The emergency-stop latch. Local, sticky, and not something the backend can clear
    // while the condition that set it is still true.
    void EngageEmergencyStop(SafetyReason reason);
    bool ClearEmergencyStop(const SensorSnapshot& sensors, int64_t now_ms);
    bool emergency_stopped() const { return estop_; }
    SafetyReason emergency_reason() const { return estop_reason_; }

private:
    SafetyVerdict CheckHazards(Direction direction, const SensorSnapshot& sensors) const;

    SafetyConfig config_;
    bool estop_ = false;
    SafetyReason estop_reason_ = SafetyReason::kNone;
};

}  // namespace nilo
