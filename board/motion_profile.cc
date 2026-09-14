#include "motion_profile.h"

#include <algorithm>
#include <cstdlib>

namespace nilo {
namespace {

constexpr int32_t kMilli = 1000;

int32_t Clamp(int32_t value, int32_t low, int32_t high) {
    return std::max(low, std::min(high, value));
}

// Integer square root, so the triangular-profile case needs no floating point. The
// firmware has an FPU on some targets and not on others, and a motion planner that
// behaves differently per target is a motion planner that is only tested on one.
int32_t IntSqrt(int64_t value) {
    if (value <= 0) {
        return 0;
    }
    int64_t guess = value;
    int64_t next = (guess + 1) / 2;
    while (next < guess) {
        guess = next;
        next = (guess + value / guess) / 2;
    }
    return static_cast<int32_t>(guess);
}

// The trapezoid, in one place, because a move and a turn differ only in what the distance
// means. `distance_mm` is unsigned here; the caller puts the sign back.
MotionPlan PlanTrapezoid(int32_t distance_mm, int32_t speed_mmps, const MotionLimits& limits) {
    MotionPlan plan;
    if (distance_mm <= 0) {
        return plan;
    }
    const int32_t accel = std::max<int32_t>(1, limits.max_accel_mmps2);
    int32_t cruise = Clamp(speed_mmps, limits.min_speed_mmps, limits.max_speed_mmps);

    // Distance needed to reach the cruise speed and come back down again. Below it the
    // profile is a triangle and the cruise speed is never reached — the case that
    // overshoots when it is missed.
    int64_t ramp_mm = (static_cast<int64_t>(cruise) * cruise) / accel;
    if (ramp_mm > distance_mm) {
        cruise = IntSqrt(static_cast<int64_t>(distance_mm) * accel);
        cruise = std::max(cruise, std::min(limits.min_speed_mmps, limits.max_speed_mmps));
        ramp_mm = std::min<int64_t>(distance_mm, (static_cast<int64_t>(cruise) * cruise) / accel);
    }

    plan.cruise_speed_mmps = cruise;
    plan.accel_ms = static_cast<int32_t>((static_cast<int64_t>(cruise) * kMilli) / accel);
    plan.decel_ms = plan.accel_ms;
    const int64_t cruise_mm = distance_mm - ramp_mm;
    plan.cruise_ms = cruise > 0 ? static_cast<int32_t>((cruise_mm * kMilli) / cruise) : 0;
    plan.cruise_ms = std::max<int32_t>(0, plan.cruise_ms);
    return plan;
}

}  // namespace

MotionPlan PlanMove(int32_t distance_mm, int32_t requested_speed_mmps, const MotionLimits& limits) {
    const int32_t wanted = requested_speed_mmps > 0 ? requested_speed_mmps : limits.max_speed_mmps;
    MotionPlan plan = PlanTrapezoid(std::abs(distance_mm), wanted, limits);
    plan.distance_mm = distance_mm;
    if (distance_mm == 0) {
        plan.cruise_speed_mmps = 0;
    }
    return plan;
}

int32_t TurnArcMm(int32_t angle_deg, const ChassisCalibration& chassis) {
    // Arc = angle (radians) * radius. 355/113 is pi to seven digits, which keeps this in
    // integers without a floating-point unit and without a table.
    const int64_t radius_mm = chassis.track_width_mm / 2;
    const int64_t arc = (static_cast<int64_t>(std::abs(angle_deg)) * radius_mm * 355) / (113 * 180);
    return static_cast<int32_t>(arc);
}

MotionPlan PlanTurn(int32_t angle_deg, int32_t requested_speed_dps, const MotionLimits& limits,
                    const ChassisCalibration& chassis) {
    const int32_t wanted_dps =
        Clamp(requested_speed_dps > 0 ? requested_speed_dps : limits.max_turn_dps, 1,
              limits.max_turn_dps);
    const int32_t arc_mm = TurnArcMm(angle_deg, chassis);
    // Convert the rotational speed into the linear speed of one wheel on the track circle,
    // so the same trapezoid applies.
    const int32_t wheel_speed =
        angle_deg == 0 ? 0
                       : static_cast<int32_t>((static_cast<int64_t>(arc_mm) * wanted_dps) /
                                              std::max(1, std::abs(angle_deg)));
    MotionPlan plan = PlanTrapezoid(arc_mm, wheel_speed, limits);
    plan.distance_mm = angle_deg >= 0 ? arc_mm : -arc_mm;
    if (angle_deg == 0) {
        plan.cruise_speed_mmps = 0;
    }
    return plan;
}

int32_t SpeedAtMs(const MotionPlan& plan, int32_t elapsed_ms) {
    if (plan.IsEmpty() || elapsed_ms < 0 || elapsed_ms >= plan.TotalMs()) {
        return 0;
    }
    const int32_t sign = plan.distance_mm < 0 ? -1 : 1;
    if (elapsed_ms < plan.accel_ms) {
        const int64_t speed =
            plan.accel_ms > 0
                ? (static_cast<int64_t>(plan.cruise_speed_mmps) * elapsed_ms) / plan.accel_ms
                : 0;
        return sign * static_cast<int32_t>(speed);
    }
    if (elapsed_ms < plan.accel_ms + plan.cruise_ms) {
        return sign * plan.cruise_speed_mmps;
    }
    const int32_t into_decel = elapsed_ms - plan.accel_ms - plan.cruise_ms;
    const int64_t remaining = plan.decel_ms > 0 ? plan.decel_ms - into_decel : 0;
    const int64_t speed =
        plan.decel_ms > 0
            ? (static_cast<int64_t>(plan.cruise_speed_mmps) * remaining) / plan.decel_ms
            : 0;
    return sign * static_cast<int32_t>(std::max<int64_t>(0, speed));
}

int32_t DistanceAtMs(const MotionPlan& plan, int32_t elapsed_ms) {
    if (plan.IsEmpty() || elapsed_ms <= 0) {
        return 0;
    }
    const int32_t sign = plan.distance_mm < 0 ? -1 : 1;
    const int32_t total = plan.TotalMs();
    if (elapsed_ms >= total) {
        return plan.distance_mm;
    }
    int64_t travelled = 0;
    const int32_t accel_part = std::min(elapsed_ms, plan.accel_ms);
    // Area under the ramp: half the peak speed over the time spent on it.
    travelled += plan.accel_ms > 0
                     ? (static_cast<int64_t>(plan.cruise_speed_mmps) * accel_part * accel_part) /
                           (2LL * plan.accel_ms * kMilli)
                     : 0;
    if (elapsed_ms > plan.accel_ms) {
        const int32_t cruise_part = std::min(elapsed_ms - plan.accel_ms, plan.cruise_ms);
        travelled += (static_cast<int64_t>(plan.cruise_speed_mmps) * cruise_part) / kMilli;
    }
    if (elapsed_ms > plan.accel_ms + plan.cruise_ms && plan.decel_ms > 0) {
        const int32_t decel_part = elapsed_ms - plan.accel_ms - plan.cruise_ms;
        const int64_t start = static_cast<int64_t>(plan.cruise_speed_mmps) * decel_part;
        const int64_t lost =
            (static_cast<int64_t>(plan.cruise_speed_mmps) * decel_part * decel_part) /
            (2LL * plan.decel_ms);
        travelled += (start - lost) / kMilli;
    }
    return sign * static_cast<int32_t>(std::min<int64_t>(std::abs(plan.distance_mm), travelled));
}

WheelSpeeds SplitStraight(int32_t speed_mmps, const ChassisCalibration& chassis) {
    const int32_t trim = Clamp(chassis.speed_trim_permille, -200, 200);
    WheelSpeeds speeds;
    speeds.left_mmps =
        static_cast<int32_t>((static_cast<int64_t>(speed_mmps) * (kMilli + trim)) / kMilli);
    speeds.right_mmps =
        static_cast<int32_t>((static_cast<int64_t>(speed_mmps) * (kMilli - trim)) / kMilli);
    return speeds;
}

WheelSpeeds SplitTurn(int32_t speed_mmps, const ChassisCalibration& chassis) {
    const WheelSpeeds straight = SplitStraight(speed_mmps, chassis);
    // Positive angles turn left, so the left wheel goes backwards.
    return WheelSpeeds{-straight.left_mmps, straight.right_mmps};
}

int32_t DeadlineMs(const MotionPlan& plan) {
    if (plan.IsEmpty()) {
        return 0;
    }
    // A quarter over, plus half a second. Generous enough that a loaded CPU is not a fault
    // and tight enough that a stalled motor is one within a second.
    return plan.TotalMs() + plan.TotalMs() / 4 + 500;
}

}  // namespace nilo
