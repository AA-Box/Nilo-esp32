// Motion planning, with no hardware in it.
//
// Everything in this file is plain C++: no ESP-IDF, no FreeRTOS, no GPIO. That is
// deliberate and it is the point of the split. The question "how fast should the wheels be
// turning 380 ms into a 250 mm move" has one right answer, it is easy to get subtly wrong,
// and it is impossible to test on a desk if it is tangled up with a PWM register.
//
// The profile is trapezoidal: accelerate to a cruise speed, hold it, decelerate to a stop.
// When the distance is too short to reach the cruise speed the profile degenerates to a
// triangle, which is the case a naive implementation overshoots on.
//
// Every number here is a calibration knob, not a constant. A real chassis never matches
// the paper one: wheels are moulded a percent small, gearboxes have backlash, one motor is
// always slightly faster than the other, and battery voltage moves all of it. A model that
// cannot be tuned is wrong on the bench.

#pragma once

#include <cstdint>

namespace nilo {

// What the chassis can physically do. Every field is tunable, and none of them may be
// raised past the hard ceilings the safety supervisor enforces.
struct MotionLimits {
    int32_t max_speed_mmps = 300;          // straight-line speed ceiling
    int32_t min_speed_mmps = 30;           // below this the motors stall instead of turning
    int32_t max_accel_mmps2 = 400;         // how hard it may accelerate or brake
    int32_t max_turn_dps = 120;            // rotation speed ceiling
    int32_t max_turn_accel_dps2 = 240;     // rotational acceleration
    int32_t emergency_decel_mmps2 = 1200;  // braking used for a stop, not for a plan
};

// The physical machine, as measured rather than as designed.
//
// `speed_trim_permille` is the one that surprises people: two identical motors are not
// identical, and a robot commanded to drive straight curves until this is set. Positive
// values speed the left wheel up relative to the right.
struct ChassisCalibration {
    int32_t wheel_diameter_mm = 65;
    int32_t track_width_mm = 140;            // centre to centre between the wheels
    int32_t gear_ratio_permille = 1000;      // output turns per motor turn, x1000
    int32_t speed_trim_permille = 0;         // left/right balance, -200..200
    int32_t distance_scale_permille = 1000;  // measured distance / commanded distance, x1000
    int32_t turn_scale_permille = 1000;      // measured angle / commanded angle, x1000
};

// A planned move: the shape of the speed curve, in milliseconds and mm/s.
struct MotionPlan {
    int32_t distance_mm = 0;        // signed: negative drives backwards
    int32_t cruise_speed_mmps = 0;  // always positive; `distance_mm` carries the direction
    int32_t accel_ms = 0;
    int32_t cruise_ms = 0;
    int32_t decel_ms = 0;

    int32_t TotalMs() const { return accel_ms + cruise_ms + decel_ms; }
    bool IsEmpty() const { return distance_mm == 0 || cruise_speed_mmps == 0; }
};

// Plan a straight move. `requested_speed_mmps` is clamped into the limits; a zero or
// negative request means "use the ceiling", because a caller that does not care should get
// the sane default rather than a robot that refuses to move.
MotionPlan PlanMove(int32_t distance_mm, int32_t requested_speed_mmps, const MotionLimits& limits);

// Plan a turn in place. The wheels travel in opposite directions along the track circle,
// so the same trapezoid applies once the angle is converted to arc length.
MotionPlan PlanTurn(int32_t angle_deg, int32_t requested_speed_dps, const MotionLimits& limits,
                    const ChassisCalibration& chassis);

// Speed at a point in time, in mm/s, signed. Outside the plan it is zero, which is what
// makes the caller's loop safe to run one tick late.
int32_t SpeedAtMs(const MotionPlan& plan, int32_t elapsed_ms);

// Distance covered by the plan up to `elapsed_ms`, in millimetres, signed.
int32_t DistanceAtMs(const MotionPlan& plan, int32_t elapsed_ms);

// The two wheel speeds for a commanded body speed, with the trim applied. This is the only
// place left/right asymmetry exists, so a chassis that pulls is corrected in one number.
struct WheelSpeeds {
    int32_t left_mmps = 0;
    int32_t right_mmps = 0;
};

WheelSpeeds SplitStraight(int32_t speed_mmps, const ChassisCalibration& chassis);
WheelSpeeds SplitTurn(int32_t speed_mmps, const ChassisCalibration& chassis);

// The arc one wheel travels when the robot turns `angle_deg` in place, in millimetres.
int32_t TurnArcMm(int32_t angle_deg, const ChassisCalibration& chassis);

// How long a plan should be given before it is called late, in milliseconds. The watchdog
// uses this: a move that has not reported completion by its deadline is a fault, not a
// slow move.
int32_t DeadlineMs(const MotionPlan& plan);

}  // namespace nilo
