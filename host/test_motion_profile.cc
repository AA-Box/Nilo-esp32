// The motion planner: the trapezoid, the triangle it degenerates to, and the calibration
// knobs a real chassis needs.

#include "check.h"
#include "motion_profile.h"

using namespace nilo;

namespace {

void TrapezoidCoversTheDistance() {
    MotionLimits limits;
    const MotionPlan plan = PlanMove(500, 200, limits);

    CHECK(plan.cruise_speed_mmps == 200);
    CHECK(plan.accel_ms > 0);
    CHECK(plan.cruise_ms > 0);
    CHECK_EQ(plan.accel_ms, plan.decel_ms);
    // The integrated distance is the commanded distance: a planner that plans 500 and
    // drives 460 is a robot that stops short of everything.
    CHECK_NEAR(DistanceAtMs(plan, plan.TotalMs()), 500, 2);
}

void AShortMoveNeverReachesCruiseSpeed() {
    MotionLimits limits;
    // Far too short to reach 300 mm/s and stop again. A planner that misses this case
    // commands full speed and overshoots.
    const MotionPlan plan = PlanMove(20, 300, limits);

    CHECK(plan.cruise_speed_mmps < 300);
    // Very nearly no cruise phase: integer arithmetic leaves a millimetre sliver rather
    // than rounding the ramp up, because rounding it up would overshoot.
    CHECK(plan.cruise_ms < 20);
    CHECK_NEAR(DistanceAtMs(plan, plan.TotalMs()), 20, 2);
}

void SpeedRampsUpAndBackDown() {
    MotionLimits limits;
    const MotionPlan plan = PlanMove(600, 200, limits);

    CHECK_EQ(SpeedAtMs(plan, 0), 0);
    CHECK(SpeedAtMs(plan, plan.accel_ms / 2) > 0);
    CHECK(SpeedAtMs(plan, plan.accel_ms / 2) < 200);
    CHECK_EQ(SpeedAtMs(plan, plan.accel_ms + plan.cruise_ms / 2), 200);
    CHECK(SpeedAtMs(plan, plan.TotalMs() - 1) < 200);
    // Past the end is zero, so a loop that ticks late leaves the wheels stopped rather
    // than holding the last speed it saw.
    CHECK_EQ(SpeedAtMs(plan, plan.TotalMs()), 0);
    CHECK_EQ(SpeedAtMs(plan, plan.TotalMs() + 5000), 0);
}

void NegativeDistanceDrivesBackwards() {
    MotionLimits limits;
    const MotionPlan plan = PlanMove(-300, 200, limits);

    CHECK(plan.distance_mm < 0);
    CHECK(plan.cruise_speed_mmps > 0);
    CHECK(SpeedAtMs(plan, plan.accel_ms + 1) < 0);
    CHECK_NEAR(DistanceAtMs(plan, plan.TotalMs()), -300, 2);
}

void SpeedIsClampedToTheLimits() {
    MotionLimits limits;
    limits.max_speed_mmps = 150;

    CHECK_EQ(PlanMove(1000, 900, limits).cruise_speed_mmps, 150);
    // A request below the stall speed is raised, not honoured: motors that do not turn are
    // not slower, they are stopped and hot.
    CHECK_EQ(PlanMove(1000, 1, limits).cruise_speed_mmps, limits.min_speed_mmps);
    // Zero means "you choose", because a caller that does not care should get a sane
    // default rather than a robot that refuses to move.
    CHECK_EQ(PlanMove(1000, 0, limits).cruise_speed_mmps, 150);
}

void ZeroDistanceIsAnEmptyPlan() {
    MotionLimits limits;
    CHECK(PlanMove(0, 200, limits).IsEmpty());
    CHECK_EQ(PlanMove(0, 200, limits).TotalMs(), 0);
}

void ATurnBecomesAnArc() {
    MotionLimits limits;
    ChassisCalibration chassis;
    chassis.track_width_mm = 140;

    // Half a turn of a 140 mm track: pi * 70 mm of arc per wheel, near enough 220 mm.
    CHECK_NEAR(TurnArcMm(180, chassis), 219, 3);
    const MotionPlan plan = PlanTurn(90, 90, limits, chassis);
    CHECK(plan.distance_mm > 0);
    CHECK_NEAR(plan.distance_mm, TurnArcMm(90, chassis), 1);
    // A 90 degree turn at 90 deg/s takes about a second.
    CHECK_NEAR(plan.TotalMs(), 1000, 400);
}

void ANegativeAngleTurnsTheOtherWay() {
    MotionLimits limits;
    ChassisCalibration chassis;
    CHECK(PlanTurn(-90, 90, limits, chassis).distance_mm < 0);
    CHECK(PlanTurn(90, 90, limits, chassis).distance_mm > 0);
}

void TheTrimIsTheOnlyPlaceWheelsDiffer() {
    ChassisCalibration chassis;
    const WheelSpeeds even = SplitStraight(200, chassis);
    CHECK_EQ(even.left_mmps, even.right_mmps);

    // A robot that curves left is corrected here and nowhere else.
    chassis.speed_trim_permille = 50;
    const WheelSpeeds trimmed = SplitStraight(200, chassis);
    CHECK(trimmed.left_mmps > trimmed.right_mmps);
    CHECK_EQ(trimmed.left_mmps + trimmed.right_mmps, 400);
}

void ATurnDrivesTheWheelsInOppositeDirections() {
    ChassisCalibration chassis;
    const WheelSpeeds speeds = SplitTurn(200, chassis);
    CHECK(speeds.left_mmps < 0);
    CHECK(speeds.right_mmps > 0);
}

void TheDeadlineIsGenerousButFinite() {
    MotionLimits limits;
    const MotionPlan plan = PlanMove(500, 200, limits);
    CHECK(DeadlineMs(plan) > plan.TotalMs());
    CHECK(DeadlineMs(plan) < plan.TotalMs() * 2 + 1000);
    CHECK_EQ(DeadlineMs(PlanMove(0, 200, limits)), 0);
}

}  // namespace

int main() {
    TrapezoidCoversTheDistance();
    AShortMoveNeverReachesCruiseSpeed();
    SpeedRampsUpAndBackDown();
    NegativeDistanceDrivesBackwards();
    SpeedIsClampedToTheLimits();
    ZeroDistanceIsAnEmptyPlan();
    ATurnBecomesAnArc();
    ANegativeAngleTurnsTheOtherWay();
    TheTrimIsTheOnlyPlaceWheelsDiffer();
    ATurnDrivesTheWheelsInOppositeDirections();
    TheDeadlineIsGenerousButFinite();
    return check::Report("motion_profile");
}
