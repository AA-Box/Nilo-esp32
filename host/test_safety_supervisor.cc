// The local protections, one test each, plus the two properties that matter most: they
// cannot be widened by configuration, and they cannot be turned off by the backend.

#include "check.h"
#include "safety_supervisor.h"

using namespace nilo;

namespace {

SensorSnapshot Healthy(int64_t now_ms = 1000) {
    SensorSnapshot sensors;
    sensors.front_distance_mm = 1500;
    sensors.battery_percent = 80;
    sensors.link_up = true;
    sensors.last_command_ms = now_ms;
    return sensors;
}

void AHealthyRobotMayMove() {
    SafetySupervisor safety;
    const SensorSnapshot sensors = Healthy();
    CHECK(safety.Admit(Direction::kForward, sensors, 1000).allowed);
    CHECK(safety.Supervise(Direction::kForward, sensors, 1000, 900).allowed);
}

void ACliffRefusesForwardButNotReverse() {
    SafetySupervisor safety;
    SensorSnapshot sensors = Healthy();
    sensors.cliff_left = true;

    CHECK(!safety.Admit(Direction::kForward, sensors, 1000).allowed);
    CHECK(safety.Admit(Direction::kForward, sensors, 1000).reason == SafetyReason::kCliff);
    CHECK(!safety.Admit(Direction::kRotate, sensors, 1000).allowed);
    // Backing away from a drop is how a robot recovers from finding one.
    CHECK(safety.Admit(Direction::kBackward, sensors, 1000).allowed);
}

void ACliffStopsAMotionInFlight() {
    SafetySupervisor safety;
    SensorSnapshot sensors = Healthy();
    sensors.cliff_right = true;
    const SafetyVerdict verdict = safety.Supervise(Direction::kForward, sensors, 1000, 900);
    CHECK(!verdict.allowed);
    CHECK(verdict.reason == SafetyReason::kCliff);
}

void AnObstacleInsideTheClearanceRefusesForward() {
    SafetySupervisor safety;
    SensorSnapshot sensors = Healthy();
    sensors.front_distance_mm = 60;

    CHECK(!safety.Admit(Direction::kForward, sensors, 1000).allowed);
    CHECK(safety.Admit(Direction::kForward, sensors, 1000).reason == SafetyReason::kObstacle);
    CHECK(safety.Admit(Direction::kBackward, sensors, 1000).allowed);
}

void BeingPickedUpRefusesEverything() {
    SafetySupervisor safety;
    SensorSnapshot sensors = Healthy();
    sensors.picked_up = true;

    CHECK(!safety.Admit(Direction::kForward, sensors, 1000).allowed);
    CHECK(!safety.Admit(Direction::kBackward, sensors, 1000).allowed);
    CHECK(!safety.Admit(Direction::kRotate, sensors, 1000).allowed);
    CHECK(safety.Admit(Direction::kForward, sensors, 1000).reason == SafetyReason::kPickedUp);
}

void ADeadLinkStopsAMotionInFlight() {
    SafetySupervisor safety;
    SensorSnapshot sensors = Healthy(1000);
    sensors.link_up = false;

    // Inside the grace window the motion continues: a momentary reconnect is not a fault.
    CHECK(safety.Supervise(Direction::kForward, sensors, 1500, 900).allowed);
    // Past it, nothing is coming to stop the robot, so it stops itself.
    const SafetyVerdict verdict = safety.Supervise(Direction::kForward, sensors, 3000, 900);
    CHECK(!verdict.allowed);
    CHECK(verdict.reason == SafetyReason::kLinkLost);
}

void AnExpiredHeartbeatStopsAMotionInFlight() {
    SafetySupervisor safety;
    const SensorSnapshot sensors = Healthy(1000);
    const SafetyVerdict verdict = safety.Supervise(Direction::kForward, sensors, 9000, 900);
    CHECK(!verdict.allowed);
    CHECK(verdict.reason == SafetyReason::kHeartbeatExpired);
}

void AMotionThatRunsTooLongIsStopped() {
    SafetySupervisor safety;
    SensorSnapshot sensors = Healthy(100000);
    const SafetyVerdict verdict = safety.Supervise(Direction::kForward, sensors, 100000, 10000);
    CHECK(!verdict.allowed);
    CHECK(verdict.reason == SafetyReason::kMotionTimeout);
}

void ACriticalBatteryRefusesMotion() {
    SafetySupervisor safety;
    SensorSnapshot sensors = Healthy();
    sensors.battery_percent = 3;
    CHECK(!safety.Admit(Direction::kForward, sensors, 1000).allowed);
    CHECK(safety.Admit(Direction::kForward, sensors, 1000).reason ==
          SafetyReason::kBatteryCritical);

    // On the dock it is fine: the robot is not about to brown out mid-move.
    sensors.charging = true;
    CHECK(safety.Admit(Direction::kForward, sensors, 1000).allowed);
}

void TheEmergencyStopLatchesAndRefusesEverything() {
    SafetySupervisor safety;
    const SensorSnapshot sensors = Healthy();

    safety.EngageEmergencyStop(SafetyReason::kEmergencyStop);
    CHECK(safety.emergency_stopped());
    CHECK(!safety.Admit(Direction::kForward, sensors, 1000).allowed);
    CHECK(!safety.Admit(Direction::kBackward, sensors, 1000).allowed);
    CHECK(!safety.Supervise(Direction::kForward, sensors, 1000, 900).allowed);
}

void TheLatchWillNotClearWhileTheHazardIsStillThere() {
    SafetySupervisor safety;
    SensorSnapshot sensors = Healthy();
    sensors.cliff_left = true;
    safety.EngageEmergencyStop(SafetyReason::kCliff);

    // A latch that clears while the robot is still on the edge of the table is not a latch.
    CHECK(!safety.ClearEmergencyStop(sensors, 2000));
    CHECK(safety.emergency_stopped());

    sensors.cliff_left = false;
    CHECK(safety.ClearEmergencyStop(sensors, 3000));
    CHECK(!safety.emergency_stopped());
    CHECK(safety.Admit(Direction::kForward, sensors, 3000).allowed);
}

void ConfigurationCanOnlyMakeItMoreCautious() {
    SafetyConfig reckless;
    reckless.min_clearance_mm = 1;           // "there is no such thing as too close"
    reckless.heartbeat_timeout_ms = 600000;  // "ten minutes of silence is fine"
    reckless.max_motion_ms = 3600000;        // "an hour-long move"
    reckless.link_grace_ms = 120000;         // "two minutes without a link"

    const SafetyConfig clamped = reckless.Clamped();
    CHECK(clamped.min_clearance_mm >= SafetyConfig::kMinClearanceFloorMm);
    CHECK(clamped.heartbeat_timeout_ms <= SafetyConfig::kMaxHeartbeatTimeoutMs);
    CHECK(clamped.max_motion_ms <= SafetyConfig::kMaxMotionMs);
    CHECK(clamped.link_grace_ms <= SafetyConfig::kMaxLinkGraceMs);

    // And a cautious configuration is left alone.
    SafetyConfig careful;
    careful.min_clearance_mm = 400;
    CHECK_EQ(careful.Clamped().min_clearance_mm, 400);
}

void ReasonsHaveNames() {
    CHECK_STREQ(SafetyReasonName(SafetyReason::kCliff), "cliff");
    CHECK_STREQ(SafetyReasonName(SafetyReason::kHeartbeatExpired), "heartbeat_expired");
    CHECK_STREQ(SafetyReasonName(SafetyReason::kNone), "none");
}

}  // namespace

int main() {
    AHealthyRobotMayMove();
    ACliffRefusesForwardButNotReverse();
    ACliffStopsAMotionInFlight();
    AnObstacleInsideTheClearanceRefusesForward();
    BeingPickedUpRefusesEverything();
    ADeadLinkStopsAMotionInFlight();
    AnExpiredHeartbeatStopsAMotionInFlight();
    AMotionThatRunsTooLongIsStopped();
    ACriticalBatteryRefusesMotion();
    TheEmergencyStopLatchesAndRefusesEverything();
    TheLatchWillNotClearWhileTheHazardIsStillThere();
    ConfigurationCanOnlyMakeItMoreCautious();
    ReasonsHaveNames();
    return check::Report("safety_supervisor");
}
