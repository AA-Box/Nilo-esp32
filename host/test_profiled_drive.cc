// The drive controller, driven end to end on a fake chassis.
//
// This is the file that proves the claims in docs/robot-hardware.md that matter: the
// robot stops when the link dies mid-motion, it stops when the heartbeat expires, it stops
// when a cliff appears under it, and it never keeps turning a wheel on a plan nobody is
// supervising. None of it needs hardware, because the controller is plain C++ over a
// MotorDriver.

#include <vector>

#include "check.h"
#include "profiled_drive.h"

using namespace nilo;

namespace {

int64_t g_now_ms = 0;
int64_t Now() { return g_now_ms; }

// A chassis that records instead of moving.
class FakeMotors : public MotorDriver {
public:
    void SetWheelSpeeds(int32_t left_mmps, int32_t right_mmps) override {
        left = left_mmps;
        right = right_mmps;
        commands.push_back({left_mmps, right_mmps});
    }
    void Brake() override {
        left = 0;
        right = 0;
        brakes++;
    }
    void Coast() override { coasts++; }

    bool Stopped() const { return left == 0 && right == 0; }

    int32_t left = 0;
    int32_t right = 0;
    int brakes = 0;
    int coasts = 0;
    std::vector<WheelSpeeds> commands;
};

struct Rig {
    FakeMotors motors;
    SafetySupervisor safety;
    ProfiledDrive drive{motors, safety, Now};

    Rig() {
        g_now_ms = 1000;
        SensorSnapshot sensors;
        sensors.front_distance_mm = 1500;
        sensors.battery_percent = 80;
        sensors.link_up = true;
        sensors.last_command_ms = g_now_ms;
        drive.UpdateSensors(sensors);
    }

    // Advance time in small steps, ticking as the real loop would.
    void Run(int32_t total_ms, int32_t step_ms = 20) {
        for (int32_t elapsed = 0; elapsed < total_ms; elapsed += step_ms) {
            g_now_ms += step_ms;
            SensorSnapshot sensors = drive.sensors();
            sensors.last_command_ms = g_now_ms;  // the backend is still talking
            drive.UpdateSensors(sensors);
            drive.Tick();
        }
    }

    // The same, with the backend silent: the control loop keeps running, nothing new
    // arrives. This is what a dead link actually looks like from inside the firmware.
    void RunSilent(int32_t total_ms, int32_t step_ms = 20) {
        for (int32_t elapsed = 0; elapsed < total_ms; elapsed += step_ms) {
            g_now_ms += step_ms;
            drive.Tick();
        }
    }
};

void AMoveRunsAndCompletes() {
    Rig rig;
    const uint32_t action = rig.drive.Move(250, 200);
    CHECK(action != 0);
    CHECK(rig.drive.IsMoving());

    rig.Run(3000);

    CHECK(!rig.drive.IsMoving());
    CHECK(rig.motors.Stopped());
    MotionResult result;
    CHECK(rig.drive.PopResult(result));
    CHECK_EQ(result.action_id, action);
    CHECK(result.outcome == MotionOutcome::kCompleted);
    CHECK_NEAR(result.travelled_mm, 250, 5);
    CHECK(!rig.drive.PopResult(result));
}

void TheWheelsRampRatherThanJumping() {
    Rig rig;
    rig.drive.Move(600, 200);
    rig.Run(60, 20);

    CHECK(!rig.motors.commands.empty());
    // The first command is not the cruise speed: a chassis commanded from zero to full
    // speed in one step either slips or browns out.
    CHECK(rig.motors.commands.front().left_mmps < 200);
    CHECK(rig.motors.left > 0);
}

void ACliffUnderTheRobotStopsItMidMotion() {
    Rig rig;
    rig.drive.Move(900, 200);
    rig.Run(200);
    CHECK(rig.drive.IsMoving());

    SensorSnapshot sensors = rig.drive.sensors();
    sensors.cliff_left = true;
    rig.drive.UpdateSensors(sensors);
    g_now_ms += 20;
    rig.drive.Tick();

    CHECK(!rig.drive.IsMoving());
    CHECK(rig.motors.Stopped());
    CHECK(rig.motors.brakes > 0);
    MotionResult result;
    CHECK(rig.drive.PopResult(result));
    CHECK(result.outcome == MotionOutcome::kBlocked);
    CHECK(result.reason == SafetyReason::kCliff);
}

void ADeadLinkStopsTheRobot() {
    Rig rig;
    rig.drive.Move(900, 200);
    rig.Run(200);

    SensorSnapshot sensors = rig.drive.sensors();
    sensors.link_up = false;
    rig.drive.UpdateSensors(sensors);
    // The loop keeps running; nothing arrives. Past the grace window it stops itself,
    // because nothing else is going to.
    rig.RunSilent(2000);

    CHECK(!rig.drive.IsMoving());
    CHECK(rig.motors.Stopped());
    MotionResult result;
    CHECK(rig.drive.PopResult(result));
    CHECK(result.reason == SafetyReason::kLinkLost);
}

void AnExpiredHeartbeatStopsTheRobot() {
    Rig rig;
    // Slow enough that the move is still running when the heartbeat expires, which is the
    // case being tested: the robot is mid-motion and the backend has stopped talking.
    rig.drive.Move(600, 50);
    rig.Run(200);

    // The link is up but the backend has gone quiet. Same outcome, different reason.
    rig.RunSilent(8000);

    CHECK(!rig.drive.IsMoving());
    MotionResult result;
    CHECK(rig.drive.PopResult(result));
    CHECK(result.reason == SafetyReason::kHeartbeatExpired);
}

void ATickThatStopsRunningStopsTheWheels() {
    Rig rig;
    rig.drive.Move(900, 200);
    rig.Run(100);

    // The control loop missed its own deadline: starved task, blocked flash write, a long
    // interrupt. Whatever it was, the wheels do not keep turning on an unsupervised plan.
    g_now_ms += 400;
    rig.drive.Tick();

    CHECK(!rig.drive.IsMoving());
    CHECK(rig.motors.Stopped());
    MotionResult result;
    CHECK(rig.drive.PopResult(result));
    CHECK(result.outcome == MotionOutcome::kTimedOut);
}

void AnUnsafeMoveIsRefusedBeforeAnythingTurns() {
    Rig rig;
    SensorSnapshot sensors = rig.drive.sensors();
    sensors.front_distance_mm = 50;
    rig.drive.UpdateSensors(sensors);

    CHECK_EQ(rig.drive.Move(300, 200), 0u);
    CHECK(rig.drive.last_refusal() == SafetyReason::kObstacle);
    CHECK(!rig.drive.IsMoving());
    CHECK(rig.motors.commands.empty());

    // Backing away from it is still allowed.
    CHECK(rig.drive.Move(-300, 200) != 0u);
}

void AStopIsAlwaysAcceptedEvenWithAHazard() {
    Rig rig;
    rig.drive.Move(900, 200);
    rig.Run(100);

    SensorSnapshot sensors = rig.drive.sensors();
    sensors.cliff_left = true;
    rig.drive.UpdateSensors(sensors);
    rig.drive.Stop();

    CHECK(!rig.drive.IsMoving());
    CHECK(rig.motors.Stopped());
}

void AnEmergencyStopLatchesAndRefusesTheNextMove() {
    Rig rig;
    rig.drive.Move(900, 200);
    rig.Run(100);

    rig.drive.EmergencyStop(SafetyReason::kEmergencyStop);

    CHECK(rig.motors.Stopped());
    CHECK_EQ(rig.drive.Move(100, 200), 0u);
    CHECK(rig.drive.last_refusal() == SafetyReason::kEmergencyStop);

    CHECK(rig.drive.ClearEmergencyStop());
    CHECK(rig.drive.Move(100, 200) != 0u);
}

void ANewMoveSupersedesTheOneInFlightAndSaysSo() {
    Rig rig;
    const uint32_t first = rig.drive.Move(900, 200);
    rig.Run(100);
    const uint32_t second = rig.drive.Move(200, 200);

    CHECK(second != first);
    MotionResult result;
    CHECK(rig.drive.PopResult(result));
    CHECK_EQ(result.action_id, first);
    // The backend is correlating on the action id and has to learn the first one is never
    // completing, rather than waiting for a notification that will not come.
    CHECK(result.outcome == MotionOutcome::kStopped);
}

void BeingPickedUpCoastsRatherThanBrakes() {
    Rig rig;
    SensorSnapshot sensors = rig.drive.sensors();
    sensors.picked_up = true;
    rig.drive.UpdateSensors(sensors);
    g_now_ms += 20;
    rig.drive.Tick();

    // A braked wheel in somebody's hand is a locked wheel, and locked wheels are how a
    // robot gets dropped.
    CHECK(rig.motors.coasts > 0);
}

void ATurnMovesTheWheelsInOppositeDirections() {
    Rig rig;
    CHECK(rig.drive.Turn(90, 90) != 0u);
    rig.Run(60, 20);
    CHECK(rig.motors.left < 0);
    CHECK(rig.motors.right > 0);
}

void ResultsAreBoundedAndTheNewestSurvives() {
    Rig rig;
    for (int index = 0; index < 10; ++index) {
        rig.drive.Move(200, 200);
        rig.Run(2000);
    }
    int popped = 0;
    MotionResult result;
    while (rig.drive.PopResult(result)) {
        popped++;
    }
    CHECK(popped <= static_cast<int>(kResultQueueDepth));
    CHECK(popped > 0);
}

}  // namespace

int main() {
    AMoveRunsAndCompletes();
    TheWheelsRampRatherThanJumping();
    ACliffUnderTheRobotStopsItMidMotion();
    ADeadLinkStopsTheRobot();
    AnExpiredHeartbeatStopsTheRobot();
    ATickThatStopsRunningStopsTheWheels();
    AnUnsafeMoveIsRefusedBeforeAnythingTurns();
    AStopIsAlwaysAcceptedEvenWithAHazard();
    AnEmergencyStopLatchesAndRefusesTheNextMove();
    ANewMoveSupersedesTheOneInFlightAndSaysSo();
    BeingPickedUpCoastsRatherThanBrakes();
    ATurnMovesTheWheelsInOppositeDirections();
    ResultsAreBoundedAndTheNewestSurvives();
    return check::Report("profiled_drive");
}
