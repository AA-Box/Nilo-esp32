// The robot, assembled: the control loop, the notifications, and the one place that knows
// both the device state machine and the drive controller.
//
// Three responsibilities, and they are together because separating them would mean three
// tasks racing for the same drive controller:
//
//   1. Tick.          Sample the sensors, hand them to the drive controller, tick it, tick
//                     the face. Every ROBOT_TICK_MS, on its own task, at a priority above
//                     the protocol so a busy network cannot stall a moving robot.
//   2. Report.        Turn what happened into backend notifications: telemetry, pose,
//                     battery, obstacle, cliff, touch, motion started/completed/failed,
//                     emergency stop.
//   3. Follow state.  Watch the firmware's own device state machine and react to it,
//                     rather than inventing a second one. The robot has no state enum of
//                     its own; it derives what it needs from the device's.
//
// The loop keeps running when the network does not. That is the point: every protection in
// safety_supervisor.h is evaluated here, on this chip, on a task that has no dependency on
// a socket being open.

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <string>

#include "face_display.h"
#include "profiled_drive.h"
#include "robot_actuators.h"
#include "robot_hardware.h"
#include "robot_sensing.h"
#include "robot_tool_schema.h"

class Camera;

namespace nilo {

class RobotService {
public:
    RobotService(MotorDriver& motors, HeadController& head, LiftController& lift,
                 DisplayExpressionController& face, GpioRobotSensors& sensors,
                 BatteryMonitor& battery, Camera* camera);

    // Start the control loop. Separate from the constructor so a board can build everything
    // and only then let the wheels become live.
    void Start();
    void Stop();

    DriveController& drive() { return drive_; }
    HeadController& head() { return head_; }
    LiftController& lift() { return lift_; }
    DisplayExpressionController& face() { return face_; }
    const ToolSpec* tools() const { return tools_; }

    // The backend said something. Feeds the heartbeat: silence is what the watchdog
    // measures, and only this call resets it.
    void NoteBackendActivity();

    // Answers for the read-only tools. Strings rather than cJSON so the tool layer stays
    // free of allocation lifetimes it would have to get right.
    std::string StatusJson();
    std::string DistanceJson();
    std::string ImuJson();
    std::string CliffJson();
    std::string BatteryJson();
    std::string Capture(const std::string& question);

private:
    static void TaskEntry(void* argument);
    void Loop();
    void Tick();
    void ReportMotionResults();
    void ReportPeriodic(int64_t now_ms);
    void ReportSensorChanges(const SensorSnapshot& sensors, int64_t now_ms);
    void FollowDeviceState();
    void Notify(const char* method, const std::string& params);
    std::string PosePayload() const;
    std::string MotionPayload() const;
    std::string SensorPayload(const SensorSnapshot& sensors) const;
    std::string BatteryPayload();

    MotorDriver& motors_;
    HeadController& head_;
    LiftController& lift_;
    DisplayExpressionController& face_;
    GpioRobotSensors& sensors_;
    BatteryMonitor& battery_;
    Camera* camera_;

    MotionLimits limits_;
    ChassisCalibration chassis_;
    SafetyConfig safety_config_;
    SafetySupervisor safety_;
    ProfiledDrive drive_;
    const ToolSpec* tools_ = nullptr;

    TaskHandle_t task_ = nullptr;
    bool running_ = false;
    int64_t last_command_ms_ = 0;
    int64_t next_telemetry_ms_ = 0;
    int64_t next_pose_ms_ = 0;
    bool last_cliff_ = false;
    bool last_bump_ = false;
    bool last_touch_ = false;
    bool last_obstacle_ = false;
    bool last_estop_ = false;
    int32_t last_battery_percent_ = -1;
    int last_device_state_ = -1;
};

}  // namespace nilo
