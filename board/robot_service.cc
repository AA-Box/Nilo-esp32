#include "robot_service.h"

#include <esp_log.h>
#include <esp_timer.h>

#include <algorithm>

#include "application.h"
#include "boards/common/camera.h"
#include "config.h"
#include "device_state.h"
#include "settings.h"

#define TAG "NiloRobot"

namespace nilo {
namespace {

// Above the protocol task. A busy network must not be able to delay the loop that decides
// whether the wheels should still be turning.
constexpr UBaseType_t kTaskPriority = 6;
constexpr uint32_t kTaskStack = 4096;

int64_t NowMs() { return esp_timer_get_time() / 1000; }

// Calibration from NVS, falling back to the designed values in config.h. Measured numbers
// belong on the device that was measured, not in a header shared by every build — two
// chassis off the same line do not have the same wheel diameter to a percent.
ChassisCalibration LoadChassis() {
    Settings settings("nilo", false);
    ChassisCalibration chassis;
    chassis.wheel_diameter_mm = settings.GetInt("wheel_mm", CAL_WHEEL_DIAMETER_MM);
    chassis.track_width_mm = settings.GetInt("track_mm", CAL_TRACK_WIDTH_MM);
    chassis.gear_ratio_permille = settings.GetInt("gear", CAL_GEAR_RATIO_PERMILLE);
    chassis.speed_trim_permille = settings.GetInt("trim", CAL_SPEED_TRIM_PERMILLE);
    chassis.distance_scale_permille = settings.GetInt("dist_scale", CAL_DISTANCE_SCALE_PERMILLE);
    chassis.turn_scale_permille = settings.GetInt("turn_scale", CAL_TURN_SCALE_PERMILLE);
    return chassis;
}

MotionLimits LoadLimits() {
    MotionLimits limits;
    limits.max_speed_mmps = LIMIT_MAX_SPEED_MMPS;
    limits.min_speed_mmps = LIMIT_MIN_SPEED_MMPS;
    limits.max_accel_mmps2 = LIMIT_MAX_ACCEL_MMPS2;
    limits.max_turn_dps = LIMIT_MAX_TURN_DPS;
    return limits;
}

SafetyConfig LoadSafety() {
    SafetyConfig config;
    config.min_clearance_mm = LIMIT_MIN_CLEARANCE_MM;
    config.heartbeat_timeout_ms = LIMIT_HEARTBEAT_TIMEOUT_MS;
    config.max_motion_ms = LIMIT_MAX_MOTION_MS;
    // Clamped on the way in as well as inside the supervisor, so the number this board
    // reports as its limit is the number that is actually enforced.
    return config.Clamped();
}

std::string Pair(const char* key, long long value) {
    return "\"" + std::string(key) + "\":" + std::to_string(value);
}

std::string Flag(const char* key, bool value) {
    return "\"" + std::string(key) + "\":" + (value ? "true" : "false");
}

std::string Text(const char* key, const std::string& value) {
    return "\"" + std::string(key) + "\":\"" + value + "\"";
}

}  // namespace

RobotService::RobotService(MotorDriver& motors, HeadController& head, LiftController& lift,
                           DisplayExpressionController& face, GpioRobotSensors& sensors,
                           BatteryMonitor& battery, Camera* camera)
    : motors_(motors),
      head_(head),
      lift_(lift),
      face_(face),
      sensors_(sensors),
      battery_(battery),
      camera_(camera),
      limits_(LoadLimits()),
      chassis_(LoadChassis()),
      safety_config_(LoadSafety()),
      safety_(safety_config_),
      drive_(motors, safety_, NowMs, limits_, chassis_),
      tools_(BuildToolTable(limits_, safety_config_)) {
    // Built in the initialiser list rather than assigned: the drive controller holds
    // references to the motors and the supervisor, so there is no version of it that can
    // be replaced after construction.
    last_command_ms_ = NowMs();
    ESP_LOGI(TAG, "robot ready: %d mm wheels, %d mm track, trim %d, clearance %d mm",
             static_cast<int>(chassis_.wheel_diameter_mm),
             static_cast<int>(chassis_.track_width_mm),
             static_cast<int>(chassis_.speed_trim_permille),
             static_cast<int>(safety_config_.min_clearance_mm));
}

void RobotService::Start() {
    if (running_) {
        return;
    }
    running_ = true;
    xTaskCreate(TaskEntry, "nilo_robot", kTaskStack, this, kTaskPriority, &task_);
    ESP_LOGI(TAG, "robot control loop running at %d ms", ROBOT_TICK_MS);
}

void RobotService::Stop() {
    running_ = false;
    // The wheels stop before the loop does. Stopping the supervisor first would leave a
    // window in which the robot is moving and nothing is watching it.
    drive_.Stop();
    motors_.Coast();
}

void RobotService::TaskEntry(void* argument) {
    static_cast<RobotService*>(argument)->Loop();
    vTaskDelete(nullptr);
}

void RobotService::Loop() {
    TickType_t last_wake = xTaskGetTickCount();
    while (running_) {
        Tick();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ROBOT_TICK_MS));
    }
    motors_.Coast();
}

void RobotService::NoteBackendActivity() { last_command_ms_ = NowMs(); }

void RobotService::FollowDeviceState() {
    // The firmware already has a device state machine. The robot does not get a second
    // one; it reads that one and reacts, so there is no pair of states that can disagree.
    const int state = static_cast<int>(Application::GetInstance().GetDeviceState());
    if (state == last_device_state_) {
        return;
    }
    last_device_state_ = state;
    switch (static_cast<DeviceState>(state)) {
        case kDeviceStateUpgrading:
            // A robot that drives during an OTA is a robot that drives with half a
            // firmware image in flash.
            drive_.EmergencyStop(SafetyReason::kEmergencyStop);
            break;
        case kDeviceStateFatalError:
            drive_.EmergencyStop(SafetyReason::kEmergencyStop);
            break;
        case kDeviceStateIdle:
        case kDeviceStateListening:
        case kDeviceStateSpeaking:
            break;
        default:
            // Starting, connecting, configuring: nothing has asked the robot to move yet,
            // and anything already moving should not be.
            drive_.Stop();
            break;
    }
}

void RobotService::Tick() {
    const int64_t now = NowMs();
    FollowDeviceState();

    sensors_.SetBattery(battery_.Percent(), battery_.IsCharging());
    // The link is the protocol's opinion, not this loop's: `last_command_ms_` only moves
    // when the backend actually said something.
    sensors_.SetLink(Application::GetInstance().GetDeviceState() != kDeviceStateStarting,
                     last_command_ms_);
    const SensorSnapshot snapshot = sensors_.Read();

    drive_.UpdateSensors(snapshot);
    drive_.Tick();
    head_.Tick();
    lift_.Tick();
    face_.Tick();

    ReportMotionResults();
    ReportSensorChanges(snapshot, now);
    ReportPeriodic(now);
}

void RobotService::ReportMotionResults() {
    MotionResult result;
    while (drive_.PopResult(result)) {
        const std::string base = Pair("action_id", result.action_id) + "," +
                                 Text("kind", MotionKindName(result.kind)) + "," +
                                 Pair("travelled_mm", result.travelled_mm) + "," +
                                 Pair("elapsed_ms", result.elapsed_ms) +
                                 ",\"pose\":" + PosePayload() + ",\"motion\":" + MotionPayload();
        if (result.outcome == MotionOutcome::kCompleted) {
            Notify("notifications/motion_completed", "{" + base + "}");
            continue;
        }
        Notify("notifications/motion_failed",
               "{" + base + "," + Text("reason", MotionOutcomeName(result.outcome)) + "," +
                   Text("detail", SafetyReasonName(result.reason)) +
                   ",\"sensors\":" + SensorPayload(sensors_.Read()) + "}");
    }
}

void RobotService::ReportSensorChanges(const SensorSnapshot& sensors, int64_t now_ms) {
    (void)now_ms;
    const bool cliff = sensors.cliff_left || sensors.cliff_right;
    if (cliff != last_cliff_) {
        last_cliff_ = cliff;
        Notify("notifications/cliff",
               "{" + Flag("detected", cliff) + ",\"sensors\":" + SensorPayload(sensors) + "}");
    }
    const bool obstacle = sensors.front_distance_mm < safety_.config().min_clearance_mm;
    if (obstacle != last_obstacle_) {
        last_obstacle_ = obstacle;
        Notify("notifications/obstacle", "{" + Flag("detected", obstacle) + "," +
                                             Pair("front_mm", sensors.front_distance_mm) + "}");
    }
    if (sensors.bump != last_bump_) {
        last_bump_ = sensors.bump;
        Notify("notifications/sensor", SensorPayload(sensors));
    }
    const bool touched = sensors_.touched();
    if (touched != last_touch_) {
        last_touch_ = touched;
        Notify("notifications/touch", "{" + Flag("detected", touched) + "}");
    }
    const bool estopped = safety_.emergency_stopped();
    if (estopped != last_estop_) {
        last_estop_ = estopped;
        Notify("notifications/emergency_stop",
               "{" + Flag("engaged", estopped) + "," +
                   Text("reason", SafetyReasonName(safety_.emergency_reason())) + "}");
    }
    if (sensors.battery_percent != last_battery_percent_) {
        last_battery_percent_ = sensors.battery_percent;
        Notify("notifications/battery", BatteryPayload());
    }
}

void RobotService::ReportPeriodic(int64_t now_ms) {
    if (now_ms >= next_telemetry_ms_) {
        next_telemetry_ms_ = now_ms + ROBOT_TELEMETRY_MS;
        Notify("notifications/telemetry",
               "{\"pose\":" + PosePayload() + ",\"motion\":" + MotionPayload() + ",\"battery\":" +
                   BatteryPayload() + ",\"sensors\":" + SensorPayload(sensors_.Read()) +
                   ",\"expression\":{" + Text("emotion", face_.current_expression()) + "}}");
    }
    // Pose only while moving. A stationary robot reporting its unchanged pose five times a
    // second is a robot spending its battery on saying nothing.
    if (drive_.IsMoving() && now_ms >= next_pose_ms_) {
        next_pose_ms_ = now_ms + ROBOT_POSE_MS;
        Notify("notifications/pose", PosePayload());
    }
}

void RobotService::Notify(const char* method, const std::string& params) {
    // A JSON-RPC notification: no id, so no answer is expected and a backend that has gone
    // away costs nothing.
    Application::GetInstance().SendMcpMessage("{\"jsonrpc\":\"2.0\",\"method\":\"" +
                                              std::string(method) + "\",\"params\":" + params +
                                              "}");
}

std::string RobotService::PosePayload() const {
    const Pose pose = drive_.GetPose();
    return "{" + Pair("x_mm", pose.x_mm) + "," + Pair("y_mm", pose.y_mm) + "," +
           Pair("heading_mdeg", pose.heading_mdeg) + "}";
}

std::string RobotService::MotionPayload() const {
    return "{" + Flag("moving", drive_.IsMoving()) + "," +
           Pair("action_id", drive_.active_action_id()) + "}";
}

std::string RobotService::SensorPayload(const SensorSnapshot& sensors) const {
    return "{" + Flag("cliff_detected", sensors.cliff_left || sensors.cliff_right) + "," +
           Flag("bump_detected", sensors.bump) + "," + Flag("picked_up", sensors.picked_up) + "," +
           Flag("touch_detected", sensors_.touched()) + ",\"readings\":{" +
           Pair("front_mm", sensors.front_distance_mm) + "}}";
}

std::string RobotService::BatteryPayload() {
    return "{" + Pair("percent", battery_.Percent()) + "," +
           Flag("charging", battery_.IsCharging()) + "," +
           Pair("millivolts", battery_.MilliVolts()) + "}";
}

std::string RobotService::StatusJson() {
    const SensorSnapshot sensors = sensors_.Read();
    return "{\"pose\":" + PosePayload() + ",\"motion\":" + MotionPayload() +
           ",\"battery\":" + BatteryPayload() + ",\"sensors\":" + SensorPayload(sensors) +
           ",\"head\":{" + Pair("pitch_deg", head_.pitch_deg()) + "," +
           Pair("yaw_deg", head_.yaw_deg()) + "},\"lift\":{" +
           Pair("height_pct", lift_.position_pct()) + "},\"expression\":{" +
           Text("emotion", face_.current_expression()) + "}," +
           Flag("emergency_stopped", safety_.emergency_stopped()) + "}";
}

std::string RobotService::DistanceJson() {
    return "{\"readings\":{" + Pair("front_mm", sensors_.DistanceMm(0)) + "}}";
}

std::string RobotService::ImuJson() {
    const ImuReading imu = sensors_.ReadImu();
    if (!imu.valid) {
        // Honest rather than plausible: a backend that cannot tell "level" from "no
        // sensor" will act on the wrong one eventually.
        return "{" + Flag("available", false) + "}";
    }
    return "{" + Flag("available", true) + ",\"accel_mg\":[" + std::to_string(imu.accel_mg[0]) +
           "," + std::to_string(imu.accel_mg[1]) + "," + std::to_string(imu.accel_mg[2]) +
           "],\"gyro_mdps\":[" + std::to_string(imu.gyro_mdps[0]) + "," +
           std::to_string(imu.gyro_mdps[1]) + "," + std::to_string(imu.gyro_mdps[2]) + "]}";
}

std::string RobotService::CliffJson() {
    const SensorSnapshot sensors = sensors_.Read();
    return "{" + Flag("left", sensors.cliff_left) + "," + Flag("right", sensors.cliff_right) + "," +
           Flag("picked_up", sensors.picked_up) + "}";
}

std::string RobotService::BatteryJson() { return BatteryPayload(); }

std::string RobotService::Capture(const std::string& question) {
    if (camera_ == nullptr) {
        return "{" + Flag("captured", false) + "," + Text("error", "no camera is fitted") + "}";
    }
    if (!camera_->Capture()) {
        return "{" + Flag("captured", false) + "," + Text("error", "the capture failed") + "}";
    }
    if (question.empty()) {
        return "{" + Flag("captured", true) + "}";
    }
    const auto explained = camera_->Explain(question);
    if (!explained.has_value()) {
        return "{" + Flag("captured", true) + "," + Text("error", explained.error()) + "}";
    }
    return "{" + Flag("captured", true) + "," + Text("answer", explained.value()) + "}";
}

}  // namespace nilo
