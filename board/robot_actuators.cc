#include "robot_actuators.h"

#include <esp_log.h>

#include <algorithm>
#include <cstdlib>

#include "config.h"

#define TAG "NiloActuators"

namespace nilo {
namespace {

constexpr ledc_timer_t kMotorTimer = LEDC_TIMER_0;
constexpr ledc_timer_t kServoTimer = LEDC_TIMER_1;
constexpr ledc_mode_t kMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_channel_t kLeftChannel = LEDC_CHANNEL_0;
constexpr ledc_channel_t kRightChannel = LEDC_CHANNEL_1;
constexpr ledc_channel_t kPitchChannel = LEDC_CHANNEL_2;
constexpr ledc_channel_t kYawChannel = LEDC_CHANNEL_3;
constexpr ledc_channel_t kLiftChannel = LEDC_CHANNEL_4;

// The servo timer runs at 50 Hz with 14 bits of resolution, so one microsecond of pulse is
// 16384 / 20000 counts. Kept as integers: the duty for 1500 us is exactly 1228.
constexpr uint32_t kServoResolutionBits = 14;
constexpr uint32_t kServoPeriodUs = 1000000 / SERVO_PWM_FREQ_HZ;

void ConfigureTimer(ledc_timer_t timer, uint32_t freq_hz, ledc_timer_bit_t resolution) {
    ledc_timer_config_t config = {};
    config.speed_mode = kMode;
    config.duty_resolution = resolution;
    config.timer_num = timer;
    config.freq_hz = freq_hz;
    config.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&config));
}

void ConfigureChannel(ledc_channel_t channel, ledc_timer_t timer, gpio_num_t pin) {
    ledc_channel_config_t config = {};
    config.gpio_num = pin;
    config.speed_mode = kMode;
    config.channel = channel;
    config.timer_sel = timer;
    config.duty = 0;
    config.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&config));
}

void ConfigureOutput(gpio_num_t pin, int level) {
    gpio_config_t config = {};
    config.pin_bit_mask = 1ULL << pin;
    config.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&config));
    gpio_set_level(pin, level);
}

uint32_t PulseToDuty(int32_t pulse_us) {
    const uint32_t max_duty = (1U << kServoResolutionBits) - 1;
    const uint32_t clamped = static_cast<uint32_t>(std::max<int32_t>(0, pulse_us));
    return static_cast<uint32_t>((static_cast<uint64_t>(clamped) * max_duty) / kServoPeriodUs);
}

}  // namespace

PwmMotorDriver::PwmMotorDriver(gpio_num_t left_pwm, gpio_num_t left_dir, gpio_num_t right_pwm,
                               gpio_num_t right_dir, gpio_num_t enable,
                               const ChassisCalibration& chassis, const MotionLimits& limits)
    : left_dir_(left_dir),
      right_dir_(right_dir),
      enable_(enable),
      chassis_(chassis),
      limits_(limits) {
    ConfigureTimer(kMotorTimer, MOTOR_PWM_FREQ_HZ,
                   static_cast<ledc_timer_bit_t>(MOTOR_PWM_RESOLUTION_BITS));
    ConfigureChannel(kLeftChannel, kMotorTimer, left_pwm);
    ConfigureChannel(kRightChannel, kMotorTimer, right_pwm);
    ConfigureOutput(left_dir, 0);
    ConfigureOutput(right_dir, 0);
    // The H-bridge starts disabled. A board that powers up with its motors live is a board
    // that moves before anything has decided it should.
    ConfigureOutput(enable, 0);
    ESP_LOGI(TAG, "drive ready: wheels %d mm, track %d mm, trim %d permille",
             static_cast<int>(chassis_.wheel_diameter_mm),
             static_cast<int>(chassis_.track_width_mm),
             static_cast<int>(chassis_.speed_trim_permille));
}

uint32_t PwmMotorDriver::DutyFor(int32_t speed_mmps) const {
    const uint32_t max_duty = (1U << MOTOR_PWM_RESOLUTION_BITS) - 1;
    const int32_t ceiling = std::max<int32_t>(1, limits_.max_speed_mmps);
    const int32_t magnitude = std::min<int32_t>(std::abs(speed_mmps), ceiling);
    return static_cast<uint32_t>((static_cast<uint64_t>(magnitude) * max_duty) / ceiling);
}

void PwmMotorDriver::Drive(ledc_channel_t channel, gpio_num_t dir_pin, int32_t speed_mmps) {
    gpio_set_level(dir_pin, speed_mmps < 0 ? 1 : 0);
    ESP_ERROR_CHECK(ledc_set_duty(kMode, channel, DutyFor(speed_mmps)));
    ESP_ERROR_CHECK(ledc_update_duty(kMode, channel));
}

void PwmMotorDriver::SetWheelSpeeds(int32_t left_mmps, int32_t right_mmps) {
    const bool moving = left_mmps != 0 || right_mmps != 0;
    gpio_set_level(enable_, moving ? 1 : 0);
    Drive(kLeftChannel, left_dir_, left_mmps);
    Drive(kRightChannel, right_dir_, right_mmps);
}

void PwmMotorDriver::Brake() {
    // Both sides driven low with the bridge still enabled: the motor is shorted through
    // the bridge and stops quickly, rather than freewheeling to a halt.
    ESP_ERROR_CHECK(ledc_set_duty(kMode, kLeftChannel, 0));
    ESP_ERROR_CHECK(ledc_update_duty(kMode, kLeftChannel));
    ESP_ERROR_CHECK(ledc_set_duty(kMode, kRightChannel, 0));
    ESP_ERROR_CHECK(ledc_update_duty(kMode, kRightChannel));
    gpio_set_level(enable_, 1);
}

void PwmMotorDriver::Coast() {
    ESP_ERROR_CHECK(ledc_set_duty(kMode, kLeftChannel, 0));
    ESP_ERROR_CHECK(ledc_update_duty(kMode, kLeftChannel));
    ESP_ERROR_CHECK(ledc_set_duty(kMode, kRightChannel, 0));
    ESP_ERROR_CHECK(ledc_update_duty(kMode, kRightChannel));
    gpio_set_level(enable_, 0);
}

ServoAxis::ServoAxis(gpio_num_t pin, ledc_channel_t channel, int32_t min_deg, int32_t max_deg,
                     int32_t trim_deg)
    : channel_(channel), min_deg_(min_deg), max_deg_(max_deg), trim_deg_(trim_deg) {
    ConfigureChannel(channel, kServoTimer, pin);
    SetAngle(0);
}

void ServoAxis::SetAngle(int32_t degrees) {
    // Clamped, not refused: the stops are a property of the machine and a caller asking for
    // 60 degrees on a servo that reaches 40 wants the servo at 40, not an error.
    angle_deg_ = std::max(min_deg_, std::min(max_deg_, degrees));
    const int32_t trimmed = angle_deg_ + trim_deg_;
    const int32_t span_us = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;
    const int32_t centre_us = SERVO_MIN_PULSE_US + span_us / 2;
    const int32_t pulse_us = centre_us + (trimmed * span_us) / 180;
    ESP_ERROR_CHECK(ledc_set_duty(kMode, channel_, PulseToDuty(pulse_us)));
    ESP_ERROR_CHECK(ledc_update_duty(kMode, channel_));
}

ServoHead::ServoHead(gpio_num_t pitch_pin, gpio_num_t yaw_pin, int32_t pitch_trim_deg,
                     int32_t yaw_trim_deg)
    : pitch_(pitch_pin, kPitchChannel, -25, 40, pitch_trim_deg),
      yaw_(yaw_pin, kYawChannel, -90, 90, yaw_trim_deg) {
    ConfigureTimer(kServoTimer, SERVO_PWM_FREQ_HZ,
                   static_cast<ledc_timer_bit_t>(kServoResolutionBits));
}

void ServoHead::SetAngle(int32_t pitch_deg, int32_t yaw_deg) {
    pitch_.SetAngle(pitch_deg);
    yaw_.SetAngle(yaw_deg);
}

void ServoHead::LookAt(int32_t x_pct, int32_t y_pct) {
    // Percent of the frame into angles, using the lens's field of view. The field of view
    // is a calibration value rather than a constant because it is a property of the lens
    // that was actually fitted.
    const int32_t yaw =
        ((50 - std::max<int32_t>(0, std::min<int32_t>(100, x_pct))) * CAL_CAMERA_HFOV_DEG) / 100;
    const int32_t pitch =
        ((50 - std::max<int32_t>(0, std::min<int32_t>(100, y_pct))) * CAL_CAMERA_VFOV_DEG) / 100;
    SetAngle(pitch_.angle_deg() + pitch, yaw_.angle_deg() + yaw);
}

ServoLift::ServoLift(gpio_num_t pin, int32_t min_pulse_us, int32_t max_pulse_us)
    : channel_(kLiftChannel), min_pulse_us_(min_pulse_us), max_pulse_us_(max_pulse_us) {
    ConfigureChannel(kLiftChannel, kServoTimer, pin);
    SetPosition(0);
}

void ServoLift::SetPosition(int32_t height_pct) {
    position_pct_ = std::max<int32_t>(0, std::min<int32_t>(100, height_pct));
    const int32_t pulse_us =
        min_pulse_us_ + ((max_pulse_us_ - min_pulse_us_) * position_pct_) / 100;
    ESP_ERROR_CHECK(ledc_set_duty(kMode, channel_, PulseToDuty(pulse_us)));
    ESP_ERROR_CHECK(ledc_update_duty(kMode, channel_));
}

}  // namespace nilo
