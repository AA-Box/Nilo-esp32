// The hardware implementations of the motion interfaces: LEDC PWM for the wheels, LEDC for
// the servos. This is the only file in the board that writes a duty cycle.
//
// Thin on purpose. Everything interesting — the ramp, the timeout, the wheel trim, the
// safety hooks — is one layer up in ProfiledDrive, where it can be tested without a robot.
// What is left here is "turn a millimetre-per-second into a duty cycle", which is the part
// that genuinely needs the chip.

#pragma once

#include <driver/gpio.h>
#include <driver/ledc.h>

#include "robot_hardware.h"

namespace nilo {

// Two brushed motors behind a dual H-bridge. Speeds in mm/s, converted with the chassis
// calibration; no caller above this line knows what a duty cycle is.
class PwmMotorDriver : public MotorDriver {
public:
    PwmMotorDriver(gpio_num_t left_pwm, gpio_num_t left_dir, gpio_num_t right_pwm,
                   gpio_num_t right_dir, gpio_num_t enable, const ChassisCalibration& chassis,
                   const MotionLimits& limits);

    void SetWheelSpeeds(int32_t left_mmps, int32_t right_mmps) override;
    void Brake() override;
    void Coast() override;

private:
    void Drive(ledc_channel_t channel, gpio_num_t dir_pin, int32_t speed_mmps);
    uint32_t DutyFor(int32_t speed_mmps) const;

    gpio_num_t left_dir_;
    gpio_num_t right_dir_;
    gpio_num_t enable_;
    ChassisCalibration chassis_;
    MotionLimits limits_;
};

// A servo on a LEDC channel, in degrees, with a trim and mechanical stops. The stops are a
// property of the machine, so a request past one is clamped rather than refused: the
// caller cannot know where they are.
class ServoAxis {
public:
    ServoAxis(gpio_num_t pin, ledc_channel_t channel, int32_t min_deg, int32_t max_deg,
              int32_t trim_deg);

    void SetAngle(int32_t degrees);
    int32_t angle_deg() const { return angle_deg_; }

private:
    ledc_channel_t channel_;
    int32_t min_deg_;
    int32_t max_deg_;
    int32_t trim_deg_;
    int32_t angle_deg_ = 0;
};

class ServoHead : public HeadController {
public:
    ServoHead(gpio_num_t pitch_pin, gpio_num_t yaw_pin, int32_t pitch_trim_deg,
              int32_t yaw_trim_deg);

    void SetAngle(int32_t pitch_deg, int32_t yaw_deg) override;
    void LookAt(int32_t x_pct, int32_t y_pct) override;
    int32_t pitch_deg() const override { return pitch_.angle_deg(); }
    int32_t yaw_deg() const override { return yaw_.angle_deg(); }
    void Tick() override {}

private:
    ServoAxis pitch_;
    ServoAxis yaw_;
};

class ServoLift : public LiftController {
public:
    ServoLift(gpio_num_t pin, int32_t min_pulse_us, int32_t max_pulse_us);

    void SetPosition(int32_t height_pct) override;
    int32_t position_pct() const override { return position_pct_; }
    void Tick() override {}

private:
    ledc_channel_t channel_;
    int32_t min_pulse_us_;
    int32_t max_pulse_us_;
    int32_t position_pct_ = 0;
};

}  // namespace nilo
