// What the robot can feel, and what is left in the battery.
//
// One snapshot, taken together. Separate getters would let the safety supervisor read a
// cliff flag from one moment and a distance from another, and decide something about a
// world that never existed.
//
// The cliff sensors are the load-bearing ones and they are debounced here rather than in
// the supervisor: a reflective sensor over a dark rug flickers, and a supervisor that
// stops on one noisy sample is a robot that cannot cross a rug. Debouncing is asymmetric
// on purpose — a cliff is believed immediately and disbelieved slowly.

#pragma once

#include <driver/gpio.h>

#include <cstdint>

#include "robot_hardware.h"

namespace nilo {

// How many consecutive clear samples before a cliff is believed gone. At a 20 ms tick this
// is a fifth of a second of solid floor.
constexpr int kCliffClearSamples = 10;

class GpioRobotSensors : public RobotSensors {
public:
    GpioRobotSensors(gpio_num_t cliff_left, gpio_num_t cliff_right, gpio_num_t bump,
                     gpio_num_t touch, gpio_num_t distance_trig, gpio_num_t distance_echo);

    SensorSnapshot Read() override;
    ImuReading ReadImu() override;
    int32_t DistanceMm(uint8_t index) override;
    uint8_t DistanceSensorCount() const override { return 1; }

    // Pushed in by the board, which is what owns the battery monitor and the link state.
    void SetBattery(int32_t percent, bool charging);
    void SetLink(bool up, int64_t last_command_ms);

    bool touched() const { return touched_; }

private:
    bool ReadCliff(gpio_num_t pin, int& clear_run) const;

    gpio_num_t cliff_left_;
    gpio_num_t cliff_right_;
    gpio_num_t bump_;
    gpio_num_t touch_;
    gpio_num_t distance_trig_;
    gpio_num_t distance_echo_;

    int left_clear_run_ = kCliffClearSamples;
    int right_clear_run_ = kCliffClearSamples;
    bool touched_ = false;
    int32_t battery_percent_ = 100;
    bool charging_ = false;
    bool link_up_ = false;
    int64_t last_command_ms_ = 0;
};

// The battery, through an ADC divider. Percent is a straight line between the empty and
// full voltages, which is wrong about lithium chemistry and right about being monotonic —
// and monotonic is what a low-battery behaviour actually needs.
class AdcBattery : public BatteryMonitor {
public:
    AdcBattery();
    ~AdcBattery() override;

    int32_t Percent() override;
    int32_t MilliVolts() override;
    bool IsCharging() override;

private:
    void* handle_ = nullptr;  // adc_oneshot_unit_handle_t, kept opaque to the header
    void* calibration_ = nullptr;
    int32_t last_mv_ = 4200;
};

}  // namespace nilo
