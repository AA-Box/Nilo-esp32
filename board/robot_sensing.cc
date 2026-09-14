#include "robot_sensing.h"

#include <driver/gpio.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <rom/ets_sys.h>

#include <algorithm>

#include "config.h"

#define TAG "NiloSensing"

namespace nilo {
namespace {

void ConfigureInput(gpio_num_t pin) {
    gpio_config_t config = {};
    config.pin_bit_mask = 1ULL << pin;
    config.mode = GPIO_MODE_INPUT;
    config.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&config));
}

void ConfigureOutput(gpio_num_t pin) {
    gpio_config_t config = {};
    config.pin_bit_mask = 1ULL << pin;
    config.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&config));
    gpio_set_level(pin, 0);
}

}  // namespace

GpioRobotSensors::GpioRobotSensors(gpio_num_t cliff_left, gpio_num_t cliff_right, gpio_num_t bump,
                                   gpio_num_t touch, gpio_num_t distance_trig,
                                   gpio_num_t distance_echo)
    : cliff_left_(cliff_left),
      cliff_right_(cliff_right),
      bump_(bump),
      touch_(touch),
      distance_trig_(distance_trig),
      distance_echo_(distance_echo) {
    ConfigureInput(cliff_left);
    ConfigureInput(cliff_right);
    ConfigureInput(bump);
    ConfigureInput(touch);
    ConfigureOutput(distance_trig);
    ConfigureInput(distance_echo);
}

bool GpioRobotSensors::ReadCliff(gpio_num_t pin, int& clear_run) const {
    // The sensor pulls low over floor and floats high over a drop.
    const bool over_drop = gpio_get_level(pin) != 0;
    if (over_drop) {
        clear_run = 0;
        return true;
    }
    if (clear_run < kCliffClearSamples) {
        clear_run++;
    }
    // Believed immediately, disbelieved slowly. A robot that trusts one clear sample after
    // a drop is a robot that drives off the table on the second one.
    return clear_run < kCliffClearSamples;
}

int32_t GpioRobotSensors::DistanceMm(uint8_t index) {
    if (index != 0) {
        return -1;
    }
    // A 10 us trigger, then time the echo. Out of range reads as the ceiling rather than
    // as an error: "nothing within two metres" is the useful answer.
    gpio_set_level(distance_trig_, 0);
    ets_delay_us(4);
    gpio_set_level(distance_trig_, 1);
    ets_delay_us(10);
    gpio_set_level(distance_trig_, 0);

    const int64_t deadline = esp_timer_get_time() + 30000;
    while (gpio_get_level(distance_echo_) == 0) {
        if (esp_timer_get_time() > deadline) {
            return 2000;
        }
    }
    const int64_t started = esp_timer_get_time();
    while (gpio_get_level(distance_echo_) != 0) {
        if (esp_timer_get_time() > deadline) {
            return 2000;
        }
    }
    // 343 m/s, there and back: micros * 343 / 2000 gives millimetres.
    const int64_t micros = esp_timer_get_time() - started;
    return static_cast<int32_t>(std::min<int64_t>(2000, (micros * 343) / 2000));
}

SensorSnapshot GpioRobotSensors::Read() {
    SensorSnapshot snapshot;
    snapshot.cliff_left = ReadCliff(cliff_left_, left_clear_run_);
    snapshot.cliff_right = ReadCliff(cliff_right_, right_clear_run_);
    snapshot.bump = gpio_get_level(bump_) == 0;
    touched_ = gpio_get_level(touch_) != 0;
    // "Picked up" is both cliff sensors seeing nothing at once, which is what being lifted
    // off a table looks like from underneath.
    snapshot.picked_up = snapshot.cliff_left && snapshot.cliff_right;
    snapshot.front_distance_mm = DistanceMm(0);
    snapshot.battery_percent = battery_percent_;
    snapshot.charging = charging_;
    snapshot.link_up = link_up_;
    snapshot.last_command_ms = last_command_ms_;
    return snapshot;
}

ImuReading GpioRobotSensors::ReadImu() {
    // No IMU is fitted on the reference build. Reported as invalid rather than as zeros:
    // a backend that cannot tell "level" from "no sensor" will eventually act on the wrong
    // one (docs/robot-hardware.md, hardware validation).
    ImuReading reading;
    reading.valid = false;
    return reading;
}

void GpioRobotSensors::SetBattery(int32_t percent, bool charging) {
    battery_percent_ = percent;
    charging_ = charging;
}

void GpioRobotSensors::SetLink(bool up, int64_t last_command_ms) {
    link_up_ = up;
    last_command_ms_ = last_command_ms;
}

AdcBattery::AdcBattery() {
    adc_oneshot_unit_init_cfg_t unit = {};
    unit.unit_id = ADC_UNIT_1;
    adc_oneshot_unit_handle_t handle = nullptr;
    if (adc_oneshot_new_unit(&unit, &handle) != ESP_OK) {
        ESP_LOGW(TAG, "no ADC unit; the battery will read as full");
        return;
    }
    handle_ = handle;

    adc_oneshot_chan_cfg_t channel = {};
    channel.atten = ADC_ATTEN_DB_12;
    channel.bitwidth = ADC_BITWIDTH_DEFAULT;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(handle, BATTERY_ADC_CHANNEL, &channel));

    adc_cali_curve_fitting_config_t cali = {};
    cali.unit_id = ADC_UNIT_1;
    cali.chan = BATTERY_ADC_CHANNEL;
    cali.atten = ADC_ATTEN_DB_12;
    cali.bitwidth = ADC_BITWIDTH_DEFAULT;
    adc_cali_handle_t calibration = nullptr;
    if (adc_cali_create_scheme_curve_fitting(&cali, &calibration) == ESP_OK) {
        calibration_ = calibration;
    } else {
        ESP_LOGW(TAG, "no ADC calibration; battery voltage is approximate");
    }
}

AdcBattery::~AdcBattery() {
    if (calibration_ != nullptr) {
        adc_cali_delete_scheme_curve_fitting(static_cast<adc_cali_handle_t>(calibration_));
    }
    if (handle_ != nullptr) {
        adc_oneshot_del_unit(static_cast<adc_oneshot_unit_handle_t>(handle_));
    }
}

int32_t AdcBattery::MilliVolts() {
    if (handle_ == nullptr) {
        return BATTERY_FULL_MV;
    }
    int raw = 0;
    if (adc_oneshot_read(static_cast<adc_oneshot_unit_handle_t>(handle_), BATTERY_ADC_CHANNEL,
                         &raw) != ESP_OK) {
        return last_mv_;
    }
    int millivolts = raw;
    if (calibration_ != nullptr) {
        adc_cali_raw_to_voltage(static_cast<adc_cali_handle_t>(calibration_), raw, &millivolts);
    }
    last_mv_ =
        static_cast<int32_t>(millivolts) * BATTERY_DIVIDER_NUMERATOR / BATTERY_DIVIDER_DENOMINATOR;
    return last_mv_;
}

int32_t AdcBattery::Percent() {
    const int32_t millivolts = MilliVolts();
    const int32_t span = BATTERY_FULL_MV - BATTERY_EMPTY_MV;
    const int32_t above = millivolts - BATTERY_EMPTY_MV;
    return std::max<int32_t>(0, std::min<int32_t>(100, (above * 100) / std::max<int32_t>(1, span)));
}

bool AdcBattery::IsCharging() {
    // No charge-detect line on the reference build. Reported honestly as "not charging"
    // rather than guessed from a rising voltage, which is indistinguishable from a warming
    // pack (docs/robot-hardware.md).
    return false;
}

}  // namespace nilo
