// Pin map and calibration for the Nilo robot board.
//
// Two kinds of number live here and they are not the same kind of thing:
//
//   Pins are facts.        They match the schematic or the board does not work.
//   Calibration is a
//   starting point.        Wheel diameter, gear ratio, servo trims and sensor offsets all
//                          drift on real hardware. The defaults below are the designed
//                          values; the measured ones are read from NVS at boot and these
//                          are only the fallback. A model that cannot be tuned is wrong on
//                          the bench (docs/robot-hardware.md).
//
// Nothing in this file is reachable from the network. The tool layer speaks millimetres
// and degrees; which GPIO that turns into is a property of this board alone.

#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

// -- audio ---------------------------------------------------------------------------------

#define AUDIO_INPUT_SAMPLE_RATE 24000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

#define AUDIO_I2S_GPIO_MCLK GPIO_NUM_16
#define AUDIO_I2S_GPIO_WS GPIO_NUM_45
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_9
#define AUDIO_I2S_GPIO_DIN GPIO_NUM_10
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_8

#define AUDIO_CODEC_PA_PIN GPIO_NUM_46
#define AUDIO_CODEC_I2C_SDA_PIN GPIO_NUM_17
#define AUDIO_CODEC_I2C_SCL_PIN GPIO_NUM_18
#define AUDIO_CODEC_ES8311_ADDR ES8311_CODEC_DEFAULT_ADDR

// -- buttons -------------------------------------------------------------------------------

#define BOOT_BUTTON_GPIO GPIO_NUM_0

// -- drive ---------------------------------------------------------------------------------
//
// Two brushed motors through a dual H-bridge: one PWM pin and one direction pin each.
// There is no tool that reaches these; the drive controller is the only writer.

#define MOTOR_LEFT_PWM_GPIO GPIO_NUM_4
#define MOTOR_LEFT_DIR_GPIO GPIO_NUM_5
#define MOTOR_RIGHT_PWM_GPIO GPIO_NUM_6
#define MOTOR_RIGHT_DIR_GPIO GPIO_NUM_7
#define MOTOR_ENABLE_GPIO GPIO_NUM_15
#define MOTOR_PWM_FREQ_HZ 20000  // above hearing, so the robot does not whine
#define MOTOR_PWM_RESOLUTION_BITS 10

// -- head and lift servos ------------------------------------------------------------------

#define HEAD_PITCH_SERVO_GPIO GPIO_NUM_11
#define HEAD_YAW_SERVO_GPIO GPIO_NUM_12
#define LIFT_SERVO_GPIO GPIO_NUM_13
#define SERVO_PWM_FREQ_HZ 50
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500

// -- sensors -------------------------------------------------------------------------------

#define CLIFF_LEFT_GPIO GPIO_NUM_38
#define CLIFF_RIGHT_GPIO GPIO_NUM_39
#define BUMP_GPIO GPIO_NUM_40
#define TOUCH_GPIO GPIO_NUM_41
#define DISTANCE_TRIG_GPIO GPIO_NUM_42
#define DISTANCE_ECHO_GPIO GPIO_NUM_2
#define IMU_I2C_ADDR 0x68

// The battery divider: 2:1, so the ADC reads half the pack voltage.
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_3
#define BATTERY_DIVIDER_NUMERATOR 2
#define BATTERY_DIVIDER_DENOMINATOR 1
#define BATTERY_FULL_MV 4200
#define BATTERY_EMPTY_MV 3300

// -- display -------------------------------------------------------------------------------

#define DISPLAY_SPI_SCK_PIN GPIO_NUM_47
#define DISPLAY_SPI_MOSI_PIN GPIO_NUM_48
#define DISPLAY_DC_PIN GPIO_NUM_21
#define DISPLAY_SPI_CS_PIN GPIO_NUM_14
#define DISPLAY_RST_PIN GPIO_NUM_3
#define DISPLAY_BACKLIGHT_PIN GPIO_NUM_1
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false

#define DISPLAY_WIDTH 240
#define DISPLAY_HEIGHT 240
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false
#define DISPLAY_OFFSET_X 0
#define DISPLAY_OFFSET_Y 0

// -- calibration defaults ------------------------------------------------------------------
//
// Overridden by whatever is in NVS under the "nilo" namespace. Measure, do not guess:
// docs/robot-hardware.md has the procedure and the order to do it in.

#define CAL_WHEEL_DIAMETER_MM 65
#define CAL_TRACK_WIDTH_MM 140
#define CAL_GEAR_RATIO_PERMILLE 1000
#define CAL_SPEED_TRIM_PERMILLE 0         // +ve speeds the left wheel up; fixes a robot that curves
#define CAL_DISTANCE_SCALE_PERMILLE 1000  // measured / commanded, x1000
#define CAL_TURN_SCALE_PERMILLE 1000
#define CAL_HEAD_PITCH_TRIM_DEG 0
#define CAL_HEAD_YAW_TRIM_DEG 0
#define CAL_LIFT_MIN_PULSE_US 700
#define CAL_LIFT_MAX_PULSE_US 2300
#define CAL_CAMERA_VFOV_DEG 45
#define CAL_CAMERA_HFOV_DEG 60

// -- motion limits -------------------------------------------------------------------------
//
// The ceilings the drive controller plans inside. The safety supervisor has its own,
// lower, non-negotiable ones; these are what a well-behaved command is planned against.

#define LIMIT_MAX_SPEED_MMPS 300
#define LIMIT_MIN_SPEED_MMPS 30
#define LIMIT_MAX_ACCEL_MMPS2 400
#define LIMIT_MAX_TURN_DPS 120
#define LIMIT_MIN_CLEARANCE_MM 120
#define LIMIT_HEARTBEAT_TIMEOUT_MS 5000
#define LIMIT_MAX_MOTION_MS 20000

// How often the robot control loop runs, and how often it reports.
#define ROBOT_TICK_MS 20
#define ROBOT_TELEMETRY_MS 1000
#define ROBOT_POSE_MS 200

#endif  // _BOARD_CONFIG_H_
