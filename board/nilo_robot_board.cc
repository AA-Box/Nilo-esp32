// The Nilo robot board: a Wi-Fi board with wheels, a head, a lift, a face and sensors.
//
// A new board, not an edit to an existing one. That is not tidiness: OTA updates are keyed
// on the reported board type, so a board that reuses another's identity gets the other
// board's firmware pushed onto it — and a robot that receives a smart speaker's image
// stops being a robot at the worst possible moment. The identity in config.json is unique
// and it stays that way after release.
//
// What this file does is construct hardware and hand it to interfaces. What it deliberately
// does not do is contain any robot logic: the motion planning, the safety rules, the tool
// schemas and the control loop are all elsewhere, in code that a host compiler can build
// and a test can drive.

#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>

#include "application.h"
#include "audio/codecs/es8311_audio_codec.h"
#include "board.h"
#include "boards/common/backlight.h"
#include "boards/common/button.h"
#include "boards/common/wifi_board.h"
#include "config.h"
#include "display/lcd_display.h"
#include "face_display.h"
#include "mcp_server.h"
#include "robot_actuators.h"
#include "robot_mcp_tools.h"
#include "robot_sensing.h"
#include "robot_service.h"

#define TAG "NiloRobotBoard"

class NiloRobotBoard : public WifiBoard {
public:
    NiloRobotBoard()
        : boot_button_(BOOT_BUTTON_GPIO),
          motors_(MOTOR_LEFT_PWM_GPIO, MOTOR_LEFT_DIR_GPIO, MOTOR_RIGHT_PWM_GPIO,
                  MOTOR_RIGHT_DIR_GPIO, MOTOR_ENABLE_GPIO, nilo::ChassisCalibration(),
                  nilo::MotionLimits()),
          head_(HEAD_PITCH_SERVO_GPIO, HEAD_YAW_SERVO_GPIO, CAL_HEAD_PITCH_TRIM_DEG,
                CAL_HEAD_YAW_TRIM_DEG),
          lift_(LIFT_SERVO_GPIO, CAL_LIFT_MIN_PULSE_US, CAL_LIFT_MAX_PULSE_US),
          sensors_(CLIFF_LEFT_GPIO, CLIFF_RIGHT_GPIO, BUMP_GPIO, TOUCH_GPIO, DISTANCE_TRIG_GPIO,
                   DISTANCE_ECHO_GPIO) {
        InitializeI2c();
        InitializeSpi();
        InitializeDisplay();
        InitializeButtons();
        InitializeRobot();
        GetBacklight()->SetBrightness(80);
    }

    AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec codec(
            codec_i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR);
        return &codec;
    }

    Display* GetDisplay() override { return display_; }

    Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        level = static_cast<int>(battery_.Percent());
        charging = battery_.IsCharging();
        discharging = !charging;
        return true;
    }

private:
    void InitializeI2c() {
        i2c_master_bus_config_t config = {};
        config.i2c_port = I2C_NUM_0;
        config.sda_io_num = AUDIO_CODEC_I2C_SDA_PIN;
        config.scl_io_num = AUDIO_CODEC_I2C_SCL_PIN;
        config.clk_source = I2C_CLK_SRC_DEFAULT;
        config.glitch_ignore_cnt = 7;
        config.intr_priority = 0;
        config.trans_queue_depth = 0;
        config.flags.enable_internal_pullup = 1;
        ESP_ERROR_CHECK(i2c_new_master_bus(&config, &codec_i2c_bus_));
    }

    void InitializeSpi() {
        spi_bus_config_t config = {};
        config.mosi_io_num = DISPLAY_SPI_MOSI_PIN;
        config.miso_io_num = GPIO_NUM_NC;
        config.sclk_io_num = DISPLAY_SPI_SCK_PIN;
        config.quadwp_io_num = GPIO_NUM_NC;
        config.quadhd_io_num = GPIO_NUM_NC;
        config.max_transfer_sz =
            DISPLAY_WIDTH * DISPLAY_HEIGHT * static_cast<int>(sizeof(uint16_t));
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &config, SPI_DMA_CH_AUTO));
    }

    void InitializeDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_SPI_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &panel_io));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new SpiLcdDisplay(panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                     DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X,
                                     DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
        // Hold the button and the robot stops and latches. A physical stop that needs the
        // network to work is not a physical stop.
        boot_button_.OnLongPress([this]() {
            if (service_ != nullptr) {
                ESP_LOGW(TAG, "emergency stop from the button");
                service_->drive().EmergencyStop(nilo::SafetyReason::kEmergencyStop);
            }
        });
    }

    void InitializeRobot() {
        face_ = new nilo::FaceDisplay(display_);
        service_ =
            new nilo::RobotService(motors_, head_, lift_, *face_, sensors_, battery_, GetCamera());
        nilo::RegisterRobotTools(*service_);
        // Tools first, loop second: a tool that is published before the loop can service it
        // is a tool that answers into a robot nobody is supervising.
        service_->Start();
    }

    i2c_master_bus_handle_t codec_i2c_bus_ = nullptr;
    LcdDisplay* display_ = nullptr;
    Button boot_button_;

    nilo::PwmMotorDriver motors_;
    nilo::ServoHead head_;
    nilo::ServoLift lift_;
    nilo::GpioRobotSensors sensors_;
    nilo::AdcBattery battery_;
    nilo::FaceDisplay* face_ = nullptr;
    nilo::RobotService* service_ = nullptr;
};

DECLARE_BOARD(NiloRobotBoard);
