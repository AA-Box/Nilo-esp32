#include "robot_tool_schema.h"

#include <cstring>

namespace nilo {
namespace {

// The table is built once into file-scope storage. Building it needs the live limits, and
// a tool table that reallocates while the MCP server holds pointers into it is a crash
// nobody reproduces.
constexpr size_t kMaxParams = 4;

ParamSpec g_move_params[kMaxParams];
ParamSpec g_turn_params[kMaxParams];
ParamSpec g_head_params[kMaxParams];
ParamSpec g_look_params[kMaxParams];
ParamSpec g_lift_params[kMaxParams];
ParamSpec g_expression_params[kMaxParams];
ParamSpec g_animation_params[kMaxParams];
ParamSpec g_camera_params[kMaxParams];
ParamSpec g_follow_params[kMaxParams];

constexpr size_t kToolCount = 14;
ToolSpec g_tools[kToolCount];
bool g_built = false;

ParamSpec Integer(const char* name, const char* description, int32_t min_value, int32_t max_value,
                  bool required, int32_t default_value = 0) {
    ParamSpec spec;
    spec.name = name;
    spec.description = description;
    spec.type = ParamType::kInteger;
    spec.required = required;
    spec.min_value = min_value;
    spec.max_value = max_value;
    spec.default_value = default_value;
    return spec;
}

ParamSpec String(const char* name, const char* description, size_t max_length, bool required) {
    ParamSpec spec;
    spec.name = name;
    spec.description = description;
    spec.type = ParamType::kString;
    spec.required = required;
    spec.max_length = max_length;
    return spec;
}

ToolSpec Tool(const char* name, const char* description, const ParamSpec* params, size_t count,
              bool asynchronous = false) {
    ToolSpec spec;
    spec.name = name;
    spec.description = description;
    spec.params = params;
    spec.param_count = count;
    spec.asynchronous = asynchronous;
    return spec;
}

}  // namespace

const char* const kExpressions[] = {"neutral", "happy",     "excited", "curious", "confused", "sad",
                                    "sleepy",  "surprised", "annoyed", "scared",  "focused"};

size_t ExpressionCount() { return sizeof(kExpressions) / sizeof(kExpressions[0]); }

bool IsKnownExpression(const char* name) {
    if (name == nullptr) {
        return false;
    }
    for (size_t index = 0; index < ExpressionCount(); ++index) {
        if (std::strcmp(kExpressions[index], name) == 0) {
            return true;
        }
    }
    return false;
}

size_t ToolCount() { return kToolCount; }

const ToolSpec* BuildToolTable(const MotionLimits& limits, const SafetyConfig& safety) {
    const SafetyConfig clamped = safety.Clamped();
    const int32_t max_distance_mm = 1000;
    const int32_t max_angle_deg = 180;
    const int32_t max_motion_ms = clamped.max_motion_ms;

    g_move_params[0] = Integer("distance_mm", "How far to drive. Negative drives backwards.",
                               -max_distance_mm, max_distance_mm, true);
    g_move_params[1] =
        Integer("speed_mmps", "Drive speed in millimetres per second.", limits.min_speed_mmps,
                limits.max_speed_mmps, false, limits.max_speed_mmps / 2);

    g_turn_params[0] = Integer("angle_deg", "How far to turn in place. Positive turns left.",
                               -max_angle_deg, max_angle_deg, true);
    g_turn_params[1] = Integer("speed_dps", "Turn speed in degrees per second.", 10,
                               limits.max_turn_dps, false, limits.max_turn_dps / 2);

    g_head_params[0] = Integer("pitch_deg", "Head pitch. Negative looks down.", -25, 40, false, 0);
    g_head_params[1] = Integer("yaw_deg", "Head yaw. Positive looks left.", -90, 90, false, 0);

    g_look_params[0] =
        Integer("x_pct", "Horizontal target, percent of the camera frame.", 0, 100, false, 50);
    g_look_params[1] =
        Integer("y_pct", "Vertical target, percent of the camera frame.", 0, 100, false, 50);

    g_lift_params[0] = Integer("height_pct", "Lift position, percent of travel.", 0, 100, true);

    g_expression_params[0] = String("emotion", "Which face to show.", 24, true);
    g_expression_params[1] =
        Integer("intensity_pct", "How strongly to show it.", 0, 100, false, 100);

    g_animation_params[0] = String("name", "Which pre-authored animation to play.", 48, true);
    g_animation_params[1] =
        Integer("duration_ms", "How long to play it for.", 100, max_motion_ms, false, 2000);

    g_camera_params[0] = String("question", "What to ask about the frame. Optional.", 200, false);

    g_follow_params[0] = String("target_id", "Which tracked target to follow.", 64, true);
    g_follow_params[1] =
        Integer("duration_ms", "How long to follow for.", 100, max_motion_ms, false, 5000);
    g_follow_params[2] = Integer("stop_distance_mm", "How close to get before stopping.",
                                 clamped.min_clearance_mm, 3000, false, 600);

    size_t index = 0;
    g_tools[index++] =
        Tool("robot.get_status", "Everything the robot knows about itself right now.", nullptr, 0);
    g_tools[index++] =
        Tool("robot.motion.move", "Drive straight by a bounded distance.", g_move_params, 2, true);
    g_tools[index++] =
        Tool("robot.motion.turn", "Turn in place by a bounded angle.", g_turn_params, 2, true);
    g_tools[index++] = Tool("robot.motion.stop", "Stop whatever is moving, now.", nullptr, 0);
    g_tools[index++] =
        Tool("robot.head.set_angle", "Point the head at an absolute angle.", g_head_params, 2);
    g_tools[index++] = Tool("robot.head.look_at", "Point the head at a spot in the camera frame.",
                            g_look_params, 2);
    g_tools[index++] =
        Tool("robot.lift.set_position", "Raise or lower the lift.", g_lift_params, 1);
    g_tools[index++] =
        Tool("robot.expression.set", "Show an expression on the face.", g_expression_params, 2);
    g_tools[index++] =
        Tool("robot.animation.play", "Play a pre-authored animation.", g_animation_params, 2, true);
    g_tools[index++] =
        Tool("robot.camera.capture", "Capture one camera frame.", g_camera_params, 1);
    g_tools[index++] = Tool("robot.sensor.get_distance", "Read the distance sensors.", nullptr, 0);
    g_tools[index++] = Tool("robot.sensor.get_imu", "Read the inertial sensor.", nullptr, 0);
    g_tools[index++] = Tool("robot.sensor.get_cliff", "Read the cliff sensors.", nullptr, 0);
    g_tools[index++] = Tool("robot.power.get_battery", "Read the battery.", nullptr, 0);
    g_built = index == kToolCount;
    return g_built ? g_tools : nullptr;
}

const ToolSpec* FindTool(const ToolSpec* table, const char* name) {
    if (table == nullptr || name == nullptr) {
        return nullptr;
    }
    for (size_t index = 0; index < kToolCount; ++index) {
        if (std::strcmp(table[index].name, name) == 0) {
            return &table[index];
        }
    }
    return nullptr;
}

const ParamSpec* FindParam(const ToolSpec& tool, const char* name) {
    if (name == nullptr) {
        return nullptr;
    }
    for (size_t index = 0; index < tool.param_count; ++index) {
        if (std::strcmp(tool.params[index].name, name) == 0) {
            return &tool.params[index];
        }
    }
    return nullptr;
}

bool ValidateInteger(const ParamSpec& param, int32_t value) {
    if (param.type != ParamType::kInteger) {
        return false;
    }
    return value >= param.min_value && value <= param.max_value;
}

bool ValidateString(const ParamSpec& param, const char* value) {
    if (param.type != ParamType::kString || value == nullptr) {
        return false;
    }
    if (param.max_length > 0 && std::strlen(value) > param.max_length) {
        return false;
    }
    return true;
}

}  // namespace nilo
