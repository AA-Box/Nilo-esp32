// The published tool table: what it contains, what it rejects, and the two properties that
// make it a safety boundary rather than a convenience.

#include <cstring>
#include <initializer_list>

#include "check.h"
#include "robot_tool_schema.h"

using namespace nilo;

namespace {

const ToolSpec* Table() {
    static MotionLimits limits;
    static SafetyConfig safety;
    return BuildToolTable(limits, safety);
}

// Words that must never appear in a published schema. One of these means the backend has
// been handed an actuator, and every check in safety_supervisor.h has become decorative.
const char* const kActuatorWords[] = {"pwm",     "duty",     "servo_us", "wheel_speed", "wheel_rpm",
                                      "voltage", "register", "gpio",     "torque",      "encoder"};

bool MentionsActuator(const char* text) {
    if (text == nullptr) {
        return false;
    }
    for (const char* word : kActuatorWords) {
        if (std::strstr(text, word) != nullptr) {
            return true;
        }
    }
    return false;
}

void EveryRequiredToolIsPublished() {
    const ToolSpec* table = Table();
    CHECK(table != nullptr);
    for (const char* name :
         {"robot.get_status", "robot.motion.move", "robot.motion.turn", "robot.motion.stop",
          "robot.head.set_angle", "robot.lift.set_position", "robot.expression.set",
          "robot.animation.play", "robot.camera.capture", "robot.sensor.get_distance",
          "robot.sensor.get_imu", "robot.sensor.get_cliff", "robot.power.get_battery"}) {
        if (FindTool(table, name) == nullptr) {
            ::check::Fail(__FILE__, __LINE__, "tool is published", name);
        }
    }
}

void NoSchemaNamesAnActuator() {
    const ToolSpec* table = Table();
    for (size_t index = 0; index < ToolCount(); ++index) {
        const ToolSpec& tool = table[index];
        if (MentionsActuator(tool.name) || MentionsActuator(tool.description)) {
            ::check::Fail(__FILE__, __LINE__, "tool mentions an actuator", tool.name);
        }
        for (size_t param = 0; param < tool.param_count; ++param) {
            if (MentionsActuator(tool.params[param].name) ||
                MentionsActuator(tool.params[param].description)) {
                ::check::Fail(__FILE__, __LINE__, "parameter mentions an actuator",
                              tool.params[param].name);
            }
        }
    }
}

void EveryParameterCarriesItsUnitAndItsBounds() {
    const ToolSpec* table = Table();
    for (size_t index = 0; index < ToolCount(); ++index) {
        const ToolSpec& tool = table[index];
        for (size_t param = 0; param < tool.param_count; ++param) {
            const ParamSpec& spec = tool.params[param];
            if (spec.type == ParamType::kInteger) {
                // An unbounded integer is not a schema, it is a suggestion.
                if (spec.max_value <= spec.min_value) {
                    ::check::Fail(__FILE__, __LINE__, "integer parameter has a usable range",
                                  spec.name);
                }
            }
            if (spec.type == ParamType::kString && spec.max_length == 0) {
                ::check::Fail(__FILE__, __LINE__, "string parameter has a maximum length",
                              spec.name);
            }
        }
    }
}

void MovementBoundsMatchTheDesignNote() {
    const ToolSpec* table = Table();
    const ToolSpec* move = FindTool(table, "robot.motion.move");
    CHECK(move != nullptr);
    const ParamSpec* distance = FindParam(*move, "distance_mm");
    CHECK(distance != nullptr);
    CHECK_EQ(distance->min_value, -1000);
    CHECK_EQ(distance->max_value, 1000);
    CHECK(distance->required);

    const ParamSpec* speed = FindParam(*move, "speed_mmps");
    CHECK(speed != nullptr);
    CHECK(speed->min_value >= 1);
    // "maximum: configured limit", not a number somebody wrote in a header.
    CHECK_EQ(speed->max_value, MotionLimits().max_speed_mmps);
    CHECK(!speed->required);
}

void TheSpeedCeilingFollowsTheConfiguredLimit() {
    MotionLimits limits;
    limits.max_speed_mmps = 120;
    SafetyConfig safety;
    const ToolSpec* table = BuildToolTable(limits, safety);
    const ParamSpec* speed = FindParam(*FindTool(table, "robot.motion.move"), "speed_mmps");
    CHECK_EQ(speed->max_value, 120);
    CHECK(!ValidateInteger(*speed, 300));
    CHECK(ValidateInteger(*speed, 120));
}

void UnsafeValuesAreRejected() {
    const ToolSpec* table = Table();
    const ParamSpec* distance = FindParam(*FindTool(table, "robot.motion.move"), "distance_mm");

    CHECK(ValidateInteger(*distance, 250));
    CHECK(ValidateInteger(*distance, -250));
    CHECK(ValidateInteger(*distance, 1000));
    CHECK(!ValidateInteger(*distance, 1001));
    CHECK(!ValidateInteger(*distance, -1001));
    CHECK(!ValidateInteger(*distance, 100000));

    const ParamSpec* angle = FindParam(*FindTool(table, "robot.motion.turn"), "angle_deg");
    CHECK(ValidateInteger(*angle, 180));
    CHECK(!ValidateInteger(*angle, 181));
    CHECK(!ValidateInteger(*angle, -360));
}

void PercentagesAreBoundedToAHundred() {
    const ToolSpec* table = Table();
    const ParamSpec* height = FindParam(*FindTool(table, "robot.lift.set_position"), "height_pct");
    CHECK(ValidateInteger(*height, 0));
    CHECK(ValidateInteger(*height, 100));
    CHECK(!ValidateInteger(*height, 101));
    CHECK(!ValidateInteger(*height, -1));
}

void StringsAreBounded() {
    const ToolSpec* table = Table();
    const ParamSpec* name = FindParam(*FindTool(table, "robot.animation.play"), "name");
    CHECK(ValidateString(*name, "excited_greeting"));
    CHECK(!ValidateString(*name, nullptr));

    char oversized[256];
    std::memset(oversized, 'a', sizeof(oversized) - 1);
    oversized[sizeof(oversized) - 1] = '\0';
    CHECK(!ValidateString(*name, oversized));
}

void TheExpressionSetIsClosed() {
    CHECK(IsKnownExpression("happy"));
    CHECK(IsKnownExpression("curious"));
    CHECK(!IsKnownExpression("smug"));
    CHECK(!IsKnownExpression(""));
    CHECK(!IsKnownExpression(nullptr));
    CHECK(ExpressionCount() > 5);
}

void MotionToolsReportCompletionAsynchronously() {
    const ToolSpec* table = Table();
    // A move finishes later and reports by notification; reading a sensor does not.
    CHECK(FindTool(table, "robot.motion.move")->asynchronous);
    CHECK(FindTool(table, "robot.motion.turn")->asynchronous);
    CHECK(!FindTool(table, "robot.motion.stop")->asynchronous);
    CHECK(!FindTool(table, "robot.power.get_battery")->asynchronous);
}

void AnUnknownToolIsNotFound() {
    CHECK(FindTool(Table(), "robot.motion.teleport") == nullptr);
    CHECK(FindTool(Table(), nullptr) == nullptr);
    CHECK(FindTool(nullptr, "robot.motion.move") == nullptr);
}

}  // namespace

int main() {
    EveryRequiredToolIsPublished();
    NoSchemaNamesAnActuator();
    EveryParameterCarriesItsUnitAndItsBounds();
    MovementBoundsMatchTheDesignNote();
    TheSpeedCeilingFollowsTheConfiguredLimit();
    UnsafeValuesAreRejected();
    PercentagesAreBoundedToAHundred();
    StringsAreBounded();
    TheExpressionSetIsClosed();
    MotionToolsReportCompletionAsynchronously();
    AnUnknownToolIsNotFound();
    return check::Report("robot_tool_schema");
}
