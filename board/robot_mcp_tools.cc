#include "robot_mcp_tools.h"

#include <esp_log.h>

#include <string>

#include "mcp_server.h"
#include "robot_service.h"
#include "robot_tool_schema.h"

#define TAG "NiloTools"

namespace nilo {
namespace {

// Turn one entry of our table into the firmware's property object. One conversion, so the
// published schema is the table by construction rather than by somebody keeping two lists
// in step.
Property ToProperty(const ParamSpec& spec) {
    switch (spec.type) {
        case ParamType::kInteger:
            return spec.required
                       ? Property(spec.name, kPropertyTypeInteger, spec.min_value, spec.max_value)
                       : Property(spec.name, kPropertyTypeInteger, spec.default_value,
                                  spec.min_value, spec.max_value);
        case ParamType::kBoolean:
            return spec.required
                       ? Property(spec.name, kPropertyTypeBoolean)
                       : Property(spec.name, kPropertyTypeBoolean, spec.default_value != 0);
        case ParamType::kString:
        default:
            break;
    }
    Property property = spec.required ? Property(spec.name, kPropertyTypeString)
                                      : Property(spec.name, kPropertyTypeString, std::string());
    if (spec.max_length > 0) {
        property.SetMaxLength(spec.max_length);
    }
    return property;
}

PropertyList PropertiesOf(const ToolSpec& tool) {
    PropertyList properties;
    for (size_t index = 0; index < tool.param_count; ++index) {
        properties.AddProperty(ToProperty(tool.params[index]));
    }
    return properties;
}

std::string Json(const std::string& body) { return "{" + body + "}"; }

std::string Quoted(const char* key, const std::string& value) {
    return "\"" + std::string(key) + "\":\"" + value + "\"";
}

std::string Number(const char* key, long long value) {
    return "\"" + std::string(key) + "\":" + std::to_string(value);
}

std::string Boolean(const char* key, bool value) {
    return "\"" + std::string(key) + "\":" + (value ? "true" : "false");
}

// A refusal the backend can act on: a typed reason, not a sentence. "no" and "no, there is
// a drop" are different answers to a person standing in the room.
ToolResult Refused(SafetyReason reason) {
    return std::unexpected(std::string("refused: ") + SafetyReasonName(reason));
}

void AddTool(const ToolSpec& tool, ToolCallback callback) {
    McpServer::GetInstance().AddTool(tool.name, tool.description, PropertiesOf(tool), callback);
}

}  // namespace

void RegisterRobotTools(RobotService& service) {
    const ToolSpec* table = service.tools();
    if (table == nullptr) {
        ESP_LOGE(TAG, "the tool table did not build; no robot tools are published");
        return;
    }

    AddTool(*FindTool(table, "robot.get_status"),
            [&service](const PropertyList&) -> ToolResult { return service.StatusJson(); });

    AddTool(*FindTool(table, "robot.motion.move"),
            [&service](const PropertyList& properties) -> ToolResult {
                const int distance_mm = properties["distance_mm"].value<int>();
                const int speed_mmps = properties["speed_mmps"].value<int>();
                const uint32_t action_id = service.drive().Move(distance_mm, speed_mmps);
                if (action_id == 0) {
                    return Refused(service.drive().last_refusal());
                }
                return Json(Boolean("accepted", true) + "," + Number("action_id", action_id) + "," +
                            Quoted("state", "moving") + "," + Number("distance_mm", distance_mm));
            });

    AddTool(*FindTool(table, "robot.motion.turn"),
            [&service](const PropertyList& properties) -> ToolResult {
                const int angle_deg = properties["angle_deg"].value<int>();
                const int speed_dps = properties["speed_dps"].value<int>();
                const uint32_t action_id = service.drive().Turn(angle_deg, speed_dps);
                if (action_id == 0) {
                    return Refused(service.drive().last_refusal());
                }
                return Json(Boolean("accepted", true) + "," + Number("action_id", action_id) + "," +
                            Quoted("state", "turning") + "," + Number("angle_deg", angle_deg));
            });

    AddTool(*FindTool(table, "robot.motion.stop"), [&service](const PropertyList&) -> ToolResult {
        // Never refused. A robot that will not stop because a sensor says it is unsafe to
        // move is the wrong failure.
        service.drive().Stop();
        return Json(Boolean("stopped", true));
    });

    AddTool(*FindTool(table, "robot.head.set_angle"),
            [&service](const PropertyList& properties) -> ToolResult {
                const int pitch_deg = properties["pitch_deg"].value<int>();
                const int yaw_deg = properties["yaw_deg"].value<int>();
                service.head().SetAngle(pitch_deg, yaw_deg);
                return Json(Number("pitch_deg", service.head().pitch_deg()) + "," +
                            Number("yaw_deg", service.head().yaw_deg()));
            });

    AddTool(*FindTool(table, "robot.head.look_at"),
            [&service](const PropertyList& properties) -> ToolResult {
                service.head().LookAt(properties["x_pct"].value<int>(),
                                      properties["y_pct"].value<int>());
                return Json(Number("pitch_deg", service.head().pitch_deg()) + "," +
                            Number("yaw_deg", service.head().yaw_deg()));
            });

    AddTool(*FindTool(table, "robot.lift.set_position"),
            [&service](const PropertyList& properties) -> ToolResult {
                service.lift().SetPosition(properties["height_pct"].value<int>());
                return Json(Number("height_pct", service.lift().position_pct()));
            });

    AddTool(*FindTool(table, "robot.expression.set"),
            [&service](const PropertyList& properties) -> ToolResult {
                const std::string emotion = properties["emotion"].value<std::string>();
                const int intensity = properties["intensity_pct"].value<int>();
                if (!service.face().SetExpression(emotion.c_str(), intensity)) {
                    return std::unexpected("unknown expression: " + emotion);
                }
                return Json(Quoted("emotion", emotion) + "," + Number("intensity_pct", intensity));
            });

    AddTool(*FindTool(table, "robot.animation.play"),
            [&service](const PropertyList& properties) -> ToolResult {
                const std::string name = properties["name"].value<std::string>();
                const int duration_ms = properties["duration_ms"].value<int>();
                if (!service.face().PlayAnimation(name.c_str(), duration_ms)) {
                    return std::unexpected("unknown animation: " + name);
                }
                return Json(Boolean("accepted", true) + "," + Quoted("animation", name) + "," +
                            Number("duration_ms", duration_ms));
            });

    AddTool(*FindTool(table, "robot.camera.capture"),
            [&service](const PropertyList& properties) -> ToolResult {
                return service.Capture(properties["question"].value<std::string>());
            });

    AddTool(*FindTool(table, "robot.sensor.get_distance"),
            [&service](const PropertyList&) -> ToolResult { return service.DistanceJson(); });

    AddTool(*FindTool(table, "robot.sensor.get_imu"),
            [&service](const PropertyList&) -> ToolResult { return service.ImuJson(); });

    AddTool(*FindTool(table, "robot.sensor.get_cliff"),
            [&service](const PropertyList&) -> ToolResult { return service.CliffJson(); });

    AddTool(*FindTool(table, "robot.power.get_battery"),
            [&service](const PropertyList&) -> ToolResult { return service.BatteryJson(); });

    ESP_LOGI(TAG, "published %d robot tools", static_cast<int>(ToolCount()));
}

}  // namespace nilo
