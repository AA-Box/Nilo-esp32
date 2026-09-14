// The tool table: what this robot publishes, and exactly what each tool accepts.
//
// Data, not code, and plain C++ with no ESP-IDF in it. The MCP layer turns this table into
// the server's property objects; a host test reads the same table and checks the bounds.
// One definition, two readers, so a schema and its validation cannot drift apart.
//
// Every parameter is an integer, a bounded string or a boolean. There is no float anywhere
// and there never will be: the transport's type system does not carry one, and a parameter
// that cannot survive the wire is a bug waiting for a decimal point.
//
// Nothing here names an actuator. No PWM, no duty cycle, no servo microseconds, no wheel
// speeds. The vocabulary is the safety boundary: a backend that could set a duty cycle
// would make every check in safety_supervisor.h decorative.

#pragma once

#include <cstddef>
#include <cstdint>

#include "motion_profile.h"
#include "safety_supervisor.h"

namespace nilo {

enum class ParamType : uint8_t { kInteger, kBoolean, kString };

// One parameter of one tool.
struct ParamSpec {
    const char* name = "";
    const char* description = "";
    ParamType type = ParamType::kInteger;
    bool required = false;
    int32_t min_value = 0;
    int32_t max_value = 0;
    int32_t default_value = 0;
    size_t max_length = 0;  // strings only
};

// One published tool.
struct ToolSpec {
    const char* name = "";
    const char* description = "";
    const ParamSpec* params = nullptr;
    size_t param_count = 0;
    // True when the tool starts something that finishes later and reports by notification.
    // The backend correlates the completion by the action id the call returns.
    bool asynchronous = false;
};

// How many tools this board publishes.
size_t ToolCount();

// The table, with the movement bounds filled in from the live limits. Built rather than
// constant because "maximum: the configured limit" has to mean the limit this robot is
// actually running, not the one somebody wrote in a header.
const ToolSpec* BuildToolTable(const MotionLimits& limits, const SafetyConfig& safety);

// Look one up by its published name. Null when it is not published.
const ToolSpec* FindTool(const ToolSpec* table, const char* name);

// Whether a value is acceptable for a parameter. The MCP layer checks this before anything
// reaches a controller, so an out-of-range request is an error the backend can read rather
// than a robot that clamps silently and drives half as far as it was asked to.
bool ValidateInteger(const ParamSpec& param, int32_t value);
bool ValidateString(const ParamSpec& param, const char* value);

// The parameter of a tool, by name. Null when the tool has no such parameter.
const ParamSpec* FindParam(const ToolSpec& tool, const char* name);

// The expressions the face understands. A closed set, so a typo shows up as a rejected
// call rather than as a robot with a blank screen at three in the morning.
extern const char* const kExpressions[];
size_t ExpressionCount();
bool IsKnownExpression(const char* name);

}  // namespace nilo
