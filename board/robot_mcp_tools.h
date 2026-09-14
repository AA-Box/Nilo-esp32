// The tools this robot publishes, registered with the firmware's MCP server.
//
// Every handler in here does the same three things and nothing else: read the already
// validated arguments, call one method on one controller interface, and answer. There is
// no GPIO in this file, no duty cycle, no servo pulse and no register. That separation is
// the whole reason the controllers are interfaces: a tool handler that reached a pin would
// be a tool handler that has to be right about hardware as well as about protocol.
//
// Ranges come from robot_tool_schema.h, which is the same table the host tests check, so
// the bounds a backend reads and the bounds a call is rejected against cannot drift apart.
//
// Motion tools return an action id immediately and report completion as a notification.
// Waiting for a robot to finish driving inside a tool call would hold the protocol task
// for seconds.

#pragma once

#include "robot_hardware.h"

namespace nilo {

class RobotService;

// Register every tool. Called once, from the board constructor, after the controllers
// exist. Registering before them would publish a tool that cannot work yet.
void RegisterRobotTools(RobotService& service);

}  // namespace nilo
