// The face: a closed set of expressions and a small set of animations, over whatever
// display this board has.
//
// The backend sends a *name*, never pixels. What "happy" looks like is a property of this
// screen — its resolution, its colours, whether it is an OLED or an emote panel — and a
// backend that shipped a frame buffer would be a backend that has to know all of that and
// be updated when the screen changes.
//
// Unknown names are refused rather than rendered as nothing. A typo that shows a blank
// face at three in the morning is a bug nobody finds; a typo that comes back as an error
// is a bug somebody fixes.

#pragma once

#include <cstdint>
#include <string>

#include "robot_hardware.h"

class Display;

namespace nilo {

class FaceDisplay : public DisplayExpressionController {
public:
    explicit FaceDisplay(Display* display);

    bool SetExpression(const char* emotion, int32_t intensity_pct) override;
    bool PlayAnimation(const char* name, int32_t duration_ms) override;
    const char* current_expression() const override { return expression_.c_str(); }
    void Tick() override;

    const std::string& current_animation() const { return animation_; }

private:
    Display* display_;
    std::string expression_ = "neutral";
    std::string animation_;
    int64_t animation_until_ms_ = 0;
};

}  // namespace nilo
