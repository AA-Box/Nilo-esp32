#include "face_display.h"

#include <esp_log.h>
#include <esp_timer.h>

#include <algorithm>

#include "display/display.h"
#include "robot_tool_schema.h"

#define TAG "NiloFace"

namespace nilo {
namespace {

// The animations this board can play. Data rather than code, so adding one is adding a
// line: the backend names an animation and this decides what the face does with it.
struct AnimationSpec {
    const char* name;
    const char* expression;
    int32_t default_ms;
};

constexpr AnimationSpec kAnimations[] = {
    {"greeting", "happy", 1500},    {"excited_greeting", "excited", 2000},
    {"thinking", "curious", 1200},  {"speaking", "happy", 800},
    {"confused", "confused", 1500}, {"sleepy", "sleepy", 3000},
    {"nod", "happy", 900},          {"shake", "annoyed", 900},
};

const AnimationSpec* FindAnimation(const char* name) {
    if (name == nullptr) {
        return nullptr;
    }
    for (const AnimationSpec& spec : kAnimations) {
        if (std::string(spec.name) == name) {
            return &spec;
        }
    }
    return nullptr;
}

int64_t NowMs() { return esp_timer_get_time() / 1000; }

}  // namespace

FaceDisplay::FaceDisplay(Display* display) : display_(display) {}

bool FaceDisplay::SetExpression(const char* emotion, int32_t intensity_pct) {
    if (!IsKnownExpression(emotion)) {
        ESP_LOGW(TAG, "refusing unknown expression '%s'", emotion == nullptr ? "(null)" : emotion);
        return false;
    }
    (void)intensity_pct;  // this panel has one intensity; a richer display would use it
    expression_ = emotion;
    if (display_ != nullptr) {
        display_->SetEmotion(emotion);
    }
    return true;
}

bool FaceDisplay::PlayAnimation(const char* name, int32_t duration_ms) {
    const AnimationSpec* spec = FindAnimation(name);
    if (spec == nullptr) {
        ESP_LOGW(TAG, "refusing unknown animation '%s'", name == nullptr ? "(null)" : name);
        return false;
    }
    animation_ = spec->name;
    animation_until_ms_ = NowMs() + (duration_ms > 0 ? duration_ms : spec->default_ms);
    SetExpression(spec->expression, 100);
    return true;
}

void FaceDisplay::Tick() {
    if (animation_.empty() || NowMs() < animation_until_ms_) {
        return;
    }
    // Animations end on their own and leave the face neutral. A face that keeps the last
    // frame of an animation forever reads as a robot that is stuck.
    animation_.clear();
    SetExpression("neutral", 100);
}

}  // namespace nilo
