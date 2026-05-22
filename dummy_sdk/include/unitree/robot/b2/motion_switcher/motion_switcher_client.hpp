// Dummy unitree_sdk2 — motion-switcher client (no service to talk to).
#pragma once

#include <unitree/dummy_prelude.hpp>

namespace unitree {
namespace robot {
namespace b2 {

class MotionSwitcherClient {
   public:
    void SetTimeout(float) {}
    void Init() {}
    // Leaves `name` empty so the controller's release loop exits immediately.
    int32_t CheckMode(std::string& /*form*/, std::string& /*name*/) { return 0; }
    int32_t ReleaseMode() { return 0; }
};

}  // namespace b2
}  // namespace robot
}  // namespace unitree
