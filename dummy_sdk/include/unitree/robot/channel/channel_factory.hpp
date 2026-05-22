// Dummy unitree_sdk2 — DDS channel factory (no-op).
#pragma once

#include <unitree/dummy_prelude.hpp>
#include <unitree/dummy_simulator.hpp>

namespace unitree {
namespace robot {

class ChannelFactory {
   public:
    static ChannelFactory* Instance() {
        static ChannelFactory instance;
        return &instance;
    }
    // Called once at the start of controller construction; drop any refresh hooks
    // left over from a previously destroyed controller before new ones register.
    void Init(int, const std::string& = "") { unitree_dummy::lowstate_refreshers().clear(); }
};

}  // namespace robot
}  // namespace unitree
