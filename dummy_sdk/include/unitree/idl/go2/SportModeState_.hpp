// Dummy unitree_sdk2 — sport-mode (odometry) state message.
#pragma once

#include <unitree/dummy_prelude.hpp>

namespace unitree_go {
namespace msg {
namespace dds_ {

struct SportModeState_ {
    UT_DUMMY_FIELD(position, std::array<float, 3>)
    UT_DUMMY_FIELD(velocity, std::array<float, 3>)
};

}  // namespace dds_
}  // namespace msg
}  // namespace unitree_go
