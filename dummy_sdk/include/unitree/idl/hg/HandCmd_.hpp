// Dummy unitree_sdk2 — dexterous-hand command message.
#pragma once

#include <unitree/dummy_prelude.hpp>
#include <unitree/idl/hg/_msg_common.hpp>

namespace unitree_hg {
namespace msg {
namespace dds_ {

struct HandCmd_ {
    // accessed via motor_cmd().resize(n) and motor_cmd().at(i)
    UT_DUMMY_FIELD(motor_cmd, std::vector<MotorCmd_>)
};

}  // namespace dds_
}  // namespace msg
}  // namespace unitree_hg
