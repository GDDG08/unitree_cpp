// Dummy unitree_sdk2 — low-level command message.
#pragma once

#include <unitree/dummy_prelude.hpp>
#include <unitree/idl/hg/_msg_common.hpp>

namespace unitree_hg {
namespace msg {
namespace dds_ {

struct LowCmd_ {
    UT_DUMMY_FIELD(mode_pr, uint8_t)
    UT_DUMMY_FIELD(mode_machine, uint8_t)
    UT_DUMMY_FIELD(crc, uint32_t)
    UT_DUMMY_FIELD(motor_cmd, std::array<MotorCmd_, 64>)  // accessed via motor_cmd().at(i)
};

}  // namespace dds_
}  // namespace msg
}  // namespace unitree_hg
