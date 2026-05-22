// Dummy unitree_sdk2 — low-level robot state message.
#pragma once

#include <unitree/dummy_prelude.hpp>
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/_msg_common.hpp>

namespace unitree_hg {
namespace msg {
namespace dds_ {

struct LowState_ {
    UT_DUMMY_FIELD(tick, uint32_t)
    UT_DUMMY_FIELD(mode_machine, uint8_t)
    UT_DUMMY_FIELD(motor_state, std::array<MotorState_, 64>)  // indexed: motor_state()[i]
    UT_DUMMY_FIELD(imu_state, IMUState_)
    UT_DUMMY_FIELD(wireless_remote, std::array<uint8_t, 40>)
    // `crc` MUST stay last: the controller CRCs all words except the final one,
    // so the dummy data path can fill a matching CRC (see dummy_simulator.hpp).
    UT_DUMMY_FIELD(crc, uint32_t)
};

}  // namespace dds_
}  // namespace msg
}  // namespace unitree_hg
