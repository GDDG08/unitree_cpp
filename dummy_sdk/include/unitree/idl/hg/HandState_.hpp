// Dummy unitree_sdk2 — dexterous-hand state message (included but unused here).
#pragma once

#include <unitree/dummy_prelude.hpp>
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/_msg_common.hpp>

namespace unitree_hg {
namespace msg {
namespace dds_ {

struct HandState_ {
    UT_DUMMY_FIELD(motor_state, std::vector<MotorState_>)
    UT_DUMMY_FIELD(imu_state, IMUState_)
};

}  // namespace dds_
}  // namespace msg
}  // namespace unitree_hg
