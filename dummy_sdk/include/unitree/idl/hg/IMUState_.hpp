// Dummy unitree_sdk2 — IMU state message.
#pragma once

#include <unitree/dummy_prelude.hpp>

namespace unitree_hg {
namespace msg {
namespace dds_ {

struct IMUState_ {
    UT_DUMMY_FIELD(quaternion, std::array<float, 4>)
    UT_DUMMY_FIELD(gyroscope, std::array<float, 3>)
    UT_DUMMY_FIELD(accelerometer, std::array<float, 3>)
    UT_DUMMY_FIELD(rpy, std::array<float, 3>)
};

}  // namespace dds_
}  // namespace msg
}  // namespace unitree_hg
