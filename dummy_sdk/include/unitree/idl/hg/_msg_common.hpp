// Dummy unitree_sdk2 — shared per-motor message records.
#pragma once

#include <unitree/dummy_prelude.hpp>

namespace unitree_hg {
namespace msg {
namespace dds_ {

struct MotorState_ {
    UT_DUMMY_FIELD(q, float)
    UT_DUMMY_FIELD(dq, float)
    UT_DUMMY_FIELD(tau_est, float)
    UT_DUMMY_FIELD(motorstate, uint8_t)
};

struct MotorCmd_ {
    UT_DUMMY_FIELD(mode, uint8_t)
    UT_DUMMY_FIELD(q, float)
    UT_DUMMY_FIELD(dq, float)
    UT_DUMMY_FIELD(kp, float)
    UT_DUMMY_FIELD(kd, float)
    UT_DUMMY_FIELD(tau, float)
};

}  // namespace dds_
}  // namespace msg
}  // namespace unitree_hg
