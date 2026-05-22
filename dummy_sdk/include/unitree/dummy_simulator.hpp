// Dummy unitree_sdk2 — synthetic data simulation.
//
// Without real DDS there is no hardware to talk to, so the dummy subscribers
// (see channel_subscriber.hpp) synthesize messages and hand them straight to the
// controller's callbacks at subscription time. This populates the controller's
// state buffers so `self_check()` passes and the pipeline runs on a machine
// without the SDK (e.g. macOS): a stationary robot with zeroed joints, an
// identity base IMU and a slightly tilted torso IMU.
//
// Delivery is synchronous (no background threads): every message is produced
// while the controller is fully constructed and alive, which avoids the
// use-after-free that a teardown-racing callback thread would hit.
#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

namespace unitree_dummy {

// Mirror of the controller's Crc32Core (poly 0x04c11db7, init 0xFFFFFFFF) so a
// synthesized LowState_ passes the controller's CRC check.
inline uint32_t crc32(const uint32_t* ptr, uint32_t len) {
    uint32_t crc = 0xFFFFFFFF;
    const uint32_t poly = 0x04c11db7;
    for (uint32_t i = 0; i < len; ++i) {
        uint32_t xbit = 1u << 31;
        uint32_t data = ptr[i];
        for (uint32_t bit = 0; bit < 32; ++bit) {
            if (crc & 0x80000000)
                crc = (crc << 1) ^ poly;
            else
                crc <<= 1;
            if (data & xbit)
                crc ^= poly;
            xbit >>= 1;
        }
    }
    return crc;
}

// LowState_ aggregates the latest torso-IMU / sport buffers when it is handled,
// so after those arrive we re-deliver LowState_ to refresh the assembled
// RobotState. These hooks are registered by LowState_ subscribers and invoked by
// other subscribers, all during controller construction. ChannelFactory::Init()
// clears them so a freshly constructed controller never calls a stale hook.
inline std::vector<std::function<void()>>& lowstate_refreshers() {
    static std::vector<std::function<void()>> hooks;
    return hooks;
}

inline void refresh_lowstate() {
    for (auto& hook : lowstate_refreshers()) {
        if (hook) {
            hook();
        }
    }
}

}  // namespace unitree_dummy

// Customization point used by the dummy subscriber. The generic version leaves
// the default-constructed message untouched; overloads below fill plausible data.
template <typename MSG>
inline void dummy_fill(MSG&, uint32_t /*tick*/) {}

inline void dummy_fill(unitree_hg::msg::dds_::LowState_& s, uint32_t tick) {
    s.tick() = (tick == 0) ? 1u : tick;  // self_check rejects tick == 0
    s.mode_machine() = 4;
    s.imu_state().quaternion() = {1.0f, 0.0f, 0.0f, 0.0f};  // identity base IMU (w-first)
    // motors and wireless_remote stay zero-initialized.
    // crc is the final member of LowState_; CRC every preceding word.
    s.crc() = unitree_dummy::crc32(reinterpret_cast<const uint32_t*>(&s), (sizeof(s) >> 2) - 1);
}

inline void dummy_fill(unitree_hg::msg::dds_::IMUState_& s, uint32_t /*tick*/) {
    // ~5 deg pitch (w-first), distinct from identity so the consumer treats it as
    // a real torso-IMU sample rather than "no data yet".
    s.quaternion() = {0.99905f, 0.0f, 0.04362f, 0.0f};
}
