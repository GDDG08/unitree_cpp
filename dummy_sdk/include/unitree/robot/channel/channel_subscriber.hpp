// Dummy unitree_sdk2 — DDS subscriber.
//
// Real DDS delivers messages from the network on a background thread. With no
// network available, this dummy synthesizes messages (see dummy_simulator.hpp)
// and delivers them synchronously to the registered callback at subscription
// time, so the controller observes a populated (stationary) robot state without
// any background thread that could outlive the controller.
#pragma once

#include <type_traits>

#include <unitree/dummy_prelude.hpp>
#include <unitree/dummy_simulator.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

namespace unitree {
namespace robot {

template <typename MSG>
class ChannelSubscriber {
   public:
    explicit ChannelSubscriber(const std::string& topic) : topic_(topic) {}

    // Accepts the std::bind(...) handler used by the controller and immediately
    // feeds it synthetic data.
    template <typename Handler>
    void InitChannel(Handler&& handler, int = 0) {
        handler_ = std::forward<Handler>(handler);

        if constexpr (std::is_same_v<MSG, unitree_hg::msg::dds_::LowState_>) {
            // LowState_ assembles RobotState from dependent buffers, so register a
            // refresh hook (re-invoked once torso/sport data arrives) and deliver now.
            auto deliver = [this]() { this->Deliver(); };
            deliver();
            unitree_dummy::lowstate_refreshers().push_back(deliver);
        } else {
            // torso IMU / sport state: deliver, then refresh LowState so the
            // assembled RobotState includes this freshly-populated buffer.
            Deliver();
            unitree_dummy::refresh_lowstate();
        }
    }

   private:
    void Deliver() {
        MSG msg{};
        dummy_fill(msg, ++tick_);
        if (handler_) {
            handler_(static_cast<const void*>(&msg));
        }
    }

    std::string topic_;
    std::function<void(const void*)> handler_;
    uint32_t tick_ = 0;
};

template <typename MSG>
using ChannelSubscriberPtr = std::shared_ptr<ChannelSubscriber<MSG>>;

}  // namespace robot
}  // namespace unitree
