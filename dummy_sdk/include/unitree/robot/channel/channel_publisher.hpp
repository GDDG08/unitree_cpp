// Dummy unitree_sdk2 — DDS publisher (drops every write).
#pragma once

#include <unitree/dummy_prelude.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

namespace unitree {
namespace robot {

template <typename MSG>
class ChannelPublisher {
   public:
    explicit ChannelPublisher(const std::string& topic) : topic_(topic) {}
    void InitChannel() {}
    void Write(const MSG&) {}

   private:
    std::string topic_;
};

template <typename MSG>
using ChannelPublisherPtr = std::shared_ptr<ChannelPublisher<MSG>>;

}  // namespace robot
}  // namespace unitree
