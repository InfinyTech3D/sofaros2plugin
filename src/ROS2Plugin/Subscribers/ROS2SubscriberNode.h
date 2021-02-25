#pragma once

#include <ROS2Plugin/toolbox.h>
#include <ROS2Plugin/types.h>

namespace sofa {
namespace ros2 {
/**
 * ROS2 Subscriber node which is deployed in the ROS2Context
 *
 * @tparam ROS2_MSG
 */
template <class ROS2_MSG>
class ROS2SubscriberNode : public rclcpp::Node {
   public:
    explicit ROS2SubscriberNode(const std::string &node_name = "DefaultNodeName", std::string topic_name = "DefaultTopicName", size_t buffer_size = 1)
        : Node(node_name) {
        m_subscription =
            this->create_subscription<ROS2_MSG>(topic_name, buffer_size, std::bind(&ROS2SubscriberNode::callback, this, std::placeholders::_1));
    }

    ROS2_MSG get() const { return m_message; }

   protected:
    virtual void callback(const typename ROS2_MSG::SharedPtr msg) { m_message = ROS2_MSG(*msg); }

   private:
    typename rclcpp::Subscription<ROS2_MSG>::SharedPtr m_subscription;
    ROS2_MSG                                           m_message;
};
}  // namespace ros2
}  // namespace sofa