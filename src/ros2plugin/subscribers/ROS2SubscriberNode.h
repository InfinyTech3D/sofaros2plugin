#pragma once

#include <ros2plugin/common/conversions.h>
#include <ros2plugin/common/types.h>

namespace sofa {
namespace ros2 {
/**
 * ROS2 Subscriber node which is deployed in the ROS2Context
 *
 * TODO: Experiment different callback strategies without copy when heavier messages (e.g point-clouds) are used
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

    ROS2_MSG get() const { return ROS2_MSG(*m_msg_ptr); /* could this copy be avoided ? */ }

   protected:
    virtual void callback(const typename ROS2_MSG::SharedPtr msg) { m_msg_ptr = msg; }

   private:
    typename rclcpp::Subscription<ROS2_MSG>::SharedPtr m_subscription;
    typename ROS2_MSG::SharedPtr                       m_msg_ptr;
};
}  // namespace ros2
}  // namespace sofa