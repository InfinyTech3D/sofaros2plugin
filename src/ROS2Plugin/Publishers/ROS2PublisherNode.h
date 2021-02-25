#pragma once

#include <ROS2Plugin/types.h>

namespace sofa {
namespace ros2 {
/**
 * ROS2 Publisher node which is deployed in the ROS2Context
 *
 * @tparam ROS2_MSG
 */
template <class ROS2_MSG>
class ROS2PublisherNode : public rclcpp::Node {
   public:
    explicit ROS2PublisherNode(const std::string &node_name = "DefaultNodeName", std::string topic_name = "DefaultTopicName", size_t buffer_size = 1)
        : Node(node_name) {
        m_publisher = this->create_publisher<ROS2_MSG>(topic_name, buffer_size);
    }

    void publish(const ROS2_MSG &msg) { m_publisher->publish(msg); }

   private:
    typename rclcpp::Publisher<ROS2_MSG>::SharedPtr m_publisher;
};
}  // namespace ros2
}  // namespace sofa