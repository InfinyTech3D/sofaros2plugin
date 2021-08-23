#pragma once

#include <ros2plugin/common/types.h>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace sofa
{
namespace ros2
{
/**
 * ROS2 Subscriber node using image transportwhich is deployed in the ROS2Context
 */
class ROS2SubscriberImageNode : public rclcpp::Node
{
public:
    explicit ROS2SubscriberImageNode(const std::string& node_name = "DefaultNodeName", std::string topic_name = "DefaultTopicName") : Node(node_name)
    {
        m_subscription =
            image_transport::create_subscription(this, topic_name, std::bind(&ROS2SubscriberImageNode::callback, this, std::placeholders::_1), "raw");
    }

    ImageMsg get() const { return m_msg_ptr ? ImageMsg(*m_msg_ptr) : ImageMsg(); /* could this copy be avoided ? */ }

protected:
    virtual void callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) { m_msg_ptr = msg; }

private:
    image_transport::Subscriber m_subscription;
    sensor_msgs::msg::Image::ConstSharedPtr m_msg_ptr = nullptr;
};
}  // namespace ros2
}  // namespace sofa