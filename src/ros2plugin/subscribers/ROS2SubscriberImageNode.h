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
    explicit ROS2SubscriberImageNode(const std::string& node_name = "DefaultNodeName", std::string topic_name = "DefaultTopicName")
        : Node(node_name), m_node_name(node_name), m_topic_name(topic_name)
    {
    }

    void init(std::string /*encoding*/, std::string compression)
    {
        try
        {
            m_subscription = image_transport::create_subscription(
                this, m_topic_name, std::bind(&ROS2SubscriberImageNode::callback, this, std::placeholders::_1), compression);
        }
        catch (const std::exception& ex)
        {
            std::cerr << "Image transport exeption: " << ex.what();
        }
    }

    GenericImageMsg::ConstSharedPtr get() const { return m_msg_ptr ? m_msg_ptr : nullptr; /* could this copy be avoided ? */ }

    const std::string m_node_name;
    const std::string m_topic_name;

protected:
    virtual void callback(const GenericImageMsg::ConstSharedPtr& msg) { m_msg_ptr = msg; }

private:
    image_transport::Subscriber m_subscription;
    GenericImageMsg::ConstSharedPtr m_msg_ptr = nullptr;
};
}  // namespace ros2
}  // namespace sofa
