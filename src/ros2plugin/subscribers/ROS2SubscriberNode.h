#pragma once

#include <ros2plugin/common/types.h>
#include "atomic"
#include <mutex>
#include <rclcpp/qos.hpp>

namespace sofa
{
namespace ros2
{
/**
 * ROS2 Subscriber node which is deployed in the ROS2Context
 *
 * TODO: Experiment different callback strategies without copy when heavier messages (e.g point-clouds) are used
 *
 * @tparam ROS2_MSG
 */
template <class ROS2_MSG>
class ROS2SubscriberNode : public rclcpp::Node
{
public:
    explicit ROS2SubscriberNode(const std::string &node_name = "DefaultNodeName", std::string topic_name = "DefaultTopicName", size_t buffer_size = 1)
        : Node(node_name)
    {

		rclcpp::QoS qos(buffer_size);
		qos.best_effort();
		qos.history(rclcpp::HistoryPolicy::KeepLast);
		qos.lifespan(rclcpp::Duration(1,0));
		qos.durability(rclcpp::DurabilityPolicy::Volatile);
        m_subscription =
            this->create_subscription<ROS2_MSG>(topic_name, qos, std::bind(&ROS2SubscriberNode::callback, this, std::placeholders::_1));

    }

    ROS2_MSG get()
    {
        const std::lock_guard<std::mutex> lock(msg_mutex);
        return m_msg;
    }//_ptr.get() ? ROS2_MSG(*(m_msg_ptr.get())) : ROS2_MSG(); /* could this copy be avoided ? */ }



protected:
    virtual void callback(const typename ROS2_MSG::SharedPtr msg)
    {
//		std::cout<<"Callback"<<std::endl;
        const std::lock_guard<std::mutex> lock(msg_mutex);
        m_msg = ROS2_MSG(*(msg.get()));
    }

private:
    typename rclcpp::Subscription<ROS2_MSG>::SharedPtr m_subscription;
//    typename ROS2_MSG::SharedPtr m_msg_ptr;
    ROS2_MSG m_msg;

    std::mutex msg_mutex;

};
}  // namespace ros2
}  // namespace sofa

