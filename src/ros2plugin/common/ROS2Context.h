#pragma once

#include <sofa/core/objectmodel/BaseObject.h>

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>
#include <string>
#include <thread>

namespace sofa
{
namespace ros2
{
/**
 * ROS2Context is a SOFA component responsible for managing the ROS context initialization and thread deployment.
 * Only one ROS2Context can be initialized in a single process - so only one per scene.
 * Be sure to check scenes/example.scn for a simple example of publishing SOFA positions to ROS2 and echoing the topics using subscribers.
 *
 * Developer observations:
 * Every ROS node in the scene should posses a SingleLink to the context and add itself during it's init() method.
 * The example below was extracted from ROS2SubscriberBase.h:
    class ROS2SubscriberBase : public core::objectmodel::BaseObject {
       public:
        SOFA_CLASS(SOFA_TEMPLATE2(ROS2SubscriberBase, DataTypes, ROS2_MSG), core::objectmodel::BaseObject);
        core::objectmodel::SingleLink<ROS2SubscriberBase<DataTypes, ROS2_MSG>, ROS2Context, BaseLink::FLAG_STRONGLINK | BaseLink::FLAG_STOREPATH>
        l_ros2Context;
        ...
        virtual void init() {
            m_ros2node = std::make_shared<ROS2SubscriberNode<ROS2_MSG>>(d_NodeName.getValue(), d_TopicName.getValue());
            l_ros2Context->addNode(m_ros2node);
    }
 *
 */
class ROS2Context : public core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(ROS2Context, core::objectmodel::BaseObject);

    ROS2Context() { rclcpp::init(0, nullptr); };

    virtual ~ROS2Context() = default;

    void init() override {}

    void bwdInit() override
    {
		for (auto node : m_workers)
		{
			auto thread = new std::thread([node]
										  {
											  rclcpp::executors::StaticSingleThreadedExecutor executor;
											  executor.add_node(node);
											  executor.spin();
										  });
			thread->detach();
		}
    }

    void cleanup() override { rclcpp::shutdown(); }

    void addNode(const std::shared_ptr<rclcpp::Node>& new_node) { m_workers.push_back(new_node); }

private:
    std::vector<std::shared_ptr<rclcpp::Node>> m_workers;
};

}  // namespace ros2
}  // end namespace sofa
