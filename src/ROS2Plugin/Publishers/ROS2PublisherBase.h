#pragma once
#include <ROS2Plugin/Publishers/ROS2PublisherNode.h>
#include <ROS2Plugin/ROS2Context.h>
#include <ROS2Plugin/toolbox.h>
#include <ROS2Plugin/types.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataCallback.h>

namespace sofa {
namespace ros2 {
/**
 * Base class for ROS2Plugin publishers.
 * When inheriting from this component, the Publisher Node insertion into the ROS2 context is managed automatically.
 * Only non-templated child classes have been tested for the moment.
 *
 * TODO: an automatic SOFA type deduction would benefit scene readability and prevent user-level bugs.
 *
 * @tparam DataTypes SOFA data type should be informed by the child class implementation
 * @tparam ROS2_MSG ROS message type should be informed by the child class implementation
 */
template <class DataTypes, class ROS2_MSG>
class ROS2PublisherBase : public core::objectmodel::BaseObject {
   public:
    SOFA_CLASS(SOFA_TEMPLATE2(ROS2PublisherBase, DataTypes, ROS2_MSG), core::objectmodel::BaseObject);

    core::objectmodel::SingleLink<ROS2PublisherBase<DataTypes, ROS2_MSG>, ROS2Context, BaseLink::FLAG_STRONGLINK | BaseLink::FLAG_STOREPATH>
        l_ros2Context;

    sofa::Data<DataTypes>           d_input;
    core::objectmodel::DataCallback c_callback;
    sofa::Data<std::string>         d_NodeName;
    sofa::Data<std::string>         d_TopicName;

    std::shared_ptr<ROS2PublisherNode<ROS2_MSG>> m_ros2node;

    ROS2PublisherBase()
        : l_ros2Context(initLink("ros2Context", "ROS2 context link"))
        , d_input(initData(&d_input, DataTypes(), "input", "input"))
        , d_NodeName(initData(&d_NodeName, std::string("DefaultNodeName"), "nodeName", "Name of the ROS2 node"))
        , d_TopicName(initData(&d_TopicName, std::string("DefaultTopicName"), "topicName", "Name of the ROS2 topic in which to publish")) {
        this->f_listening.setValue(true);
    }

    virtual ~ROS2PublisherBase() = default;

    virtual void init() {
        c_callback.addInput(&d_input);
        c_callback.addCallback([this] { update(); });
        m_ros2node = std::make_shared<ROS2PublisherNode<ROS2_MSG>>(d_NodeName.getValue(), d_TopicName.getValue());
        l_ros2Context->addNode(m_ros2node);
    }

    virtual void update() {
        DataTypes input = d_input.getValue();
        ROS2_MSG  msg   = toROS(input);
        m_ros2node->publish(msg);
    }
};

}  // namespace ros2
}  // end namespace sofa
