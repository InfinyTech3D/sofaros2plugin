#pragma once
#include <ROS2Plugin/ROS2Context.h>
#include <ROS2Plugin/ROS2SubscriberNode.h>
#include <ROS2Plugin/toolbox.h>
#include <ROS2Plugin/types.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <sofa/simulation/AnimateBeginEvent.h>

namespace sofa {
namespace ros2 {
template <class DataTypes, class ROS2_MSG>
class ROS2SubscriberBase : public core::objectmodel::BaseObject {
   public:
    SOFA_CLASS(SOFA_TEMPLATE2(ROS2SubscriberBase, DataTypes, ROS2_MSG), core::objectmodel::BaseObject);

    core::objectmodel::SingleLink<ROS2SubscriberBase<DataTypes, ROS2_MSG>, ROS2Context, BaseLink::FLAG_STRONGLINK | BaseLink::FLAG_STOREPATH>
        l_ros2Context;

    sofa::Data<DataTypes>           d_output;
    sofa::Data<std::string>         d_NodeName;
    sofa::Data<std::string>         d_TopicName;

    std::shared_ptr<ROS2SubscriberNode<ROS2_MSG>> m_ros2node;

    ROS2SubscriberBase()
        : l_ros2Context(initLink("ros2Context", "ROS2 context link"))
        , d_output(initData(&d_output, DataTypes(), "output", "output"))
        , d_NodeName(initData(&d_NodeName, std::string("DefaultNodeName"), "nodeName", "Name of the ROS2 node"))
        , d_TopicName(initData(&d_TopicName, std::string("DefaultTopicName"), "topicName", "Name of the ROS2 topic in which to publish")) {
        this->f_listening.setValue(true);
    }

    virtual ~ROS2SubscriberBase() = default;

    virtual void init() {
        m_ros2node = std::make_shared<ROS2SubscriberNode<ROS2_MSG>>(d_NodeName.getValue(), d_TopicName.getValue());
        l_ros2Context->addNode(m_ros2node);
    }

    virtual void handleEvent(sofa::core::objectmodel::Event* event) {
        if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event)) {
            auto msg = m_ros2node->get();
            d_output.setValue(toSofa(msg));
        }
    }
};

}  // namespace ros2
}  // end namespace sofa
