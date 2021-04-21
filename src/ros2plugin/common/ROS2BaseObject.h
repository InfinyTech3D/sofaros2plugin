#pragma once
#include <ros2plugin/common/ROS2Context.h>
#include <ros2plugin/common/types.h>
#include <sofa/core/objectmodel/BaseObject.h>

namespace sofa {
namespace ros2 {

template <class DataTypes, class ROS2_MSG>
class ROS2BaseObject : public core::objectmodel::BaseObject {
   public:
    using Inherit = core::objectmodel::BaseObject;
    SOFA_CLASS(SOFA_TEMPLATE2(ROS2BaseObject, DataTypes, ROS2_MSG), Inherit);

    core::objectmodel::SingleLink<ROS2BaseObject<DataTypes, ROS2_MSG>, ROS2Context, BaseLink::FLAG_STRONGLINK | BaseLink::FLAG_STOREPATH>
        l_ros2Context;

    sofa::Data<std::string> d_NodeName;
    sofa::Data<std::string> d_TopicName;
    std::shared_ptr<rclcpp::TimeSource> m_ts;

    explicit ROS2BaseObject()
        : l_ros2Context(initLink("ros2Context", "ROS2 context link"))
        , d_NodeName(initData(&d_NodeName, std::string("DefaultNodeName"), "nodeName", "Name of the ROS2 node"))
        , d_TopicName(initData(&d_TopicName, std::string("DefaultTopicName"), "topicName", "Name of the ROS2 topic in which to publish")){};

    virtual ~ROS2BaseObject() = default;

    virtual void init() = 0;

    template <class ROS2NodeType>
    void createNode(std::shared_ptr<ROS2NodeType>& node_ptr)
    {
        node_ptr = std::make_shared<ROS2NodeType>(this->d_NodeName.getValue(), this->d_TopicName.getValue());
        m_ts     = std::make_shared<rclcpp::TimeSource>(node_ptr);
        this->l_ros2Context->addNode(node_ptr);
    }

    static bool canCreate(ROS2BaseObject<DataTypes, ROS2_MSG>* o, core::objectmodel::BaseContext* context,
                          core::objectmodel::BaseObjectDescription* arg)
    {
        // Automatically locate context if it does not exist
        std::string context_path = arg->getAttribute("ros2Context", "");
        if (context_path.empty()) {
            // Try to find a ROS2Context component
            const auto context_candidates = context->getObjects<ROS2Context>(core::objectmodel::BaseContext::SearchDirection::Local);
            if (context_candidates.size() == 0) {
                arg->logError("No ROS2Context was found! Please create one in order to use ROS2Subscriber component.");
                return false;
            }
            context_path = context_candidates[0]->getPathName();
            // If a context was found, link it to this component
            if (Inherit::canCreate(o, context, arg)) {
                arg->setAttribute("ros2Context", "@" + context_path);
                return true;
            }
        }
        return Inherit::canCreate(o, context, arg);
    }
};

}  // namespace ros2
}  // end namespace sofa
