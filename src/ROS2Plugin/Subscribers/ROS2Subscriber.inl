#pragma once
#include <ROS2Plugin/Subscribers/ROS2Subscriber.h>

namespace sofa {
namespace ros2 {

template <class DataTypes, class ROS2_MSG>
ROS2Subscriber<DataTypes, ROS2_MSG>::ROS2Subscriber()
    : l_ros2Context(initLink("ros2Context", "ROS2 context link"))
    , d_output(initData(&d_output, DataTypes(), "output", "output"))
    , d_NodeName(initData(&d_NodeName, std::string("DefaultNodeName"), "nodeName", "Name of the ROS2 node"))
    , d_TopicName(initData(&d_TopicName, std::string("DefaultTopicName"), "topicName", "Name of the ROS2 topic in which to publish"))
    , d_drawScale(initData(&d_drawScale, 0.1, "drawScale", "Scale imposed to draw function in SOFA"))
    , d_draw(initData(&d_draw, false, "draw", "If true, position is drawn on screen"))
{
    this->f_listening.setValue(true);
}

template <class DataTypes, class ROS2_MSG>
void ROS2Subscriber<DataTypes, ROS2_MSG>::init()
{
    m_ros2node = std::make_shared<ROS2SubscriberNode<ROS2_MSG>>(d_NodeName.getValue(), d_TopicName.getValue());
    l_ros2Context->addNode(m_ros2node);
}

template <class DataTypes, class ROS2_MSG>
void ROS2Subscriber<DataTypes, ROS2_MSG>::handleEvent(sofa::core::objectmodel::Event *event)
{
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event)) {
        auto msg = m_ros2node->get();
        d_output.setValue(toolbox::toSofa(msg));
    }
}

template <class DataTypes, class ROS2_MSG>
void ROS2Subscriber<DataTypes, ROS2_MSG>::draw(const sofa::core::visual::VisualParams *vparams)
{
    if (d_draw.getValue()) {
        const auto &output = d_output.getValue();
        toolbox::draw(vparams, output, d_drawScale.getValue());
    }
}

template <class DataTypes, class ROS2_MSG>
bool ROS2Subscriber<DataTypes, ROS2_MSG>::canCreate(ROS2Subscriber<DataTypes, ROS2_MSG> *o, core::objectmodel::BaseContext *context, core::objectmodel::BaseObjectDescription *arg)
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

}  // namespace ros2
}  // end namespace sofa
