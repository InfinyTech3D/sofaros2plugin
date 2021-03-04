#pragma once
#include <ROS2Plugin/Publishers/ROS2Publisher.h>
namespace sofa {
namespace ros2 {

template <class DataTypes, class ROS2_MSG>
ROS2Publisher<DataTypes, ROS2_MSG>::ROS2Publisher()
    : l_ros2Context(initLink("ros2Context", "ROS2 context link"))
    , d_input(initData(&d_input, DataTypes(), "input", "input"))
    , d_NodeName(initData(&d_NodeName, std::string("DefaultNodeName"), "nodeName", "Name of the ROS2 node"))
    , d_TopicName(initData(&d_TopicName, std::string("DefaultTopicName"), "topicName", "Name of the ROS2 topic in which to publish"))
    , d_drawScale(initData(&d_drawScale, 0.1, "drawScale", "Scale imposed to draw function in SOFA"))
    , d_draw(initData(&d_draw, false, "draw", "If true, position is drawn on screen"))
{
    this->f_listening.setValue(true);
}

template <class DataTypes, class ROS2_MSG>
void ROS2Publisher<DataTypes, ROS2_MSG>::init()
{
    c_callback.addInput(&d_input);
    c_callback.addCallback([this] { update(); });
    m_ros2node = std::make_shared<ROS2PublisherNode<ROS2_MSG>>(d_NodeName.getValue(), d_TopicName.getValue());
    l_ros2Context->addNode(m_ros2node);
}

template <class DataTypes, class ROS2_MSG>
void ROS2Publisher<DataTypes, ROS2_MSG>::update()
{
    DataTypes input = d_input.getValue();
    ROS2_MSG msg    = toolbox::toROS(input);
    m_ros2node->publish(msg);
}

template <class DataTypes, class ROS2_MSG>
void ROS2Publisher<DataTypes, ROS2_MSG>::draw(const sofa::core::visual::VisualParams *vparams)
{
    if (d_draw.getValue()) {
        const auto &input = d_input.getValue();
        toolbox::draw(vparams, input, d_drawScale.getValue());
    }
}

}  // namespace ros2
}  // namespace sofa