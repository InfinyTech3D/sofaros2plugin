#pragma once
#include <ros2plugin/publishers/ROS2Publisher.h>
namespace sofa {
namespace ros2 {

template <class DataTypes, class ROS2_MSG>
ROS2Publisher<DataTypes, ROS2_MSG>::ROS2Publisher()
    : d_input(initData(&d_input, DataTypes(), "input", "input"))
    , d_drawScale(initData(&d_drawScale, 0.1, "drawScale", "Scale imposed to draw function in SOFA"))
    , d_draw(initData(&d_draw, false, "draw", "If true, position is drawn on screen"))
{
    this->f_listening.setValue(true);
}

template <class DataTypes, class ROS2_MSG>
void ROS2Publisher<DataTypes, ROS2_MSG>::init()
{
    this->createNode(m_ros2node);
    c_callback.addInput(&d_input);
    c_callback.addCallback([this] { update(); });
}

template <class DataTypes, class ROS2_MSG>
void ROS2Publisher<DataTypes, ROS2_MSG>::update()
{
    DataTypes input = d_input.getValue();
    ROS2_MSG msg    = MessageWrapper<DataTypes, ROS2_MSG>::toROS(input);
    m_ros2node->publish(msg);
}

template <class DataTypes, class ROS2_MSG>
void ROS2Publisher<DataTypes, ROS2_MSG>::draw(const sofa::core::visual::VisualParams *vparams)
{
    if (d_draw.getValue()) {
        const auto &input = d_input.getValue();
        MessageWrapper<DataTypes, ROS2_MSG>::draw(vparams, input, d_drawScale.getValue());
    }
}

}  // namespace ros2
}  // namespace sofa