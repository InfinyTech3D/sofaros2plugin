#pragma once
#include <ros2plugin/subscribers/ROS2Subscriber.h>

namespace sofa {
namespace ros2 {

template <class DataTypes, class ROS2_MSG>
ROS2Subscriber<DataTypes, ROS2_MSG>::ROS2Subscriber()
    : d_output(initData(&d_output, DataTypes(), "output", "output"))
    , d_initialValue(initData(&d_initialValue, DataTypes(), "initialValue", "initial value for the output if no topic is found"))
    , d_drawScale(initData(&d_drawScale, 0.1, "drawScale", "Scale imposed to draw function in SOFA"))
    , d_draw(initData(&d_draw, false, "draw", "If true, position is drawn on screen"))
{
    this->f_listening.setValue(true);
}

template <class DataTypes, class ROS2_MSG>
void ROS2Subscriber<DataTypes, ROS2_MSG>::init()
{
    d_output.setValue(d_initialValue.getValue());
    this->createNode(m_ros2node);
}

template <class DataTypes, class ROS2_MSG>
void ROS2Subscriber<DataTypes, ROS2_MSG>::handleEvent(sofa::core::objectmodel::Event *event)
{
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event)) {
        auto msg = m_ros2node->get();
        d_output.setValue(MessageWrapper<DataTypes, ROS2_MSG>::toSofa(msg));
    }
}

template <class DataTypes, class ROS2_MSG>
void ROS2Subscriber<DataTypes, ROS2_MSG>::draw(const sofa::core::visual::VisualParams *vparams)
{
    if (d_draw.getValue()) {
        const auto &output = d_output.getValue();
        MessageWrapper<DataTypes, ROS2_MSG>::draw(vparams, output);
    }
}

}  // namespace ros2
}  // end namespace sofa
