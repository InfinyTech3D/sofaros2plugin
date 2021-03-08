#pragma once
#include <ROS2Plugin/Subscribers/ROS2Subscriber.h>

namespace sofa {
namespace ros2 {

template <class DataTypes, class ROS2_MSG>
ROS2Subscriber<DataTypes, ROS2_MSG>::ROS2Subscriber()
    : d_output(initData(&d_output, DataTypes(), "output", "output"))
    , d_drawScale(initData(&d_drawScale, 0.1, "drawScale", "Scale imposed to draw function in SOFA"))
    , d_draw(initData(&d_draw, false, "draw", "If true, position is drawn on screen"))
{
    this->f_listening.setValue(true);
}

template <class DataTypes, class ROS2_MSG>
void ROS2Subscriber<DataTypes, ROS2_MSG>::init()
{
    this->createNode(m_ros2node);
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

}  // namespace ros2
}  // end namespace sofa
