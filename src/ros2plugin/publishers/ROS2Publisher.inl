#pragma once
#include <ros2plugin/publishers/ROS2Publisher.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>

namespace sofa
{
namespace ros2
{
template <class DataTypes, class ROS2_MSG>
ROS2Publisher<DataTypes, ROS2_MSG>::ROS2Publisher()
    : d_input(initData(&d_input, DataTypes(), "input", "input"))
    , d_drawScale(initData(&d_drawScale, 0.1, "drawScale", "Scale imposed to draw function in SOFA"))
    , d_scale(initData(&d_scale,1.0, "scale", "Scale to apply to the data. The way it is applied depends on the ToSofa method."))
    , d_canPublish(initData(&d_canPublish, false, "canPublish", "If true, publisher start to publish"))
    , d_draw(initData(&d_draw, false, "draw", "If true, position is drawn on screen"))
{
    this->f_listening.setValue(true);
}

template <class DataTypes, class ROS2_MSG>
void ROS2Publisher<DataTypes, ROS2_MSG>::init()
{
    this->createNode(m_ros2node);
}

template <class DataTypes, class ROS2_MSG>
void ROS2Publisher<DataTypes, ROS2_MSG>::handleEvent(sofa::core::objectmodel::Event *event)
{

    if (dynamic_cast<sofa::simulation::AnimateEndEvent *>(event))
    {
        if(d_canPublish.getValue())
        {
            DataTypes input = d_input.getValue();
                ROS2_MSG msg = MessageWrapper<DataTypes, ROS2_MSG>::toROS(input,d_scale.getValue());
            m_ros2node->publish(msg);
        }
    }
    if (sofa::core::objectmodel::KeypressedEvent* ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event)) {
        if (ev->getKey() == 'p' ||ev->getKey() == 'P')
        {
            d_canPublish.setValue(!d_canPublish.getValue());
        }
    }
}

template <class DataTypes, class ROS2_MSG>
void ROS2Publisher<DataTypes, ROS2_MSG>::draw(const sofa::core::visual::VisualParams *vparams)
{
    if (d_draw.getValue())
    {
        const auto &input = d_input.getValue();
        MessageWrapper<DataTypes, ROS2_MSG>::draw(vparams, input, d_drawScale.getValue());
    }
}

}  // namespace ros2
}  // namespace sofa