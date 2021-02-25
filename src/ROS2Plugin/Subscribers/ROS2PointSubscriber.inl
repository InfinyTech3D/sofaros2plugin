#pragma once

#include <ROS2Plugin/Subscribers/ROS2PointSubscriber.h>

namespace sofa {
namespace ros2 {

ROS2PointSubscriber::ROS2PointSubscriber()
    : ROS2SubscriberBase<SofaType, ROS2Type>(), d_draw(initData(&d_draw, false, "draw", "If true, position is drawn on screen")) {
    this->f_listening.setValue(true);
}

void ROS2PointSubscriber::draw(const sofa::core::visual::VisualParams *vparams) {
    if (d_draw.getValue()) {
        const auto &output = d_output.getValue();
        vparams->drawTool()->drawSphere(output, 0.1, RGBAColor(1, 1, 1, 1));
    }
}

}  // namespace ros2
}  // end namespace sofa
