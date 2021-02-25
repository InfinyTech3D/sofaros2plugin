#pragma once

#include <ROS2Plugin/Publishers/ROS2PointPublisher.h>

namespace sofa {
namespace ros2 {

ROS2PointPublisher::ROS2PointPublisher()
    : ROS2PublisherBase<SofaType, ROS2Type>(), d_draw(initData(&d_draw, false, "draw", "If true, position is drawn on screen")) {
    this->f_listening.setValue(true);
}

void ROS2PointPublisher::draw(const sofa::core::visual::VisualParams *vparams) {
    if (d_draw.getValue()) {
        const auto &input = d_input.getValue();
        vparams->drawTool()->drawSphere(input, 0.1, RGBAColor(1, 1, 1, 1));
    }
}

}  // namespace ros2
}  // end namespace sofa
