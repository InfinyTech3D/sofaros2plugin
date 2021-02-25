#pragma once

#include <ROS2Plugin/Publishers/ROS2PosePublisher.h>

namespace sofa {
namespace ros2 {

ROS2PosePublisher::ROS2PosePublisher()
    : ROS2PublisherBase<SofaType, ROS2Type>(), d_draw(initData(&d_draw, false, "draw", "If true, position is drawn on screen")) {
    this->f_listening.setValue(true);
}

void ROS2PosePublisher::draw(const sofa::core::visual::VisualParams *vparams) {
    if (d_draw.getValue()) {
        const auto &input = d_input.getValue();
        vparams->drawTool()->drawFrame(input.getCenter(), input.getOrientation(), Vec3d(0.1, 0.1, 0.1));
    }
}

}  // namespace ros2
}  // end namespace sofa
