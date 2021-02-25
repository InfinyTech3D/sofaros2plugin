#pragma once

#include <ROS2Plugin/Subscribers/ROS2PoseSubscriber.h>

namespace sofa {
namespace ros2 {

ROS2PoseSubscriber::ROS2PoseSubscriber()
    : ROS2SubscriberBase<SofaType, ROS2Type>(), d_draw(initData(&d_draw, false, "draw", "If true, position is drawn on screen")) {
    this->f_listening.setValue(true);
}

void ROS2PoseSubscriber::draw(const sofa::core::visual::VisualParams *vparams) {
    if (d_draw.getValue()) {
        const auto &output = d_output.getValue();
        vparams->drawTool()->drawFrame(output.getCenter(), output.getOrientation(), Vec3d(0.1, 0.1, 0.1));
    }
}

}  // namespace ros2
}  // end namespace sofa
