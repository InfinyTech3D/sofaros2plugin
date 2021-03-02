#pragma once

#include <ROS2Plugin/ROS2Context.h>
#include <ROS2Plugin/Subscribers/ROS2SubscriberBase.h>
#include <ROS2Plugin/toolbox.h>
#include <ROS2Plugin/types.h>

namespace sofa {
namespace ros2 {
class ROS2PointSubscriber : public ROS2SubscriberBase<Vec3d, PointMsg> {
   public:
    using SofaType = Vec3d;
    using ROS2Type = PointMsg;

    SOFA_CLASS(ROS2PointSubscriber, SOFA_TEMPLATE2(ROS2SubscriberBase, SofaType, ROS2Type));
    explicit ROS2PointSubscriber() : ROS2SubscriberBase(){};
};

}  // namespace ros2
}  // end namespace sofa
