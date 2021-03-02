#pragma once

#include <ROS2Plugin/Publishers/ROS2PublisherBase.h>
#include <ROS2Plugin/ROS2Context.h>
#include <ROS2Plugin/toolbox.h>
#include <ROS2Plugin/types.h>
#include <sofa/core/visual/VisualParams.h>

namespace sofa {
namespace ros2 {
class ROS2PosePublisher : public ROS2PublisherBase<Rigid, PoseMsg> {
   public:
    using SofaType = Rigid;
    using ROS2Type = PoseMsg;

    SOFA_CLASS(ROS2PosePublisher, SOFA_TEMPLATE2(ROS2PublisherBase, SofaType, ROS2Type));
    explicit ROS2PosePublisher() : ROS2PublisherBase(){};
};

}  // namespace ros2
}  // end namespace sofa
