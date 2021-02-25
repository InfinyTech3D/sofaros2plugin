#pragma once

#include <ROS2Plugin/ROS2Context.h>
#include <ROS2Plugin/ROS2PublisherBase.h>
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

    Data<bool> d_draw;

    explicit ROS2PosePublisher();

    virtual void draw(const sofa::core::visual::VisualParams *vparams);
};

}  // namespace ros2
}  // end namespace sofa
