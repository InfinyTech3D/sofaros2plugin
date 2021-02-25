#pragma once

#include <ROS2Plugin/ROS2Context.h>
#include <ROS2Plugin/ROS2PublisherBase.h>
#include <ROS2Plugin/toolbox.h>
#include <ROS2Plugin/types.h>
#include <sofa/core/visual/VisualParams.h>

namespace sofa {
namespace ros2 {
class ROS2PointPublisher : public ROS2PublisherBase<Vec3d, PointMsg> {
   public:
    using SofaType = Vec3d;
    using ROS2Type = PointMsg;

    SOFA_CLASS(ROS2PointPublisher, SOFA_TEMPLATE2(ROS2PublisherBase, SofaType, ROS2Type));

    Data<bool> d_draw;

    explicit ROS2PointPublisher();

    virtual void draw(const sofa::core::visual::VisualParams *vparams);
};

}  // namespace ros2
}  // end namespace sofa
