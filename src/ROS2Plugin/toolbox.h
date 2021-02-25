#pragma once
#include <ROS2Plugin/types.h>

namespace sofa {
namespace ros2 {
inline PointMsg toROS(const Vec3d& vec3d) {
    auto point = PointMsg();
    point.x    = vec3d[0];
    point.y    = vec3d[1];
    point.z    = vec3d[2];
    return point;
}

inline PoseMsg toROS(const Rigid& rigid) {
    auto pose          = PoseMsg();
    pose.position      = toROS(rigid.getCenter());
    pose.orientation.x = rigid.getOrientation()[0];
    pose.orientation.y = rigid.getOrientation()[1];
    pose.orientation.z = rigid.getOrientation()[2];
    pose.orientation.w = rigid.getOrientation()[3];
    return pose;
}
}  // namespace ros2
}  // namespace sofa