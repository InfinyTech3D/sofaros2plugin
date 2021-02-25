#pragma once
#include <ROS2Plugin/types.h>

namespace sofa {
namespace ros2 {
/** SOFA -> ROS2 decode */
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

/** ROS2 -> SOFA decode */
inline Vec3d toSofa(const PointMsg& point) {
    auto vec3d = Vec3d();
    vec3d[0] = point.x;
    vec3d[1] = point.y;
    vec3d[2] = point.z;
    return vec3d;
}

inline Rigid toSofa(const PoseMsg& pose) {
    Vec3d vec3d = toSofa(pose.position);
    Quat quat = Quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    return Rigid(vec3d, quat);
}
}  // namespace ros2
}  // namespace sofa