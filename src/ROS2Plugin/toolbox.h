#pragma once
#include <ROS2Plugin/types.h>

namespace sofa {
namespace ros2 {

/** ROS2 messages -> SOFA types */
inline Vec3d toSofa(const PointMsg& point) { return Vec3d(point.x, point.y, point.z); }

inline Quat toSofa(const QuatMsg& quat) { return Quat(quat.x, quat.y, quat.z, quat.w); }

inline Rigid toSofa(const PoseMsg& pose) { return Rigid(toSofa(pose.position), toSofa(pose.orientation)); }

/** SOFA types -> ROS2 messages */
inline PointMsg toROS(const Vec3d& vec3d) {
    auto point = PointMsg();
    point.x    = vec3d[0];
    point.y    = vec3d[1];
    point.z    = vec3d[2];
    return point;
}

inline QuatMsg toROS(const Quat& orientation) {
    auto quat = QuatMsg();
    quat.x    = orientation[0];
    quat.y    = orientation[1];
    quat.z    = orientation[2];
    quat.w    = orientation[3];
    return quat;
}

inline PoseMsg toROS(const Rigid& rigid) {
    auto pose        = PoseMsg();
    pose.position    = toROS(rigid.getCenter());
    pose.orientation = toROS(rigid.getOrientation());
    return pose;
}

}  // namespace ros2
}  // namespace sofa