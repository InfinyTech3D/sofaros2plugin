#pragma once
#include <ROS2Plugin/types.h>
#include <sofa/core/visual/VisualParams.h>

namespace sofa {
namespace ros2 {
namespace toolbox {
/** ROS2 messages -> SOFA types */
inline double toSofa(const Float64Msg& msg) { return msg.data; }

inline DoubleArray toSofa(const Float64ArrayMsg& msg) { return DoubleArray(msg.data.begin(), msg.data.end()); }

inline Vec3d toSofa(const PointMsg& point) { return Vec3d(point.x, point.y, point.z); }

inline Quat toSofa(const QuatMsg& quat) { return Quat(quat.x, quat.y, quat.z, quat.w); }

inline Rigid toSofa(const PoseMsg& pose) { return Rigid(toSofa(pose.position), toSofa(pose.orientation)); }

/** SOFA types -> ROS2 messages */
inline Float64Msg toROS(const double& value)
{
    auto msg = Float64Msg();
    msg.data = value;
    return msg;
}

inline Float64ArrayMsg toROS(const DoubleArray& array)
{
    auto msg = Float64ArrayMsg();
    msg.data = array;
    return msg;
}

inline PointMsg toROS(const Vec3d& vec3d)
{
    auto point = PointMsg();
    point.x    = vec3d[0];
    point.y    = vec3d[1];
    point.z    = vec3d[2];
    return point;
}

inline QuatMsg toROS(const Quat& orientation)
{
    auto quat = QuatMsg();
    quat.x    = orientation[0];
    quat.y    = orientation[1];
    quat.z    = orientation[2];
    quat.w    = orientation[3];
    return quat;
}

inline PoseMsg toROS(const Rigid& rigid)
{
    auto pose        = PoseMsg();
    pose.position    = toROS(rigid.getCenter());
    pose.orientation = toROS(rigid.getOrientation());
    return pose;
}

/** Drawing */
inline void draw(const sofa::core::visual::VisualParams*, const double&, const double&) {}

inline void draw(const sofa::core::visual::VisualParams*, const DoubleArray&, const double&) {}

inline void draw(const sofa::core::visual::VisualParams* vparams, const Vec3d& vec3d, const double& scale, RGBAColor color = RGBAColor(1, 1, 1, 1)) { vparams->drawTool()->drawSphere(vec3d, scale, color); }

inline void draw(const sofa::core::visual::VisualParams* vparams, const Rigid& pose, const double& scale) { vparams->drawTool()->drawFrame(pose.getCenter(), pose.getOrientation(), scale * Vec3d(1, 1, 1)); }
}  // namespace toolbox
}  // namespace ros2
}  // namespace sofa