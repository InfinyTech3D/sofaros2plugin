#pragma once
#include <ros2plugin/common/types.h>
#include <sofa/core/visual/VisualParams.h>

namespace sofa {
namespace ros2 {

template <class DataTypes, class ROS2_MSG>
class MessageWrapper {
   public:
    static inline void draw(const sofa::core::visual::VisualParams* /*vparams*/, const DataTypes& /*sofa_type*/, const double& /*scale*/) {}
    static inline DataTypes toSofa(const ROS2_MSG& /*ros_msg*/)
    {
        msg_info("ROS2Plugin") << "in function toSofa: Types informed at template are unknown";
    }
    static inline ROS2_MSG toROS(const DataTypes& /*sofa_type*/)
    {
        msg_info("ROS2Plugin") << "in function toROS: Types informed at template are unknown";
    }
};

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *     double                    Float64Msg
 */
template <>
inline double MessageWrapper<double, Float64Msg>::toSofa(const Float64Msg& ros_msg)
{
    return ros_msg.data;
}
template <>
inline Float64Msg MessageWrapper<double, Float64Msg>::toROS(const double& sofa_type)
{
    auto ros_msg = Float64Msg();
    ros_msg.data = sofa_type;
    return ros_msg;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *     DoubleArray               Float64ArrayMsg
 */
template <>
inline DoubleArray MessageWrapper<DoubleArray, Float64ArrayMsg>::toSofa(const Float64ArrayMsg& ros_msg)
{
    return DoubleArray(ros_msg.data.begin(), ros_msg.data.end());
}
template <>
inline Float64ArrayMsg MessageWrapper<DoubleArray, Float64ArrayMsg>::toROS(const DoubleArray& sofa_type)
{
    auto ros_msg = Float64ArrayMsg();
    ros_msg.data = sofa_type;
    return ros_msg;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *      Vec3d                      PointMsg
 */
template <>
inline void MessageWrapper<Vec3d, PointMsg>::draw(const sofa::core::visual::VisualParams* vparams, const Vec3d& vec3d, const double& scale)
{
    vparams->drawTool()->drawSphere(vec3d, scale, RGBAColor(1, 1, 1, 1));
}
template <>
inline Vec3d MessageWrapper<Vec3d, PointMsg>::toSofa(const PointMsg& point)
{
    return Vec3d(point.x, point.y, point.z);
}
template <>
inline PointMsg MessageWrapper<Vec3d, PointMsg>::toROS(const Vec3d& vec3d)
{
    auto point = PointMsg();
    point.x    = vec3d[0];
    point.y    = vec3d[1];
    point.z    = vec3d[2];
    return point;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *      Quat                      QuatMsg
 */
template <>
inline Quat MessageWrapper<Quat, QuatMsg>::toSofa(const QuatMsg& quat)
{
    return Quat(quat.x, quat.y, quat.z, quat.w);
}
template <>
inline QuatMsg MessageWrapper<Quat, QuatMsg>::toROS(const Quat& orientation)
{
    auto quat = QuatMsg();
    quat.x    = orientation[0];
    quat.y    = orientation[1];
    quat.z    = orientation[2];
    quat.w    = orientation[3];
    return quat;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *      Rigid                      PoseMsg
 */
template <>
inline void MessageWrapper<Rigid, PoseMsg>::draw(const sofa::core::visual::VisualParams* vparams, const Rigid& pose, const double& scale)
{
    vparams->drawTool()->drawFrame(pose.getCenter(), pose.getOrientation(), scale * Vec3d(1, 1, 1));
}
template <>
inline Rigid MessageWrapper<Rigid, PoseMsg>::toSofa(const PoseMsg& pose)
{
    return Rigid(MessageWrapper<Vec3d, PointMsg>::toSofa(pose.position), MessageWrapper<Quat, QuatMsg>::toSofa(pose.orientation));
}
template <>
inline PoseMsg MessageWrapper<Rigid, PoseMsg>::toROS(const Rigid& rigid)
{
    auto pose        = PoseMsg();
    pose.position    = MessageWrapper<Vec3d, PointMsg>::toROS(rigid.getCenter());
    pose.orientation = MessageWrapper<Quat, QuatMsg>::toROS(rigid.getOrientation());
    return pose;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *      Rigid                      PoseStampedMsg
 */
template <>
inline void MessageWrapper<Rigid, PoseStampedMsg>::draw(const sofa::core::visual::VisualParams* vparams, const Rigid& pose, const double& scale)
{
    MessageWrapper<Rigid, PoseMsg>::draw(vparams, pose, scale);
}
template <>
inline Rigid MessageWrapper<Rigid, PoseStampedMsg>::toSofa(const PoseStampedMsg& msg)
{
    return MessageWrapper<Rigid, PoseMsg>::toSofa(msg.pose);
}
template <>
inline PoseStampedMsg MessageWrapper<Rigid, PoseStampedMsg>::toROS(const Rigid& rigid)
{
    auto msg   = PoseStampedMsg();
    msg.pose   = MessageWrapper<Rigid, PoseMsg>::toROS(rigid);
    msg.header.stamp = rclcpp::Time();
    return msg;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *    DoubleArray               JointStateMsg
 * Note: At the moment, only position data is transmitted
 * TODO: Integrate another DataType which accounts for velocity and forces (MechanicalStates ?)
 */
template <>
inline DoubleArray MessageWrapper<DoubleArray, JointStateMsg>::toSofa(const JointStateMsg& joint_msg)
{
    return DoubleArray(joint_msg.position.begin(), joint_msg.position.end());
}
template <>
inline JointStateMsg MessageWrapper<DoubleArray, JointStateMsg>::toROS(const DoubleArray& array)
{
    auto joint_msg = JointStateMsg();
    joint_msg.name.resize(array.size());
    joint_msg.position.resize(array.size());
    for (size_t i = 0; i < array.size(); i++) {
        joint_msg.name[i]     = "joint_" + std::to_string(i);
        joint_msg.position[i] = array[i];
    }
    return joint_msg;
}

}  // namespace ros2
}  // namespace sofa