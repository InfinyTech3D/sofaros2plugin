#pragma once
#include <ros2plugin/common/types.h>
#include <sofa/core/visual/VisualParams.h>

namespace sofa
{
namespace ros2
{
template <class DataTypes, class ROS2_MSG>
class MessageWrapper
{
public:
    static inline void draw(const sofa::core::visual::VisualParams* /*vparams*/, const DataTypes& /*sofa_type*/, const double& /*scale*/) {}
    static inline DataTypes toSofa(const ROS2_MSG& /*ros_msg*/)
    {
        msg_info("ROS2Plugin") << "in function toSofa: Types informed at template are unknown";
        return DataTypes();
    }
    static inline ROS2_MSG toROS(const DataTypes& /*sofa_type*/)
    {
        msg_info("ROS2Plugin") << "in function toROS: Types informed at template are unknown";
        return ROS2_MSG();
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
    point.x = vec3d[0];
    point.y = vec3d[1];
    point.z = vec3d[2];
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
    quat.x = orientation[0];
    quat.y = orientation[1];
    quat.z = orientation[2];
    quat.w = orientation[3];
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
    auto pose = PoseMsg();
    pose.position = MessageWrapper<Vec3d, PointMsg>::toROS(rigid.getCenter());
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
    auto msg = PoseStampedMsg();
    msg.pose = MessageWrapper<Rigid, PoseMsg>::toROS(rigid);
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
    for (size_t i = 0; i < array.size(); i++)
    {
        joint_msg.name[i] = "joint_a" + std::to_string(i);
        joint_msg.position[i] = array[i];
    }
    joint_msg.header.stamp = rclcpp::Time();
    return joint_msg;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *      helper::vector<Vec3d>       PointArrayMsg
 */
template <>
inline void MessageWrapper<helper::vector<Vec3d>, PoseArrayMsg>::draw(const sofa::core::visual::VisualParams* vparams,
                                                                      const helper::vector<Vec3d>& vec3d, const double& scale)
{
    for (unsigned i = 0; i < vec3d.size(); i++) vparams->drawTool()->drawSphere(vec3d[i], scale, RGBAColor(1, 1, 1, 1));
}
template <>
inline helper::vector<Vec3d> MessageWrapper<helper::vector<Vec3d>, PoseArrayMsg>::toSofa(const PoseArrayMsg& points)
{
    helper::vector<Vec3d> returnVec;

    for (unsigned i = 0; i < points.poses.size(); i++)
        returnVec.push_back(Vec3d(points.poses[i].position.x, points.poses[i].position.y, points.poses[i].position.z));

    return returnVec;
}
template <>
inline PoseArrayMsg MessageWrapper<helper::vector<Vec3d>, PoseArrayMsg>::toROS(const helper::vector<Vec3d>& vec3d)
{
    auto points = PoseArrayMsg();

    for (unsigned i = 0; i < vec3d.size(); i++)
    {
        PoseArrayMsg::_poses_type::value_type P;
        P.position.set__x(vec3d[i][0]).set__y(vec3d[i][1]).set__z(vec3d[i][2]);
        points.poses.push_back(P);
    }

    return points;
}

/********************************************************************************************************
 *      SOFA         <===>          ROS2
 *      helper::vector<Rigid>      PoseArray
 */
template <>
inline void MessageWrapper<helper::vector<Rigid>, PoseArrayMsg>::draw(const sofa::core::visual::VisualParams* vparams,
                                                                      const helper::vector<Rigid>& vec3d, const double& scale)
{
    for (unsigned i = 0; i < vec3d.size(); i++) MessageWrapper<Rigid, PoseMsg>::draw(vparams, vec3d[i], scale);
}
template <>
inline helper::vector<Rigid> MessageWrapper<helper::vector<Rigid>, PoseArrayMsg>::toSofa(const PoseArrayMsg& msg)
{
    helper::vector<Rigid> returnVec;

    for (unsigned i = 0; i < msg.poses.size(); i++) returnVec.push_back(MessageWrapper<Rigid, PoseMsg>::toSofa(msg.poses[i]));

    return returnVec;
}
template <>
inline PoseArrayMsg MessageWrapper<helper::vector<Rigid>, PoseArrayMsg>::toROS(const helper::vector<Rigid>& vec3d)
{
    auto points = PoseArrayMsg();

    for (unsigned i = 0; i < vec3d.size(); i++)
    {
        points.poses.push_back(MessageWrapper<Rigid, PoseMsg>::toROS(vec3d[i]));
    }

    return points;
}

}  // namespace ros2
}  // namespace sofa