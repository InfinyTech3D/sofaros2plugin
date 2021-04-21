#include <sofa/core/ObjectFactory.h>

#include <ros2plugin/subscribers/ROS2Subscriber.inl>

namespace sofa
{
namespace ros2
{
/** Define template names for every specialization */
template <>
std::string ROS2Subscriber<double, Float64Msg>::templateName(const ROS2Subscriber<double, Float64Msg> *)
{
    return "RosFloat64";
}
template class ROS2Subscriber<double, Float64Msg>;

template <>
std::string ROS2Subscriber<DoubleArray, Float64ArrayMsg>::templateName(const ROS2Subscriber<DoubleArray, Float64ArrayMsg> *)
{
    return "RosFloat64Array";
}
template class ROS2Subscriber<DoubleArray, Float64ArrayMsg>;

template <>
std::string ROS2Subscriber<Vec3d, PointMsg>::templateName(const ROS2Subscriber<Vec3d, PointMsg> *)
{
    return "RosVec3d";
}
template class ROS2Subscriber<Vec3d, PointMsg>;

template <>
std::string ROS2Subscriber<Rigid, PoseMsg>::templateName(const ROS2Subscriber<Rigid, PoseMsg> *)
{
    return "RosRigid";
}
template class ROS2Subscriber<Rigid, PoseMsg>;

template <>
std::string ROS2Subscriber<Rigid, PoseStampedMsg>::templateName(const ROS2Subscriber<Rigid, PoseStampedMsg> *)
{
    return "RosPoseStamped";
}
template class ROS2Subscriber<Rigid, PoseStampedMsg>;

template <>
std::string ROS2Subscriber<DoubleArray, JointStateMsg>::templateName(const ROS2Subscriber<DoubleArray, JointStateMsg> *)
{
    return "RosJointState";
}
template class ROS2Subscriber<DoubleArray, JointStateMsg>;

template <>
std::string ROS2Subscriber<helper::vector<Vec3d>, PoseArrayMsg>::templateName(const ROS2Subscriber<helper::vector<Vec3d>, PoseArrayMsg> *)
{
    return "RosPointArray";
}
template class ROS2Subscriber<helper::vector<Vec3d>, PoseArrayMsg>;

template <>
std::string ROS2Subscriber<helper::vector<Rigid>, PoseArrayMsg>::templateName(const ROS2Subscriber<helper::vector<Rigid>, PoseArrayMsg> *)
{
    return "RosPoseArray";
}
template class ROS2Subscriber<helper::vector<Rigid>, PoseArrayMsg>;

static int ROS2SubscriberClass = sofa::core::RegisterObject("")
                                     .add<ROS2Subscriber<double, Float64Msg>>()
                                     .add<ROS2Subscriber<DoubleArray, Float64ArrayMsg>>()
                                     .add<ROS2Subscriber<Vec3d, PointMsg>>()
                                     .add<ROS2Subscriber<Rigid, PoseMsg>>()
                                     .add<ROS2Subscriber<Rigid, PoseStampedMsg>>()
                                     .add<ROS2Subscriber<DoubleArray, JointStateMsg>>()
                                     .add<ROS2Subscriber<helper::vector<Vec3d>, PoseArrayMsg>>()
                                     .add<ROS2Subscriber<helper::vector<Rigid>, PoseArrayMsg>>();

}  // namespace ros2
}  // namespace sofa