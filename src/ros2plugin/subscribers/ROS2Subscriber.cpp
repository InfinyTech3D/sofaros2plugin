#include <sofa/core/ObjectFactory.h>

#include <ros2plugin/subscribers/ROS2Subscriber.inl>

namespace sofa {
namespace ros2 {

/** Define template names for every specialization */
template <>
std::string ROS2Subscriber<double, Float64Msg>::templateName(const ROS2Subscriber<double, Float64Msg> *)
{
    return "Float64";
}
template class ROS2Subscriber<double, Float64Msg>;

template <>
std::string ROS2Subscriber<DoubleArray, Float64ArrayMsg>::templateName(const ROS2Subscriber<DoubleArray, Float64ArrayMsg> *)
{
    return "Float64Array";
}
template class ROS2Subscriber<DoubleArray, Float64ArrayMsg>;

template <>
std::string ROS2Subscriber<Vec3d, PointMsg>::templateName(const ROS2Subscriber<Vec3d, PointMsg> *)
{
    return "Vec3d";
}
template class ROS2Subscriber<Vec3d, PointMsg>;

template <>
std::string ROS2Subscriber<Rigid, PoseMsg>::templateName(const ROS2Subscriber<Rigid, PoseMsg> *)
{
    return "Rigid3d";
}
template class ROS2Subscriber<Rigid, PoseMsg>;

template <>
std::string ROS2Subscriber<DoubleArray, JointStateMsg>::templateName(const ROS2Subscriber<DoubleArray, JointStateMsg> *)
{
    return "JointState";
}
template class ROS2Subscriber<DoubleArray, JointStateMsg>;

static int ROS2SubscriberClass = sofa::core::RegisterObject("")
                                     .add<ROS2Subscriber<double, Float64Msg>>()
                                     .add<ROS2Subscriber<DoubleArray, Float64ArrayMsg>>()
                                     .add<ROS2Subscriber<Vec3d, PointMsg>>()
                                     .add<ROS2Subscriber<Rigid, PoseMsg>>()
                                     .add<ROS2Subscriber<DoubleArray, JointStateMsg>>();

}  // namespace ros2
}  // namespace sofa