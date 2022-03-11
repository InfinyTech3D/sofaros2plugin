#include <sofa/core/ObjectFactory.h>

#include <ros2plugin/subscribers/ROS2Subscriber.inl>
#include <ros2plugin/subscribers/ROS2SubscriberArray.inl>

namespace sofa
{
namespace ros2
{
/** Define template names for every specialization */
template <>
std::string ROS2SubscriberArray<DoubleArray, Float64ArrayMsg>::templateName(const ROS2SubscriberArray<DoubleArray, Float64ArrayMsg> *)
{
    return "RosFloat64Array";
}
template class ROS2SubscriberArray<DoubleArray, Float64ArrayMsg>;


template <>
std::string ROS2SubscriberArray<PoseArray, PoseArrayMsg>::templateName(const ROS2SubscriberArray<PoseArray, PoseArrayMsg> *)
{
    return "RosPoseArray";
}
template class ROS2SubscriberArray<PoseArray, PoseArrayMsg>;

//template <>
//std::string ROS2SubscriberArray<PointArray, TrackerArrayMsg>::templateName(const ROS2SubscriberArray<PointArray, TrackerArrayMsg> *)
//{
//    return "RosTrackerArray";
//}
//template class ROS2SubscriberArray<PointArray, TrackerArrayMsg>;

//template <>
//std::string ROS2SubscriberArray<PoseArray, RigidArrayMsg>::templateName(const ROS2SubscriberArray<PoseArray, RigidArrayMsg> *)
//{
//    return "RosRigidArray";
//}
//template class ROS2SubscriberArray<PoseArray, RigidArrayMsg>;

template <>
std::string ROS2SubscriberArray<DoubleArrayArray, JointStateMsg>::templateName(const ROS2SubscriberArray<DoubleArrayArray, JointStateMsg> *)
{
    return "RosJointState";
}
template class ROS2SubscriberArray<DoubleArrayArray, JointStateMsg>;


static int ROS2SubscriberArrayClass = sofa::core::RegisterObject("")
                                          .add<ROS2SubscriberArray<DoubleArray, Float64ArrayMsg>>()
                                          .add<ROS2SubscriberArray<PoseArray, PoseArrayMsg>>()
//                                          .add<ROS2SubscriberArray<PointArray, TrackerArrayMsg>>()
                                          .add<ROS2SubscriberArray<DoubleArrayArray, JointStateMsg>>();
//                                          .add<ROS2SubscriberArray<PoseArray, RigidArrayMsg>>();

}  // namespace ros2
}  // namespace sofa
