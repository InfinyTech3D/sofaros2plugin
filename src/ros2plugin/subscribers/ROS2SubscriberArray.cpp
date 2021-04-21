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
std::string ROS2Subscriber<PointArray, PoseArrayMsg>::templateName(const ROS2Subscriber<PointArray, PoseArrayMsg> *)
{
    return "RosPointArray";
}
template class ROS2Subscriber<PointArray, PoseArrayMsg>;

template <>
std::string ROS2SubscriberArray<PoseArray, PoseArrayMsg>::templateName(const ROS2SubscriberArray<PoseArray, PoseArrayMsg> *)
{
    return "RosPoseArray";
}
template class ROS2SubscriberArray<PoseArray, PoseArrayMsg>;

template <>
std::string ROS2SubscriberArray<PointArray, TrackerArrayMsg>::templateName(const ROS2SubscriberArray<PointArray, TrackerArrayMsg> *)
{
    return "RosTrackerArray";
}
template class ROS2SubscriberArray<PointArray, TrackerArrayMsg>;

static int ROS2SubscriberArrayClass = sofa::core::RegisterObject("")
                                          .add<ROS2SubscriberArray<DoubleArray, Float64ArrayMsg>>()
                                          .add<ROS2Subscriber<PointArray, PoseArrayMsg>>()
                                          .add<ROS2SubscriberArray<PoseArray, PoseArrayMsg>>()
                                          .add<ROS2SubscriberArray<PointArray, TrackerArrayMsg>>();

}  // namespace ros2
}  // namespace sofa