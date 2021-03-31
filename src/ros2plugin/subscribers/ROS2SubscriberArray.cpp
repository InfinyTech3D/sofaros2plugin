#include <sofa/core/ObjectFactory.h>

#include <ros2plugin/subscribers/ROS2SubscriberArray.inl>
#include <ros2plugin/subscribers/ROS2Subscriber.inl>

namespace sofa {
namespace ros2 {

/** Define template names for every specialization */
template <>
std::string ROS2SubscriberArray<helper::vector<Rigid>, PoseArrayMsg>::templateName(const ROS2SubscriberArray<helper::vector<Rigid>, PoseArrayMsg> *)
{
return "PoseArray";
}
template class ROS2SubscriberArray<helper::vector<Rigid>, PoseArrayMsg>;

template <>
std::string ROS2SubscriberArray<helper::vector<Vec3d>, TrackerArrayMsg>::templateName(const ROS2SubscriberArray<helper::vector<Vec3d>, TrackerArrayMsg> *)
{
    return "TrackerArray";
}
template class ROS2SubscriberArray<helper::vector<Vec3d>, TrackerArrayMsg>;


static int ROS2SubscriberArrayClass = sofa::core::RegisterObject("")
                                     .add<ROS2SubscriberArray<helper::vector<Rigid>, PoseArrayMsg>>()
                                     .add<ROS2SubscriberArray<helper::vector<Vec3d>, TrackerArrayMsg>>();


}  // namespace ros2
}  // namespace sofa