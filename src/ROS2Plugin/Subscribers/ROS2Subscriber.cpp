#include <sofa/core/ObjectFactory.h>

#include <ROS2Plugin/Subscribers/ROS2Subscriber.inl>

namespace sofa {
namespace ros2 {

/** Define template names for every specialization */
template <>
std::string ROS2Subscriber<Vec3d, PointMsg>::templateName(const ROS2Subscriber<Vec3d, PointMsg> *)
{
    return "Vec3d";
}

template <>
std::string ROS2Subscriber<Rigid, PoseMsg>::templateName(const ROS2Subscriber<Rigid, PoseMsg> *)
{
    return "Rigid3d";
}

// This will force the compiler to compile the class with some template type
template class ROS2Subscriber<Vec3d, PointMsg>;
template class ROS2Subscriber<Rigid, PoseMsg>;

// clang-format off
static int ROS2SubscriberClass = sofa::core::RegisterObject("")
                                             .add<ROS2Subscriber<Vec3d, PointMsg>>()
                                             .add<ROS2Subscriber<Rigid, PoseMsg>>();
// clang-format on

}  // namespace ros2
}  // namespace sofa