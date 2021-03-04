#include <sofa/core/ObjectFactory.h>

#include <ROS2Plugin/Publishers/ROS2Publisher.inl>

namespace sofa {
namespace ros2 {

/** Define template names for every specialization */
template <>
std::string ROS2Publisher<Vec3d, PointMsg>::templateName(const ROS2Publisher<Vec3d, PointMsg> *)
{
    return "Vec3d";
}

template <>
std::string ROS2Publisher<Rigid, PoseMsg>::templateName(const ROS2Publisher<Rigid, PoseMsg> *)
{
    return "Rigid3d";
}

// This will force the compiler to compile the class with some template type
template class ROS2Publisher<Vec3d, PointMsg>;
template class ROS2Publisher<Rigid, PoseMsg>;

// clang-format off
static int ROS2PublisherClass = sofa::core::RegisterObject("")
    .add<ROS2Publisher<Vec3d, PointMsg>>()
    .add<ROS2Publisher<Rigid, PoseMsg>>();
// clang-format on

}  // namespace ros2
}  // namespace sofa