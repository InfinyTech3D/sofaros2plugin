#include <sofa/core/ObjectFactory.h>

#include <ros2plugin/publishers/ROS2Publisher.inl>

namespace sofa {
namespace ros2 {

/** Define template names for every specialization */
template <>
std::string ROS2Publisher<double, Float64Msg>::templateName(const ROS2Publisher<double, Float64Msg> *)
{
    return "Float64";
}
template class ROS2Publisher<double, Float64Msg>;

template <>
std::string ROS2Publisher<DoubleArray, Float64ArrayMsg>::templateName(const ROS2Publisher<DoubleArray, Float64ArrayMsg> *)
{
    return "Float64Array";
}
template class ROS2Publisher<DoubleArray, Float64ArrayMsg>;

template <>
std::string ROS2Publisher<Vec3d, PointMsg>::templateName(const ROS2Publisher<Vec3d, PointMsg> *)
{
    return "Vec3d";
}
template class ROS2Publisher<Vec3d, PointMsg>;

template <>
std::string ROS2Publisher<Rigid, PoseMsg>::templateName(const ROS2Publisher<Rigid, PoseMsg> *)
{
    return "Rigid";
}
template class ROS2Publisher<Rigid, PoseMsg>;

template <>
std::string ROS2Publisher<Rigid, PoseStampedMsg>::templateName(const ROS2Publisher<Rigid, PoseStampedMsg> *)
{
    return "PoseStamped";
}
template class ROS2Publisher<Rigid, PoseStampedMsg>;

template <>
std::string ROS2Publisher<DoubleArray, JointStateMsg>::templateName(const ROS2Publisher<DoubleArray, JointStateMsg> *)
{
    return "JointState";
}
template class ROS2Publisher<DoubleArray, JointStateMsg>;

template <>
std::string ROS2Publisher<helper::vector<Vec3d>, PoseArrayMsg>::templateName(const ROS2Publisher<helper::vector<Vec3d>, PoseArrayMsg> *)
{
    return "PointArray";
}
template class ROS2Publisher<helper::vector<Vec3d>, PoseArrayMsg>;

template <>
std::string ROS2Publisher<helper::vector<Rigid>, PoseArrayMsg>::templateName(const ROS2Publisher<helper::vector<Rigid>, PoseArrayMsg> *)
{
    return "PoseArray";
}
template class ROS2Publisher<helper::vector<Rigid>, PoseArrayMsg>;

static int ROS2PublisherClass = sofa::core::RegisterObject("")
                                    .add<ROS2Publisher<double, Float64Msg>>()
                                    .add<ROS2Publisher<DoubleArray, Float64ArrayMsg>>()
                                    .add<ROS2Publisher<Vec3d, PointMsg>>()
                                    .add<ROS2Publisher<Rigid, PoseMsg>>()
                                    .add<ROS2Publisher<Rigid, PoseStampedMsg>>()
                                    .add<ROS2Publisher<DoubleArray, JointStateMsg>>()
                                    .add<ROS2Publisher<helper::vector<Vec3d>, PoseArrayMsg>>()
                                    .add<ROS2Publisher<helper::vector<Rigid>, PoseArrayMsg>>();

}  // namespace ros2
}  // namespace sofa