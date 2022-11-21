#include <sofa/core/ObjectFactory.h>

#include <ros2plugin/subscribers/ROS2Subscriber.inl>

namespace sofa
{
namespace ros2
{


/*******ATOMIC******/

/** Define template names for every specialization */
template <>
std::string ROS2Subscriber<double, DoubleMsg>::templateName(const ROS2Subscriber<double, DoubleMsg> *)
{
	return "RosDouble";
}
template class ROS2Subscriber<double, DoubleMsg>;


/** Define template names for every specialization */
template <>
std::string ROS2Subscriber<int, IntMsg>::templateName(const ROS2Subscriber<int, IntMsg> *)
{
	return "RosInt";
}
template class ROS2Subscriber<int , IntMsg>;

/** Define template names for every specialization */
template <>
std::string ROS2Subscriber<unsigned, UnsignedMsg>::templateName(const ROS2Subscriber<unsigned, UnsignedMsg> *)
{
	return "RosUnsigned";
}
template class ROS2Subscriber<unsigned, UnsignedMsg>;


/** Define template names for every specialization */
template <>
std::string ROS2Subscriber<Vec3d, Vec3dMsg>::templateName(const ROS2Subscriber<Vec3d, Vec3dMsg> *)
{
	return "RosVec3d";
}
template class ROS2Subscriber<Vec3d , Vec3dMsg>;

/** Define template names for every specialization */
template <>
std::string ROS2Subscriber<Rigid, RigidMsg>::templateName(const ROS2Subscriber<Rigid, RigidMsg> *)
{
	return "RosRigid";
}
template class ROS2Subscriber<Rigid, RigidMsg>;

/** Define template names for every specialization */
template <>
std::string ROS2Subscriber<String, StringMsg>::templateName(const ROS2Subscriber<String, StringMsg> *)
{
	return "RosString";
}
template class ROS2Subscriber<String, StringMsg>;

/** Define template names for every specialization */
template <>
std::string ROS2Subscriber<Byte, ByteMsg>::templateName(const ROS2Subscriber<Byte, ByteMsg> *)
{
	return "RosByte";
}
template class ROS2Subscriber<Byte, ByteMsg>;




/*******ARRAY******/

/** Define template names for every specialization */
template <>
std::string ROS2Subscriber<DoubleArray , DoubleArrayMsg>::templateName(const ROS2Subscriber<DoubleArray, DoubleArrayMsg> *)
{
	return "RosDoubleArray";
}
template class ROS2Subscriber<DoubleArray, DoubleArrayMsg>;


/** Define template names for every specialization */
template <>
std::string ROS2Subscriber<IntArray, IntArrayMsg>::templateName(const ROS2Subscriber<IntArray, IntArrayMsg> *)
{
	return "RosIntArray";
}
template class ROS2Subscriber<IntArray , IntArrayMsg>;

/** Define template names for every specialization */
template <>
std::string ROS2Subscriber<UnsignedArray , UnsignedArrayMsg>::templateName(const ROS2Subscriber<UnsignedArray, UnsignedArrayMsg> *)
{
	return "RosUnsignedArray";
}
template class ROS2Subscriber<UnsignedArray, UnsignedArrayMsg>;


/** Define template names for every specialization */
template <>
std::string ROS2Subscriber<Vec3dArray, Vec3dArrayMsg>::templateName(const ROS2Subscriber<Vec3dArray, Vec3dArrayMsg> *)
{
	return "RosVec3dArray";
}
template class ROS2Subscriber<Vec3dArray , Vec3dArrayMsg>;

/** Define template names for every specialization */
template <>
std::string ROS2Subscriber<RigidArray, RigidArrayMsg>::templateName(const ROS2Subscriber<RigidArray, RigidArrayMsg> *)
{
	return "RosRigidArray";
}

/** Define template names for every specialization */
template <>
std::string ROS2Subscriber<ByteArray, ByteArrayMsg>::templateName(const ROS2Subscriber<ByteArray, ByteArrayMsg> *)
{
return "RosByteArray";
}
template class ROS2Subscriber<ByteArray, ByteArrayMsg>;


template class ROS2Subscriber<RigidArray, RigidArrayMsg>;
static int ROS2SubscriberClass = sofa::core::RegisterObject("")
.add<ROS2Subscriber<double, DoubleMsg>>()
.add<ROS2Subscriber<int, IntMsg>>()
.add<ROS2Subscriber<unsigned, UnsignedMsg>>()
.add<ROS2Subscriber<Vec3d, Vec3dMsg>>()
.add<ROS2Subscriber<Rigid, RigidMsg>>()
.add<ROS2Subscriber<String, StringMsg>>()
.add<ROS2Subscriber<Byte, ByteMsg>>()
.add<ROS2Subscriber<DoubleArray , DoubleArrayMsg>>()
.add<ROS2Subscriber<IntArray , IntArrayMsg>>()
.add<ROS2Subscriber<UnsignedArray , UnsignedArrayMsg>>()
.add<ROS2Subscriber<Vec3dArray , Vec3dArrayMsg>>()
.add<ROS2Subscriber<RigidArray , RigidArrayMsg>>()
.add<ROS2Subscriber<ByteArray , ByteArrayMsg>>();


}  // namespace ros2
}  // namespace sofa
