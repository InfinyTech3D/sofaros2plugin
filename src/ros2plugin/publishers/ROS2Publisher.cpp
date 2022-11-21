#include <sofa/core/ObjectFactory.h>

#include <ros2plugin/publishers/ROS2Publisher.inl>

namespace sofa
{
	namespace ros2
	{


/*******ATOMIC******/

/** Define template names for every specialization */
template <>
std::string ROS2Publisher<double, DoubleMsg>::templateName(const ROS2Publisher<double, DoubleMsg> *)
{
	return "RosDouble";
}
template class ROS2Publisher<double, DoubleMsg>;


/** Define template names for every specialization */
template <>
std::string ROS2Publisher<int, IntMsg>::templateName(const ROS2Publisher<int, IntMsg> *)
{
	return "RosInt";
}
template class ROS2Publisher<int , IntMsg>;

/** Define template names for every specialization */
template <>
std::string ROS2Publisher<unsigned, UnsignedMsg>::templateName(const ROS2Publisher<unsigned, UnsignedMsg> *)
{
	return "RosUnsigned";
}
template class ROS2Publisher<unsigned, UnsignedMsg>;


/** Define template names for every specialization */
template <>
std::string ROS2Publisher<Vec3d, Vec3dMsg>::templateName(const ROS2Publisher<Vec3d, Vec3dMsg> *)
{
	return "RosVec3d";
}
template class ROS2Publisher<Vec3d , Vec3dMsg>;

/** Define template names for every specialization */
template <>
std::string ROS2Publisher<Rigid, RigidMsg>::templateName(const ROS2Publisher<Rigid, RigidMsg> *)
{
	return "RosRigid";
}
template class ROS2Publisher<Rigid, RigidMsg>;

/** Define template names for every specialization */
template <>
std::string ROS2Publisher<String, StringMsg>::templateName(const ROS2Publisher<String, StringMsg> *)
{
	return "RosString";
}
template class ROS2Publisher<String, StringMsg>;

/** Define template names for every specialization */
template <>
std::string ROS2Publisher<Byte, ByteMsg>::templateName(const ROS2Publisher<Byte, ByteMsg> *)
{
	return "RosByte";
}
template class ROS2Publisher<Byte, ByteMsg>;




/*******ARRAY******/

/** Define template names for every specialization */
template <>
std::string ROS2Publisher<DoubleArray , DoubleArrayMsg>::templateName(const ROS2Publisher<DoubleArray, DoubleArrayMsg> *)
{
	return "RosDoubleArray";
}
template class ROS2Publisher<DoubleArray, DoubleArrayMsg>;


/** Define template names for every specialization */
template <>
std::string ROS2Publisher<IntArray, IntArrayMsg>::templateName(const ROS2Publisher<IntArray, IntArrayMsg> *)
{
	return "RosIntArray";
}
template class ROS2Publisher<IntArray , IntArrayMsg>;

/** Define template names for every specialization */
template <>
std::string ROS2Publisher<UnsignedArray , UnsignedArrayMsg>::templateName(const ROS2Publisher<UnsignedArray, UnsignedArrayMsg> *)
{
	return "RosUnsignedArray";
}
template class ROS2Publisher<UnsignedArray, UnsignedArrayMsg>;


/** Define template names for every specialization */
template <>
std::string ROS2Publisher<Vec3dArray, Vec3dArrayMsg>::templateName(const ROS2Publisher<Vec3dArray, Vec3dArrayMsg> *)
{
	return "RosVec3dArray";
}
template class ROS2Publisher<Vec3dArray , Vec3dArrayMsg>;

/** Define template names for every specialization */
template <>
std::string ROS2Publisher<RigidArray, RigidArrayMsg>::templateName(const ROS2Publisher<RigidArray, RigidArrayMsg> *)
{
	return "RosRigidArray";
}

/** Define template names for every specialization */
template <>
std::string ROS2Publisher<ByteArray, ByteArrayMsg>::templateName(const ROS2Publisher<ByteArray, ByteArrayMsg> *)
{
	return "RosByteArray";
}
template class ROS2Publisher<ByteArray, ByteArrayMsg>;


		template class ROS2Publisher<RigidArray, RigidArrayMsg>;
static int ROS2PublisherClass = sofa::core::RegisterObject("")
		.add<ROS2Publisher<double, DoubleMsg>>()
		.add<ROS2Publisher<int, IntMsg>>()
		.add<ROS2Publisher<unsigned, UnsignedMsg>>()
		.add<ROS2Publisher<Vec3d, Vec3dMsg>>()
		.add<ROS2Publisher<Rigid, RigidMsg>>()
		.add<ROS2Publisher<String, StringMsg>>()
		.add<ROS2Publisher<Byte, ByteMsg>>()
		.add<ROS2Publisher<DoubleArray , DoubleArrayMsg>>()
		.add<ROS2Publisher<IntArray , IntArrayMsg>>()
		.add<ROS2Publisher<UnsignedArray , UnsignedArrayMsg>>()
		.add<ROS2Publisher<Vec3dArray , Vec3dArrayMsg>>()
		.add<ROS2Publisher<RigidArray , RigidArrayMsg>>()
		.add<ROS2Publisher<ByteArray , ByteArrayMsg>>();


}  // namespace ros2
}  // namespace sofa
