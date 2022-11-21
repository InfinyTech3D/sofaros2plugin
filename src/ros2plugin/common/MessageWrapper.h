#pragma once
//#include <cv_bridge/cv_bridge.h>
#include <ros2plugin/common/types.h>
#include <sofa/core/visual/VisualParams.h>
#include <sstream>

namespace sofa
{
namespace ros2
{
template <class DataTypes, class ROS2_MSG>
class MessageWrapper
{
public:
    static inline void draw(const sofa::core::visual::VisualParams* /*vparams*/, const DataTypes& /*sofa_type*/) {}
    static inline DataTypes toSofa(const ROS2_MSG& /*ros_msg*/)
    {
        msg_info("ROS2Plugin") << "in function toSofa: Types informed at template are unknown";
        return DataTypes();
    }
    static inline ROS2_MSG toROS(const DataTypes& /*sofa_type*/)
    {
        msg_info("ROS2Plugin") << "in function toROS: Types informed at template are unknown";
        return ROS2_MSG();
    }
};

/************************************************************/
/************************** ATOMIC **************************/
/************************************************************
 *      SOFA         <===>          ROS2
 *     double                    DoubleMsg
 */
template <>
inline double MessageWrapper<double, DoubleMsg>::toSofa(const DoubleMsg& ros_msg)
{
    return ros_msg.data ;
}
template <>
inline DoubleMsg MessageWrapper<double, DoubleMsg>::toROS(const double& sofa_type)
{
    auto ros_msg = DoubleMsg();
    ros_msg.data = sofa_type ;
    return ros_msg;
}

/************************************************************
 *      SOFA         <===>          ROS2
 *     int                    IntMsg
 */
template <>
inline int MessageWrapper<int, IntMsg>::toSofa(const IntMsg & ros_msg)
{
    return ros_msg.data ;
}
template <>
inline IntMsg MessageWrapper<int, IntMsg>::toROS(const int & sofa_type)
{
    auto ros_msg = IntMsg();
    ros_msg.data = sofa_type ;
    return ros_msg;
}

/************************************************************
 *      SOFA         <===>          ROS2
 *     int                    UnsignedMsg
 */
template <>
inline unsigned MessageWrapper<unsigned, UnsignedMsg>::toSofa(const UnsignedMsg & ros_msg)
{
    return ros_msg.data ;
}
template <>
inline UnsignedMsg MessageWrapper<unsigned, UnsignedMsg>::toROS(const unsigned & sofa_type)
{
    auto ros_msg = UnsignedMsg();
    ros_msg.data = sofa_type ;
    return ros_msg;
}


/************************************************************
 *      SOFA         <===>          ROS2
 *      Vec3d                      PointMsg
 */
template <>
inline Vec3d MessageWrapper<Vec3d, Vec3dMsg>::toSofa(const Vec3dMsg& point)
{

	return Vec3d(point.x, point.y, point.z);
}
template <>
inline Vec3dMsg MessageWrapper<Vec3d, Vec3dMsg>::toROS(const Vec3d& vec3d)
{
	auto point = Vec3dMsg();

	point.x = vec3d[0];
	point.y = vec3d[1];
	point.z = vec3d[2];


	return point;
}


/************************************************************
 *      SOFA         <===>          ROS2
 *      Rigid                      PoseMsg
 */

template <>
inline Rigid MessageWrapper<Rigid, RigidMsg>::toSofa(const RigidMsg& pose)
{
	return Rigid(MessageWrapper<Vec3d, Vec3dMsg>::toSofa(pose.position), Quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
}
template <>
inline RigidMsg MessageWrapper<Rigid, RigidMsg>::toROS(const Rigid& rigid)
{
	auto pose = RigidMsg();
	auto quat = QuatMsg();
	quat.x = rigid.getOrientation()[0];
	quat.y = rigid.getOrientation()[1];
	quat.z = rigid.getOrientation()[2];
	quat.w = rigid.getOrientation()[3];

	pose.position = MessageWrapper<Vec3d, Vec3dMsg>::toROS(rigid.getCenter());
	pose.orientation = quat;
	return pose;
}

/************************************************************
 *      SOFA         <===>          ROS2
 *      String                      StringMsg
 */

template <>
inline String MessageWrapper<String, StringMsg>::toSofa(const StringMsg& ros_msg)
{
	return ros_msg.data;
}
template <>
inline StringMsg MessageWrapper<String, StringMsg>::toROS(const String& sofa_type)
{
	auto msg = StringMsg();
	msg.set__data(sofa_type);
	return msg;
}

/************************************************************
 *      SOFA         <===>          ROS2
 *      Byte                      ByteMsg
 */

template <>
inline Byte MessageWrapper<Byte, ByteMsg>::toSofa(const ByteMsg& ros_msg)
{
	return ros_msg.data;
}
template <>
inline ByteMsg MessageWrapper<Byte, ByteMsg>::toROS(const Byte& sofa_type)
{
	auto msg = ByteMsg();
	msg.set__data(sofa_type);
	return msg;
}

/************************************************************/
/****************************** ARRAY ***********************/
/************************************************************
 *      SOFA         <===>          ROS2
 *     DoubleArray               Float64ArrayMsg
 */
template <>
inline DoubleArray MessageWrapper<DoubleArray, DoubleArrayMsg>::toSofa(const DoubleArrayMsg& ros_msg)
{
    DoubleArray temp(ros_msg.data.begin(), ros_msg.data.end());
    return temp;
}
template <>
inline DoubleArrayMsg MessageWrapper<DoubleArray, DoubleArrayMsg>::toROS(const DoubleArray& sofa_type)
{
    auto ros_msg = DoubleArrayMsg();
    ros_msg.data = sofa_type;
    return ros_msg;
}

/************************************************************
 *      SOFA         <===>          ROS2
 *     DoubleArray               Float64ArrayMsg
 */
template <>
inline IntArray MessageWrapper<IntArray , IntArrayMsg>::toSofa(const IntArrayMsg& ros_msg)
{
	IntArray temp(ros_msg.data.begin(), ros_msg.data.end());
    return temp;
}
template <>
inline IntArrayMsg MessageWrapper<IntArray, IntArrayMsg>::toROS(const IntArray& sofa_type)
{
    auto ros_msg = IntArrayMsg();
    ros_msg.data = sofa_type;
    return ros_msg;
}


/************************************************************
 *      SOFA         <===>          ROS2
 *     DoubleArray               Float64ArrayMsg
 */
template <>
inline UnsignedArray MessageWrapper<UnsignedArray , UnsignedArrayMsg>::toSofa(const UnsignedArrayMsg& ros_msg)
{
	UnsignedArray temp(ros_msg.data.begin(), ros_msg.data.end());
    return temp;
}
template <>
inline UnsignedArrayMsg MessageWrapper<UnsignedArray, UnsignedArrayMsg>::toROS(const UnsignedArray& sofa_type)
{
    auto ros_msg = UnsignedArrayMsg();
    ros_msg.data = sofa_type;
    return ros_msg;
}





/************************************************************
 *      SOFA         <===>          ROS2
 *      Vec3dArray       Vec3dArrayMsg
 */
template <>
inline Vec3dArray MessageWrapper<Vec3dArray, Vec3dArrayMsg>::toSofa(const Vec3dArrayMsg& points)
{
    sofa::type::vector<Vec3d> returnVec;

    for (unsigned i = 0; i < points.points.size(); i++)
        returnVec.push_back(Vec3d(points.points[i].x, points.points[i].y, points.points[i].z) );

    return returnVec;
}
template <>
inline Vec3dArrayMsg MessageWrapper<Vec3dArray, Vec3dArrayMsg>::toROS(const Vec3dArray& vec3d)
{
    auto points = Vec3dArrayMsg();

    for (unsigned i = 0; i < vec3d.size(); i++)
    {
		Vec3dArrayMsg::_points_type::value_type P;
        P.set__x(vec3d[i][0] ).set__y(vec3d[i][1] ).set__z(vec3d[i][2] );
        points.points.push_back(P);
    }

    return points;
}

/************************************************************
 *      SOFA         <===>          ROS2
 *      RigidArray                  RigidArrayMsg
 */
template <>
inline RigidArray MessageWrapper<RigidArray, RigidArrayMsg>::toSofa(const RigidArrayMsg& msg)
{
	RigidArray returnVec;

    for (unsigned i = 0; i < msg.poses.size(); i++) returnVec.push_back(MessageWrapper<Rigid, RigidMsg>::toSofa(msg.poses[i]));

    return returnVec;
}
template <>
inline RigidArrayMsg MessageWrapper<RigidArray, RigidArrayMsg>::toROS(const RigidArray& vec3d)
{
    auto points = RigidArrayMsg();

    for (unsigned i = 0; i < vec3d.size(); i++)
    {
        points.poses.push_back(MessageWrapper<Rigid, RigidMsg>::toROS(vec3d[i]));
    }

    return points;
}

/************************************************************
 *      SOFA         <===>          ROS2
 *      ByteArray                  ByteArrayMsg
 */
template <>
inline ByteArray MessageWrapper<ByteArray, ByteArrayMsg>::toSofa(const ByteArrayMsg& msg)
{
	ByteArray returnVec;

    for (unsigned i = 0; i < msg.data.size(); i++) returnVec.push_back(msg.data[i]);

    return returnVec;
}
template <>
inline ByteArrayMsg MessageWrapper<ByteArray, ByteArrayMsg>::toROS(const ByteArray& byteArray)
{
    auto points = ByteArrayMsg();

    for (unsigned i = 0; i < byteArray.size(); i++)
    {
        points.data.push_back(byteArray[i]);
    }

    return points;
}

}  // namespace ros2

}  // namespace sofa
