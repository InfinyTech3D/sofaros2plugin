#pragma once

#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/type/Quat.h>


#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

namespace sofa
{
namespace ros2
{
/** SOFA Types */
using Vec3d = defaulttype::Vec3Types::Coord;
using Quat = sofa::type::Quatd;
using Rigid = sofa::defaulttype::Rigid3dTypes::Coord;


using DoubleArray = sofa::type::vector<double>;
using IntArray = sofa::type::vector<int>;
using UnsignedArray = sofa::type::vector<unsigned>;
using Vec3dArray = sofa::type::vector<Vec3d>;
using RigidArray = sofa::type::vector<Rigid>;



/** ROS2 Types */
using DoubleMsg = std_msgs::msg::Float64;
using IntMsg = std_msgs::msg::Int32;
using UnsignedMsg = std_msgs::msg::UInt32;
using Vec3dMsg = geometry_msgs::msg::Point;
using RigidMsg = geometry_msgs::msg::Pose;
using QuatMsg = geometry_msgs::msg::Quaternion;


using DoubleArrayMsg = std_msgs::msg::Float64MultiArray;
using IntArrayMsg = std_msgs::msg::Int32MultiArray;
using UnsignedArrayMsg = std_msgs::msg::UInt32MultiArray;
using Vec3dArrayMsg = sensor_msgs::msg::PointCloud;
using RigidArrayMsg = geometry_msgs::msg::PoseArray;
}  // namespace ros2

}  // namespace sofa
