#pragma once

#include <optitrack_msgs/msg/tracker_array.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/defaulttype/VecTypes.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <optitrack_msgs/msg/tracker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace sofa
{
namespace ros2
{
/** SOFA Types */
using Vec3d = defaulttype::Vec3Types::Coord;
using Quat = defaulttype::Quat;
using Rigid = sofa::defaulttype::Rigid3dTypes::Coord;
using Transform = sofa::defaulttype::SolidTypes<double>::Transform;
typedef sofa::helper::types::RGBAColor RGBAColor;

using DoubleArray = helper::vector<double>;
using PointArray = helper::vector<Vec3d>;
using PoseArray = helper::vector<Rigid>;


/** ROS2 Types */
using Float64Msg = std_msgs::msg::Float64;
using PointMsg = geometry_msgs::msg::Point;
using QuatMsg = geometry_msgs::msg::Quaternion;
using PoseMsg = geometry_msgs::msg::Pose;

using JointStateMsg = sensor_msgs::msg::JointState;


using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using Float64ArrayMsg = std_msgs::msg::Float64MultiArray;
using PoseArrayMsg = geometry_msgs::msg::PoseArray;
using TrackerArrayMsg = optitrack_msgs::msg::TrackerArray;

}  // namespace ros2

}  // namespace sofa
