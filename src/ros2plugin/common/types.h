#pragma once

#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/defaulttype/VecTypes.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace sofa {

namespace ros2 {

/** SOFA Types */
using DoubleArray = helper::vector<double>;

using Vec3d     = defaulttype::Vec3Types::Coord;
using Quat      = defaulttype::Quat;
using Rigid     = sofa::defaulttype::Rigid3dTypes::Coord;
using Transform = sofa::defaulttype::SolidTypes<double>::Transform;

typedef sofa::helper::types::RGBAColor RGBAColor;

/** ROS2 Types */
using Float64Msg = std_msgs::msg::Float64;
using PointMsg   = geometry_msgs::msg::Point;
using QuatMsg    = geometry_msgs::msg::Quaternion;
using PoseMsg    = geometry_msgs::msg::Pose;

using Float64ArrayMsg = std_msgs::msg::Float64MultiArray;

using JointStateMsg = sensor_msgs::msg::JointState;

}  // namespace ros2

}  // namespace sofa
