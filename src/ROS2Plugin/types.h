#pragma once

#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/defaulttype/VecTypes.h>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sofa {

namespace ros2 {

/** SOFA Types */
typedef defaulttype::Vec3Types::Coord                    Vec3d;
typedef defaulttype::Quat                                Quat;
typedef sofa::defaulttype::Rigid3dTypes::Coord           Rigid;
typedef sofa::defaulttype::SolidTypes<double>::Transform Transform;

typedef sofa::helper::types::RGBAColor RGBAColor;

/** ROS2 Types */
typedef geometry_msgs::msg::Point      PointMsg;
typedef geometry_msgs::msg::Quaternion QuatMsg;
typedef geometry_msgs::msg::Pose       PoseMsg;

}  // namespace ros2

}  // namespace sofa
