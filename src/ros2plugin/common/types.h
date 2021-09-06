#pragma once

#include <optitrack_msgs/msg/tracker_array.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/opencvplugin/BaseOpenCVData.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <optitrack_msgs/msg/tracker_array.hpp>
#include <optitrack_msgs/msg/sphere_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <optitrack_msgs/msg/rigid_array.hpp>
#include <sofa/opencvplugin/BaseOpenCVData.h>

namespace sofa
{
namespace ros2
{
/** SOFA Types */
using Vec3d = defaulttype::Vec3Types::Coord;
using Vec6d = type::Vec6d;
using Quat = type::Quatd;
using Rigid = sofa::defaulttype::Rigid3dTypes::Coord;
using Transform = sofa::defaulttype::SolidTypes<double>::Transform;
typedef sofa::type::RGBAColor RGBAColor;

using DoubleArray = sofa::type::vector<double>;
using DoubleArrayArray = sofa::type::vector<sofa::type::vector<double> >;
using PointArray = sofa::type::vector<Vec3d>;
using PoseArray = sofa::type::vector<Rigid>;
using CameraInfo = opencvplugin::ProjectionMatrixData;
using SofaTwist = type::Vec6d;
using SofaSphere = type::Vec4d;

using SofaImage = sofa::opencvplugin::ImageData;


/** ROS2 Types */
using Float64Msg = std_msgs::msg::Float64;
using PointMsg = geometry_msgs::msg::Point;
using QuatMsg = geometry_msgs::msg::Quaternion;
using PoseMsg = geometry_msgs::msg::Pose;
using TransformMsg = geometry_msgs::msg::Transform;
using TransformStampedMsg = geometry_msgs::msg::TransformStamped;
using WrenchMsg = geometry_msgs::msg::WrenchStamped;

using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
using GenericImageMsg = sensor_msgs::msg::Image;
using ImageMsg = sensor_msgs::msg::CompressedImage;

using JointStateMsg = sensor_msgs::msg::JointState;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using Float64ArrayMsg = std_msgs::msg::Float64MultiArray;
using PoseArrayMsg = geometry_msgs::msg::PoseArray;
using TrackerArrayMsg = optitrack_msgs::msg::TrackerArray;
using RigidArrayMsg = optitrack_msgs::msg::RigidArray;
using TwistMsg = geometry_msgs::msg::TwistStamped;
using SphereArrayMsg = optitrack_msgs::msg::SphereArray;

}  // namespace ros2

}  // namespace sofa
