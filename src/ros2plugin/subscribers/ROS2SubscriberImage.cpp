#include <sofa/core/ObjectFactory.h>

#include <ros2plugin/subscribers/ROS2SubscriberImage.inl>

namespace sofa
{
namespace ros2
{
static int ROS2SubscriberImageClass = sofa::core::RegisterObject("").add < ROS2SubscriberImage >();

}  // namespace ros2
}  // namespace sofa