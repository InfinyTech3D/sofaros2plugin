#include <sofa/core/ObjectFactory.h>

#include <ROS2Plugin/Subscribers/ROS2PointSubscriber.inl>

namespace sofa {
namespace ros2 {

int ROS2PointSubscriberClass = core::RegisterObject("").add<ROS2PointSubscriber>();

}
}  // namespace sofa
