#include <sofa/core/ObjectFactory.h>

#include <ROS2Plugin/Publishers/ROS2PointPublisher.inl>

namespace sofa {
namespace ros2 {

int ROS2PointPublisherClass = core::RegisterObject("").add<ROS2PointPublisher>();

}
}  // namespace sofa
