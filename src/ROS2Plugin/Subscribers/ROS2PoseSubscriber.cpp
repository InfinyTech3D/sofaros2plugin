#include <sofa/core/ObjectFactory.h>

#include <ROS2Plugin/Subscribers/ROS2PoseSubscriber.h>

namespace sofa {
namespace ros2 {

int ROS2PoseSubscriberClass = core::RegisterObject("").add<ROS2PoseSubscriber>();

}
}  // namespace sofa
