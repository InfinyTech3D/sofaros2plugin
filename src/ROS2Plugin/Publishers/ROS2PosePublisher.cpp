#include <sofa/core/ObjectFactory.h>

#include <ROS2Plugin/Publishers/ROS2PosePublisher.h>

namespace sofa {
namespace ros2 {

int ROS2PosePublisherClass = core::RegisterObject("").add<ROS2PosePublisher>();

}
}  // namespace sofa
