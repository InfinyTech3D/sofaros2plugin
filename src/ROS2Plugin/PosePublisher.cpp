#include <ROS2Plugin/PosePublisher.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa {
namespace ros2 {

int PosePublisherClass = core::RegisterObject("Random forces applied to all points").add<PosePublisher>();

}
}  // namespace sofa
