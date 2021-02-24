#include <ROS2Plugin/Vec3dPublisher.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa {
namespace ros2 {

int Vec3dPublisherClass = core::RegisterObject("Random forces applied to all points").add<Vec3dPublisher>();

}
}  // namespace sofa
