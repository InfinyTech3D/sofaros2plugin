#include <ROS2Plugin/DummyRigid.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa {
namespace ros2 {

int DummyRigidClass = core::RegisterObject("Random forces applied to all points").add<DummyRigid>();

}
}  // namespace sofa
