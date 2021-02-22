#include <ROS2Plugin/DummyVec3d.inl>

namespace sofa {
namespace ros2 {

int DummyVec3dClass = core::RegisterObject("Random forces applied to all points").add<DummyVec3d>();

}
}  // namespace sofa
