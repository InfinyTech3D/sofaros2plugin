#include <ROS2Plugin/DummyVec3d.inl>

namespace sofa {
namespace component {
namespace controller {

int DummyVec3dClass = core::RegisterObject("Random forces applied to all points")
        .add<DummyVec3d>();

}
}
}
