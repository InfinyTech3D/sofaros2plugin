#pragma once

#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/SolidTypes.h>

namespace sofa {

namespace ros2 {

typedef sofa::defaulttype::Vec3d                         Vec3d;
typedef defaulttype::Quat                                Quat;
typedef sofa::defaulttype::Rigid3dTypes::Coord           Rigid;
typedef sofa::defaulttype::SolidTypes<double>::Transform Transform;

typedef sofa::helper::types::RGBAColor RGBAColor;

}  // namespace ros2

}  // namespace sofa
