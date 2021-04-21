#include <ros2plugin/common/ROS2Context.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa
{
namespace ros2
{
int ROS2ContextClass = core::RegisterObject("Initializes and handles ROS2 context").add<ROS2Context>();

}
}  // namespace sofa
