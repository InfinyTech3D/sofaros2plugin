#pragma once
#include <ros2plugin/common/MessageArrayWrapper.h>
#include <ros2plugin/common/ROS2BaseObject.h>
#include <ros2plugin/common/ROS2Context.h>
#include <ros2plugin/common/types.h>
#include <ros2plugin/subscribers/ROS2Subscriber.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>

namespace sofa {
namespace ros2 {

template <class DataTypes, class ROS2_MSG>
class ROS2SubscriberArray : public ROS2Subscriber<DataTypes, ROS2_MSG> {
   public:
    SOFA_CLASS(SOFA_TEMPLATE2(ROS2SubscriberArray, DataTypes, ROS2_MSG), SOFA_TEMPLATE2(ROS2Subscriber, DataTypes, ROS2_MSG));

    Data<helper::vector<int>> d_indexes;

    explicit ROS2SubscriberArray();
    virtual ~ROS2SubscriberArray() = default;

    virtual void handleEvent(sofa::core::objectmodel::Event* event) override;
    virtual void draw(const sofa::core::visual::VisualParams* vparams) override;

    static std::string templateName(const ROS2SubscriberArray<DataTypes, ROS2_MSG>* = nullptr) { return "Unknown"; }
};

}  // namespace ros2
}  // end namespace sofa
