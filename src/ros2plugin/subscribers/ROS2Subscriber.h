#pragma once
#include <ros2plugin/common/ROS2BaseObject.h>
#include <ros2plugin/common/ROS2Context.h>
#include <ros2plugin/common/MessageWrapper.h>
#include <ros2plugin/common/types.h>
#include <ros2plugin/subscribers/ROS2SubscriberNode.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>

namespace sofa {
namespace ros2 {

/**
 * Base template class for ros2plugin subscribers.
 * In order to add a new message type, be sure the functions toSofa(ROS2MSG) with the corresponding ROS2MSG is defined.
 * Then you may add new template specializations to the ROS2Subscriber.cpp file.
 *
 * Troubleshoot:
 *      - init() method should call base class createNode method
 *      - toSofa() message conversions should be implemented for any new message
 *      - draw() needs to be defined for the corresponding SofaType
 *
 * @tparam DataTypes SOFA data type should be informed by the child class implementation
 * @tparam ROS2_MSG ROS message type should be informed by the child class implementation
 */
template <class DataTypes, class ROS2_MSG>
class ROS2Subscriber : public ROS2BaseObject<DataTypes, ROS2_MSG> {
   public:
    SOFA_CLASS(SOFA_TEMPLATE2(ROS2Subscriber, DataTypes, ROS2_MSG), SOFA_TEMPLATE2(ROS2BaseObject, DataTypes, ROS2_MSG));

    sofa::Data<DataTypes> d_output;
    Data<double> d_drawScale;
    Data<bool> d_draw;

    explicit ROS2Subscriber();
    virtual ~ROS2Subscriber() = default;

    std::shared_ptr<ROS2SubscriberNode<ROS2_MSG>> m_ros2node;

    virtual void init() override;
    virtual void handleEvent(sofa::core::objectmodel::Event* event) override;
    virtual void draw(const sofa::core::visual::VisualParams* vparams) override;

    static std::string templateName(const ROS2Subscriber<DataTypes, ROS2_MSG>* = nullptr) { return "Unknown"; }
};

}  // namespace ros2
}  // end namespace sofa
