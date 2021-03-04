#pragma once
#include <ROS2Plugin/ROS2Context.h>
#include <ROS2Plugin/Subscribers/ROS2SubscriberNode.h>
#include <ROS2Plugin/toolbox.h>
#include <ROS2Plugin/types.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>

namespace sofa {
namespace ros2 {

/**
 * Base class for ROS2Plugin subscribers.
 * When inheriting from this component, the Subscriber Node insertion into the ROS2 context is managed automatically.
 * Only non-templated child classes have been tested for the moment.
 *
 * TODO: an automatic SOFA type deduction would benefit scene readability and prevent user-level bugs.
 *
 * Troubleshoot:
 *      - toSofa() message conversions should be implemented for any new message
 *      - draw() needs to be defined for the corresponding SofaType
 *
 * @tparam DataTypes SOFA data type should be informed by the child class implementation
 * @tparam ROS2_MSG ROS message type should be informed by the child class implementation
 */
template <class DataTypes, class ROS2_MSG>
class ROS2Subscriber : public core::objectmodel::BaseObject {
   public:
    using Inherit  = core::objectmodel::BaseObject;
    SOFA_CLASS(SOFA_TEMPLATE2(ROS2Subscriber, DataTypes, ROS2_MSG), Inherit);

    core::objectmodel::SingleLink<ROS2Subscriber<DataTypes, ROS2_MSG>, ROS2Context, BaseLink::FLAG_STRONGLINK | BaseLink::FLAG_STOREPATH> l_ros2Context;

    sofa::Data<DataTypes> d_output;
    sofa::Data<std::string> d_NodeName;
    sofa::Data<std::string> d_TopicName;
    Data<double> d_drawScale;
    Data<bool> d_draw;

    std::shared_ptr<ROS2SubscriberNode<ROS2_MSG>> m_ros2node;

    explicit ROS2Subscriber();
    virtual ~ROS2Subscriber() = default;

    virtual void init() override;
    virtual void handleEvent(sofa::core::objectmodel::Event* event) override;
    virtual void draw(const sofa::core::visual::VisualParams* vparams) override;

    [[nodiscard]] std::string getTemplateName() const override { return templateName(this); }
    static std::string templateName(const ROS2Subscriber<DataTypes, ROS2_MSG>* = nullptr) { return "Unknown"; }
    static bool canCreate(ROS2Subscriber<DataTypes, ROS2_MSG>* o, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg);
};

}  // namespace ros2
}  // end namespace sofa
