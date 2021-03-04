#pragma once
#include <ROS2Plugin/Publishers/ROS2PublisherNode.h>
#include <ROS2Plugin/ROS2Context.h>
#include <ROS2Plugin/toolbox.h>
#include <ROS2Plugin/types.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataCallback.h>

namespace sofa {
namespace ros2 {
/**
 * Base class for ROS2Plugin publishers.
 * When inheriting from this component, the Publisher Node insertion into the ROS2 context is managed automatically.
 * Only non-templated child classes have been tested for the moment.
 *
 * TODO: an automatic SOFA type deduction would benefit scene readability and prevent user-level bugs.
 *
 * Troubleshoot:
 *      - toROS() message conversions should be implemented for any new message types
 *      - draw() needs to be defined for the corresponding SofaType
 *
 * @tparam DataTypes SOFA data type should be informed by the child class implementation
 * @tparam ROS2_MSG ROS message type should be informed by the child class implementation
 */
template <class DataTypes, class ROS2_MSG>
class ROS2Publisher : public core::objectmodel::BaseObject {
   public:
    using Inherit  = core::objectmodel::BaseObject;
    SOFA_CLASS(SOFA_TEMPLATE2(ROS2Publisher, DataTypes, ROS2_MSG), Inherit);

    core::objectmodel::SingleLink<ROS2Publisher<DataTypes, ROS2_MSG>, ROS2Context, BaseLink::FLAG_STRONGLINK | BaseLink::FLAG_STOREPATH> l_ros2Context;

    sofa::Data<DataTypes> d_input;
    core::objectmodel::DataCallback c_callback;
    sofa::Data<std::string> d_NodeName;
    sofa::Data<std::string> d_TopicName;
    Data<double> d_drawScale;
    Data<bool> d_draw;

    std::shared_ptr<ROS2PublisherNode<ROS2_MSG>> m_ros2node;

    explicit ROS2Publisher();

    virtual ~ROS2Publisher() = default;

    virtual void init() override;

    virtual void update();

    virtual void draw(const sofa::core::visual::VisualParams* vparams) override;

    [[nodiscard]] std::string getTemplateName() const override { return templateName(this); }
    static std::string templateName(const ROS2Publisher<DataTypes, ROS2_MSG>* = nullptr) { return "Unknown"; }
    static bool canCreate(ROS2Publisher<DataTypes, ROS2_MSG>* o, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg);
};

}  // namespace ros2
}  // end namespace sofa
