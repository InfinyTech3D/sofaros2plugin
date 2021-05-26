#pragma once

#include <ros2plugin/common/MessageWrapper.h>
#include <ros2plugin/common/ROS2BaseObject.h>
#include <ros2plugin/common/ROS2Context.h>
#include <ros2plugin/common/types.h>
#include <ros2plugin/publishers/ROS2PublisherNode.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/simulation/AnimateEndEvent.h>

namespace sofa
{
namespace ros2
{
/**
 * Base template class for ros2plugin publishers.
 * In order to add a new message type, be sure the functions toROS(SOFAType) with the corresponding SOFAType is defined.
 * Then you may add new template specializations to the ROS2Publisher.cpp file.
 *
 * TODO: an automatic SOFA type deduction would benefit scene readability and prevent user-level bugs.
 *
 * Troubleshoot:
 *      - init() method should call base class createNode method
 *      - toROS() message conversions should be implemented for any new message types
 *      - draw() needs to be defined for the corresponding SofaType
 *
 * @tparam DataTypes SOFA data type should be informed by the child class implementation
 * @tparam ROS2_MSG ROS message type should be informed by the child class implementation
 */
template <class DataTypes, class ROS2_MSG>
class ROS2Publisher : public ROS2BaseObject<DataTypes, ROS2_MSG>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(ROS2Publisher, DataTypes, ROS2_MSG), SOFA_TEMPLATE2(ROS2BaseObject, DataTypes, ROS2_MSG));

    sofa::Data<DataTypes> d_input;
    Data<double> d_drawScale;
    Data<bool> d_canPublish;
    Data<bool> d_draw;

    std::shared_ptr<ROS2PublisherNode<ROS2_MSG>> m_ros2node;

    explicit ROS2Publisher();

    virtual ~ROS2Publisher() = default;

    virtual void init() override;

    virtual void handleEvent(sofa::core::objectmodel::Event* event) override;

    virtual void draw(const sofa::core::visual::VisualParams* vparams) override;

    static std::string templateName(const ROS2Publisher<DataTypes, ROS2_MSG>* = nullptr) { return "Unknown"; }
};

}  // namespace ros2
}  // end namespace sofa
