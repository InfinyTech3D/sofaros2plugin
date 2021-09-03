#pragma once
#include <ros2plugin/common/MessageArrayWrapper.h>
#include <ros2plugin/common/ROS2BaseObject.h>
#include <ros2plugin/common/ROS2Context.h>
#include <ros2plugin/common/types.h>
#include <ros2plugin/subscribers/ROS2SubscriberImageNode.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>

namespace sofa
{
namespace ros2
{
class ROS2SubscriberImage : public ROS2BaseObject<SofaImage, GenericImageMsg>
{
public:
    SOFA_CLASS(ROS2SubscriberImage, SOFA_TEMPLATE2(ROS2BaseObject, SofaImage, GenericImageMsg));

    sofa::Data<std::string> d_encoding;
    sofa::Data<std::string> d_compression;
    sofa::Data<SofaImage> d_output;
    std::shared_ptr<ROS2SubscriberImageNode> m_ros2node;

    explicit ROS2SubscriberImage();
    virtual ~ROS2SubscriberImage() = default;

    virtual void init() override;
    virtual void handleEvent(sofa::core::objectmodel::Event* event) override;

    virtual SofaImage toSofa(const GenericImageMsg::ConstSharedPtr& msg);

    virtual bool isEncodingValid();

    virtual bool isCompressionValid();

    static std::string templateName(const ROS2SubscriberImage* = nullptr) { return "Unknown"; }
};

}  // namespace ros2
}  // end namespace sofa
