#pragma once
#include <ros2plugin/subscribers/ROS2SubscriberImage.h>

namespace sofa
{
namespace ros2
{

ROS2SubscriberImage::ROS2SubscriberImage()
    : d_output(initData(&d_output, SofaImage(), "output", "output"))
    , d_compression(initData(&d_compression, std::string("raw"), "compression", "raw, compressed, theora"))
    , d_encoding(initData(&d_encoding, std::string("bgr8"), "encoding", "mono8, bgr8, bgra8, rgb8, rgba8, mono16"))
{
    this->f_listening.setValue(true);
}

void ROS2SubscriberImage::init()
{
    d_output.setValue(SofaImage());
    isEncodingValid();
    isCompressionValid();
    this->createNode(m_ros2node);
    m_ros2node->init(d_encoding.getValue(), d_compression.getValue());
}

void ROS2SubscriberImage::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent*>(event))
    {
        GenericImageMsg::ConstSharedPtr msg = this->m_ros2node->get();
        this->d_output.setValue(toSofa(msg));
    }
}

inline SofaImage ROS2SubscriberImage::toSofa(const GenericImageMsg::ConstSharedPtr& msg)
{
    if (!msg) return SofaImage();
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, std::string(d_encoding.getValue()));
    }
    catch (cv_bridge::Exception& e)
    {
        msg_error("ROS2Plugin") << "cv_bridge exception: " << e.what();
        return SofaImage();
    }
    return SofaImage(cv_ptr->image);
}

bool ROS2SubscriberImage::isCompressionValid()
{
    auto compression = d_compression.getValue();
    if (compression == "raw" || compression == "compressed" || compression == "theora")
    {
        return true;
    }
    else
    {
        msg_error("ROS2SubscriberImage") << "Image compression " << compression << " not recognized, resetting it to default raw";
        d_compression.setValue("raw");
        return false;
    }
}

bool ROS2SubscriberImage::isEncodingValid()
{
    auto encoding = d_encoding.getValue();
    if (encoding == "mono8" || encoding == "bgr8" || encoding == "bgra8" || encoding == "rgb8" || encoding == "rgba8" || encoding == "mono16")
    {
        return true;
    }
    else
    {
        msg_error("ROS2SubscriberImage") << "Image encoding " << encoding << " not recognized, resetting it to default bgr8";
        d_encoding.setValue("bgr8");
        return false;
    }
}

}  // namespace ros2

}  // end namespace sofa
