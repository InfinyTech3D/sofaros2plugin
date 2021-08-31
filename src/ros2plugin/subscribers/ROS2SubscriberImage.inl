#pragma once
#include <ros2plugin/subscribers/ROS2SubscriberImage.h>

namespace sofa
{
namespace ros2
{
ROS2SubscriberImage::ROS2SubscriberImage() : d_output(initData(&d_output, SofaImage(), "output", "output")) { this->f_listening.setValue(true); }

void ROS2SubscriberImage::init()
{
    d_output.setValue(SofaImage());
    this->createNode(m_ros2node);
}

void ROS2SubscriberImage::handleEvent(sofa::core::objectmodel::Event *event)
{
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
        auto msg = this->m_ros2node->get();
        this->d_output.setValue(MessageWrapper<SofaImage, GenericImageMsg>::toSofa(msg));
    }
}


}  // namespace ros2
}  // end namespace sofa
