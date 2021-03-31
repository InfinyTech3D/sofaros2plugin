#pragma once
#include <ros2plugin/subscribers/ROS2SubscriberArray.h>

namespace sofa {
namespace ros2 {

template <class DataTypes, class ROS2_MSG>
ROS2SubscriberArray<DataTypes, ROS2_MSG>::ROS2SubscriberArray() : d_indexes(initData(&d_indexes, "indexes", "index list to subscribe to"))
{
    this->f_listening.setValue(true);
}


template <class DataTypes, class ROS2_MSG>
void ROS2SubscriberArray<DataTypes, ROS2_MSG>::handleEvent(sofa::core::objectmodel::Event *event)
{
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event)) {
        auto msg = this->m_ros2node->get();
        this->d_output.setValue(MessageArrayWrapper<DataTypes, ROS2_MSG>::toSofa(msg, d_indexes.getValue()));
    }
}

template <class DataTypes, class ROS2_MSG>
void ROS2SubscriberArray<DataTypes, ROS2_MSG>::draw(const sofa::core::visual::VisualParams *vparams)
{
    if (this->d_draw.getValue()) {
        const auto &output = this->d_output.getValue();
        MessageArrayWrapper<DataTypes, ROS2_MSG>::draw(vparams, output, this->d_drawScale.getValue(), d_indexes.getValue());
    }
}

}  // namespace ros2
}  // end namespace sofa
