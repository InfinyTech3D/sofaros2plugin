#pragma once

#include <ROS2Plugin/types.h>
#include <ROS2Plugin/ROS2Publisher.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>

#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"


namespace sofa {
    namespace ros2 {
        class PosePublisher : public core::objectmodel::BaseObject {
        public:
            SOFA_CLASS(PosePublisher, core::objectmodel::BaseObject);

            sofa::Data<Rigid> d_input;
            core::objectmodel::DataCallback c_callback;
            sofa::Data<std::string> d_NodeName;
            sofa::Data<std::string> d_TopicName;
            sofa::Data<bool> d_draw;


            std::shared_ptr<ROS2Publisher<geometry_msgs::msg::Pose>> m_r2p_ptr;


            PosePublisher();

            virtual ~PosePublisher();

            virtual void init();

            virtual void draw(const sofa::core::visual::VisualParams *vparams);

            void update();
        };

    }  // namespace ros2
}  // end namespace sofa
