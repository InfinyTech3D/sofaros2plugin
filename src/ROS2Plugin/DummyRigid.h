#pragma once

#include <ROS2Plugin/types.h>
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

        using namespace sofa::defaulttype;
        using core::objectmodel::Data;

        using namespace sofa::defaulttype;

        class Ros2Publisher : public rclcpp::Node {
        public:
            Ros2Publisher() : Node("sofaRos2_publisher") {
                publisher = this->create_publisher<geometry_msgs::msg::Pose>("sofa/pose", 10);
            }

            void run() {
                rclcpp::spin(std::make_shared<Ros2Publisher>());
                rclcpp::shutdown();
            }

            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher;

            std::unique_ptr<std::thread> spawn() {
                return std::make_unique<std::thread>(&Ros2Publisher::run, this);
            }


        };

        class DummyRigid : public core::objectmodel::BaseObject {
        public:
            SOFA_CLASS(DummyRigid, core::objectmodel::BaseObject);

            sofa::Data<Rigid> d_input;
            sofa::Data<Rigid> d_output;
            core::objectmodel::DataCallback c_callback;
            sofa::Data<bool> d_draw;


            std::unique_ptr<Ros2Publisher> m_r2p_ptr;
            std::unique_ptr<std::thread> m_thread;


            DummyRigid();

            virtual ~DummyRigid();

            virtual void init();

            virtual void handleEvent(sofa::core::objectmodel::Event *event);

            virtual void draw(const sofa::core::visual::VisualParams *vparams);

            void update();
        };

    }  // namespace ros2
}  // end namespace sofa
