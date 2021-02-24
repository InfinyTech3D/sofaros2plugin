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
#include "geometry_msgs/msg/point.hpp"


namespace sofa {
    namespace ros2 {

        using namespace sofa::defaulttype;
        using core::objectmodel::Data;

        using namespace sofa::defaulttype;

        class Ros2Publisher : public rclcpp::Node {
        public:
            Ros2Publisher() : Node("sofaRos2_publisher") {
                m_publisher = this->create_publisher<geometry_msgs::msg::Point>("sofa/point", 10);
            }

            void run() {
                rclcpp::spin(std::make_shared<Ros2Publisher>());
            }

            rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr m_publisher;

            std::unique_ptr<std::thread> spawn() {
                return std::make_unique<std::thread>(&Ros2Publisher::run, this);
            }


        };

        class Vec3dPublisher : public core::objectmodel::BaseObject {
        public:
            SOFA_CLASS(Vec3dPublisher, core::objectmodel::BaseObject);

            sofa::Data<Vec3d> d_input;
            core::objectmodel::DataCallback c_callback;
            sofa::Data<bool> d_draw;


            std::unique_ptr<Ros2Publisher> m_r2p_ptr;
            std::unique_ptr<std::thread> m_thread;


            Vec3dPublisher();

            virtual ~Vec3dPublisher();

            virtual void init();

            virtual void draw(const sofa::core::visual::VisualParams *vparams);

            void update();
        };

    }  // namespace ros2
}  // end namespace sofa
