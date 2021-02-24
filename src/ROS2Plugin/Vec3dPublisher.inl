#pragma once

#include <ROS2Plugin/Vec3dPublisher.h>


namespace sofa {
    namespace ros2 {

        Vec3dPublisher::Vec3dPublisher()
                : d_input(initData(&d_input, Vec3d(0, 0, 0), "input", "input")),
                  d_draw(initData(&d_draw, false, "draw", "If true, position is drawn on screen")) {
            this->f_listening.setValue(true);
            m_thread.reset(nullptr);
        }

        Vec3dPublisher::~Vec3dPublisher() {
            if (!m_thread) return;
            if (m_thread->joinable()) m_thread->join();
            m_thread.reset(nullptr);
            msg_info("ROS2Plugin") << "Thread terminated";
        }

        void Vec3dPublisher::init() {
            if (!m_thread) {
                m_r2p_ptr = std::make_unique<Ros2Publisher>();
                m_thread = m_r2p_ptr->spawn();
                m_thread->detach();
            }

            c_callback.addInput(&d_input);
            c_callback.addCallback(std::bind(&Vec3dPublisher::update, this));
        }

        void Vec3dPublisher::update() {
            msg_info("Vec3dPublisher") << "Every time input changes, this callback is called";
            geometry_msgs::msg::Point point = geometry_msgs::msg::Point();
            point.x = d_input.getValue()[0];
            point.y = d_input.getValue()[1];
            point.z = d_input.getValue()[2];
            m_r2p_ptr->m_publisher->publish(point);

        }

        void Vec3dPublisher::draw(const sofa::core::visual::VisualParams *vparams) {
            if (d_draw.getValue()) {
                const Vec3d &input = d_input.getValue();
                vparams->drawTool()->drawSphere(input, 0.1, RGBAColor(1.0, 1.0, 1.0, 1.0));
            }
        }

    }  // namespace ros2
}  // end namespace sofa
