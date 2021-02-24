#pragma once

#include <ROS2Plugin/DummyRigid.h>


namespace sofa {
    namespace ros2 {

        DummyRigid::DummyRigid()
                : d_input(initData(&d_input, Rigid(Vec3d(0, 0, 0), Quat(0, 0, 0, 0)), "input", "input")),
                  d_output(initData(&d_output, Rigid(Vec3d(0, 0, 0), Quat(0, 0, 0, 0)), "output", "output")),
                  d_draw(initData(&d_draw, false, "draw", "If true, position is drawn on screen")) {
            this->f_listening.setValue(true);
            m_thread.reset(nullptr);
            rclcpp::init(0, nullptr);
        }

        DummyRigid::~DummyRigid() {
            if (!m_thread) return;
            if (m_thread->joinable()) m_thread->join();
            m_thread.reset(nullptr);
            msg_info("ROS2Plugin") << "Thread terminated";
        }

        void DummyRigid::init() {
            if (!m_thread) {
                m_r2p_ptr = std::make_unique<Ros2Publisher>();
                m_thread = m_r2p_ptr->spawn();
                m_thread->detach();
            }

            c_callback.addInput(&d_input);
            c_callback.addCallback(std::bind(&DummyRigid::update, this));
        }

        void DummyRigid::update() {

            msg_info("DummyRigid") << "Every time input changes, this callback is called";
            geometry_msgs::msg::Pose pose = geometry_msgs::msg::Pose();
            pose.position.x = d_input.getValue().getCenter()[0];
            pose.position.y = d_input.getValue().getCenter()[1];
            pose.position.z = d_input.getValue().getCenter()[2];
            pose.orientation.x = d_input.getValue().getOrientation()[0];
            pose.orientation.y = d_input.getValue().getOrientation()[1];
            pose.orientation.z = d_input.getValue().getOrientation()[2];
            pose.orientation.w = d_input.getValue().getOrientation()[3];
            m_r2p_ptr->publisher->publish(pose);

        }

        void DummyRigid::handleEvent(sofa::core::objectmodel::Event *event) {
            if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event)) {
                msg_info("DummyRigid") << "In every simulation begin event the output is updated";
                d_output.setValue(d_input.getValue());
            }
        }

        void DummyRigid::draw(const sofa::core::visual::VisualParams *vparams) {
            if (d_draw.getValue()) {
                const Rigid &output = d_output.getValue();
                vparams->drawTool()->drawFrame(output.getCenter(), output.getOrientation(), Vec3d(0.1, 0.1, 0.1));
            }
        }

    }  // namespace ros2
}  // end namespace sofa
