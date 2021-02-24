#pragma once

#include <sofa/core/objectmodel/BaseObject.h>

#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

namespace sofa {
namespace ros2 {

class ROS2Context : public core::objectmodel::BaseObject {
   public:
    SOFA_CLASS(ROS2Context, core::objectmodel::BaseObject);

    ROS2Context() {
        rclcpp::init(0, nullptr);
    };

    virtual ~ROS2Context() = default;

    void init() override {}

    void bwdInit() override {
        auto thread = std::thread([this] {
            rclcpp::executors::MultiThreadedExecutor executor;
            for (auto node : m_workers) executor.add_node(node);
            executor.spin();
        });
        thread.detach();
    }

    void cleanup() override { rclcpp::shutdown(); }

    void addNode(const std::shared_ptr<rclcpp::Node>& new_node) { m_workers.push_back(new_node); }

   private:
    std::vector<std::shared_ptr<rclcpp::Node>> m_workers;
};

}  // namespace ros2
}  // end namespace sofa
