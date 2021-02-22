#pragma once

#include <ROS2Plugin/DummyVec3d.h>

namespace sofa {
namespace ros2 {

DummyVec3d::DummyVec3d()
    : d_input(initData(&d_input, sofa::defaulttype::Vec3d(0, 0, 0), "input", "input"))
    , d_output(initData(&d_output, sofa::defaulttype::Vec3d(0, 0, 0), "output", "output"))
    , d_draw(initData(&d_draw, false, "draw", "If true, position is drawn on screen"))

{
    this->f_listening.setValue(true);
    c_callback.addInput(&d_input);
    c_callback.addCallback(std::bind(&DummyVec3d::update, this));
}

DummyVec3d::~DummyVec3d() {}

void DummyVec3d::init() {}

void DummyVec3d::update() { msg_info("DummyVec3d") << "Every time input changes, this callback is called"; }

void DummyVec3d::handleEvent(sofa::core::objectmodel::Event *event) {
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event)) {
        msg_info("DummyVec3d") << "In every simulation begin event the output is updated";
        d_output.setValue(d_input.getValue());
    }
}

void DummyVec3d::draw(const sofa::core::visual::VisualParams *vparams) {
    if (d_draw.getValue()) {
        vparams->drawTool()->drawSphere(d_output.getValue(), 0.1, RGBAColor(1.0, 1.0, 1.0, 1.0));
    }
}

}  // namespace ros2
}  // end namespace sofa
