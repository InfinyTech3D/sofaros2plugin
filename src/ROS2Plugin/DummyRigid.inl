#pragma once

#include <ROS2Plugin/DummyRigid.h>

namespace sofa {
namespace ros2 {

DummyRigid::DummyRigid()
    : d_input(initData(&d_input, Rigid(Vec3d(0, 0, 0), Quat(0, 0, 0, 0)), "input", "input"))
    , d_output(initData(&d_output, Rigid(Vec3d(0, 0, 0), Quat(0, 0, 0, 0)), "output", "output"))
    , d_draw(initData(&d_draw, false, "draw", "If true, position is drawn on screen"))

{
    this->f_listening.setValue(true);
    c_callback.addInput(&d_input);
    c_callback.addCallback(std::bind(&DummyRigid::update, this));
}

DummyRigid::~DummyRigid() {}

void DummyRigid::init() {}

void DummyRigid::update() { msg_info("DummyRigid") << "Every time input changes, this callback is called"; }

void DummyRigid::handleEvent(sofa::core::objectmodel::Event *event) {
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event)) {
        msg_info("DummyRigid") << "In every simulation begin event the output is updated";
        d_output.setValue(d_input.getValue());
    }
}

void DummyRigid::draw(const sofa::core::visual::VisualParams *vparams) {
    if (d_draw.getValue()) {
        const Rigid& output = d_output.getValue();
        vparams->drawTool()->drawFrame(output.getCenter(), output.getOrientation(), Vec3d(0.1, 0.1, 0.1));
    }
}

}  // namespace ros2
}  // end namespace sofa
