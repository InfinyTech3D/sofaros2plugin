#pragma once

#include <ROS2Plugin/types.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>

namespace sofa {
namespace ros2 {

using namespace sofa::defaulttype;
using core::objectmodel::Data;

using namespace sofa::defaulttype;
class DummyRigid : public core::objectmodel::BaseObject {
   public:
    SOFA_CLASS(DummyRigid, core::objectmodel::BaseObject);
    sofa::Data<Rigid>               d_input;
    sofa::Data<Rigid>               d_output;
    core::objectmodel::DataCallback c_callback;
    sofa::Data<bool>                d_draw;

    DummyRigid();
    virtual ~DummyRigid();
    virtual void init();
    virtual void handleEvent(sofa::core::objectmodel::Event* event);
    virtual void draw(const sofa::core::visual::VisualParams* vparams);
    void         update();
};

}  // namespace ros2
}  // end namespace sofa
