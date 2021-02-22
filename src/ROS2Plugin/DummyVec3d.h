#pragma once

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/simulation/AnimateBeginEvent.h>

namespace sofa {
namespace component {

using namespace sofa::defaulttype;
using core::objectmodel::Data;

using namespace sofa::defaulttype;
class DummyVec3d : public core::objectmodel::BaseObject {
   public:
    SOFA_CLASS(DummyVec3d, core::objectmodel::BaseObject);
    sofa::Data<sofa::defaulttype::Vec3d> d_input;
    sofa::Data<sofa::defaulttype::Vec3d> d_output;
    core::objectmodel::DataCallback      c_callback;
    sofa::Data<bool>                     d_draw;

    DummyVec3d();
    virtual ~DummyVec3d();
    virtual void init();
    virtual void handleEvent(sofa::core::objectmodel::Event* event);
    virtual void draw(const sofa::core::visual::VisualParams* vparams);
    void         update();
};

}  // end namespace component
}  // end namespace sofa
