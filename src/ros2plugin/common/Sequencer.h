/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_CONTROLLER_Sequencer_H
#define SOFA_COMPONENT_CONTROLLER_Sequencer_H
//#include "config.h"


#include <sofa/type/vector.h>
#include <sofa/component/controller/Controller.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include <iostream>
#include <chrono>
#include <thread>

namespace sofa
{

namespace component
{

namespace controller
{

/**
 * @brief The Sequencer class goal is to be able to give a file
 * containing a vector of force norm to be used with MaxMotionForcefield.
 * The goal was to be able to make a minimisation of the error by changing
 * the input force using a python script. So there was the need to get those
 * normes from an external file.
 */

class Sequencer : public Controller
{
public:
    SOFA_CLASS(Sequencer,Controller);

    Data<double> d_frequency;
    Data<int> d_saveLagAt;
    Data<std::string> d_filename;

    Sequencer();

    virtual void init();
    virtual void handleEvent(sofa::core::objectmodel::Event *event);
    virtual void cleanup();
    void saveInfile();

protected:
    std::chrono::high_resolution_clock::time_point m_T;
    double m_period;
    std::vector<double> m_lag;
    long m_stepCounter;
};


} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_Sequencer_H
