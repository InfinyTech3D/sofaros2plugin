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
#ifndef SOFA_COMPONENT_CONTROLLER_Sequencer_INL
#define SOFA_COMPONENT_CONTROLLER_Sequencer_INL

#include "Sequencer.h"
#include <sofa/simulation/AnimateEndEvent.h>
#include <ctime>
#include <fstream>


namespace sofa
{

namespace component
{

namespace controller
{




Sequencer::Sequencer()
: d_frequency(initData(&d_frequency,40.0, "frequency", "Frenquency"))
, d_saveLagAt(initData(&d_saveLagAt,0,"saveLagAt","Step id at which save the lag vector, if 0 never save, if -1 save at end"))
, d_filename(initData(&d_filename,std::string("lag.bin"),"filename","filename"))
{
    f_listening.setValue(true);
}

void Sequencer::init()
{
    m_T = std::chrono::high_resolution_clock::now();
    m_period = 1000000/d_frequency.getValue();
    m_stepCounter = 0;
    if(d_saveLagAt.getValue() !=0) m_lag.reserve(d_saveLagAt.getValue()+2);
}

void Sequencer::handleEvent(sofa::core::objectmodel::Event *event)
{
    if (dynamic_cast<sofa::simulation::AnimateEndEvent*>(event))
    {


        std::chrono::high_resolution_clock::time_point curr_Time = std::chrono::high_resolution_clock::now();
        long R = m_period - std::chrono::duration_cast<std::chrono::microseconds>(curr_Time-m_T).count();
        if((d_saveLagAt.getValue() !=0)&&(m_stepCounter<d_saveLagAt.getValue()))
        {
            m_lag.push_back(R/m_period);
        }
        if ((d_saveLagAt.getValue() !=0)&&(m_stepCounter==d_saveLagAt.getValue()))
        {
            saveInfile();
        }
        ++m_stepCounter;
        std::this_thread::sleep_for(std::chrono::duration<long,std::micro>(R));
        m_T = std::chrono::high_resolution_clock::now();

    }
}

void Sequencer::cleanup()
{
    if (d_saveLagAt.getValue()==-1)
    {
        saveInfile();
    }
}

void Sequencer::saveInfile()
{
    if(!(m_lag.size()==0))
    {
        std::ofstream OfFile;
        OfFile.open(d_filename.getValue().c_str(),std::ofstream::binary);
        if(OfFile.is_open())
        {
            for(int i=0;i<m_lag.size();i++)
            {
                OfFile << m_lag[i] <<std::endl;
            }
            OfFile.close();
            std::cout<<"Sequencer file written"<<std::endl;
        }
        else
        {
           msg_error()<<"Couldn't open file";
        }

    }
    else
    {
        std::cout<<"Sequencer no lag to write"<<std::endl;
    }
}

} // namespace contoller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_Sequencer_INL



