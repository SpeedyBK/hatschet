/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)

    Copyright (C) 2018

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "MoovacResAwScheduler.h"
#include <HatScheT/utility/Utility.h>

namespace HatScheT
{

MoovacResAwScheduler::MoovacResAwScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist, XilinxFPGA& fpga)
        : MoovacScheduler(g, resourceModel, solverWishlist), fpga(fpga) {
  this->minII = this->computeMinII(&g,&resourceModel);
  this->maxII = Utility::calcMaxII(&g, &resourceModel);
  this->SLMax = 0;

  this->resourceModelIsValid();
}

void MoovacResAwScheduler::schedule()
{
  cout << "MoovacResAwScheduler.schedule: start" << endl;

  cout << "MoovacResAwScheduler.schedule: finished" << endl;
  cout << "II: " << this->getII() << endl;
}

void MoovacResAwScheduler::constructProblem()
{

}

void MoovacResAwScheduler::setObjective()
{

}

void MoovacResAwScheduler::setGeneralConstraints()
{

}

bool MoovacResAwScheduler::resourceModelIsValid()
{
  for(auto it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); ++it){
    Resource* r  = *it;
    //skip unlimited resources
    if(r->getLimit() == -1) continue;

    if(this->fpga.constraintExists(r->getName()) == false) return false;
  }

  return true;
}

}