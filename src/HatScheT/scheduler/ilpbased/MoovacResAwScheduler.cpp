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

MoovacResAwScheduler::MoovacResAwScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist, Target& target)
        : MoovacScheduler(g, resourceModel, solverWishlist), target(target) {
  if(Utility::resourceModelAndTargetValid(resourceModel,target) == false){
    throw HatScheT::Exception("MoovacResAwScheduler.MoovacResAwScheduler: ERROR Resource Model and Hardware Target are not corresponding!");
  }

  this->minII = this->computeMinII(&g,&resourceModel);
  this->maxII = Utility::calcMaxII(&g, &resourceModel);
  if (this->minII >= this->maxII) this->maxII = this->minII+1;
  this->SLMax = 0;
}

void MoovacResAwScheduler::schedule()
{
  cout << "MoovacResAwScheduler.schedule: start " << this->g.getName() << endl;

  this->setUpSolverSettings();
  this->constructProblem();

  cout << "MoovacResAwScheduler.schedule: finished " << this->g.getName() << endl;
  cout << "Identified II: " << this->getII() << endl;
}

void MoovacResAwScheduler::constructProblem()
{
  this->setMaxLatency();
}

void MoovacResAwScheduler::setObjective()
{

}

void MoovacResAwScheduler::setGeneralConstraints()
{

}

}