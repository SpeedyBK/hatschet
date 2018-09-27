/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Julian Oppermann (oppermann@esa.tu-darmstadt.de)

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

#include <HatScheT/scheduler/ilpbased/EichenbergerDavidson97Scheduler.h>
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/Exception.h"

#include <iostream>
#include <map>

namespace HatScheT
{

EichenbergerDavidson97Scheduler::EichenbergerDavidson97Scheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist) : SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist)
{
  this->minII = -1; //this->computeMinII(&g,&resourceModel);
  this->maxII = Utility::calcMaxII(&g, &resourceModel);
  if (minII > maxII)
    throw HatScheT::Exception("Inconsistent II bounds");
}

void EichenbergerDavidson97Scheduler::schedule()
{
  this->II = this->maxII;
  for (auto *v : g.Vertices())
    this->startTimes.insert(std::make_pair(v, v->getId()));
}

void EichenbergerDavidson97Scheduler::constructProblem()
{
  std::cout << "ED97::constructProblem" << std::endl;
}

void EichenbergerDavidson97Scheduler::setObjective()
{
  std::cout << "ED97::setObjective" << std::endl;
}

}
