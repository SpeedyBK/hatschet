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

#include <HatScheT/scheduler/ilpbased/RationalIIScheduler.h>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/utility/Utility.h>

namespace HatScheT
{
    RationalIIScheduler::RationalIIScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist)
    : MoovacScheduler(g, resourceModel, solverWishlist)
    {
      throw HatScheT::Exception("rationalIIScheduler::rationalIIScheduler: This constructor is currently under construction and disabled!");
  /*this->minII = this->computeMinII(&g,&resourceModel);
  HatScheT::ASAPScheduler asap(g,resourceModel);
  this->maxII = Utility::calcMaxII(&asap);
  this->SLMax = 0;*/
}

}
