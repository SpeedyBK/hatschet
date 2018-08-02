/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Patrick Sittel (sittel@uni-kassel.de)
  All rights reserved.
*/
#pragma once

#include <HatScheT/scheduler/ilpbased/MoovacScheduler.h>

namespace HatScheT
{

class rationalIIScheduler : public MoovacScheduler
{
public:
    rationalIIScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);

    virtual int getII() { return this->II;}
protected:


};
}
