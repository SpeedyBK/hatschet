//
// Created by nfiege on 17/10/19.
//

#include "ModuloQScheduler.h"

namespace HatScheT {
	ModuloQScheduler::ModuloQScheduler(HatScheT::Graph &g, HatScheT::ResourceModel &resourceModel,
																							 std::list<std::string> solverWishlist) :
		SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist)
	{

	}

	void ModuloQScheduler::schedule() {
		std::cout << "graph: " << std::endl;
		std::cout << this->g << std::endl;
		std::cout << "resource model: " << std::endl;
		std::cout << this->resourceModel << std::endl;
		throw HatScheT::Exception("ModuloQScheduler::schedule: I don't work yet :(");
	}
}


