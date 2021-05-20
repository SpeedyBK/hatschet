//
// Created by nfiege on 17/02/21.
//

#include "RationalIIModuloSDCScheduler.h"
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/scheduler/ALAPScheduler.h>
#include <math.h>

namespace HatScheT {
	RationalIIModuloSDCScheduler::RationalIIModuloSDCScheduler(Graph &g, ResourceModel &resourceModel,
																														 std::list<std::string> solverWishlist) :
		RationalIISchedulerLayer(g, resourceModel), ILPSchedulerBase(solverWishlist){
		// do nothing
	}

	void RationalIIModuloSDCScheduler::scheduleIteration() {
		// do nothing
	}

	void RationalIIModuloSDCScheduler::constructProblem() {
		// do nothing
	}

	void RationalIIModuloSDCScheduler::setObjective() {
		// do nothing
	}

	void RationalIIModuloSDCScheduler::resetContainer() {
		// do nothing
	}
}
