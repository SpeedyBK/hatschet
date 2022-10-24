//
// Created by nfiege on 6/21/22.
//

#ifndef HATSCHET_SATCOMBINEDRATIISCHEDULER_H
#define HATSCHET_SATCOMBINEDRATIISCHEDULER_H

#ifdef USE_CADICAL
#include <cadical.hpp>
#include <HatScheT/layers/RationalIISchedulerLayer.h>
#include <string>
#include <map>
#include <utility>
#include <memory>
#include <HatScheT/scheduler/satbased/CaDiCaLTerminator.h>

namespace HatScheT {
	class SATCombinedRatIIScheduler : public RationalIISchedulerLayer {
	public:
		SATCombinedRatIIScheduler(Graph& g, ResourceModel &resourceModel);
		double getSolvingTime() const { return this->solvingTime; }
		void scheduleIteration() override;
		void setSolverTimeout(unsigned int newTimeoutInSec);
	private:
		void initScheduler();
		unsigned int solverTimeout;
		bool optimalResult;
		double solvingTime;
		Graph unrolledGraph;
		ResourceModel unrolledResourceModel;
		std::map<Vertex*, std::vector<Vertex*>> vertexMappings;
	};
}

#endif //USE_CADICAL
#endif //HATSCHET_SATCOMBINEDRATIISCHEDULER_H
