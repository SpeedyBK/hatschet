//
// Created by nfiege on 5/25/22.
//

#ifndef HATSCHET_SATCOMBINEDSCHEDULER_H
#define HATSCHET_SATCOMBINEDSCHEDULER_H

#ifdef USE_CADICAL
#include <cadical.hpp>
#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <string>
#include <map>
#include <utility>
#include <memory>
#include <HatScheT/scheduler/satbased/CaDiCaLTerminator.h>

namespace HatScheT {
	class SATCombinedScheduler : public SchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase {
	public:
		enum backendSchedulerType_t {
			BACKEND_SAT,
			BACKEND_SATLAT,
			BACKEND_SATBIN
		};
		SATCombinedScheduler(Graph& g, ResourceModel &resourceModel, int II=-1);
		double getSolvingTime() const { return this->solvingTime; }
		void schedule() override;
		void setSolverTimeout(unsigned int newTimeoutInSec);
		void setBackendSchedulerType(backendSchedulerType_t newBackendSchedulerType);
	private:
		void initScheduler();
		unsigned int solverTimeout;
		bool optimalResult;
		int candidateII;
		double solvingTime;
		backendSchedulerType_t backendSchedulerType;
	};
}
#endif //USE_CADICAL
#endif //HATSCHET_SATCOMBINEDSCHEDULER_H
