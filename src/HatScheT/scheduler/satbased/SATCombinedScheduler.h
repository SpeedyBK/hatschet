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
		SATCombinedScheduler(Graph& g, ResourceModel &resourceModel, int II=-1);
		double getSolvingTime() const { return this->solvingTime; }
		void schedule() override;
		void setSolverTimeout(unsigned int newTimeoutInSec);
		void setUseOptLatScheduler(bool newUseOptLatScheduler);
	private:
		void initScheduler();
		unsigned int solverTimeout;
		bool optimalResult;
		int candidateII;
		double solvingTime;
		bool useOptLatScheduler;
	};
}
#endif //USE_CADICAL
#endif //HATSCHET_SATCOMBINEDSCHEDULER_H
