//
// Created by nfiege on 5/25/22.
//

#include "SATCombinedScheduler.h"
#include <cmath>
#ifdef USE_CADICAL
#include <HatScheT/scheduler/satbased/SATScheduler.h>
#include <HatScheT/scheduler/satbased/SATSCCScheduler.h>
namespace HatScheT {

	SATCombinedScheduler::SATCombinedScheduler(Graph &g, ResourceModel &resourceModel)
		: SchedulerBase(g,resourceModel), solverTimeout(300)
	{
		this->II = -1;
		this->timeouts = 0;
		this->scheduleFound = false;
		this->optimalResult = false;
		computeMinII(&g, &resourceModel);
		this->minII = ceil(this->minII);
		computeMaxII(&g, &resourceModel);
		this->solvingTime = -1.0;
		this->candidateII = -1;
	}

	void SATCombinedScheduler::schedule() {
		this->initScheduler();
		for (this->candidateII = (int)this->minII; this->candidateII <= (int)this->maxII; ++this->candidateII) {
			if (!this->quiet) {
				std::cout << "SATCombinedScheduler: candidate II=" << this->candidateII << std::endl;
			}
			// prove II infeasible or compute valid schedule with SCC-based scheduler
			SATSCCScheduler s1(this->g, this->resourceModel, this->candidateII);
			s1.setSolverTimeout(this->solverTimeout);
			s1.setQuiet(this->quiet);
			s1.setMaxLatencyConstraint(this->maxLatencyConstraint);
			s1.schedule();
			int lat;
			if (s1.getScheduleFound()) {
				// found schedule for II
				this->scheduleFound = true;
				this->II = this->candidateII;
				this->secondObjectiveOptimal = false;
				this->startTimes = s1.getSchedule();
				lat = s1.getScheduleLength();
			}
			else {
				// no schedule found
				if (s1.getIIFeasible()) {
					// timeout
					this->firstObjectiveOptimal = false;
				}
				else {
					// II infeasible
					std::cout << "#q# SCC-BASED SCHEDULER PROVED II = " << this->candidateII << " INFEASIBLE" << std::endl;
					this->firstObjectiveOptimal = true;
				}
				// try next II
				continue;
			}
			auto sccTime = s1.getSolvingTime();
			if (sccTime > this->solverTimeout) {
				// sanity check for timeout
				continue;
			}
			auto satTimeout = (unsigned int)(this->solverTimeout - sccTime);
			// refine latency with normal scheduler
			SATScheduler s2(this->g, this->resourceModel, this->candidateII);
			s2.setSolverTimeout(satTimeout);
			s2.setQuiet(this->quiet);
			if (this->maxLatencyConstraint >= 0) {
				lat = min(this->maxLatencyConstraint, lat);
			}
			s2.setMaxLatencyConstraint(lat);
			s2.schedule();
			if (s2.getScheduleFound()) {
				// ayy we got a schedule :)
				this->startTimes = s2.getSchedule();
				// check if it's latency-optimal
				this->secondObjectiveOptimal = s2.getObjectivesOptimal().second;
			}
			break;
		}
	}

	void SATCombinedScheduler::setSolverTimeout(unsigned int newTimeoutInSec) {
		this->solverTimeout = newTimeoutInSec;
	}

	void SATCombinedScheduler::initScheduler() {
		// let's be optimistic :)
		this->firstObjectiveOptimal = true;
		this->secondObjectiveOptimal = true;
	}
}
#endif //USE_CADICAL