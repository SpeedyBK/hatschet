//
// Created by nfiege on 5/25/22.
//

#include "SATCombinedScheduler.h"
#include <cmath>
#ifdef USE_CADICAL
#include <HatScheT/scheduler/satbased/SATScheduler.h>
#include <HatScheT/scheduler/satbased/SATSchedulerLatOpt.h>
#include <HatScheT/scheduler/satbased/SATSchedulerBinEnc.h>
#include <HatScheT/scheduler/satbased/SATSCCScheduler.h>
#include <memory>
namespace HatScheT {

	SATCombinedScheduler::SATCombinedScheduler(Graph &g, ResourceModel &resourceModel, int II)
		: SchedulerBase(g,resourceModel), solverTimeout(300), backendSchedulerType(BACKEND_SAT)
	{
		this->II = -1;
		this->timeouts = 0;
		this->scheduleFound = false;
		this->optimalResult = false;
		if (II <= 0) {
			computeMinII(&g, &resourceModel);
			this->minII = ceil(this->minII);
			computeMaxII(&g, &resourceModel);
		}
		else {
			this->minII = II;
			this->maxII = II;
			this->resMinII = II;
			this->recMinII = II;
		}
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
			SATSCCScheduler s1(this->g, this->resourceModel, this->candidateII, this->backendSchedulerType==BACKEND_SATBIN);
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
				if (!this->quiet) {
					std::cout << "SATCombinedScheduler: found initial schedule with SCC heuristic:" << std::endl;
					for (auto &v : this->g.Vertices()) {
						std::cout << "  " << v->getName() << " - " << this->startTimes.at(v) << std::endl;
					}
				}
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
					this->firstObjectiveOptimal = true;
				}
				// try next II
				continue;
			}
			auto sccTime = s1.getSolvingTime();
			if (sccTime > this->solverTimeout) {
				// sanity check for timeout
				if (s1.getScheduleFound()) {
					break;
				}
				else {
					continue;
				}
			}
			if (!this->quiet) {
				std::cout << "SATCombinedScheduler: SAT-based SCC scheduler found solution for II=" << this->candidateII << " and schedule length " << lat << std::endl;
			}
			auto satTimeout = (unsigned int)(this->solverTimeout - sccTime);
			// refine latency with normal scheduler
			lat--; // do "-1" do avoid checking a latency twice
			std::unique_ptr<SchedulerBase> s2;
			switch (this->backendSchedulerType) {
				case BACKEND_SAT:
					s2 = std::unique_ptr<SchedulerBase>(new SATScheduler(this->g, this->resourceModel, this->candidateII));
					dynamic_cast<SATScheduler*>(s2.get())->setSolverTimeout(satTimeout);
					dynamic_cast<SATScheduler*>(s2.get())->setQuiet(this->quiet);
					break;
				case BACKEND_SATLAT:
					s2 = std::unique_ptr<SchedulerBase>(new SATSchedulerLatOpt(this->g, this->resourceModel, this->candidateII));
					dynamic_cast<SATSchedulerLatOpt*>(s2.get())->setSolverTimeout(satTimeout);
					dynamic_cast<SATSchedulerLatOpt*>(s2.get())->setQuiet(this->quiet);
					break;
				case BACKEND_SATBIN:
					s2 = std::unique_ptr<SchedulerBase>(new SATSchedulerBinEnc(this->g, this->resourceModel, this->candidateII));
					dynamic_cast<SATSchedulerBinEnc*>(s2.get())->setSolverTimeout(satTimeout);
					dynamic_cast<SATSchedulerBinEnc*>(s2.get())->setQuiet(this->quiet);
					break;
				default:
					throw HatScheT::Exception("Requested unsupported backend scheduler type");
			}
			if (this->maxLatencyConstraint >= 0) {
				lat = min(this->maxLatencyConstraint, lat);
			}
			s2->setMaxLatencyConstraint(lat);
			s2->schedule();
			// check if the schedule is latency-optimal or if it is proven that there is no better solution than the SCC one
			this->secondObjectiveOptimal = dynamic_cast<ModuloSchedulerBase*>(s2.get())->getObjectivesOptimal().second;
			if (s2->getScheduleFound()) {
				// ayy we got a schedule :)
				this->startTimes = s2->getSchedule();
				if (!this->quiet) {
					std::cout << "SATCombinedScheduler: SAT scheduler refined schedule length to " << s2->getScheduleLength() << std::endl;
				}
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

	void SATCombinedScheduler::setBackendSchedulerType(backendSchedulerType_t newBackendSchedulerType) {
		this->backendSchedulerType = newBackendSchedulerType;
	}
}
#endif //USE_CADICAL