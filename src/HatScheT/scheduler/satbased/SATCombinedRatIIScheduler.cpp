//
// Created by nfiege on 6/21/22.
//

#include "SATCombinedRatIIScheduler.h"
#include <HatScheT/utility/Utility.h>
#ifdef USE_CADICAL
#ifdef USE_SCALP
#include <HatScheT/scheduler/satbased/SATRatIIScheduler.h>
#include <HatScheT/scheduler/satbased/SATSCCScheduler.h>
namespace HatScheT {

	SATCombinedRatIIScheduler::SATCombinedRatIIScheduler(Graph &g, ResourceModel &resourceModel)
		: RationalIISchedulerLayer(g, resourceModel), solverTimeout(300), solvingTime(-1.0)
	{
		// nothing to do here
	}

	void SATCombinedRatIIScheduler::scheduleIteration() {
		this->initScheduler();
		// prove II infeasible or compute valid schedule with SCC-based scheduler
		SATSCCScheduler s1(this->unrolledGraph, this->unrolledResourceModel, this->modulo);
		s1.setSolverTimeout(this->solverTimeout);
		s1.setQuiet(this->quiet);
		s1.setMaxLatencyConstraint(this->maxLatencyConstraint);
		s1.schedule();
		int lat;
		if (s1.getScheduleFound()) {
			// found schedule for II
			this->scheduleFound = true;
			this->secondObjectiveOptimal = false;
			auto sccStartTimes = s1.getSchedule();
			this->startTimesVector.resize(this->samples);
			for (auto &v : this->g.Vertices()) {
				for (int s=0; s<this->samples; s++) {
					auto *vUnrolled = this->vertexMappings.at(v).at(s);
					auto t = sccStartTimes.at(vUnrolled);
					this->startTimesVector.at(s)[v] = t;
					if (s==0) {
						this->startTimes[v] = t;
					}
				}
			}
			lat = s1.getScheduleLength();
			if (!this->quiet) {
				std::cout << "SATCombinedRatIIScheduler: found initial schedule with schedule length " << lat << " using the SCC heuristic:" << std::endl;
				for (auto &v : this->g.Vertices()) {
					for (int s=0; s<this->samples; s++) {
						std::cout << "  " << v->getName() << " (" << s << ") - " << this->startTimesVector.at(s).at(v) << std::endl;
					}
				}
			}
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
			return;
		}
		auto sccTime = s1.getSolvingTime();
		if (sccTime > this->solverTimeout) {
			// sanity check for timeout
			return;
		}
		if (!this->quiet) {
			std::cout << "SATCombinedRatIIScheduler: SAT-based SCC scheduler found solution for II=" << this->modulo << "/" << this->samples << " and schedule length " << lat << std::endl;
		}
		auto satTimeout = (unsigned int)(this->solverTimeout - sccTime);
		// refine latency with normal scheduler
		SATRatIIScheduler s2(this->g, this->resourceModel, this->modulo, this->samples);
		s2.setMaxRuns(1);
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
			this->startTimesVector = s2.getStartTimeVector();
			// check if it's latency-optimal
			this->secondObjectiveOptimal = s2.getObjectivesOptimal().second;
			if (!this->quiet) {
				std::cout << "SATCombinedRatIIScheduler: SAT scheduler refined schedule length to " << s2.getScheduleLength() << std::endl;
			}
		}
	}

	void SATCombinedRatIIScheduler::initScheduler() {
		// clear previous vertex mappings
		this->vertexMappings.clear();
		// reset the graph + resource model
		this->unrolledGraph.reset();
		this->unrolledResourceModel.reset();
		// unroll the graph + resource model
		Utility::unroll(&this->unrolledGraph, &this->unrolledResourceModel, this->samples, this->modulo, &this->g, &this->resourceModel, &this->vertexMappings);
	}

	void SATCombinedRatIIScheduler::setSolverTimeout(unsigned int newTimeoutInSec) {
		this->solverTimeout = newTimeoutInSec;
	}
}
#endif //USE_SCALP
#endif //USE_CADICAL