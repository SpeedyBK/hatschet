//
// Created by nfiege on 6/21/22.
//

#include "SATCombinedRatIIScheduler.h"
#ifdef USE_CADICAL
#ifdef USE_SCALP
#include <HatScheT/scheduler/satbased/SATRatIIScheduler.h>
#include <HatScheT/scheduler/satbased/SATSCCRatIIScheduler.h>
#include <HatScheT/scheduler/dev/UnrollRationalIIScheduler.h>
#include <cmath>
namespace HatScheT {

	SATCombinedRatIIScheduler::SATCombinedRatIIScheduler(Graph &g, ResourceModel &resourceModel)
		: RationalIISchedulerLayer(g, resourceModel), solverTimeout(300), solvingTime(-1.0)
	{
		// nothing to do here
	}

	void SATCombinedRatIIScheduler::scheduleIteration() {
		this->initScheduler();
		// prove II infeasible or compute valid schedule with SCC-based scheduler
		//SATSCCRatIIScheduler s1(this->unrolledGraph, this->unrolledResourceModel, this->modulo, this->samples);
		SATSCCRatIIScheduler s1(this->g, this->resourceModel, this->modulo, this->samples);
		s1.setSolverTimeout(this->solverTimeout);
		s1.setQuiet(this->quiet);
		s1.setMaxLatencyConstraint(this->maxLatencyConstraint);
		s1.disableVerifier();
		s1.schedule();
		int lat;
		if (s1.getScheduleFound()) {
			// found schedule for II
			this->scheduleFound = true;
			this->secondObjectiveOptimal = false;
			this->startTimes = s1.getSchedule();
			this->startTimesVector = s1.getStartTimeVector();
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
		auto elapsedSchedulingTime = s1.getSolvingTime();
		this->solvingTime = elapsedSchedulingTime;
		if (!this->quiet) {
			std::cout << "SATCombinedRatIIScheduler: elapsed time after SCC scheduler: " << elapsedSchedulingTime << std::endl;
		}
		if (elapsedSchedulingTime > this->solverTimeout) {
			// sanity check for timeout
			return;
		}
		if (!this->quiet) {
			std::cout << "SATCombinedRatIIScheduler: SAT-based SCC scheduler found solution for II=" << this->modulo << "/" << this->samples << " and schedule length " << lat << std::endl;
		}
		auto schedulerTimeout = (unsigned int)std::ceil(this->solverTimeout - elapsedSchedulingTime);
		// refine latency with exact uniform scheduler
		SATRatIIScheduler s2(this->g, this->resourceModel, this->modulo, this->samples);
		s2.setMaxRuns(1);
		s2.setSolverTimeout(schedulerTimeout);
		s2.setQuiet(this->quiet);
		if (this->maxLatencyConstraint >= 0) {
			lat = min(this->maxLatencyConstraint, lat);
		}
		s2.setMaxLatencyConstraint(lat);
		s2.disableVerifier();
		s2.schedule();
		if (s2.getScheduleFound()) {
			// ayy we got a schedule :)
			this->startTimes = s2.getSchedule();
			this->startTimesVector = s2.getStartTimeVector();
			lat = s2.getScheduleLength();
			// check if it's latency-optimal
			this->secondObjectiveOptimal = s2.getObjectivesOptimal().second;
			if (!this->quiet) {
				std::cout << "SATCombinedRatIIScheduler: uniform scheduler refined schedule length to " << lat << std::endl;
			}
		}
		elapsedSchedulingTime += s2.getSolvingTime();
		this->solvingTime = elapsedSchedulingTime;
		if (!this->quiet) {
			std::cout << "SATCombinedRatIIScheduler: elapsed time after exact uniform scheduler: " << elapsedSchedulingTime << std::endl;
		}
		if (elapsedSchedulingTime > this->solverTimeout) {
			// sanity check for timeout
			return;
		}
		schedulerTimeout = (unsigned int)std::ceil(this->solverTimeout - elapsedSchedulingTime);
		// refine latency even more with unrolling-based exact nonuniform scheduler
		UnrollRationalIIScheduler s3(this->g, this->resourceModel, {"Gurobi", "CPLEX", "SCIP", "LPSolve"}, this->modulo, this->samples);
		s3.setIntIIScheduler(SchedulerType::SAT);
		s3.setMaxRuns(1);
		s3.setSolverTimeout(schedulerTimeout);
		s3.setQuiet(this->quiet);
		if (this->maxLatencyConstraint >= 0) {
			lat = min(this->maxLatencyConstraint, lat);
		}
		s3.setMaxLatencyConstraint(lat-1);
		s3.disableVerifier();
		s3.schedule();
		if (s3.getScheduleFound()) {
			// ayy we got a schedule :)
			this->startTimes = s3.getSchedule();
			this->startTimesVector = s3.getStartTimeVector();
			lat = s3.getScheduleLength();
			// check if it's latency-optimal
			this->secondObjectiveOptimal = s3.getObjectivesOptimal().second;
			if (!this->quiet) {
				std::cout << "SATCombinedRatIIScheduler: unrolling-based nonuniform scheduler refined schedule length to " << lat << std::endl;
			}
		}
		elapsedSchedulingTime += s3.getSolvingTime();
		this->solvingTime = elapsedSchedulingTime;
		if (!this->quiet) {
			std::cout << "SATCombinedRatIIScheduler: elapsed time after exact nonuniform scheduler: " << elapsedSchedulingTime << std::endl;
		}
	}

	void SATCombinedRatIIScheduler::initScheduler() {
		// clear previous vertex mappings
		this->vertexMappings.clear();
		// reset the graph + resource model
		this->unrolledGraph.reset();
		this->unrolledResourceModel.reset();
		// unroll the graph + resource model
		//Utility::unroll(&this->unrolledGraph, &this->unrolledResourceModel, this->samples, this->modulo, &this->g, &this->resourceModel, &this->vertexMappings);
	}

	void SATCombinedRatIIScheduler::setSolverTimeout(unsigned int newTimeoutInSec) {
		this->solverTimeout = newTimeoutInSec;
	}
}
#endif //USE_SCALP
#endif //USE_CADICAL