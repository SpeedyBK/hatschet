//
// Created by nfiege on 4/21/22.
//

#ifndef HATSCHET_SATSCHEDULERLATOPT_H
#define HATSCHET_SATSCHEDULERLATOPT_H

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

	class SATSchedulerLatOpt : public SchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase {
	public:
		enum LatencyOptimizationStrategy {
			AUTO,
			REVERSE_LINEAR,
			LINEAR_JUMP
		};
		SATSchedulerLatOpt(Graph& g, ResourceModel &resourceModel, int II=-1);
		double getSolvingTime() const { return this->solvingTime; }
		void schedule() override;
		void setSolverTimeout(unsigned int newTimeoutInSec);
		void setLatencyOptimizationStrategy(const LatencyOptimizationStrategy &newLos);
		void setTargetLatency(const int &newTargetLatency);
		void setEarliestStartTimes(const std::map<Vertex*, int> &newEarliestStartTimes);
		void setLatestStartTimeDifferences(const std::map<Vertex*, int> &newLatestStartTimeDifferences);
		int linearJumpLength;

	private:
		void setLatencySearchStrategy();
		bool computeNewLatencySuccess(const bool &lastSchedulingAttemptSuccessful);
		void calculateLatestStartTimes();
		void simplifyResourceLimits();
		void restoreResourceLimits();

		void initScheduler();
		void defineLatLimits();
		void setUpSolver();
		void resetContainer();
		void createLiterals();
		void createClauses();
		void fillSolutionStructure();

		bool optimalResult;
		bool enableIIBasedLatencyLowerBound;
		int candidateII;
		int candidateLatency;
		int lastCandidateLatency;
		int minLatency;
		int maxLatency;
		int latencyLowerBound;
		int latencyUpperBound;

		unsigned int solverTimeout;

		double solvingTime;
		std::unique_ptr<CaDiCaL::Solver> solver;
		CaDiCaLTerminator terminator;

		int literalCounter;
		int scheduleTimeLiteralCounter;
		int bindingLiteralCounter;
		int timeOverlapLiteralCounter;
		int bindingOverlapLiteralCounter;
		int clauseCounter;
		int dependencyConstraintClauseCounter;
		int resourceConstraintClauseCounter;
		int scheduleTimeConstraintClauseCounter;
		int bindingConstraintClauseCounter;
		int timeOverlapClauseCounter;
		int bindingOverlapClauseCounter;
		std::map<Vertex*, int> resourceLimit;
		std::map<Vertex*, bool> vertexIsUnlimited;
		std::map<std::pair<Vertex*, int>, int> scheduleTimeLiterals;
		std::map<std::pair<Vertex*, int>, int> bindingLiterals;
		std::map<std::pair<Vertex*, Vertex*>, int> timeOverlapLiterals;
		std::map<std::pair<Vertex*, Vertex*>, int> bindingOverlapLiterals;
		std::set<int> latencyAttempts;
		LatencyOptimizationStrategy los;
		std::map<Vertex*, int> earliestStartTime;
		std::map<Vertex*, int> latestStartTime;
		std::map<Vertex*, int> lastLatestStartTime;
		std::map<Vertex*, int> latestStartTimeDifferences;
		std::map<const Resource*, int> originalResourceLimits;
	};
}
#endif //USE_CADICAL
#endif //HATSCHET_SATSCHEDULERLATOPT_H
