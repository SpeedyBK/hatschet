//
// Created by nfiege on 4/21/22.
//

#ifndef HATSCHET_SATSCHEDULER_H
#define HATSCHET_SATSCHEDULER_H

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

	class SATScheduler : public SchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase {
	public:
		enum LatencyOptimizationStrategy {
			REVERSE_LINEAR,
			LINEAR,
			LINEAR_JUMP,
			LINEAR_JUMP_LOG,
			LOGARITHMIC
		};
		SATScheduler(Graph& g, ResourceModel &resourceModel, int II=-1);
		double getSolvingTime() const { return this->solvingTime; }
		void schedule() override;
		void setLatencyOptimizationStrategy(const LatencyOptimizationStrategy &newLos);
		void setTargetLatency(const int &newTargetLatency);
		void setEarliestStartTimes(const std::map<Vertex*, int> &newEarliestStartTimes);
		void setLatestStartTimeDifferences(const std::map<Vertex*, int> &newLatestStartTimeDifferences);
		int linearJumpLength;

	    string getName() override { return "SAT-Scheduler"; }

	private:
		bool computeNewLatencySuccess(const bool &lastSchedulingAttemptSuccessful);
		void calcMinLatency();
		void calcMaxLatency();
		void calculateLatestStartTimes();
		void calculateLatestStartTimeDifferences();
		void calculateEarliestStartTimes();
		void simplifyResourceLimits();
		void restoreResourceLimits();

		void initScheduler();
		void setUpSolver();
		void resetContainer();
		void createLiterals();
		void createClauses();
		void fillSolutionStructure();

		bool optimalResult;
		bool enableIIBasedLatencyLowerBound;
		int candidateII;
		int candidateLatency;
		int minLatency;
		int maxLatency;
		int latencyLowerBound;
		int latencyUpperBound;

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
		std::map<Vertex*, int> latestStartTimeDifferences;
		std::map<const Resource*, int> originalResourceLimits;
	};
}
#endif //USE_CADICAL
#endif //HATSCHET_SATSCHEDULER_H
