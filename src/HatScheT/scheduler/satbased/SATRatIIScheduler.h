//
// Created by nfiege on 6/3/22.
//

#ifndef HATSCHET_SATRATIISCHEDULER_H
#define HATSCHET_SATRATIISCHEDULER_H
#ifdef USE_CADICAL
#include <cadical.hpp>
#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/layers/RationalIISchedulerLayer.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <string>
#include <map>
#include <utility>
#include <memory>
#include <HatScheT/scheduler/satbased/CaDiCaLTerminator.h>

namespace HatScheT {

	class SATRatIIScheduler : public RationalIISchedulerLayer {
	public:
		enum LatencyOptimizationStrategy {
			REVERSE_LINEAR,
			LINEAR,
			LINEAR_JUMP,
			LINEAR_JUMP_LOG,
			LOGARITHMIC
		};
		SATRatIIScheduler(Graph& g, ResourceModel &resourceModel, int M=-1, int S=-1);
		void scheduleIteration() override;
		void setSolverTimeout(unsigned int newTimeoutInSec);
		void setLatencyOptimizationStrategy(const LatencyOptimizationStrategy &newLos);
		void setTargetLatency(const int &newTargetLatency);
		void setEarliestStartTimes(const std::map<Vertex*, int> &newEarliestStartTimes);
		void setLatestStartTimeDifferences(const std::map<Vertex*, int> &newLatestStartTimeDifferences);
		int linearJumpLength;

	private:
		bool computeNewLatencySuccess(const bool &lastSchedulingAttemptSuccessful);
		void calcMinLatency();
		void calcMaxLatency();
		void calculateLatestStartTimes();
		void calculateLatestStartTimeDifferences();
		void calculateEarliestStartTimes();
		void simplifyResourceLimits();
		void restoreResourceLimits();

		void calcDeltaMins();
		void initScheduler();
		void setUpSolver();
		void resetContainer();
		void createLiterals();
		void createClauses();
		void fillSolutionStructure();

		bool optimalResult;
		int candidateLatency;
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
		std::map<std::tuple<Vertex*, int, int>, int> bindingLiterals;
		std::map<std::tuple<Vertex*, int, Vertex*, int>, int> timeOverlapLiterals;
		std::map<std::tuple<Vertex*, int, Vertex*, int>, int> bindingOverlapLiterals;
		std::set<int> latencyAttempts;
		LatencyOptimizationStrategy los;
		std::map<Vertex*, int> earliestStartTime;
		std::map<Vertex*, int> latestStartTime;
		std::map<Vertex*, int> latestStartTimeDifferences;
		std::map<const Resource*, int> originalResourceLimits;

		std::vector<int> initiationIntervals;
		std::map<unsigned int,unsigned int> deltaMins;
	};
}
#endif //USE_CADICAL
#endif //HATSCHET_SATRATIISCHEDULER_H
