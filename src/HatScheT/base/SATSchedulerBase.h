//
// Created by bkessler on 11/8/22.
//

#ifndef HATSCHET_SATSCHEDULERBASE_H
#define HATSCHET_SATSCHEDULERBASE_H

#ifdef USE_CADICAL

#include <HatScheT/layers/IterativeModuloSchedulerLayer.h>
#include <HatScheT/scheduler/satbased/CaDiCaLTerminator.h>
#include <cadical.hpp>
#include <memory>

namespace HatScheT {

	class SATSchedulerBase : public IterativeModuloSchedulerLayer {
	public:
		enum LatencyOptimizationStrategy {
			REVERSE_LINEAR,
			LINEAR,
			LINEAR_JUMP,
			LINEAR_JUMP_LOG,
			LOGARITHMIC,
			NO_LATENCY_OPTIMIZATION
		};

		SATSchedulerBase(Graph& g, ResourceModel &resourceModel, int II=-1);
		void setLatencyOptimizationStrategy(const LatencyOptimizationStrategy &newLos);
		void setTargetLatency(const int &newTargetLatency);
		void setEarliestStartTimes(const std::map<Vertex*, int> &newEarliestStartTimes);
		void setLatestStartTimeDifferences(const std::map<Vertex*, int> &newLatestStartTimeDifferences);
		int linearJumpLength;
		std::string getName() override { return "SATSchedulerBase"; }

	protected:
		/*
		 * methods
		 */
		// override base functions
		void scheduleIteration() override;
		void scheduleInit() override;
		void scheduleCleanup() override;

		// computing next target latency -> return false if search space is exhausted
		bool computeNewLatencySuccess(const bool &lastSchedulingAttemptSuccessful);

		// simplify resource limits if possible
		void simplifyResourceLimits();
		void restoreResourceLimits();

		/*
		 * members
		 */
		// solver related
		std::unique_ptr<CaDiCaL::Solver> solver;
		CaDiCaLTerminator terminator;
		int literalCounter = 0;
		int clauseCounter = 0;

		// restrict search space for each vertex separately
		std::map<Vertex*, int> earliestStartTime;
		std::map<Vertex*, int> latestStartTime;
		std::map<Vertex*, int> latestStartTimeDifferences;

		// resource-related
		std::map<const Resource*, int> originalResourceLimits;
		std::map<Vertex*, int> resourceLimit;
		std::map<Vertex*, bool> vertexIsUnlimited;

		// latency related
		std::set<int> latencyAttempts;
		LatencyOptimizationStrategy los;
		int latencyLowerBound;
		int latencyUpperBound;
		bool enableIIBasedLatencyLowerBound = true;
		int candidateII = -1;
		int candidateLatency = -1;
		int minLatency = -1;
		int maxLatency = -1;
		//bool minLatencyUserDef = false;
		//bool maxLatencyUserDef = false;
		bool targetSLUserDef = false;

	}; // class SATSchedulerBase
}

#endif //USE_CADICAL
#endif //HATSCHET_SATSCHEDULERBASE_H
