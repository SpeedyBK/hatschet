//
// Created by nfiege on 8/26/22.
//

#ifndef HATSCHET_SATSCHEDULERBINENC_H
#define HATSCHET_SATSCHEDULERBINENC_H
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

	class SATSchedulerBinEnc : public SchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase {
	public:
		SATSchedulerBinEnc(Graph& g, ResourceModel &resourceModel, int II=-1);
		double getSolvingTime() const { return this->solvingTime; }
		void schedule() override;
		void setSolverTimeout(unsigned int newTimeoutInSec);
		void setTargetLatency(const int &newTargetLatency);
		void setEarliestStartTimes(const std::map<Vertex*, int> &newEarliestStartTimes);
		void setLatestStartTimeDifferences(const std::map<Vertex*, int> &newLatestStartTimeDifferences);

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
		void defineLatLimits();
		void defineEarliestStartTimeOffsets();
		void setUpSolver();
		void resetContainer();
		void createLiterals();
		void createBaseClauses();
		void createAdditionalClauses();
		void fillSolutionStructure();

		void getNumberRepr(Vertex* v, int t, bool negate, std::vector<int>* literals);
		void getNumberRepr(std::map<std::pair<Vertex*, int>, int>* container, Vertex* v, const int &numDigits, const int &num, const int &globalMultiplier, std::vector<int>* literals);

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
		int unlimitedScheduleTimeLiteralCounter;
		int moduloSlotLiteralCounter;
		int timeOffsetLiteralCounter;
		int bindingLiteralCounter;
		//int timeOverlapLiteralCounter;
		//int bindingOverlapLiteralCounter;
		int clauseCounter;
		int dependencyConstraintClauseCounter;
		int resourceConstraintClauseCounter;
		int scheduleTimeConstraintClauseCounter;
		int bindingConstraintClauseCounter;
		int scheduleTimeInRangeClauseCounter;
		//int timeOverlapClauseCounter;
		//int bindingOverlapClauseCounter;
		std::map<Vertex*, int> resourceLimit;
		std::map<Vertex*, bool> vertexIsUnlimited;
		std::map<std::pair<Vertex*, int>, int> unlimitedScheduleTimeLiterals; // t_i for unlimited vertices
		std::map<std::pair<Vertex*, int>, int> moduloSlotLiterals; // m_i for limited vertices
		std::map<std::pair<Vertex*, int>, int> timeOffsetLiterals; // y_i for limited vertices
		std::map<std::pair<Vertex*, int>, int> bindingLiterals; // b_i for limited vertices
		//std::map<std::pair<Vertex*, Vertex*>, int> timeOverlapLiterals; // T_ij for pairs of limited vertices of the same resource type
		//std::map<std::pair<Vertex*, Vertex*>, int> bindingOverlapLiterals; // B_ij for pairs of limited vertices of the same resource type
		std::map<Vertex*, int> greatestExpressibleNumber; // greatest start time expressible with the given variables
		std::map<Vertex*, int> earliestStartTime; // tMin_i
		std::map<Vertex*, int> earliestStartTimeOffsets; // C_i
		std::map<Vertex*, int> numUnlimitedScheduleTimeVariables; // # t_i (unlimited)
		std::map<Vertex*, int> numModuloSlotVariables; // # m_i (limited)
		std::map<Vertex*, int> numTimeOffsetVariables; // # y_i (limited)
		std::map<Vertex*, int> numBindingVariables; // # b_i (limited)
		std::map<Vertex*, int> latestStartTime; // tMax_i
		std::map<Vertex*, int> lastLatestStartTime; // tMax_i for the last candidate latency
		std::map<Vertex*, int> latestStartTimeDifferences; // L - tMax_i = const
		std::map<const Resource*, int> originalResourceLimits;
	};
}
#endif //HATSCHET_SATSCHEDULERBINENC_H
