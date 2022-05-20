//
// Created by nfiege on 5/3/22.
//

#ifndef HATSCHET_SATMINREGSCHEDULER_H
#define HATSCHET_SATMINREGSCHEDULER_H
#ifdef USE_CADICAL
#include <cadical.hpp>
#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <string>
#include <map>
#include <utility>
#include <memory>
#include <chrono>
#include <HatScheT/scheduler/satbased/SATScheduler.h>
#ifdef USE_SCALP
#include <ScaLP/Solver.h>
#endif // USE_SCALP

namespace HatScheT {

	class SATMinRegScheduler : public SchedulerBase, public ModuloSchedulerBase {
	public:
		enum RegisterOptimizationStrategy {
			LINEAR,
			SQRT,
			LOGARITHMIC
		};
		SATMinRegScheduler(Graph& g, ResourceModel &resourceModel);
		void schedule() override;
		void setSolverTimeout(unsigned int newTimeoutInSec);
		int getNumRegs() const;
		void setRegMax(const int& newRegMax);
		void setSolverWishlist(const std::list<std::string>& newSolverWishlist);
		void setLatencyOptimizationStrategy(const RegisterOptimizationStrategy &newRos);

	private:
		bool computeNewNumRegistersSuccess(const bool &lastSchedulingAttemptSuccessful);
		void initScheduler();
		void setUpSolver();
		void resetContainer();
		void createLiterals();
		void createClauses();
		void fillSolutionStructure();
		void calculateLatestStartTimes();
		void calculateEarliestStartTimes();
#ifdef USE_SCALP
		int calculateEarliestStartTime(Vertex* v, ScaLP::Solver *s, const std::map<Vertex*, ScaLP::Variable> &vars);
#endif

		bool optimalResult;
		int candidateNumRegs;
		int numRegs;
		int regMax;
		int registerUpperBound;
		int registerLowerBound;

		unsigned int solverTimeout;

		double solvingTime;
		std::unique_ptr<CaDiCaL::Solver> solver;
		CaDiCalTerminator terminator;

		RegisterOptimizationStrategy ros;
		bool skipFirstRegAttempt;
		int sqrtJumpLength;
		std::set<int> registerAttempts;

		int literalCounter;
		int scheduleTimeLiteralCounter;
		int bindingLiteralCounter;
		int variableLiteralCounter;
		int registerBindingLiteralCounter;
		int clauseCounter;
		int dependencyConstraintClauseCounter;
		int resourceConstraintClauseCounter;
		int scheduleTimeConstraintClauseCounter;
		int bindingConstraintClauseCounter;
		int variableConstraintClauseCounter;
		int registerBindingConstraintClauseCounter;
		int registerOverlapConstraintClauseCounter;
		std::map<Vertex*, int> resourceLimit;
		std::map<Vertex*, bool> vertexIsUnlimited;
		std::map<std::pair<Vertex*, int>, int> scheduleTimeLiterals;
		std::map<std::pair<Vertex*, int>, int> bindingLiterals;
		std::map<std::pair<Vertex*, int>, int> variableLiterals;
		std::map<std::tuple<Vertex*, int, int>, int> registerBindingLiterals;
		std::map<Vertex*, std::set<int>> startTimesContainer;
		std::map<Vertex*, bool> hasOutgoingEdges;
		std::map<Vertex*, int> latestVariableReadTime;
		std::map<Vertex*, int> earliestStartTime;
		std::map<Vertex*, int> latestStartTime;
		std::list<std::string> sdcSolverWishlist;
	};
}
#endif //USE_CADICAL
#endif //HATSCHET_SATMINREGSCHEDULER_H
