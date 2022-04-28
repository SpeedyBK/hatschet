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
#include <chrono>

namespace HatScheT {

	class CaDiCalTerminator : public CaDiCaL::Terminator {
	public:
		explicit CaDiCalTerminator(double timeout);
		bool terminate () override;
		void reset(double newTimeout);
		double getElapsedTime() const;
	private:
		double maxTime;
		std::chrono::steady_clock::time_point timerStart;
	};

	class SATScheduler : public SchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase {
	public:
		SATScheduler(Graph& g, ResourceModel &resourceModel);
		void schedule() override;
		void setSolverTimeout(unsigned int newTimeoutInSec);

	private:
		void calcMinLatency();
		void calcMaxLatency();

		void initScheduler();
		void setUpSolver();
		void resetContainer();
		void createLiterals();
		void createClauses();
		void fillSolutionStructure();

		bool optimalResult;
		int candidateII;
		int candidateLatency;
		int minLatency;
		int maxLatency;

		unsigned int solverTimeout;

		double solvingTime;
		std::unique_ptr<CaDiCaL::Solver> solver;
		CaDiCalTerminator terminator;

		int literalCounter;
		int scheduleTimeLiteralCounter;
		int bindingLiteralCounter;
		int clauseCounter;
		int dependencyConstraintClauseCounter;
		int resourceConstraintClauseCounter;
		int scheduleTimeConstraintClauseCounter;
		int bindingConstraintClauseCounter;
		std::map<Vertex*, int> resourceLimit;
		std::map<Vertex*, bool> vertexIsUnlimited;
		std::map<std::pair<Vertex*, int>, int> scheduleTimeLiterals;
		std::map<std::pair<Vertex*, int>, int> bindingLiterals;
	};
}
#endif //USE_CADICAL
#endif //HATSCHET_SATSCHEDULER_H
