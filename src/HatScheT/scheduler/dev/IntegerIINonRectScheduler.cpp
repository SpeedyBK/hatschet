//
// Created by nfiege on 7/8/21.
//

#include "IntegerIINonRectScheduler.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/Exception.h"

#include <iostream>
#include <map>
#include <cmath>

namespace HatScheT {
	IntegerIINonRectScheduler::IntegerIINonRectScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist) : SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist) {
		// reset previous solutions
		this->II = -1;
		this->candII = -1;
		this->timeouts = 0;
		this->startTimes.clear();
		this->scheduleFound = false;
		this->optimalResult = true;
	}

	void IntegerIINonRectScheduler::schedule()
	{
		if(this->candII <= 0) {
			throw HatScheT::Exception("Need to set candidate II before scheduling attempt");
		}
		if(this->quiet==false){
			std::cout << "IntIINonRect: min/maxII = " << minII << " " << maxII << ", (minResII/minRecII " << this->resMinII << " / " << this->recMinII << ")" << std::endl;
			std::cout << "IntIINonRect: solver timeout = " << this->solverTimeout << " (sec)" << endl;
		}

		// iterative modulo scheduling does not make sense with this scheduler
		// because resource limits must be set according to the candidate II
		bool feasible = false;
		bool proven = false;
		scheduleAttempt(candII, feasible, proven);
		scheduleFound |= feasible;
		optimalResult &= proven;
		if (feasible) {
			II = candII;
			auto solution = solver->getResult().values;
			for (auto *i : g.Vertices())
				startTimes[i] = (int) std::lround(solution.find(t[i])->second);

			if(this->quiet==false) {
				std::cout << "IntIINonRect: found " << (optimalResult ? "optimal" : "feasible") << " solution with II=" << II << std::endl;
				for(auto it : this->startTimes) {
					std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
				}
			}
		}
		if(!feasible) if(this->quiet==false) cout << "  II" << candII << " : " << this->stat << endl;

		if(scheduleFound == false) this->II = -1;
		if(this->quiet==false) std::cout << "IntIINonRect: solving time was " << this->timeUsed << " seconds" << std::endl;
	}

	void IntegerIINonRectScheduler::setUpSolverSettings()
	{
		this->solver->quiet   = this->solverQuiet;
		this->solver->timeout = this->solverTimeout;
		this->solver->threads = this->threads;
	}

	void IntegerIINonRectScheduler::scheduleAttempt(int candII, bool &feasible, bool &proven)
	{
		solver->reset();
		solver->timeout = this->getSolverTimeout();

		constructDecisionVariables(candII);
		setObjective();
		constructConstraints(candII);
		setUpSolverSettings();

		if(this->quiet==false) {
			std::cout << "IntIINonRect: attempt II=" << candII << std::endl;
			std::cout << "IntIINonRect: solver timeout = " << this->solver->timeout << " (sec)" << endl;
		}

//		//timestamp
//		this->begin = clock();
//		//solve
//		this->stat = this->solver->solve();
//		//timestamp
//		this->end = clock();
//
//		//log time
//		if(this->solvingTime == -1.0) this->solvingTime = 0.0;
//		this->solvingTime += (double)(this->end - this->begin) / CLOCKS_PER_SEC;

		//timestamp
		startTimeTracking();
		//solve
		this->stat = this->solver->solve();
		//timestamp
		endTimeTracking();

		if(stat == ScaLP::status::TIMEOUT_INFEASIBLE) this->timeouts++;
		feasible = stat == ScaLP::status::OPTIMAL | stat == ScaLP::status::FEASIBLE   | stat == ScaLP::status::TIMEOUT_FEASIBLE;
		proven   = stat == ScaLP::status::OPTIMAL | stat == ScaLP::status::INFEASIBLE;
	}

	void IntegerIINonRectScheduler::constructDecisionVariables(int candII)
	{
		t.clear();
		b.clear(); b.resize(candII);
		k.clear();

		for (auto *i : g.Vertices()) {
			auto id = "_" + std::to_string(i->getId());
			// (1)
			for (int r = 0; r < candII; ++r) b[r][i] = ScaLP::newBinaryVariable("a_" + std::to_string(r) + id);

			// (2)
			k[i]    = ScaLP::newIntegerVariable("k" + id);
			t[i] = ScaLP::newIntegerVariable("t" + id);

			solver->addConstraint(k[i] >= 0);
			solver->addConstraint(t[i] >= 0);
		}
	}

	void IntegerIINonRectScheduler::setObjective()
	{
		// currently only one objective: minimise the schedule length
		ScaLP::Variable ss = ScaLP::newIntegerVariable("supersink");
		solver->addConstraint(ss >= 0);
		if (maxLatencyConstraint >= 0)
			solver->addConstraint(ss <= maxLatencyConstraint);

		for (auto *v : g.Vertices()) {
			solver->addConstraint(ss - t[v] >= resourceModel.getVertexLatency(v));
		}
		solver->setObjective(ScaLP::minimize(ss));
	}

	static inline int mod(int a, int b) {
		int m = a % b;
		return m >= 0 ? m : m + b;
	}

	void IntegerIINonRectScheduler::constructConstraints(int candII)
	{
		for (auto *i : g.Vertices()) {
			// anchor source vertices, but only if they are not resource-limited
			if (this->g.isSourceVertex(i) && this->resourceModel.getResource(i)->getLimit() == UNLIMITED) {
				this->solver->addConstraint(this->k[i] == 0);
				this->solver->addConstraint(this->b[0][i] == 1);
			}

			// bind result variables (2)
			ScaLP::Term sumBind;
			for (int r = 0; r < candII; ++r) {
				sumBind.add(this->b[r][i], r);
			}
			this->solver->addConstraint(this->t[i] - (this->k[i] * candII) - sumBind == 0);

			// assignment constraints (1)
			ScaLP::Term sumAssign;
			for (int r = 0; r < candII; ++r) {
				sumAssign.add(this->b[r][i], 1);
			}
			this->solver->addConstraint(sumAssign == 1);
		}

		// resource constraints (5)
		// this could be extended to general reservation tables
		for (auto r : this->resourceModel.Resources()) {
			if (r->isReservationTable()) {
				throw HatScheT::Exception("IntIINonRect formulation currently handles only simple resources");
			}
			if (r->getBlockingTime() != 1) {
				throw HatScheT::Exception("IntIINonRect formulation currently handles only fully pipelined resources");
			}
			auto vertices = resourceModel.getVerticesOfResource(r);
			for (auto congruenceClass = 0; congruenceClass < candII; congruenceClass++) {
				// skip unlimited resources
				if (r->getNonRectLimit(congruenceClass) == UNLIMITED) {
					continue;
				}
				ScaLP::Term sumRes;
				for (auto *v : vertices) {
					sumRes += this->b[congruenceClass][v];
				}
				this->solver->addConstraint(sumRes <= r->getNonRectLimit(congruenceClass));
			}
		}

		// normal dependency constraints (NOT 0-1-structured because it's easier at the moment)
		for (auto *e : g.Edges()) {
			auto vSrc = &e->getVertexSrc();
			auto vDst = &e->getVertexDst();
			auto rSrc = this->resourceModel.getResource(vSrc);
			auto latSrc = rSrc->getLatency();
			auto distance = e->getDistance();
			auto delay = e->getDelay();
			auto tSrc = this->t[vSrc];
			auto tDst = this->t[vDst];
			this->solver->addConstraint(tSrc + latSrc + delay - tDst <= candII * distance);
		}
	}
}