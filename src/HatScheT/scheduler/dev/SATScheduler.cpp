//
// Created by nfiege on 4/21/22.
//

#include "SATScheduler.h"
#include <cmath>
#include <chrono>
#include <iomanip>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/utility/Utility.h>

#ifdef USE_CADICAL
namespace HatScheT {
#define CADICAL_SAT 10

	SATScheduler::SATScheduler(HatScheT::Graph &g, HatScheT::ResourceModel &resourceModel) :
		SchedulerBase(g,resourceModel), solverTimeout(300), terminator(0.0) {
		this->quiet = false; // debugging
		this->II = -1;
		this->timeouts = 0;
		//this->startTimes.clear();
		this->scheduleFound = false;
		this->optimalResult = false;
		computeMinII(&g, &resourceModel);
		// start replace min II calculation with DFS
		/*
		this->resMinII = Utility::calcResMII(&resourceModel,nullptr);
		this->recMinII = Utility::calcRecMIIDFS(&g,&resourceModel);
		this->minII = Utility::calcMinII(this->resMinII,this->recMinII);
		 */
		// end replace min II calculation with DFS
		this->minII = ceil(this->minII);
		computeMaxII(&g, &resourceModel);

		this->solvingTime = -1.0;
		this->candidateII = -1;
		this->candidateLatency = -1;
		this->minLatency = -1;
		this->maxLatency = -1;
		this->literalCounter = -1;
		this->scheduleTimeLiteralCounter = -1;
		this->bindingLiteralCounter = -1;
		this->clauseCounter = -1;
		this->dependencyConstraintClauseCounter = -1;
		this->resourceConstraintClauseCounter = -1;
		this->scheduleTimeConstraintClauseCounter = -1;
		this->bindingConstraintClauseCounter = -1;
	}

	void SATScheduler::schedule() {
		this->initScheduler();
		for (this->candidateII = (int)this->minII; this->candidateII <= (int)this->maxII; ++this->candidateII) {
			if (!this->quiet) {
				std::cout << "SATScheduler: candidate II=" << this->candidateII << std::endl;
			}
			for (this->candidateLatency = max(this->minLatency, this->candidateII); this->candidateLatency <= this->maxLatency; this->candidateLatency++) {
				if (!this->quiet) {
					std::cout << "SATScheduler: resetting containers" << std::endl;
				}
				this->resetContainer();
				if (!this->quiet) {
					std::cout << "SATScheduler: setting up solver" << std::endl;
				}
				this->setUpSolver();
				if (!this->quiet) {
					std::cout << "SATScheduler: creating literals" << std::endl;
				}
				this->createLiterals();
				if (!this->quiet) {
					std::cout << "SATScheduler: creating clauses" << std::endl;
				}
				this->createClauses();
				if (!this->quiet) {
					std::cout << "SATScheduler: start scheduling for II=" << this->candidateII << " and latency=" << this->candidateLatency << " with '" << this->literalCounter << "' literals and '"
										<< this->clauseCounter << "' clauses" << std::endl;
					std::cout << "  '" << this->scheduleTimeLiteralCounter << "' schedule time literals" << std::endl;
					std::cout << "  '" << this->bindingLiteralCounter << "' binding literals" << std::endl;
					std::cout << "  '" << this->dependencyConstraintClauseCounter << "' dependency constraint clauses" << std::endl;
					std::cout << "  '" << this->resourceConstraintClauseCounter << "' resource constraint clauses" << std::endl;
					std::cout << "  '" << this->scheduleTimeConstraintClauseCounter << "' schedule time constraint clauses" << std::endl;
					std::cout << "  '" << this->bindingConstraintClauseCounter << "' binding constraint clauses" << std::endl;
					auto now = std::chrono::system_clock::now();
					auto inTime = std::chrono::system_clock::to_time_t(now);
					std::cout << "  current time: " << std::put_time(std::localtime(&inTime), "%Y-%m-%d %X") << std::endl;
				}
				auto elapsedTime = this->terminator.getElapsedTime();
				if (!this->quiet) {
					std::cout << "SATScheduler: time is " << elapsedTime << "sec after constructing the problem" << std::endl;
				}
				this->solvingTime += elapsedTime;
				if (elapsedTime >= this->solverTimeout) {
					// timeout after problem construction!
					if (!this->quiet) {
						std::cout << "SATScheduler: encountered timeout after " << elapsedTime << "sec (time budget was " << this->solverTimeout << "sec) - total elapsed time: " << this->solvingTime << "sec" << std::endl;
					}
					this->timeouts++;
					break;
				}
				// start solving
				auto stat = this->solver->solve();
				elapsedTime = this->terminator.getElapsedTime();
				this->solvingTime += elapsedTime;
				if (!this->quiet) {
					std::cout << "SATScheduler: finished solving with status '" <<
					  (stat == CADICAL_SAT?"SAT":"UNSAT") << "' (code '" << stat << "') after " << elapsedTime
					  << " sec (total: " << this->solvingTime << " sec)" << std::endl;
				}
				if(stat != CADICAL_SAT) {
					// check if it was due to a timeout
					if (elapsedTime >= this->solverTimeout) {
						// timeout when solving
						if (!this->quiet) {
							std::cout << "SATScheduler: encountered timeout after " << elapsedTime << "sec (time budget was " << this->solverTimeout << "sec)" << std::endl;
						}
						this->timeouts++;
						break;
					}
					// schedule attempt failed :(
					// let's try again for the next latency :)
					if (!this->quiet) {
						std::cout << "SATScheduler: latency " << this->candidateLatency << " is infeasible - trying again with modified latency limit" << std::endl;
					}
					continue;
				}
				this->scheduleFound = true;
				this->optimalResult = true;
				this->II = this->candidateII;
				this->fillSolutionStructure();
				break;
			}
			if (this->scheduleFound) {
				// schedule attempt finished :)
				// stop trying
				break;
			}
		}
	}

	void SATScheduler::calcMinLatency() {
		// unconstrained ASAP scheduler for lower limit on achievable latency
		ResourceModel asapRm;
		for (auto &r : this->resourceModel.Resources()) {
			auto &asapR = asapRm.makeResource(r->getName(), UNLIMITED, r->getLatency(), r->getBlockingTime());
			for (auto &v : this->resourceModel.getVerticesOfResource(r)) {
				asapRm.registerVertex(v, &asapR);
			}
		}
		ASAPScheduler asapScheduler(this->g, this->resourceModel);
		asapScheduler.schedule();
		this->minLatency = asapScheduler.getScheduleLength();
		if (!this->quiet) {
			std::cout << "SATScheduler: computed minimum latency " << this->minLatency << std::endl;
		}
	}

	void SATScheduler::calcMaxLatency() {
		// use upper limit from Equation (6) in:
		// [1] J. Oppermann, M. Reuter-Oppermann, L. Sommer, A. Koch, and O. Sinnen,
		// ‘Exact and Practical Modulo Scheduling for High-Level Synthesis’,
		// ACM Transactions on Reconfigurable Technology and Systems, vol. 12, no. 2, p. 26.
		this->maxLatency = 0;
		for (auto &v : this->g.Vertices()) {
			this->maxLatency += this->resourceModel.getVertexLatency(v);
		}
		for (auto &r : this->resourceModel.Resources()) {
			if (r->isUnlimited()) continue;
			auto numRegistrations = this->resourceModel.getNumVerticesRegisteredToResource(r);
			auto limit = (float)r->getLimit();
			for (int i=0; i<numRegistrations; i++) {
				this->maxLatency += (int)floor(((float)i) / limit);
			}
		}
		if (!this->quiet) {
			std::cout << "SATScheduler: computed maximum latency " << this->maxLatency << std::endl;
		}
	}

	void SATScheduler::initScheduler() {
		if (!this->quiet) {
			std::cout << "SATScheduler: start initializing scheduler" << std::endl;
		}
		// solution info
		this->scheduleFound = false;
		this->optimalResult = false;
		// latency bounds
		this->calcMinLatency();
		this->calcMaxLatency();
		if (this->maxLatencyConstraint < 0 or this->maxLatency < this->maxLatencyConstraint) {
			this->maxLatencyConstraint = this->maxLatency;
		}
		// II bounds
		if(this->maxRuns > 0) {
			int runs = (int)(this->maxII - this->minII)+1;
			if(runs > this->maxRuns) {
				this->maxII = ((int)this->minII + this->maxRuns - 1);
			}
			if(!this->quiet) {
				std::cout << "SATScheduler: maxII changed due to maxRuns value set by user!" << endl;
				std::cout << "SATScheduler: min/maxII = " << this->minII << " " << this->maxII << std::endl;
			}
		}
		if (this->minII > this->maxII) {
			throw Exception("Inconsistent II bounds");
		}
		for (auto &v : this->g.Vertices()) {
			auto r = this->resourceModel.getResource(v);
			this->vertexIsUnlimited[v] = r->isUnlimited();
			if (r->isUnlimited()) {
				this->resourceLimit[v] = (int)this->resourceModel.getNumVerticesRegisteredToResource(r);
			}
			else {
				this->resourceLimit[v] = r->getLimit();
			}
		}
		// time stuff
		this->timeouts = 0;
		this->solvingTime = 0.0;
	}

	void SATScheduler::resetContainer() {
		// just create a new solver lol
		//this->solver = CaDiCaL::Solver(); // nfiege: that creates segmentation faults!! (why tho?!)
		//this->solver = std::make_unique<CaDiCaL::Solver>();
		this->solver = std::unique_ptr<CaDiCaL::Solver>(new CaDiCaL::Solver);
		this->scheduleTimeLiterals.clear();
		this->bindingLiterals.clear();
		this->literalCounter = 0;
		this->scheduleTimeLiteralCounter = 0;
		this->bindingLiteralCounter = 0;
		this->clauseCounter = 0;
		this->dependencyConstraintClauseCounter = 0;
		this->resourceConstraintClauseCounter = 0;
		this->scheduleTimeConstraintClauseCounter = 0;
		this->bindingConstraintClauseCounter = 0;
		this->terminator = CaDiCalTerminator((double)this->solverTimeout);
	}

	void SATScheduler::createClauses() {
		// dependency constraints
		if (!this->quiet) {
			std::cout << "SATScheduler: creating dependency constraints" << std::endl;
		}
		for (auto &e : this->g.Edges()) {
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			if (!this->quiet) {
				std::cout << "SATScheduler: creating dependency constraint for edge '" << vSrc->getName() << "' -> '" << vDst->getName() << "'" << std::endl;
			}
			auto lSrc = this->resourceModel.getVertexLatency(vSrc);
			auto lDst = this->resourceModel.getVertexLatency(vDst);
			auto distance = e->getDistance();
			auto delay = e->getDelay();
			for (int tau1=0; tau1 <= this->candidateLatency - lSrc; tau1++) {
				for (int tau2=0; tau2 <= this->candidateLatency - lDst; tau2++) {
					if (tau2 + distance * this->candidateII - tau1 - lSrc - delay >= 0) {
						// dependency not violated
						continue;
					}
					// dependency violated!
					// both literals must not be 1 at the same time!
					this->solver->add(-this->scheduleTimeLiterals.at({vSrc, tau1}));
					this->solver->add(-this->scheduleTimeLiterals.at({vDst, tau2}));
					this->solver->add(0);
					this->dependencyConstraintClauseCounter++;
				}
			}
		}
		// resource constraints
		if (!this->quiet) {
			std::cout << "SATScheduler: creating resource constraints" << std::endl;
		}
		for (auto &r : this->resourceModel.Resources()) {
			auto limit = r->getLimit();
			if (limit == UNLIMITED) continue;
			//auto limit = r->isUnlimited()?(int)this->resourceModel.getNumVerticesRegisteredToResource(r):r->getLimit();
			auto lat = r->getLatency();
			for (int l=0; l<limit; l++) {
				if (!this->quiet) {
					std::cout << "SATScheduler: creating resource constraints for resource '" << r->getName() << "' instance '" << l << "'" << std::endl;
				}
				for (auto &v1 : this->g.Vertices()) {
					if (this->resourceModel.getResource(v1) != r) continue;
					auto b1 = this->bindingLiterals.at({v1, l});
					for (auto &v2 : this->g.Vertices()) {
						if (this->resourceModel.getResource(v2) != r) continue;
						if (v1->getId() >= v2->getId()) continue;
						auto b2 = this->bindingLiterals.at({v2, l});
						for (int x=0; x<this->candidateII; x++) {
							for (int tau1=0; tau1<= this->candidateLatency - lat; tau1++) {
								if (tau1 % this->candidateII != x) continue;
								auto t1 = this->scheduleTimeLiterals.at({v1, tau1});
								for (int tau2=0; tau2<= this->candidateLatency - lat; tau2++) {
									if (tau2 % this->candidateII != x) continue;
									auto t2 = this->scheduleTimeLiterals.at({v2, tau2});
									this->solver->add(-t1);
									this->solver->add(-t2);
									this->solver->add(-b1);
									this->solver->add(-b2);
									this->solver->add(0);
									this->resourceConstraintClauseCounter++;
								}
							}
						}
					}
				}
			}
		}
		// ensure exactly 1 schedule time and exactly 1 binding
		if (!this->quiet) {
			std::cout << "SATScheduler: creating schedule time and binding constraints" << std::endl;
		}
		for (auto &v : this->g.Vertices()) {
			if (!this->quiet) {
				std::cout << "SATScheduler: creating schedule time constraint for vertex '" << v->getName() << "'" << std::endl;
			}
			// schedule time
			auto lat = this->resourceModel.getVertexLatency(v);
			// all zero clause
			for (int tau=0; tau<= this->candidateLatency - lat; tau++) {
				this->solver->add(this->scheduleTimeLiterals.at({v, tau}));
			}
			this->solver->add(0);
			this->scheduleTimeConstraintClauseCounter++;
			for (int tau1=0; tau1<= this->candidateLatency - lat; tau1++) {
				auto t1 = this->scheduleTimeLiterals.at({v, tau1});
				for (int tau2=tau1+1; tau2<= this->candidateLatency - lat; tau2++) {
					this->solver->add(-t1);
					this->solver->add(-this->scheduleTimeLiterals.at({v, tau2}));
					this->solver->add(0);
					this->scheduleTimeConstraintClauseCounter++;
				}
			}
			if (!this->quiet) {
				std::cout << "SATScheduler: creating binding constraint for vertex '" << v->getName() << "'" << std::endl;
			}
			// binding
			if (this->vertexIsUnlimited.at(v)) continue;
			// all zero clause
			for (int l=0; l<this->resourceLimit.at(v); l++) {
				this->solver->add(this->bindingLiterals.at({v, l}));
			}
			this->solver->add(0);
			this->bindingConstraintClauseCounter++;
			for (int l1=0; l1<this->resourceLimit.at(v); l1++) {
				auto b1 = this->bindingLiterals.at({v, l1});
				for (int l2=l1+1; l2<this->resourceLimit.at(v); l2++) {
					this->solver->add(-b1);
					this->solver->add(-this->bindingLiterals.at({v, l2}));
					this->solver->add(0);
					this->bindingConstraintClauseCounter++;
				}
			}
		}
		this->clauseCounter = this->dependencyConstraintClauseCounter + this->resourceConstraintClauseCounter +
			this->scheduleTimeConstraintClauseCounter + this->bindingConstraintClauseCounter;
	}

	void SATScheduler::fillSolutionStructure() {
		// print solution
		if (!this->quiet) {
			std::cout << "SATScheduler: CaDiCaL solution: " << std::endl;
			for (auto &v : this->g.Vertices()) {
				std::cout << "  vertex " << v->getName() << std::endl;
				// times
				for (int tau=0; tau<= this->candidateLatency - this->resourceModel.getVertexLatency(v); tau++) {
					std::cout << "    t=" << tau << " - " << this->solver->val(this->scheduleTimeLiterals.at({v, tau})) << std::endl;
				}
				if (this->vertexIsUnlimited.at(v)) {
					std::cout << "    unlimited" << std::endl;
					continue;
				}
				// bindings
				for (int l=0; l<this->resourceLimit.at(v); l++) {
					std::cout << "    FU=" << l << " - " << this->solver->val(this->bindingLiterals.at({v, l})) << std::endl;
				}
			}
		}
		std::map<const Resource*, int> unlimitedResourceCounter;
		for (auto &v : this->g.Vertices()) {
			// schedule time
			auto t = -1;
			for (int tau=0; tau<= this->candidateLatency - this->resourceModel.getVertexLatency(v); tau++) {
				if (this->solver->val(this->scheduleTimeLiterals.at({v, tau})) < 0) {
					continue;
				}
				if (t >= 0) {
					throw Exception("Determined multiple start times ("+std::to_string(tau)+" and "+std::to_string(t)+") for vertex '"+v->getName()+"' - that should never happen!");
				}
				t = tau;
			}
			if (t < 0) {
				throw Exception("Failed to find start time for vertex '"+v->getName()+"' - that should never happen!");
			}
			this->startTimes[v] = t;
			// binding
			if (this->vertexIsUnlimited.at(v)) {
				// assign unlimited vertices unique FUs
				auto *r = this->resourceModel.getResource(v);
				this->binding[v] = unlimitedResourceCounter[r]++; // assign FU and increment counter
				continue;
			}
			auto b = -1;
			for (int l=0; l<this->resourceLimit.at(v); l++) {
				if (this->solver->val(this->bindingLiterals.at({v, l})) < 0) {
					continue;
				}
				if (b >= 0) {
					throw Exception("Determined multiple bindings ("+std::to_string(l)+" and "+std::to_string(b)+") for vertex '"+v->getName()+"' - that should never happen!");
				}
				b = l;
			}
			if (b < 0) {
				throw Exception("Failed to find binding for vertex '"+v->getName()+"' - that should never happen!");
			}
			this->binding[v] = b;
		}
	}

	void SATScheduler::setUpSolver() {
		// attach terminator to support timeout
		this->solver->connect_terminator(&this->terminator);
	}

	void SATScheduler::createLiterals() {
		for (auto &v : this->g.Vertices()) {
			// schedule time variables
			for (int tau=0; tau<= this->candidateLatency - this->resourceModel.getVertexLatency(v); tau++) {
				this->scheduleTimeLiteralCounter++;
				this->scheduleTimeLiterals[{v, tau}] = ++this->literalCounter;
			}
			// binding variables
			if (this->vertexIsUnlimited.at(v)) continue;
			for (int l=0; l<this->resourceLimit.at(v); l++) {
				this->bindingLiteralCounter++;
				this->bindingLiterals[{v, l}] = ++this->literalCounter;
			}
		}
	}

	void SATScheduler::setSolverTimeout(unsigned int newTimeoutInSec) {
		this->solverTimeout = newTimeoutInSec;
	}

	CaDiCalTerminator::CaDiCalTerminator(double timeout)
		: maxTime(timeout), timerStart(std::chrono::steady_clock::now()) {}

	bool CaDiCalTerminator::terminate() {
		return this->getElapsedTime() >= this->maxTime;
	}

	void CaDiCalTerminator::reset(double newTimeout) {
		this->timerStart = std::chrono::steady_clock::now();
		this->maxTime = newTimeout;
	}

	double CaDiCalTerminator::getElapsedTime() const {
		return chrono::duration_cast<chrono::seconds>(std::chrono::steady_clock::now() - this->timerStart).count();
	}
}
#endif
