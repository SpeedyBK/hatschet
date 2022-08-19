//
// Created by nfiege on 4/21/22.
//

#include "SATSchedulerLatOpt.h"
#include <cmath>
#include <chrono>
#include <iomanip>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/scheduler/ALAPScheduler.h>
#include <HatScheT/utility/Utility.h>

#ifdef USE_CADICAL
namespace HatScheT {
#define NEW_RESOURCE_CONSTRAINTS 1

	SATSchedulerLatOpt::SATSchedulerLatOpt(HatScheT::Graph &g, HatScheT::ResourceModel &resourceModel, int II)
		: SchedulerBase(g,resourceModel), solverTimeout(300), terminator(0.0),
			los(LatencyOptimizationStrategy::LINEAR_JUMP_LOG), linearJumpLength(-1), latencyLowerBound(-1),
			latencyUpperBound(-1), enableIIBasedLatencyLowerBound(true) {
		this->II = -1;
		this->timeouts = 0;
		this->scheduleFound = false;
		this->optimalResult = false;
		if (II >= 0) {
			this->minII = II;
			this->maxII = II;
			this->maxRuns = 1;
		}
		else {
			computeMinII(&g, &resourceModel);
			this->minII = ceil(this->minII);
			computeMaxII(&g, &resourceModel);
		}

		this->solvingTime = -1.0;
		this->candidateII = -1;
		this->candidateLatency = -1;
		this->minLatency = -1;
		this->maxLatency = -1;
		this->literalCounter = -1;
		this->scheduleTimeLiteralCounter = -1;
		this->bindingLiteralCounter = -1;
		this->timeOverlapLiteralCounter = -1;
		this->bindingOverlapLiteralCounter = -1;
		this->clauseCounter = -1;
		this->dependencyConstraintClauseCounter = -1;
		this->resourceConstraintClauseCounter = -1;
		this->scheduleTimeConstraintClauseCounter = -1;
		this->bindingConstraintClauseCounter = -1;
		this->timeOverlapClauseCounter = -1;
		this->bindingOverlapClauseCounter = -1;
	}

	void SATSchedulerLatOpt::schedule() {
		this->initScheduler();
		for (this->candidateII = (int)this->minII; this->candidateII <= (int)this->maxII; ++this->candidateII) {
			this->defineLatLimits();
			//if (!this->quiet) {
			auto currentTime1 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			std::cerr << "SATSchedulerLatOpt: trying candidate II=" << this->candidateII << " at time " << 	std::put_time(std::localtime(&currentTime1), "%Y-%m-%d %X") << std::endl;
			//}

			this->latencyAttempts.clear();
			this->candidateLatency = -1;
			this->latencyLowerBound = this->minLatency;
			if (this->enableIIBasedLatencyLowerBound) {
				// loop pipelining only makes sense if the latency of the whole graph is larger than the II...
				this->latencyLowerBound = max(this->latencyLowerBound, this->candidateII);
			}
			this->latencyUpperBound = this->maxLatency;
			bool lastAttemptSuccess = false;
			bool breakByTimeout = false;
			double elapsedTime = 0.0;
			this->terminator = CaDiCaLTerminator((double)this->solverTimeout);
			if (!this->quiet) {
				std::cout << "SATSchedulerLatOpt: candidate II=" << this->candidateII << " with min latency="
									<< this->latencyLowerBound << " and max latency=" << this->latencyUpperBound << std::endl;
			}
			while (this->computeNewLatencySuccess(lastAttemptSuccess)) {
				if (!this->quiet) {
					std::cout << "SATSchedulerLatOpt: new latency limit = " << this->candidateLatency << std::endl;
					std::cout << "SATSchedulerLatOpt: resetting containers" << std::endl;
				}
				this->resetContainer();
				if (!this->quiet) {
					std::cout << "SATSchedulerLatOpt: setting up solver" << std::endl;
				}
				this->setUpSolver();
				if (!this->quiet) {
					std::cout << "SATSchedulerLatOpt: creating literals" << std::endl;
				}
				this->createLiterals();
				if (!this->quiet) {
					std::cout << "SATSchedulerLatOpt: creating clauses" << std::endl;
				}
				this->createClauses();
				elapsedTime = this->terminator.getElapsedTime();
				if (!this->quiet) {
					std::cout << "SATSchedulerLatOpt: time is " << elapsedTime << "sec after constructing the problem" << std::endl;
				}
				if (this->terminator.terminate()) {
					// timeout during problem construction :(
					if (!this->quiet) {
						std::cout << "SATSchedulerLatOpt: encountered timeout during problem construction after " << elapsedTime << "sec (time budget was " << this->solverTimeout << "sec) - total elapsed time: " << this->solvingTime << "sec" << std::endl;
					}
					breakByTimeout = true;
					break;
				}
				if (!this->quiet) {
					std::cout << "SATSchedulerLatOpt: start scheduling for II=" << this->candidateII << " and latency=" << this->candidateLatency << " with '" << this->literalCounter << "' literals and '"
										<< this->clauseCounter << "' clauses" << std::endl;
					std::cout << "  '" << this->scheduleTimeLiteralCounter << "' schedule time literals" << std::endl;
					std::cout << "  '" << this->bindingLiteralCounter << "' binding literals" << std::endl;
					std::cout << "  '" << this->timeOverlapLiteralCounter << "' time overlap literals" << std::endl;
					std::cout << "  '" << this->bindingOverlapLiteralCounter << "' binding overlap literals" << std::endl;
					std::cout << "  '" << this->dependencyConstraintClauseCounter << "' dependency constraint clauses" << std::endl;
					std::cout << "  '" << this->resourceConstraintClauseCounter << "' resource constraint clauses" << std::endl;
					std::cout << "  '" << this->scheduleTimeConstraintClauseCounter << "' schedule time constraint clauses" << std::endl;
					std::cout << "  '" << this->bindingConstraintClauseCounter << "' binding constraint clauses" << std::endl;
					std::cout << "  '" << this->timeOverlapClauseCounter << "' time overlap clauses" << std::endl;
					std::cout << "  '" << this->bindingOverlapClauseCounter << "' binding overlap clauses" << std::endl;
					auto currentTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
					std::cout << "  current time: " << std::put_time(std::localtime(&currentTime2), "%Y-%m-%d %X") << std::endl;
				}
				if (elapsedTime >= this->solverTimeout) {
					// timeout after problem construction!
					if (!this->quiet) {
						std::cout << "SATSchedulerLatOpt: encountered timeout after " << elapsedTime << "sec (time budget was " << this->solverTimeout << "sec) - total elapsed time: " << this->solvingTime << "sec" << std::endl;
					}
					breakByTimeout = true;
					break;
				}
				// start solving
				//if (!this->quiet) {
				auto currentTime3 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				std::cerr << "SATSchedulerLatOpt: start solving at time " << 	std::put_time(std::localtime(&currentTime3), "%Y-%m-%d %X") << std::endl;
				//}
				auto stat = this->solver->solve();
				elapsedTime = this->terminator.getElapsedTime();
				lastAttemptSuccess = stat == CADICAL_SAT;
				if (!this->quiet) {
					std::cout << "SATSchedulerLatOpt: finished solving with status '" <<
										(lastAttemptSuccess?"SAT":"UNSAT") << "' (code '" << stat << "') after " << elapsedTime
										<< " sec (total: " << this->solvingTime << " sec)" << std::endl;
				}
				if(!lastAttemptSuccess) {
					//if (!this->quiet) {
					auto currentTime4 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
					std::cerr << "SATSchedulerLatOpt: failed to find solution for II=" << this->candidateII << " and SL=" << this->candidateLatency << " at " << std::put_time(std::localtime(&currentTime4), "%Y-%m-%d %X") << std::endl;
					//}
					// check if it was due to a timeout
					if (elapsedTime >= this->solverTimeout) {
						// timeout when solving
						if (!this->quiet) {
							std::cout << "SATSchedulerLatOpt: encountered timeout after " << elapsedTime << "sec (time budget was " << this->solverTimeout << "sec)" << std::endl;
						}
						breakByTimeout = true;
						break;
					}
					// schedule attempt failed :(
					// let's try again for the next latency :)
					if (!this->quiet) {
						std::cout << "SATSchedulerLatOpt: latency " << this->candidateLatency << " is infeasible - trying again with modified latency limit" << std::endl;
					}
					continue;
				}
				this->scheduleFound = true;
				this->II = this->candidateII;
				this->fillSolutionStructure();
				//if (!this->quiet) {
				auto currentTime5 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				std::cerr << "SATSchedulerLatOpt: found solution for II=" << this->candidateII << " and SL=" << this->candidateLatency << " at " << std::put_time(std::localtime(&currentTime5), "%Y-%m-%d %X") << std::endl;
				//}
			}
			this->solvingTime += elapsedTime;
			this->optimalResult = !breakByTimeout;
			if (breakByTimeout) {
				this->timeouts++;
				this->secondObjectiveOptimal = false;
				if (!this->scheduleFound) {
					this->firstObjectiveOptimal = false;
				}
			}
			if (this->scheduleFound) {
				// schedule attempt finished :)
				// stop trying
				break;
			}
		}
		this->restoreResourceLimits();
	}

	void SATSchedulerLatOpt::calcMinLatency() {
		// unconstrained ASAP scheduler for lower limit on achievable latency
		std::map<const Resource*, int> originalLimits;
		for (auto &r : this->resourceModel.Resources()) {
			originalLimits[r] = r->getLimit();
			r->setLimit(UNLIMITED, false); // disable errors during set limit function
		}
		ASAPScheduler asapScheduler(this->g, this->resourceModel);
		asapScheduler.schedule();
		this->minLatency = asapScheduler.getScheduleLength();
		if (!this->quiet) {
			std::cout << "SATSchedulerLatOpt: computed minimum latency " << this->minLatency << std::endl;
		}
		for (auto &r : this->resourceModel.Resources()) {
			r->setLimit(originalLimits.at(r));
		}
	}

	void SATSchedulerLatOpt::calcMaxLatency() {
		// use upper limit from Equation (6) in:
		// [1] J. Oppermann, M. Reuter-Oppermann, L. Sommer, A. Koch, and O. Sinnen,
		// ‘Exact and Practical Modulo Scheduling for High-Level Synthesis’,
		// ACM Transactions on Reconfigurable Technology and Systems, vol. 12, no. 2, p. 26.
		this->maxLatency = 0;
		for (auto &v : this->g.Vertices()) {
			int maxChainingDelay = 0;
			for (auto &e : this->g.Edges()) {
				if (&e->getVertexSrc() != v) continue;
				auto d = e->getDelay();
				if (d > maxChainingDelay) maxChainingDelay = d;
			}
			this->maxLatency += (this->resourceModel.getVertexLatency(v) + maxChainingDelay);
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
			std::cout << "SATSchedulerLatOpt: computed maximum latency " << this->maxLatency << std::endl;
		}
	}

	void SATSchedulerLatOpt::initScheduler() {
		if (!this->quiet) {
			std::cout << "SATSchedulerLatOpt: start initializing scheduler" << std::endl;
		}
		// solution info
		this->scheduleFound = false;
		this->optimalResult = false;
		this->firstObjectiveOptimal = true;
		this->secondObjectiveOptimal = true;
		// II bounds
		if(this->maxRuns > 0) {
			int runs = (int)(this->maxII - this->minII)+1;
			if(runs > this->maxRuns) {
				this->maxII = ((int)this->minII + this->maxRuns - 1);
			}
			if(!this->quiet) {
				std::cout << "SATSchedulerLatOpt: maxII changed due to maxRuns value set by user!" << endl;
				std::cout << "SATSchedulerLatOpt: min/maxII = " << this->minII << " " << this->maxII << std::endl;
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
		// simplify resource limits to save variables/clauses
		this->simplifyResourceLimits();
	}

	void SATSchedulerLatOpt::resetContainer() {
		// just create a new solver lol
		this->solver = std::unique_ptr<CaDiCaL::Solver>(new CaDiCaL::Solver);
		this->scheduleTimeLiterals.clear();
		this->bindingLiterals.clear();
		this->literalCounter = 0;
		this->scheduleTimeLiteralCounter = 0;
		this->bindingLiteralCounter = 0;
		this->timeOverlapLiteralCounter = 0;
		this->bindingOverlapLiteralCounter = 0;
		this->clauseCounter = 0;
		this->dependencyConstraintClauseCounter = 0;
		this->resourceConstraintClauseCounter = 0;
		this->scheduleTimeConstraintClauseCounter = 0;
		this->bindingConstraintClauseCounter = 0;
		this->timeOverlapClauseCounter = 0;
		this->bindingOverlapClauseCounter = 0;
		this->calculateLatestStartTimes();
	}

	void SATSchedulerLatOpt::createClauses() {
		// dependency constraints
		if (!this->quiet) {
			std::cout << "SATSchedulerLatOpt: creating dependency constraints" << std::endl;
		}
		for (auto &e : this->g.Edges()) {
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			if (!this->quiet) {
				std::cout << "SATSchedulerLatOpt: creating dependency constraint for edge '" << vSrc->getName() << "' -> '" << vDst->getName() << "'" << std::endl;
			}
			auto lSrc = this->resourceModel.getVertexLatency(vSrc);
			auto distance = e->getDistance();
			auto delay = e->getDelay();
			for (int tau1=this->earliestStartTime.at(vSrc); tau1 <= this->latestStartTime.at(vSrc); tau1++) {
				for (int tau2=this->earliestStartTime.at(vDst); tau2 <= this->latestStartTime.at(vDst); tau2++) {
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
				// check timeout
				if (this->terminator.terminate()) {
					return;
				}
			}
		}
		// check timeout
		if (this->terminator.terminate()) {
			return;
		}
		// resource constraints
		if (!this->quiet) {
			std::cout << "SATSchedulerLatOpt: creating resource constraints" << std::endl;
		}
#if NEW_RESOURCE_CONSTRAINTS
		for (auto &v1 : this->g.Vertices()) {
			auto limit = this->resourceModel.getResource(v1)->getLimit();
			if (limit == UNLIMITED) continue;
			auto bindingTrivial = limit == 1;
			for (auto &v2 : this->g.Vertices()) {
				if (v2->getId() <= v1->getId()) continue;
				if (this->resourceModel.getResource(v1) != this->resourceModel.getResource(v2)) continue;
				// check timeout
				if (this->terminator.terminate()) {
					return;
				}
				if (!this->quiet) {
					std::cout << "SATSchedulerLatOpt: creating resource constraint clauses for vertices '" << v1->getName() << "' and '" << v2->getName() << "'" << std::endl;
				}
				// now we got two vertices that might produce resource conflicts
				// ensure that conflicting vertices are either scheduled or bound differently
				this->solver->add(this->timeOverlapLiterals.at({v1, v2}));
				if (!bindingTrivial) {
					this->solver->add(this->bindingOverlapLiterals.at({v1, v2}));
				}
				this->solver->add(0);
				this->resourceConstraintClauseCounter++;
				// ensure that time overlap literals are set correctly
				for (auto tau1 = this->earliestStartTime.at(v1); tau1 <= this->latestStartTime.at(v1); tau1++) {
					// check timeout
					if (this->terminator.terminate()) {
						return;
					}
					for (auto tau2 = this->earliestStartTime.at(v2); tau2 <= this->latestStartTime.at(v2); tau2++) {
						if (tau1 % this->candidateII != tau2 % this->candidateII) continue;
						this->solver->add(-this->timeOverlapLiterals.at({v1, v2}));
						this->solver->add(-this->scheduleTimeLiterals.at({v1, tau1}));
						this->solver->add(-this->scheduleTimeLiterals.at({v2, tau2}));
						this->solver->add(0);
						this->timeOverlapClauseCounter++;
					}
				}
				// ensure that binding overlap literals are set correctly
				if (bindingTrivial) continue;
				for (auto k=0; k<limit; k++) {
					this->solver->add(-this->bindingOverlapLiterals.at({v1, v2}));
					this->solver->add(-this->bindingLiterals.at({v1, k}));
					this->solver->add(-this->bindingLiterals.at({v2, k}));
					this->solver->add(0);
					this->bindingOverlapClauseCounter++;
				}
			}
		}

#else
		for (auto &r : this->resourceModel.Resources()) {
			auto limit = r->getLimit();
			auto bindingTrivial = limit == 1;
			if (limit == UNLIMITED) continue;
			for (int l=0; l<limit; l++) {
				if (!this->quiet) {
					std::cout << "SATSchedulerLatOpt: creating resource constraints for resource '" << r->getName() << "' instance '" << l << "'" << std::endl;
				}
				for (auto &v1 : this->g.Vertices()) {
					if (this->resourceModel.getResource(v1) != r) continue;
					auto b1 = -1;
					if (!bindingTrivial) {
						b1 = this->bindingLiterals.at({v1, l});
					}
					// check timeout
					if (this->terminator.terminate()) {
						return;
					}
					// iterate over conflicting vertices
					for (auto &v2 : this->g.Vertices()) {
						if (this->resourceModel.getResource(v2) != r) continue;
						if (v1->getId() >= v2->getId()) continue;
						auto b2 = -1;
						if (!bindingTrivial) {
							b2 = this->bindingLiterals.at({v2, l});
						}
						for (int x=0; x<this->candidateII; x++) {
							for (int tau1=this->earliestStartTime.at(v1); tau1<= this->latestStartTime.at(v1); tau1++) {
								if (tau1 % this->candidateII != x) continue;
								auto t1 = this->scheduleTimeLiterals.at({v1, tau1});
								for (int tau2=this->earliestStartTime.at(v2); tau2<= this->latestStartTime.at(v2); tau2++) {
									if (tau2 % this->candidateII != x) continue;
									auto t2 = this->scheduleTimeLiterals.at({v2, tau2});
									this->solver->add(-t1);
									this->solver->add(-t2);
									if (!bindingTrivial) this->solver->add(-b1);
									if (!bindingTrivial) this->solver->add(-b2);
									this->solver->add(0);
									this->resourceConstraintClauseCounter++;
								}
							}
						}
					}
				}
			}
		}
#endif
		// ensure exactly/at least 1 schedule time and exactly/at least 1 binding
		if (!this->quiet) {
			std::cout << "SATSchedulerLatOpt: creating schedule time and binding constraints" << std::endl;
		}
		for (auto &v : this->g.Vertices()) {
			if (!this->quiet) {
				std::cout << "SATSchedulerLatOpt: creating schedule time constraint for vertex '" << v->getName() << "'" << std::endl;
			}
			// schedule time
			for (int tau=this->earliestStartTime.at(v); tau<= this->latestStartTime.at(v); tau++) {
				this->solver->add(this->scheduleTimeLiterals.at({v, tau}));
			}
			this->solver->add(0);
			this->scheduleTimeConstraintClauseCounter++;
			if (!this->quiet) {
				std::cout << "SATSchedulerLatOpt: creating binding constraint for vertex '" << v->getName() << "'" << std::endl;
			}
			// binding
			if (this->vertexIsUnlimited.at(v) or this->resourceLimit.at(v) == 1) continue;
			for (int l=0; l<this->resourceLimit.at(v); l++) {
				this->solver->add(this->bindingLiterals.at({v, l}));
			}
			this->solver->add(0);
			this->bindingConstraintClauseCounter++;
		}
		this->clauseCounter = this->dependencyConstraintClauseCounter + this->resourceConstraintClauseCounter +
													this->scheduleTimeConstraintClauseCounter + this->bindingConstraintClauseCounter + this->timeOverlapClauseCounter
													+ this->bindingConstraintClauseCounter;
	}

	void SATSchedulerLatOpt::fillSolutionStructure() {
		// print solution
		if (!this->quiet) {
			std::cout << "SATSchedulerLatOpt: CaDiCaL solution: " << std::endl;
			for (auto &v : this->g.Vertices()) {
				std::cout << "  vertex " << v->getName() << " (latency=" << this->resourceModel.getVertexLatency(v) << ")" << std::endl;
				// times
				for (int tau=this->earliestStartTime.at(v); tau<= this->latestStartTime.at(v); tau++) {
					std::cout << "    t=" << tau << " - " << this->solver->val(this->scheduleTimeLiterals.at({v, tau})) << std::endl;
				}
				if (this->vertexIsUnlimited.at(v)) {
					std::cout << "    unlimited" << std::endl;
					continue;
				}
				if (this->resourceLimit.at(v) == 1) {
					std::cout << "    FU=0 - 1 (trivial)" << std::endl;
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
			for (int tau=this->earliestStartTime.at(v); tau<= this->latestStartTime.at(v); tau++) {
				if (this->solver->val(this->scheduleTimeLiterals.at({v, tau})) < 0) {
					continue;
				}
				t = tau;
				break;
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
			if (this->resourceLimit.at(v) == 1) {
				// assign trivial bindings
				this->binding[v] = 0;
				continue;
			}
			auto b = -1;
			for (int l=0; l<this->resourceLimit.at(v); l++) {
				if (this->solver->val(this->bindingLiterals.at({v, l})) < 0) {
					continue;
				}
				b = l;
				break;
			}
			if (b < 0) {
				throw Exception("Failed to find binding for vertex '"+v->getName()+"' - that should never happen!");
			}
			this->binding[v] = b;
		}
		// override candidate latency in case the scheduler found a solution with a schedule length
		// which is smaller than the given candidate latency (unlikely I guess, but who knows...)
		auto actualScheduleLength = this->getScheduleLength();
		if (actualScheduleLength > this->candidateLatency) {
			// print vertices that violate the requested limit
			for (auto &it : this->startTimes) {
				if (it.second + this->resourceModel.getVertexLatency(it.first) > this->candidateLatency) {
					// start time is too late!
					std::cout << "SATSchedulerLatOpt: vertex '" << it.first->getName() << "' (t='" << it.second << "', lat='" << this->resourceModel.getVertexLatency(it.first) << "') violates requested schedule length of '" << this->candidateLatency << "'" << std::endl;
				}
			}
			throw Exception("SATSchedulerLatOpt: Found invalid schedule! Requested candidate schedule length was '"+std::to_string(this->candidateLatency)+"' but scheduler found schedule length '"+std::to_string(actualScheduleLength)+"'");
		}
		this->candidateLatency = actualScheduleLength;
	}

	void SATSchedulerLatOpt::setUpSolver() {
		// attach terminator to support timeout
		this->solver->connect_terminator(&this->terminator);
	}

	void SATSchedulerLatOpt::createLiterals() {
		for (auto &v : this->g.Vertices()) {
			// schedule time variables
			for (int tau=this->earliestStartTime.at(v); tau<= this->latestStartTime.at(v); tau++) {
				this->scheduleTimeLiteralCounter++;
				this->scheduleTimeLiterals[{v, tau}] = ++this->literalCounter;
			}
			// binding variables
			if (this->vertexIsUnlimited.at(v) or this->resourceLimit.at(v)==1) continue;
			for (int l=0; l<this->resourceLimit.at(v); l++) {
				this->bindingLiteralCounter++;
				this->bindingLiterals[{v, l}] = ++this->literalCounter;
			}
		}
#if NEW_RESOURCE_CONSTRAINTS
		for (auto &v1 : this->g.Vertices()) {
			auto limit = this->resourceModel.getResource(v1)->getLimit();
			if (limit == UNLIMITED) continue;
			auto bindingTrivial = limit == 1;
			for (auto &v2 : this->g.Vertices()) {
				if (v2->getId() <= v1->getId()) continue;
				if (this->resourceModel.getResource(v1) != this->resourceModel.getResource(v2)) continue;
				this->timeOverlapLiteralCounter++;
				this->timeOverlapLiterals[{v1, v2}] = ++this->literalCounter;
				if (bindingTrivial) continue;
				this->bindingOverlapLiteralCounter++;
				this->bindingOverlapLiterals[{v1, v2}] = ++this->literalCounter;
			}
		}
#endif
	}

	void SATSchedulerLatOpt::setSolverTimeout(unsigned int newTimeoutInSec) {
		this->solverTimeout = newTimeoutInSec;
	}

	bool SATSchedulerLatOpt::computeNewLatencySuccess(const bool &lastSchedulingAttemptSuccessful) {
		if (!this->quiet) {
			std::cout << "SATSchedulerLatOpt: computing new latency for last candidate latency=" << this->candidateLatency
								<< ", min latency=" << this->latencyLowerBound << ", max latency=" << this->latencyUpperBound
								<< " and last attempt success=" << lastSchedulingAttemptSuccessful << std::endl;
		}
		// set a new value for the candidate latency
		switch (this->los) {
			case REVERSE_LINEAR: {
				if (this->candidateLatency < 0) {
					// first attempt: try maximum latency
					this->candidateLatency = this->latencyUpperBound;
					return true;
				}
				else if (lastSchedulingAttemptSuccessful) {
					// last scheduling attempt was a success
					if (this->candidateLatency <= this->latencyLowerBound) {
						// and we reached the lower bound
						// -> we found the optimum
						return true;
					}
					else {
						// and we did not yet reach the lower bound
						// -> try again with the next latency...
						this->candidateLatency--;
						return true;
					}
				}
				else {
					// last scheduling attempt was a fail
					// -> II is either infeasible or we found the optimum
					return false;
				}
			}
			case LINEAR: {
				if (this->candidateLatency < 0) {
					// first attempt: try minimum latency
					this->candidateLatency = this->latencyLowerBound;
					return true;
				}
				else if (lastSchedulingAttemptSuccessful) {
					// last scheduling attempt was a success
					// -> we found the optimum
					return false;
				}
				else if (this->candidateLatency < this->latencyUpperBound) {
					// last scheduling attempt was a fail
					// and we did not reach the upper bound
					// -> try again with the next latency...
					this->candidateLatency++;
					return true;
				}
				else {
					// last scheduling attempt was a fail
					// and we reached the upper bound
					// -> II is infeasible
					return false;
				}
			}
			case LINEAR_JUMP: {
				if (this->candidateLatency < 0) {
					// first attempt: try the first jump after the minimum latency
					// because in practice, the minimum latency is only very rarely achievable
					this->candidateLatency = this->latencyLowerBound + this->linearJumpLength;
					if (this->candidateLatency > this->latencyUpperBound) this->candidateLatency = this->latencyUpperBound;
				}
				else if (lastSchedulingAttemptSuccessful) {
					// check if we found the optimum
					if (this->candidateLatency <= this->latencyLowerBound) return false;
					// we already got a valid solution
					// adjust upper bound
					this->latencyUpperBound = this->candidateLatency;
					// decrease latency to find optimum
					this->candidateLatency--;
				}
				else {
					// check if we found the optimum
					if (this->scheduleFound) return false;
					// check if II is infeasible
					if (this->candidateLatency >= this->latencyUpperBound) return false;
					// looks like we are still searching for a valid solution
					// adjust lower bound
					this->latencyLowerBound = this->candidateLatency+1;
					this->candidateLatency += this->linearJumpLength;
					if (this->candidateLatency > this->latencyUpperBound) this->candidateLatency = this->latencyUpperBound;
				}
				return true;
			}
			case LINEAR_JUMP_LOG: {
				// this is the same as the linear jump one, but we are using a logarithmic search
				// once we have found a solution (i.e., a "good" upper bound on the latency)
				if (this->candidateLatency < 0) {
					// first attempt: try minimum latency
					this->candidateLatency = this->latencyLowerBound + this->linearJumpLength;
					return true;
				}
				else if (this->scheduleFound) {
					// we got a solution -> do binary search to find optimum
					if (lastSchedulingAttemptSuccessful) {
						// last scheduling attempt was a success
						this->latencyUpperBound = this->candidateLatency;
						// -> floor(mean(lower bound, last latency))
						this->candidateLatency = floor(((double)this->candidateLatency + (double)this->latencyLowerBound) / 2.0);
					}
					else {
						// last scheduling attempt was a fail
						this->latencyLowerBound = this->candidateLatency+1;
						// -> ceil(mean(upper bound, last latency))
						this->candidateLatency = ceil(((double)this->candidateLatency + (double)this->latencyUpperBound) / 2.0);
					}
					auto successPair = this->latencyAttempts.insert(this->candidateLatency);
					return successPair.second;
				}
				else {
					// we do not have a solution, yet
					// check if II is infeasible
					if (this->candidateLatency >= this->latencyUpperBound) return false;
					// looks like we are still searching for a valid solution
					// -> keep jumping, baby
					// adjust lower bound
					this->latencyLowerBound = this->candidateLatency+1;
					this->candidateLatency += this->linearJumpLength;
					if (this->candidateLatency > this->latencyUpperBound) this->candidateLatency = this->latencyUpperBound;
					return true;
				}
			}
			case LOGARITHMIC: {
				if (this->candidateLatency < 0) {
					// first attempt: try average between min and max latency rounded down
					this->candidateLatency = (this->latencyLowerBound + this->latencyUpperBound) / 2;
				}
				else if (lastSchedulingAttemptSuccessful) {
					// last scheduling attempt was a success
					this->latencyUpperBound = this->candidateLatency;
					// -> floor(mean(lower bound, last latency))
					this->candidateLatency = floor(((double)this->candidateLatency + (double)this->latencyLowerBound) / 2.0);
				}
				else {
					// last scheduling attempt was a fail
					this->latencyLowerBound = this->candidateLatency;
					// -> ceil(mean(upper bound, last latency))
					this->candidateLatency = ceil(((double)this->candidateLatency + (double)this->latencyUpperBound) / 2.0);
				}
				auto successPair = this->latencyAttempts.insert(this->candidateLatency);
				return successPair.second;
			}
		}
		// something went wrong ... ABORT!
		return false;
	}

	void SATSchedulerLatOpt::setLatencyOptimizationStrategy(const SATSchedulerLatOpt::LatencyOptimizationStrategy &newLos) {
		this->los = newLos;
	}

	void SATSchedulerLatOpt::calculateLatestStartTimes() {
		for (auto &v : this->g.Vertices()) {
			this->latestStartTime[v] = this->candidateLatency - this->latestStartTimeDifferences.at(v);
		}
	}

	void SATSchedulerLatOpt::calculateLatestStartTimeDifferences() {

		// check if latest start time differences were set by user
		bool foundAll = true;
		for (auto &v : this->g.Vertices()) {
			if (this->latestStartTimeDifferences.find(v) == this->latestStartTimeDifferences.end()) {
				foundAll = false;
				break;
			}
		}
		if (foundAll) {
			return;
		}

		// use ALAP scheduler without resource constraints to calc latest start times
		std::map<const Resource*, int> originalLimits;
		for (auto &r : this->resourceModel.Resources()) {
			originalLimits[r] = r->getLimit();
			r->setLimit(UNLIMITED, false); // disable errors during set limit function
		}
		ALAPScheduler alapScheduler(this->g, this->resourceModel);
		alapScheduler.schedule();
		if (!alapScheduler.getScheduleFound()) {
			throw Exception("SATSchedulerLatOpt: failed to compute latest start times - that should never happen");
		}
		auto alapSL = alapScheduler.getScheduleLength();
		auto alapStartTimes = alapScheduler.getSchedule();
		if (!this->quiet) {
			std::cout << "SATSchedulerLatOpt: ALAP schedule length = " << alapSL << std::endl;
		}
		for (auto &v : this->g.Vertices()) {
			try {
				auto lstd = this->latestStartTimeDifferences.at(v);
				if (!this->quiet) {
					std::cout << "SATSchedulerLatOpt: vertex '" << v->getName() << "': user max time diff = '"
										<< lstd << "', ALAP max time diff = '" << alapSL - alapStartTimes.at(v) << "'"
										<< std::endl;
				}
				this->latestStartTimeDifferences[v] = std::min(lstd ,alapSL - alapStartTimes.at(v));
			}
			catch (std::out_of_range&) {
				this->latestStartTimeDifferences[v] = alapSL - alapStartTimes.at(v);
			}
		}
		// set resource limits back to original values
		for (auto &r : this->resourceModel.Resources()) {
			r->setLimit(originalLimits.at(r));
		}
	}

	void SATSchedulerLatOpt::calculateEarliestStartTimes() {

		// check if earliest start times were set by user
		bool foundAll = true;
		for (auto &v : this->g.Vertices()) {
			if (this->earliestStartTime.find(v) == this->earliestStartTime.end()) {
				foundAll = false;
				break;
			}
		}
		if (foundAll) {
			return;
		}

		// use ASAP scheduler without resource constraints for lower bounds on start times
		std::map<const Resource*, int> originalLimits;
		for (auto &r : this->resourceModel.Resources()) {
			originalLimits[r] = r->getLimit();
			r->setLimit(UNLIMITED, false); // disable errors during set limit function
		}
		ASAPScheduler asapScheduler(this->g, this->resourceModel);
		asapScheduler.schedule();
		if (!asapScheduler.getScheduleFound()) {
			throw Exception("SATSchedulerLatOpt: failed to compute earliest start times - that should never happen");
		}
		auto asapStartTimes = asapScheduler.getSchedule();
		for (auto &v : this->g.Vertices()) {
			try {
				// check if earliest start time was set by user
				this->earliestStartTime[v] = max(asapStartTimes.at(v), this->earliestStartTime.at(v));
			}
			catch (std::out_of_range&) {
				// just use the calculated one by the ASAP scheduler
				this->earliestStartTime[v] = asapStartTimes.at(v);
			}
		}
		// set resource limits back to original values
		for (auto &r : this->resourceModel.Resources()) {
			r->setLimit(originalLimits.at(r));
		}
	}

	void SATSchedulerLatOpt::setTargetLatency(const int &newTargetLatency) {
		this->minLatency = newTargetLatency;
		this->maxLatency = newTargetLatency;
		this->enableIIBasedLatencyLowerBound = false;
	}

	void SATSchedulerLatOpt::setEarliestStartTimes(const map<Vertex *, int> &newEarliestStartTimes) {
		bool foundAll = true;
		for (auto &v : this->g.Vertices()) {
			if (newEarliestStartTimes.find(v) == newEarliestStartTimes.end()) {
				std::cout << "SATSchedulerLatOpt::setEarliestStartTimes: Warning: failed to find earliest start time for vertex '" << v->getName() << "' -> skipping assignment!" << std::endl;
				foundAll = false;
			}
		}
		if (!foundAll) return;
		this->earliestStartTime = newEarliestStartTimes;
	}

	void SATSchedulerLatOpt::setLatestStartTimeDifferences(const map<Vertex *, int> &newLatestStartTimeDifferences) {
		bool foundAll = true;
		for (auto &v : this->g.Vertices()) {
			if (newLatestStartTimeDifferences.find(v) == newLatestStartTimeDifferences.end()) {
				std::cout << "SATSchedulerLatOpt::setLatestStartTimeDifferences: Warning: failed to find earliest start time for vertex '" << v->getName() << "' -> skipping assignment!" << std::endl;
				foundAll = false;
			}
		}
		if (!foundAll) return;
		this->latestStartTimeDifferences = newLatestStartTimeDifferences;
	}

	void SATSchedulerLatOpt::simplifyResourceLimits() {
		for (auto &r : this->resourceModel.Resources()) {
			auto limit = r->getLimit();
			if (limit == UNLIMITED) continue; // skip unlimited resources because this is the dream scenario anyways
			auto numVertices = this->resourceModel.getNumVerticesRegisteredToResource(r);
			if (numVertices <= limit) {
				// save original limit to restore it later
				this->originalResourceLimits[r] = limit;
				// resource limit can be ignored
				r->setLimit(UNLIMITED, false);
			}
		}
	}

	void SATSchedulerLatOpt::restoreResourceLimits() {
		for (auto &r : this->resourceModel.Resources()) {
			if (this->originalResourceLimits.find(r) == this->originalResourceLimits.end()) continue; // limit was not changed
			auto lim = this->originalResourceLimits.at(r);
			r->setLimit(lim, false);
		}
	}

	void SATSchedulerLatOpt::defineLatLimits() {
		if (this->minLatency >= 0 and this->maxLatency >= 0) {
			if (!this->quiet) {
				std::cout << "SATSchedulerLatOpt: min & max latency defined by user: " << this->minLatency << " & " << this->maxLatency << std::endl;
			}
			// limits were defined by the user
			// only set min and max times based on SDC schedule
			auto result = Utility::getSDCAsapAndAlapTimes(&this->g, &this->resourceModel, this->candidateII, this->quiet);
			this->earliestStartTime = result.first;
			int sdcScheduleLength = 0;
			for (auto &v : this->g.Vertices()) {
				auto t = result.second.at(v) + this->resourceModel.getVertexLatency(v);
				if (t > sdcScheduleLength) sdcScheduleLength = t;
			}
			for (auto &v : this->g.Vertices()) {
				this->latestStartTimeDifferences[v] = sdcScheduleLength - result.second.at(v);
			}
		}
		else if (this->maxLatency >= 0 and this->minLatency < 0) {
			// only max latency defined by the user
			// set min and max times and also min latency
			if (!this->quiet) {
				std::cout << "SATSchedulerLatOpt: max latency defined by user: " << this->maxLatency << std::endl;
			}
			auto result = Utility::getLatencyEstimation(&this->g, &this->resourceModel, this->candidateII, Utility::latencyBounds::minLatency, this->quiet);
			this->minLatency = result.minLat;
			this->earliestStartTime = result.asapStartTimes;
			int sdcScheduleLength = 0;
			for (auto &v : this->g.Vertices()) {
				auto t = result.alapStartTimes.at(v) + this->resourceModel.getVertexLatency(v);
				if (t > sdcScheduleLength) sdcScheduleLength = t;
			}
			for (auto &v : this->g.Vertices()) {
				this->latestStartTimeDifferences[v] = sdcScheduleLength - result.alapStartTimes.at(v);
			}
		}
		else if (this->minLatency >= 0 and this->maxLatency < 0) {
			// only min latency defined by the user
			// set min and max times and also max latency
			if (!this->quiet) {
				std::cout << "SATSchedulerLatOpt: min latency defined by user: " << this->minLatency << std::endl;
			}
			auto result = Utility::getLatencyEstimation(&this->g, &this->resourceModel, this->candidateII, Utility::latencyBounds::maxLatency, this->quiet);
			this->maxLatency = result.maxLat;
			this->earliestStartTime = result.asapStartTimes;
			int sdcScheduleLength = 0;
			for (auto &v : this->g.Vertices()) {
				auto t = result.alapStartTimes.at(v) + this->resourceModel.getVertexLatency(v);
				if (t > sdcScheduleLength) sdcScheduleLength = t;
			}
			for (auto &v : this->g.Vertices()) {
				this->latestStartTimeDifferences[v] = sdcScheduleLength - result.alapStartTimes.at(v);
			}
		}
		else {
			// nothing requested by the user
			// set everything
			// i.e. min and max times and also min and max latencies
			if (!this->quiet) {
				std::cout << "SATSchedulerLatOpt: no latency defined by user" << std::endl;
			}
			auto result = Utility::getLatencyEstimation(&this->g, &this->resourceModel, this->candidateII, Utility::latencyBounds::both, this->quiet);
			this->minLatency = result.minLat;
			this->maxLatency = result.maxLat;
			this->earliestStartTime = result.asapStartTimes;
			int sdcScheduleLength = 0;
			for (auto &v : this->g.Vertices()) {
				auto t = result.alapStartTimes.at(v) + this->resourceModel.getVertexLatency(v);
				if (t > sdcScheduleLength) sdcScheduleLength = t;
			}
			for (auto &v : this->g.Vertices()) {
				this->latestStartTimeDifferences[v] = sdcScheduleLength - result.alapStartTimes.at(v);
			}
		}
		// jump length for latency minimization strategy
		if (this->linearJumpLength <= 0) {
			this->linearJumpLength = (int)ceil(sqrt(this->maxLatency - this->minLatency));
		}
		// handle max latency constraint
		if (this->maxLatencyConstraint >= 0 and this->maxLatency > this->maxLatencyConstraint) {
			this->maxLatency = this->maxLatencyConstraint;
		}
		if (!this->quiet) {
			std::cout << "SATSchedulerLatOpt: latency limits: " << this->minLatency << " <= L <= " << this->maxLatency << std::endl;
			// sort vertices by name
			std::set<std::string> vertexNames;
			for (auto &v : this->g.Vertices()) {
				vertexNames.insert(v->getName());
			}
			// print sorted times
			std::cout << "SATSchedulerLatOpt: printing all start time limits" << std::endl;
			for (auto &vn : vertexNames) {
				auto *v = &this->g.getVertexByName(vn);
				std::cout << "  " << v->getName() << " earliest: " << this->earliestStartTime.at(v) << ", latest diff: "
									<< this->latestStartTimeDifferences.at(v) << std::endl;
			}
		}
	}
}
#endif
