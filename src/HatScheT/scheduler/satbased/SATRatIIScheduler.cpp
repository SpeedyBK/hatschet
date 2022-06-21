//
// Created by nfiege on 6/3/22.
//

#include "SATRatIIScheduler.h"
#include <cmath>
#include <chrono>
#include <iomanip>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/scheduler/ALAPScheduler.h>

#ifdef USE_CADICAL
namespace HatScheT {
#define CADICAL_SAT 10

	SATRatIIScheduler::SATRatIIScheduler(HatScheT::Graph &g, HatScheT::ResourceModel &resourceModel, int M, int S)
		: RationalIISchedulerLayer(g,resourceModel, M, S), solverTimeout(300), terminator(0.0),
			los(LatencyOptimizationStrategy::LINEAR_JUMP), linearJumpLength(-1), latencyLowerBound(-1),
			latencyUpperBound(-1) {
		this->timeouts = 0;
		this->scheduleFound = false;
		this->optimalResult = false;

		this->solvingTime = -1.0;
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

	void SATRatIIScheduler::scheduleIteration() {
		this->initScheduler();
		//if (!this->quiet) {
		auto currentTime1 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
		std::cerr << "SATRatIIScheduler: trying candidate II=" << this->modulo << "/" << this->samples << " at time " << 	std::put_time(std::localtime(&currentTime1), "%Y-%m-%d %X") << std::endl;
		//}

		if (!this->quiet) {
			std::cout << "SATRatIIScheduler: candidate II=" << this->modulo << "/" << this->samples << std::endl;
		}
		this->latencyAttempts.clear();
		this->candidateLatency = -1;
		this->latencyLowerBound = this->minLatency;
		this->latencyUpperBound = this->maxLatency;
		bool lastAttemptSuccess = false;
		bool breakByTimeout = false;
		double elapsedTime = 0.0;
		this->terminator = CaDiCaLTerminator((double)this->solverTimeout);
		while (this->computeNewLatencySuccess(lastAttemptSuccess)) {
			if (!this->quiet) {
				std::cout << "SATRatIIScheduler: resetting containers" << std::endl;
			}
			this->resetContainer();
			if (!this->quiet) {
				std::cout << "SATRatIIScheduler: setting up solver" << std::endl;
			}
			this->setUpSolver();
			if (!this->quiet) {
				std::cout << "SATRatIIScheduler: creating literals" << std::endl;
			}
			this->createLiterals();
			if (!this->quiet) {
				std::cout << "SATRatIIScheduler: creating clauses" << std::endl;
			}
			this->createClauses();
			if (!this->quiet) {
				std::cout << "SATRatIIScheduler: start scheduling for II=" << this->modulo << "/" << this->samples << " and latency=" << this->candidateLatency << " with '" << this->literalCounter << "' literals and '"
									<< this->clauseCounter << "' clauses" << std::endl;
				std::cout << "  '" << this->scheduleTimeLiteralCounter << "' schedule time literals" << std::endl;
				std::cout << "  '" << this->bindingLiteralCounter << "' binding literals" << std::endl;
				std::cout << "  '" << this->dependencyConstraintClauseCounter << "' dependency constraint clauses" << std::endl;
				std::cout << "  '" << this->resourceConstraintClauseCounter << "' resource constraint clauses" << std::endl;
				std::cout << "  '" << this->scheduleTimeConstraintClauseCounter << "' schedule time constraint clauses" << std::endl;
				std::cout << "  '" << this->bindingConstraintClauseCounter << "' binding constraint clauses" << std::endl;
				auto currentTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				std::cout << "  current time: " << std::put_time(std::localtime(&currentTime2), "%Y-%m-%d %X") << std::endl;
			}
			elapsedTime = this->terminator.getElapsedTime();
			if (!this->quiet) {
				std::cout << "SATRatIIScheduler: time is " << elapsedTime << "sec after constructing the problem" << std::endl;
			}
			if (elapsedTime >= this->solverTimeout) {
				// timeout after problem construction!
				if (!this->quiet) {
					std::cout << "SATRatIIScheduler: encountered timeout after " << elapsedTime << "sec (time budget was " << this->solverTimeout << "sec) - total elapsed time: " << this->solvingTime << "sec" << std::endl;
				}
				breakByTimeout = true;
				break;
			}
			// start solving
			//if (!this->quiet) {
			auto currentTime3 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			std::cerr << "SATRatIIScheduler: start solving at time " << 	std::put_time(std::localtime(&currentTime3), "%Y-%m-%d %X") << std::endl;
			//}
			auto stat = this->solver->solve();
			elapsedTime = this->terminator.getElapsedTime();
			lastAttemptSuccess = stat == CADICAL_SAT;
			if (!this->quiet) {
				std::cout << "SATRatIIScheduler: finished solving with status '" <<
									(lastAttemptSuccess?"SAT":"UNSAT") << "' (code '" << stat << "') after " << elapsedTime
									<< " sec (total: " << this->solvingTime << " sec)" << std::endl;
			}
			if(!lastAttemptSuccess) {
				//if (!this->quiet) {
				auto currentTime4 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				std::cerr << "SATRatIIScheduler: failed to find solution for II=" << this->modulo << "/" << this->samples << " and SL=" << this->candidateLatency << " at " << std::put_time(std::localtime(&currentTime4), "%Y-%m-%d %X") << std::endl;
				//}
				// check if it was due to a timeout
				if (elapsedTime >= this->solverTimeout) {
					// timeout when solving
					if (!this->quiet) {
						std::cout << "SATRatIIScheduler: encountered timeout after " << elapsedTime << "sec (time budget was " << this->solverTimeout << "sec)" << std::endl;
					}
					breakByTimeout = true;
					break;
				}
				// schedule attempt failed :(
				// let's try again for the next latency :)
				if (!this->quiet) {
					std::cout << "SATRatIIScheduler: latency " << this->candidateLatency << " is infeasible - trying again with modified latency limit" << std::endl;
				}
				continue;
			}
			this->scheduleFound = true;
			this->fillSolutionStructure();
			//if (!this->quiet) {
			auto currentTime5 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			std::cerr << "SATRatIIScheduler: found solution for II=" << this->modulo << "/" << this->samples << " and SL=" << this->candidateLatency << " at " << std::put_time(std::localtime(&currentTime5), "%Y-%m-%d %X") << std::endl;
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
		this->restoreResourceLimits();
	}

	void SATRatIIScheduler::calcMinLatency() {
		// check if min latency was set by user
		if (this->minLatency >= 0) return;
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
			std::cout << "SATRatIIScheduler: computed minimum latency " << this->minLatency << std::endl;
		}
		for (auto &r : this->resourceModel.Resources()) {
			r->setLimit(originalLimits.at(r));
		}
	}

	void SATRatIIScheduler::calcMaxLatency() {
		// check if max latency was set by user
		if (this->maxLatency >= 0) return;
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
			std::cout << "SATRatIIScheduler: computed maximum latency " << this->maxLatency << std::endl;
		}
	}

	void SATRatIIScheduler::initScheduler() {
		if (!this->quiet) {
			std::cout << "SATRatIIScheduler: start initializing scheduler" << std::endl;
		}
		// solution info
		this->scheduleFound = false;
		this->optimalResult = false;
		this->firstObjectiveOptimal = true;
		this->secondObjectiveOptimal = true;
		// latency bounds
		this->calcMinLatency();
		this->calcMaxLatency();
		// jump length for latency minimization strategy
		if (this->linearJumpLength <= 0) {
			this->linearJumpLength = (int)ceil(sqrt(this->maxLatency - this->minLatency));
		}
		// handle max latency constraint
		if (this->maxLatencyConstraint < 0 or this->maxLatency < this->maxLatencyConstraint) {
			this->maxLatencyConstraint = this->maxLatency;
		}
		// handle resource limitations
		if (this->resourceLimit.empty() and this->vertexIsUnlimited.empty()) {
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
		}
		//init latency sequence, init intervals, deltaMin containers
		this->initiationIntervals = RationalIISchedulerLayer::getOptimalInitiationIntervalSequence(this->samples,this->modulo,this->quiet);
		this->latencySequence = RationalIISchedulerLayer::getLatencySequenceFromInitiationIntervals(this->initiationIntervals,this->modulo);
		if (!this->quiet) {
			std::cout << "SATRatIIScheduler: initiation intervals = <";
			for (auto &it : this->initiationIntervals) {
				std::cout << " " << it;
			}
			std::cout << " >" << std::endl;
			std::cout << "SATRatIIScheduler: latency sequence = <";
			for (auto &it : this->latencySequence) {
				std::cout << " " << it;
			}
			std::cout << " >" << std::endl;
		}
		calcDeltaMins();
		// upper and lower bounds
		this->calculateEarliestStartTimes();
		this->calculateLatestStartTimeDifferences();
		if (!this->quiet) {
			std::cout << "SATRatIIScheduler: calculated earliest start times and latest start time diffs:" << std::endl;
			for (auto &v : this->g.Vertices()) {
				std::cout << "  " << v->getName() << " tMin=" << this->earliestStartTime.at(v) << ", diff=" << this->latestStartTimeDifferences.at(v) << std::endl;
			}
		}
		// simplify resource limits to save variables/clauses
		this->simplifyResourceLimits();
	}

	void SATRatIIScheduler::resetContainer() {
		// just create a new solver lol
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
		this->calculateLatestStartTimes();
	}

	void SATRatIIScheduler::calcDeltaMins() {
		if(this->initiationIntervals.empty() or this->latencySequence.empty())
			throw HatScheT::Exception("SATRatIIScheduler::calcDeltaMins: need to specify initiation intervals/latency sequence");
		if(this->samples<=0 or this->modulo<=0)
			throw HatScheT::Exception("SATRatIIScheduler::calcDeltaMins: need to specify samples and modulo");
		// distance 0 is trivial
		this->deltaMins[0] = 0;
		if(!this->quiet)
			std::cout << "set min delta (0) = " << 0 << std::endl;
		for(auto &e : this->g.Edges()) {
			auto edgeDistance = e->getDistance();
			// check if delta for this distance was already calculated
			if(edgeDistance==0) continue;
			if(this->deltaMins.find(edgeDistance) != this->deltaMins.end()) continue;
			// calc minimum delta
			unsigned int minDelta = 10000000; // 'infinity'
			for(auto offset=0; offset<this->samples; ++offset) {
				unsigned int delta = 0;
				for(auto d=0; d<edgeDistance; ++d) {
					delta += this->latencySequence[(offset+d)%this->samples];
				}
				if(delta<minDelta) minDelta = delta;
			}
			this->deltaMins[edgeDistance] = minDelta;
			if(!this->quiet)
				std::cout << "set min delta (" << edgeDistance << ") = " << this->deltaMins[edgeDistance] << std::endl;
		}
	}

	void SATRatIIScheduler::createClauses() {
		// dependency constraints
		if (!this->quiet) {
			std::cout << "SATRatIIScheduler: creating dependency constraints" << std::endl;
		}
		for (auto &e : this->g.Edges()) {
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			if (!this->quiet) {
				std::cout << "SATRatIIScheduler: creating dependency constraint for edge '" << vSrc->getName() << "' -> '" << vDst->getName() << "'" << std::endl;
			}
			auto lSrc = this->resourceModel.getVertexLatency(vSrc);
			auto distance = e->getDistance();
			auto delay = e->getDelay();
			for (int tau1=this->earliestStartTime.at(vSrc); tau1 <= this->latestStartTime.at(vSrc); tau1++) {
				for (int tau2=this->earliestStartTime.at(vDst); tau2 <= this->latestStartTime.at(vDst); tau2++) {
					if (tau2 + int(this->deltaMins.at(distance)) - tau1 - lSrc - delay >= 0) {
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
			std::cout << "SATRatIIScheduler: creating resource constraints" << std::endl;
		}
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
					std::cout << "SATRatIIScheduler: creating resource constraint clauses for vertices '" << v1->getName() << "' and '" << v2->getName() << "'" << std::endl;
				}
				// now we got two vertices that might produce resource conflicts
				for (int s1=0; s1<this->samples; s1++) {
					for (int s2=0; s2<this->samples; s2++) {
						// ensure that conflicting vertices are either scheduled or bound differently
						this->solver->add(this->timeOverlapLiterals.at({v1, s1, v2, s2}));
						if (!bindingTrivial) {
							this->solver->add(this->bindingOverlapLiterals.at({v1, s1, v2, s2}));
						}
						this->solver->add(0);
						this->resourceConstraintClauseCounter++;
						// ensure that time overlap literals are set correctly
						for (auto tau1Temp = this->earliestStartTime.at(v1); tau1Temp <= this->latestStartTime.at(v1); tau1Temp++) {
							// check timeout
							if (this->terminator.terminate()) {
								return;
							}
							auto tau1 = tau1Temp + this->initiationIntervals.at(s1);
							for (auto tau2Temp = this->earliestStartTime.at(v2); tau2Temp <= this->latestStartTime.at(v2); tau2Temp++) {
								auto tau2 = tau2Temp + this->initiationIntervals.at(s2);
								if (tau1 % this->modulo != tau2 % this->modulo) continue;
								this->solver->add(-this->timeOverlapLiterals.at({v1, s1, v2, s2}));
								this->solver->add(-this->scheduleTimeLiterals.at({v1, tau1Temp}));
								this->solver->add(-this->scheduleTimeLiterals.at({v2, tau2Temp}));
								this->solver->add(0);
								this->timeOverlapClauseCounter++;
							}
						}
						// ensure that binding overlap literals are set correctly
						if (bindingTrivial) continue;
						for (auto k=0; k<limit; k++) {
							this->solver->add(-this->bindingOverlapLiterals.at({v1, s1, v2, s2}));
							this->solver->add(-this->bindingLiterals.at({v1, k, s1}));
							this->solver->add(-this->bindingLiterals.at({v2, k, s2}));
							this->solver->add(0);
							this->bindingOverlapClauseCounter++;
						}
					}
				}
			}
		}
		// ensure exactly 1 schedule time and exactly 1 binding
		if (!this->quiet) {
			std::cout << "SATRatIIScheduler: creating schedule time and binding constraints" << std::endl;
		}
		for (auto &v : this->g.Vertices()) {
			if (!this->quiet) {
				std::cout << "SATRatIIScheduler: creating schedule time constraint for vertex '" << v->getName() << "'" << std::endl;
			}
			// schedule time
			for (int tau=this->earliestStartTime.at(v); tau<= this->latestStartTime.at(v); tau++) {
				this->solver->add(this->scheduleTimeLiterals.at({v, tau}));
			}
			this->solver->add(0);
			this->scheduleTimeConstraintClauseCounter++;
			if (!this->quiet) {
				std::cout << "SATRatIIScheduler: creating binding constraint for vertex '" << v->getName() << "'" << std::endl;
			}
			// binding
			if (this->vertexIsUnlimited.at(v) or this->resourceLimit.at(v) == 1) continue;
			for (int s=0; s<this->samples; s++) {
				for (int l=0; l<this->resourceLimit.at(v); l++) {
					this->solver->add(this->bindingLiterals.at({v, l, s}));
				}
				this->solver->add(0);
				this->bindingConstraintClauseCounter++;
			}
		}
		this->clauseCounter = this->dependencyConstraintClauseCounter + this->resourceConstraintClauseCounter +
													this->scheduleTimeConstraintClauseCounter + this->bindingConstraintClauseCounter;
	}

	void SATRatIIScheduler::fillSolutionStructure() {
		// print solution
		if (!this->quiet) {
			std::cout << "SATRatIIScheduler: CaDiCaL solution: " << std::endl;
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
				for (int s=0; s<this->samples; s++) {
					std::cout << "    s=" << s << std::endl;
					for (int l=0; l<this->resourceLimit.at(v); l++) {
						std::cout << "      FU=" << l << " - " << this->solver->val(this->bindingLiterals.at({v, l, s})) << std::endl;
					}
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
			this->startTimesVector.resize(this->samples);
			for (auto s=0; s<this->samples; s++) {
				this->startTimesVector[s][v] = t + this->initiationIntervals[s];
			}
			// binding
			this->ratIIbindings.resize(this->samples);
			auto *r = this->resourceModel.getResource(v);
			for (int s=0; s<this->samples; s++) {
				if (r->isUnlimited()) {
					// assign unlimited vertices unique FUs
					this->ratIIbindings[s][v] = unlimitedResourceCounter[r];
					if (s == 0) {
						this->binding[v] = unlimitedResourceCounter[r];
					}
					// increment counter
					unlimitedResourceCounter[r]++;
				}
				else if (r->getLimit() == 1) {
					// assign trivial bindings
					this->ratIIbindings[s][v] = 0;
					if (s == 0) {
						this->binding[v] = 0;
					}
				}
				else {
					auto b = -1;
					for (int l=0; l<this->resourceLimit.at(v); l++) {
						if (this->solver->val(this->bindingLiterals.at({v, l, s})) < 0) {
							continue;
						}
						b = l;
						break;
					}
					if (b < 0) {
						throw Exception("Failed to find binding for vertex '"+v->getName()+"' in sample '"+std::to_string(s)+"' - that should never happen!");
					}
					this->ratIIbindings[s][v] = b;
					if (s == 0) {
						this->binding[v] = b;
					}
				}
			}
		}
		// override candidate latency in case the scheduler found a solution with a schedule length
		// which is smaller than the given candidate latency (unlikely I guess, but who knows...)
		this->candidateLatency = this->getScheduleLength();
	}

	void SATRatIIScheduler::setUpSolver() {
		// attach terminator to support timeout
		this->solver->connect_terminator(&this->terminator);
	}

	void SATRatIIScheduler::createLiterals() {
		// time and binding variables
		for (auto &v : this->g.Vertices()) {
			// schedule time variables
			for (int tau=this->earliestStartTime.at(v); tau<= this->latestStartTime.at(v); tau++) {
				this->scheduleTimeLiteralCounter++;
				this->scheduleTimeLiterals[{v, tau}] = ++this->literalCounter;
			}
			// binding variables
			if (this->vertexIsUnlimited.at(v) or this->resourceLimit.at(v)==1) continue;
			for (int s=0; s<this->samples; s++) {
				for (int l=0; l<this->resourceLimit.at(v); l++) {
					this->bindingLiteralCounter++;
					this->bindingLiterals[{v, l, s}] = ++this->literalCounter;
				}
			}
		}
		// overlap variables
		for (auto &v1 : this->g.Vertices()) {
			auto limit = this->resourceModel.getResource(v1)->getLimit();
			if (limit == UNLIMITED) continue;
			auto bindingTrivial = limit == 1;
			for (auto &v2 : this->g.Vertices()) {
				if (v2->getId() <= v1->getId()) continue;
				if (this->resourceModel.getResource(v1) != this->resourceModel.getResource(v2)) continue;
				for (int s1=0; s1<this->samples; s1++) {
					for (int s2=0; s2<this->samples; s2++) {
						this->timeOverlapLiteralCounter++;
						this->timeOverlapLiterals[{v1, s1, v2, s2}] = ++this->literalCounter;
						if (bindingTrivial) continue;
						this->bindingOverlapLiteralCounter++;
						this->bindingOverlapLiterals[{v1, s1, v2, s2}] = ++this->literalCounter;
					}
				}
			}
		}
	}

	void SATRatIIScheduler::setSolverTimeout(unsigned int newTimeoutInSec) {
		this->solverTimeout = newTimeoutInSec;
	}

	bool SATRatIIScheduler::computeNewLatencySuccess(const bool &lastSchedulingAttemptSuccessful) {
		switch (this->los) {
			case REVERSE_LINEAR: {
				if (this->candidateLatency < 0) {
					// first attempt: try maximum latency
					this->candidateLatency = this->latencyUpperBound;
					return true;
				}
				else if (lastSchedulingAttemptSuccessful) {
					// last scheduling attempt was a success
					if (this->candidateLatency == this->latencyLowerBound) {
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
					// first attempt: try minimum latency
					this->candidateLatency = this->latencyLowerBound;
				}
				else if (lastSchedulingAttemptSuccessful) {
					// check if we found the optimum
					if (this->candidateLatency == this->latencyLowerBound) return false;
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
					this->candidateLatency = this->latencyLowerBound;
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
					// first attempt: try minimum latency
					this->candidateLatency = this->latencyLowerBound;
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

	void SATRatIIScheduler::setLatencyOptimizationStrategy(const SATRatIIScheduler::LatencyOptimizationStrategy &newLos) {
		this->los = newLos;
	}

	void SATRatIIScheduler::calculateLatestStartTimes() {
		for (auto &v : this->g.Vertices()) {
			this->latestStartTime[v] = this->candidateLatency - this->latestStartTimeDifferences.at(v);
		}
	}

	void SATRatIIScheduler::calculateLatestStartTimeDifferences() {
		/*
		// check if latest start time differences were set by user
		bool foundAll = true;
		for (auto &v : this->g.Vertices()) {
			if (this->latestStartTimeDifferences.find(v) == this->latestStartTimeDifferences.end()) {
				foundAll = false;
				break;
			}
		}
		if (foundAll) return;
		 */
		// use ALAP scheduler without resource constraints to calc latest start times
		std::map<const Resource*, int> originalLimits;
		for (auto &r : this->resourceModel.Resources()) {
			originalLimits[r] = r->getLimit();
			r->setLimit(UNLIMITED, false); // disable errors during set limit function
		}
		ALAPScheduler alapScheduler(this->g, this->resourceModel);
		alapScheduler.setQuiet(this->quiet);
		alapScheduler.schedule();
		if (!alapScheduler.getScheduleFound()) {
			throw Exception("SATRatIIScheduler: failed to compute latest start times - that should never happen");
		}
		auto alapSL = alapScheduler.getScheduleLength();
		auto alapStartTimes = alapScheduler.getSchedule();
		if (!this->quiet) {
			std::cout << "SATRatIIScheduler: ALAP schedule length = " << alapSL << std::endl;
			for (auto &v : this->g.Vertices()) {
				std::cout << "  " << v->getName() << " - " << alapStartTimes.at(v) << " (lat=" << this->resourceModel.getVertexLatency(v) << ")" << std::endl;
			}
		}
		for (auto &v : this->g.Vertices()) {
			try {
				auto lstd = this->latestStartTimeDifferences.at(v);
				if (!this->quiet) {
					std::cout << "SATRatIIScheduler: vertex '" << v->getName() << "': user max time diff = '" << lstd << "', ALAP max time diff = '" << alapSL - alapStartTimes.at(v) << "'" << std::endl;
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

	void SATRatIIScheduler::calculateEarliestStartTimes() {
		/*
		// check if earliest start times were set by user
		bool foundAll = true;
		for (auto &v : this->g.Vertices()) {
			if (this->earliestStartTime.find(v) == this->earliestStartTime.end()) {
				foundAll = false;
				break;
			}
		}
		if (foundAll) return;
		 */
		// use ASAP scheduler without resource constraints for lower bounds on start times
		std::map<const Resource*, int> originalLimits;
		for (auto &r : this->resourceModel.Resources()) {
			originalLimits[r] = r->getLimit();
			r->setLimit(UNLIMITED, false); // disable errors during set limit function
		}
		ASAPScheduler asapScheduler(this->g, this->resourceModel);
		asapScheduler.schedule();
		if (!asapScheduler.getScheduleFound()) {
			throw Exception("SATRatIIScheduler: failed to compute earliest start times - that should never happen");
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

	void SATRatIIScheduler::setTargetLatency(const int &newTargetLatency) {
		this->minLatency = newTargetLatency;
		this->maxLatency = newTargetLatency;
	}

	void SATRatIIScheduler::setEarliestStartTimes(const map<Vertex *, int> &newEarliestStartTimes) {
		bool foundAll = true;
		for (auto &v : this->g.Vertices()) {
			if (newEarliestStartTimes.find(v) == newEarliestStartTimes.end()) {
				std::cout << "SATRatIIScheduler::setEarliestStartTimes: Warning: failed to find earliest start time for vertex '" << v->getName() << "' -> skipping assignment!" << std::endl;
				foundAll = false;
			}
		}
		if (!foundAll) return;
		this->earliestStartTime = newEarliestStartTimes;
	}

	void SATRatIIScheduler::setLatestStartTimeDifferences(const map<Vertex *, int> &newLatestStartTimeDifferences) {
		bool foundAll = true;
		for (auto &v : this->g.Vertices()) {
			if (newLatestStartTimeDifferences.find(v) == newLatestStartTimeDifferences.end()) {
				std::cout << "SATRatIIScheduler::setLatestStartTimeDifferences: Warning: failed to find earliest start time for vertex '" << v->getName() << "' -> skipping assignment!" << std::endl;
				foundAll = false;
			}
		}
		if (!foundAll) return;
		this->latestStartTimeDifferences = newLatestStartTimeDifferences;
	}

	void SATRatIIScheduler::simplifyResourceLimits() {
		for (auto &r : this->resourceModel.Resources()) {
			auto limit = r->getLimit();
			if (limit == UNLIMITED) continue; // skip unlimited resources because this is the dream scenario anyways
			auto numVertices = this->resourceModel.getNumVerticesRegisteredToResource(r);
			if (numVertices * this->samples <= limit) {
				// save original limit to restore it later
				this->originalResourceLimits[r] = limit;
				// resource limit can be ignored
				r->setLimit(UNLIMITED, false);
			}
		}
	}

	void SATRatIIScheduler::restoreResourceLimits() {
		for (auto &r : this->resourceModel.Resources()) {
			if (this->originalResourceLimits.find(r) == this->originalResourceLimits.end()) continue; // limit was not changed
			auto lim = this->originalResourceLimits.at(r);
			r->setLimit(lim, false);
		}
	}
}
#endif
