//
// Created by nfiege on 8/26/22.
//

#include "SATSchedulerBinEnc.h"
#include <cmath>
#include <chrono>
#include <iomanip>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/scheduler/ALAPScheduler.h>
#include <HatScheT/utility/Utility.h>

#ifdef USE_CADICAL
namespace HatScheT {

	SATSchedulerBinEnc::SATSchedulerBinEnc(HatScheT::Graph &g, HatScheT::ResourceModel &resourceModel, int II)
		: SchedulerBase(g,resourceModel), solverTimeout(300), terminator(0.0),
			latencyLowerBound(-1), latencyUpperBound(-1), enableIIBasedLatencyLowerBound(true) {
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
		this->lastCandidateLatency = -1;
		this->minLatency = -1;
		this->maxLatency = -1;
		this->literalCounter = -1;
		this->moduloSlotLiteralCounter = -1;
		this->unlimitedScheduleTimeLiteralCounter = -1;
		this->timeOffsetLiteralCounter = -1;
		this->bindingLiteralCounter = -1;
		this->clauseCounter = -1;
		this->dependencyConstraintClauseCounter = -1;
		this->resourceConstraintClauseCounter = -1;
		this->scheduleTimeConstraintClauseCounter = -1;
		this->bindingConstraintClauseCounter = -1;
		this->scheduleTimeInRangeClauseCounter = -1;
	}

	void SATSchedulerBinEnc::schedule() {
		this->initScheduler();
		for (this->candidateII = (int)this->minII; this->candidateII <= (int)this->maxII; ++this->candidateII) {
			this->defineLatLimits();
			if (this->maxLatency < this->minLatency) continue;
			//if (!this->quiet) {
			auto currentTime1 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			std::cerr << "SATSchedulerBinEnc: trying candidate II=" << this->candidateII << " at time " << 	std::put_time(std::localtime(&currentTime1), "%Y-%m-%d %X") << std::endl;
			//}
			this->candidateLatency = this->lastCandidateLatency = -1;
			this->latencyLowerBound = this->minLatency;
			if (this->enableIIBasedLatencyLowerBound) {
				// loop pipelining only makes sense if the latency of the whole graph is larger than the II...
				this->latencyLowerBound = max(this->latencyLowerBound, this->candidateII);
			}
			this->latencyUpperBound = this->maxLatency;
			bool lastAttemptSuccess = false;
			bool breakByTimeout = false;
			double elapsedTime = 0.0;

			if (!this->quiet) {
				std::cout << "SATSchedulerBinEnc: setting up solver" << std::endl;
			}
			this->setUpSolver();

			if (!this->quiet) {
				std::cout << "SATSchedulerBinEnc: candidate II=" << this->candidateII << " with min latency="
									<< this->latencyLowerBound << " and max latency=" << this->latencyUpperBound << std::endl;
			}
			if (!this->quiet) {
				std::cout << "SATSchedulerBinEnc: resetting containers" << std::endl;
			}
			this->resetContainer();
			while (this->computeNewLatencySuccess(lastAttemptSuccess)) {
				if (!this->quiet) {
					std::cout << "SATSchedulerBinEnc: new latency limit = " << this->candidateLatency << std::endl;
				}
				this->calculateLatestStartTimes();
				this->defineEarliestStartTimeOffsets();
				if (!this->quiet) {
					std::cout << "SATSchedulerBinEnc: creating literals" << std::endl;
				}
				this->createLiterals();
				if (!this->quiet) {
					std::cout << "SATSchedulerBinEnc: creating clauses" << std::endl;
				}
				this->createBaseClauses();
				this->createAdditionalClauses();
				elapsedTime = this->terminator.getElapsedTime();
				if (!this->quiet) {
					std::cout << "SATSchedulerBinEnc: time is " << elapsedTime << "sec after constructing the problem" << std::endl;
				}
				if (this->terminator.terminate()) {
					// timeout during problem construction :(
					if (!this->quiet) {
						std::cout << "SATSchedulerBinEnc: encountered timeout during problem construction after " << elapsedTime << "sec (time budget was " << this->solverTimeout << "sec) - total elapsed time: " << this->solvingTime << "sec" << std::endl;
					}
					breakByTimeout = true;
					break;
				}
				if (!this->quiet) {
					std::cout << "SATSchedulerBinEnc: start scheduling for II=" << this->candidateII << " and latency=" << this->candidateLatency << " with '" << this->literalCounter << "' literals and '"
										<< this->clauseCounter << "' clauses" << std::endl;
					std::cout << "  '" << this->unlimitedScheduleTimeLiteralCounter << "' schedule time literals (unlimited vertices)" << std::endl;
					std::cout << "  '" << this->moduloSlotLiteralCounter << "' modulo slot literals (limited vertices)" << std::endl;
					std::cout << "  '" << this->timeOffsetLiteralCounter << "' time offset literals (limited vertices)" << std::endl;
					std::cout << "  '" << this->bindingLiteralCounter << "' binding literals" << std::endl;
					std::cout << "  '" << this->dependencyConstraintClauseCounter << "' dependency constraint clauses" << std::endl;
					std::cout << "  '" << this->resourceConstraintClauseCounter << "' resource constraint clauses" << std::endl;
					std::cout << "  '" << this->scheduleTimeConstraintClauseCounter << "' schedule time constraint clauses" << std::endl;
					std::cout << "  '" << this->bindingConstraintClauseCounter << "' binding constraint clauses" << std::endl;
					std::cout << "  '" << this->scheduleTimeInRangeClauseCounter << "' schedule time range clauses" << std::endl;
					auto currentTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
					std::cout << "  current time: " << std::put_time(std::localtime(&currentTime2), "%Y-%m-%d %X") << std::endl;
				}
				if (elapsedTime >= this->solverTimeout) {
					// timeout after problem construction!
					if (!this->quiet) {
						std::cout << "SATSchedulerBinEnc: encountered timeout after " << elapsedTime << "sec (time budget was " << this->solverTimeout << "sec) - total elapsed time: " << this->solvingTime << "sec" << std::endl;
					}
					breakByTimeout = true;
					break;
				}
				// start solving
				//if (!this->quiet) {
				auto currentTime3 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				std::cerr << "SATSchedulerBinEnc: start solving at time " << 	std::put_time(std::localtime(&currentTime3), "%Y-%m-%d %X") << std::endl;
				//}
				auto stat = this->solver->solve();
				elapsedTime = this->terminator.getElapsedTime();
				lastAttemptSuccess = stat == CADICAL_SAT;
				if (!this->quiet) {
					std::cout << "SATSchedulerBinEnc: finished solving with status '" <<
										(lastAttemptSuccess?"SAT":"UNSAT") << "' (code '" << stat << "') after " << elapsedTime
										<< " sec (total: " << this->solvingTime << " sec)" << std::endl;
				}
				if(!lastAttemptSuccess) {
					//if (!this->quiet) {
					auto currentTime4 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
					std::cerr << "SATSchedulerBinEnc: failed to find solution for II=" << this->candidateII << " and SL=" << this->candidateLatency << " at " << std::put_time(std::localtime(&currentTime4), "%Y-%m-%d %X") << std::endl;
					//}
					// check if it was due to a timeout
					if (elapsedTime >= this->solverTimeout) {
						// timeout when solving
						if (!this->quiet) {
							std::cout << "SATSchedulerBinEnc: encountered timeout after " << elapsedTime << "sec (time budget was " << this->solverTimeout << "sec)" << std::endl;
						}
						breakByTimeout = true;
						break;
					}
					// schedule attempt failed :(
					// let's try again for the next latency :)
					if (!this->quiet) {
						std::cout << "SATSchedulerBinEnc: latency " << this->candidateLatency << " is infeasible - trying again with modified latency limit" << std::endl;
					}
					continue;
				}
				this->scheduleFound = true;
				this->II = this->candidateII;
				this->fillSolutionStructure();
				//if (!this->quiet) {
				auto currentTime5 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				std::cerr << "SATSchedulerBinEnc: found solution for II=" << this->candidateII << " and SL=" << this->candidateLatency << " at " << std::put_time(std::localtime(&currentTime5), "%Y-%m-%d %X") << std::endl;
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

	void SATSchedulerBinEnc::calcMinLatency() {
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
			std::cout << "SATSchedulerBinEnc: computed minimum latency " << this->minLatency << std::endl;
		}
		for (auto &r : this->resourceModel.Resources()) {
			r->setLimit(originalLimits.at(r));
		}
	}

	void SATSchedulerBinEnc::calcMaxLatency() {
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
			std::cout << "SATSchedulerBinEnc: computed maximum latency " << this->maxLatency << std::endl;
		}
	}

	void SATSchedulerBinEnc::initScheduler() {
		if (!this->quiet) {
			std::cout << "SATSchedulerBinEnc: start initializing scheduler" << std::endl;
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
				std::cout << "SATSchedulerBinEnc: maxII changed due to maxRuns value set by user!" << endl;
				std::cout << "SATSchedulerBinEnc: min/maxII = " << this->minII << " " << this->maxII << std::endl;
			}
		}
		if (this->minII > this->maxII) {
			throw Exception("Inconsistent II bounds");
		}
		// time stuff
		this->timeouts = 0;
		this->solvingTime = 0.0;
		// simplify resource limits to save variables/clauses
		this->simplifyResourceLimits();
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

	void SATSchedulerBinEnc::resetContainer() {
		// just create a new solver lol -> actually don't (use incremental solving instead)
		//this->solver = std::unique_ptr<CaDiCaL::Solver>(new CaDiCaL::Solver);
		//this->scheduleTimeLiterals.clear();
		//this->bindingLiterals.clear();
		this->literalCounter = 0;
		this->unlimitedScheduleTimeLiteralCounter = 0;
		this->moduloSlotLiteralCounter = 0;
		this->timeOffsetLiteralCounter = 0;
		this->bindingLiteralCounter = 0;
		this->clauseCounter = 0;
		this->dependencyConstraintClauseCounter = 0;
		this->resourceConstraintClauseCounter = 0;
		this->scheduleTimeConstraintClauseCounter = 0;
		this->bindingConstraintClauseCounter = 0;
		this->scheduleTimeInRangeClauseCounter = 0;
	}

	void SATSchedulerBinEnc::createBaseClauses() {
		// only create base clauses once at the beginning
		if (this->lastCandidateLatency != -1) return;
		// dependency constraints
		if (!this->quiet) {
			std::cout << "SATSchedulerBinEnc: creating dependency constraints" << std::endl;
		}
		for (auto &e : this->g.Edges()) {
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			if (!this->quiet) {
				std::cout << "SATSchedulerBinEnc: creating dependency constraint for edge '" << vSrc->getName() << "' -> '" << vDst->getName() << "'" << std::endl;
			}
			auto lSrc = this->resourceModel.getVertexLatency(vSrc);
			auto distance = e->getDistance();
			auto delay = e->getDelay();
			int tau1Min = this->vertexIsUnlimited.at(vSrc)?this->earliestStartTime.at(vSrc):this->earliestStartTimeOffsets.at(vSrc);
			int tau2Min = this->vertexIsUnlimited.at(vDst)?this->earliestStartTime.at(vDst):this->earliestStartTimeOffsets.at(vDst);
			auto edgeConst = lSrc + delay - (distance * this->candidateII);
			for (int tau1=tau1Min; tau1 <= this->latestStartTime.at(vSrc); tau1++) {
				auto lastInvalidTau2 = tau1 + edgeConst - 1;
				auto loopLimit = std::min(lastInvalidTau2, this->latestStartTime.at(vDst));
				// only loop over values for vDst that do not satisfy the edge constraint
				for (int tau2=tau2Min; tau2 <= loopLimit; tau2++) {
					/*if (tau2 + distance * this->candidateII - tau1 - lSrc - delay >= 0) {
						// dependency not violated
						continue;
					}*/
					// dependency violated!
					// forbid setting both times simultaneously
					std::vector<int> literals;
					this->getNumberRepr(vSrc, tau1, true, &literals);
					this->getNumberRepr(vDst, tau2, true, &literals);
					if (!literals.empty()) {
						for (auto &l : literals) {
							this->solver->add(l);
						}
						this->solver->add(0);
						this->dependencyConstraintClauseCounter++;
					}
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
			std::cout << "SATSchedulerBinEnc: creating resource constraints" << std::endl;
		}
		for (auto &v1 : this->g.Vertices()) {
			auto limit = this->resourceModel.getResource(v1)->getLimit();
			if (limit == UNLIMITED) continue;
			auto bindingTrivial = limit == 1;
			auto numV1 = this->numModuloSlotVariables.at(v1);
			for (auto &v2 : this->g.Vertices()) {
				if (v2->getId() <= v1->getId()) continue;
				if (this->resourceModel.getResource(v1) != this->resourceModel.getResource(v2)) continue;
				// check timeout
				if (this->terminator.terminate()) {
					return;
				}
				if (!this->quiet) {
					std::cout << "SATSchedulerBinEnc: creating resource constraint clauses for vertices '" << v1->getName() << "' and '" << v2->getName() << "'" << std::endl;
				}
				auto numV2 = this->numModuloSlotVariables.at(v2);
				// now we got two vertices that might produce resource conflicts
				// ensure that conflicting vertices are either scheduled or bound differently
				// resourceConstraintClauseCounter
				for (auto tau = 0; tau < this->candidateII; tau++) {
					// add modulo slot variables
					//if (numV1 >= tau) break;
					//if (numV2 >= tau) break;
					std::vector<int> literals;
					this->getNumberRepr(&this->moduloSlotLiterals, v1, numV1, tau, -1, &literals);
					this->getNumberRepr(&this->moduloSlotLiterals, v2, numV2, tau, -1, &literals);
					if (bindingTrivial) {
						// trivial binding -> no need to iterate over binding variables -> just forbid modulo slot overlaps
						if (!literals.empty()) {
							for (auto &l : literals) {
								this->solver->add(l);
							}
							this->solver->add(0);
							this->resourceConstraintClauseCounter++;
							/*std::cout << "#q# forbidding m = " << tau << " overlap: ";
							for (auto &l : literals) {
								std::cout << l << " ";
							}
							std::cout << std::endl;*/
						}
					}
					else {
						// binding nontrivial
						for (auto k = 0; k < limit; k++) {
							std::vector<int> literalsWithBind = literals;
							this->getNumberRepr(&this->bindingLiterals, v1, this->numBindingVariables.at(v1), k, -1, &literalsWithBind);
							this->getNumberRepr(&this->bindingLiterals, v2, this->numBindingVariables.at(v2), k, -1, &literalsWithBind);
							if (!literalsWithBind.empty()) {
								for (auto &l : literalsWithBind) {
									this->solver->add(l);
								}
								this->solver->add(0);
								this->resourceConstraintClauseCounter++;
								/*std::cout << "#q# forbidding m = " << tau << " and b = " << k << " overlap: ";
								for (auto &l : literalsWithBind) {
									std::cout << l << " ";
								}
								std::cout << std::endl;*/
							}
						}
					}
				}
			}
		}
		// ensure schedule time and binding variables are in range
		if (!this->quiet) {
			std::cout << "SATSchedulerBinEnc: creating schedule time and binding constraints" << std::endl;
		}
		for (auto &v : this->g.Vertices()) {
			if (!this->quiet) {
				std::cout << "SATSchedulerBinEnc: creating schedule time constraint for vertex '" << v->getName() << "'" << std::endl;
			}
			// schedule time
			if (!this->vertexIsUnlimited.at(v)) {
				/*
				// limit y TODO: this should not be necessary?!
				auto &numYDigits = this->numTimeOffsetVariables.at(v);
				auto greatestY = std::pow(2, numYDigits) - 1;
				auto &latest = this->latestStartTime.at(v);
				for (int y=greatestY; y*this->candidateII > latest; y--) {
					std::vector<int> literals;
					this->getNumberRepr(&this->timeOffsetLiterals, v, numYDigits, y, -1, &literals);
					if (!literals.empty()) {
						for (auto &l : literals) {
							this->solver->add(l);
						}
						this->solver->add(0);
						this->scheduleTimeConstraintClauseCounter++;
					}
				}
				 */
				// limit m for limited vertices
				auto &numMDigits = this->numModuloSlotVariables.at(v);
				auto greatestM = std::pow(2, numMDigits) - 1;
				for (int m=this->candidateII; m <= greatestM; m++) {
					std::vector<int> literals;
					this->getNumberRepr(&this->moduloSlotLiterals, v, numMDigits, m, -1, &literals);
					if (!literals.empty()) {
						for (auto &l : literals) {
							this->solver->add(l);
						}
						this->solver->add(0);
						this->scheduleTimeConstraintClauseCounter++;
					}
				}
			}
			auto &tMax = this->greatestExpressibleNumber.at(v);
			for (int t=this->latestStartTime.at(v); t<=tMax; t++) {
				std::vector<int> literals;
				this->getNumberRepr(v, t, true, &literals);
				if (!literals.empty()) {
					for (auto &l : literals) {
						this->solver->add(l);
					}
					this->solver->add(0);
					this->scheduleTimeConstraintClauseCounter++;
				}
			}
			if (!this->quiet) {
				std::cout << "SATSchedulerBinEnc: creating binding constraint for vertex '" << v->getName() << "'" << std::endl;
			}
			// binding
			if (this->vertexIsUnlimited.at(v) or this->resourceLimit.at(v) == 1) continue;
			auto &numBindingVars = this->numBindingVariables.at(v);
			auto lMax = std::pow(2,std::ceil(numBindingVars)) - 1;
			for (int r=this->resourceLimit.at(v); r<=lMax; r++) {
				std::vector<int> literals;
				this->getNumberRepr(&this->bindingLiterals, v, numBindingVars, r, -1, &literals);
				if (!literals.empty()) {
					for (auto &l : literals) {
						this->solver->add(l);
					}
					this->solver->add(0);
					this->bindingConstraintClauseCounter++;
				}
			}
		}
		// forbid schedule times that are too late
		/*
		if (this->lastCandidateLatency > this->candidateLatency) {
			for (auto &v : this->g.Vertices()) {
				this->solver->add(-this->scheduleTimeLiterals.at({v, this->lastLatestStartTime.at(v)}));
				this->solver->add(0);
			}
		}
		 */
		this->clauseCounter = this->dependencyConstraintClauseCounter + this->resourceConstraintClauseCounter +
													this->scheduleTimeConstraintClauseCounter + this->bindingConstraintClauseCounter;
	}

	void SATSchedulerBinEnc::createAdditionalClauses() {
		// only create base clauses once at the beginning
		if (this->lastCandidateLatency == -1) return;
		// forbid last schedule time
		for (auto &v : this->g.Vertices()) {
			auto t = this->lastLatestStartTime.at(v);
			std::vector<int> literals;
			this->getNumberRepr(v, t, true, &literals);
			if (!literals.empty()) {
				for (auto &l : literals) {
					this->solver->add(l);
				}
				this->solver->add(0);
				this->scheduleTimeInRangeClauseCounter++;
			}
		}

		this->clauseCounter = this->dependencyConstraintClauseCounter + this->resourceConstraintClauseCounter +
													this->scheduleTimeConstraintClauseCounter + this->bindingConstraintClauseCounter +
													this->scheduleTimeInRangeClauseCounter;
	}

	void SATSchedulerBinEnc::fillSolutionStructure() {
		// print solution
		/*
		if (!this->quiet) {
			std::cout << "SATSchedulerBinEnc: CaDiCaL solution: " << std::endl;
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
		 */
		std::map<const Resource*, int> unlimitedResourceCounter;
		for (auto &v : this->g.Vertices()) {
			// schedule time
			int t=0;
			if (this->vertexIsUnlimited.at(v)) {
				// t = tMin + t_i
				t += this->earliestStartTime.at(v);
				auto &tauMax = this->numUnlimitedScheduleTimeVariables.at(v);
				for (int tau=0; tau<tauMax; tau++) {
					auto &lit = this->unlimitedScheduleTimeLiterals.at({v, tau});
					if (this->solver->val(lit) < 0) continue;
					t += std::pow(2, tau);
				}
				//std::cout << "#q# " << v->getName() << ": t = " << t << " = " << this->earliestStartTime.at(v) << " + " << t-this->earliestStartTime.at(v) << std::endl;
			}
			else {
				// t = C_i + m_i + II*y_i
				t += this->earliestStartTimeOffsets.at(v);
				int m = 0;
				auto &tauMaxM = this->numModuloSlotVariables.at(v);
				for (int tau=0; tau<tauMaxM; tau++) {
					auto &lit = this->moduloSlotLiterals.at({v, tau});
					if (this->solver->val(lit) < 0) continue;
					m += std::pow(2, tau);
				}
				t += m;
				int y = 0;
				auto &tauMaxY = this->numTimeOffsetVariables.at(v);
				for (int tau=0; tau<tauMaxY; tau++) {
					auto &lit = this->timeOffsetLiterals.at({v, tau});
					if (this->solver->val(lit) < 0) continue;
					y += std::pow(2, tau);
				}
				t += (y * this->candidateII);
				//std::cout << "#q# " << v->getName() << ": t = " << t << " = " << this->earliestStartTimeOffsets.at(v) << " + " << m << " + " << y << " * " << this->candidateII << std::endl;
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
				//std::cout << "#q# " << v->getName() << ": b = " << this->binding[v] << std::endl;
				continue;
			}
			if (this->resourceLimit.at(v) == 1) {
				// assign trivial bindings
				this->binding[v] = 0;
				//std::cout << "#q# " << v->getName() << ": b = " << this->binding[v] << std::endl;
				continue;
			}
			auto b = 0;
			auto &rMax = this->numBindingVariables.at(v);
			for (int r=0; r<rMax; r++) {
				auto &lit = this->bindingLiterals.at({v, r});
				if (this->solver->val(lit) < 0) continue;
				b += std::pow(2, r);
			}
			if (b < 0) {
				throw Exception("Failed to find binding for vertex '"+v->getName()+"' - that should never happen!");
			}
			this->binding[v] = b;
			//std::cout << "#q# " << v->getName() << ": b = " << this->binding[v] << std::endl;
		}
		// override candidate latency in case the scheduler found a solution with a schedule length
		// which is smaller than the given candidate latency (unlikely I guess, but who knows...)
		auto actualScheduleLength = this->getScheduleLength();
		if (actualScheduleLength > this->candidateLatency) {
			// print vertices that violate the requested limit
			for (auto &it : this->startTimes) {
				if (it.second + this->resourceModel.getVertexLatency(it.first) > this->candidateLatency) {
					// start time is too late!
					std::cout << "SATSchedulerBinEnc: vertex '" << it.first->getName() << "' (t='" << it.second << "', lat='" << this->resourceModel.getVertexLatency(it.first) << "') violates requested schedule length of '" << this->candidateLatency << "'" << std::endl;
				}
			}
			throw Exception("SATSchedulerBinEnc: Found invalid schedule! Requested candidate schedule length was '"+std::to_string(this->candidateLatency)+"' but scheduler found schedule length '"+std::to_string(actualScheduleLength)+"'");
		}
		this->candidateLatency = actualScheduleLength;
	}

	void SATSchedulerBinEnc::setUpSolver() {
		this->solver = std::unique_ptr<CaDiCaL::Solver>(new CaDiCaL::Solver);
		this->terminator = CaDiCaLTerminator((double)this->solverTimeout);
		this->solver->connect_terminator(&this->terminator);
	}

	void SATSchedulerBinEnc::createLiterals() {
		// only create literals once
		if (this->lastCandidateLatency != -1) return;
		// schedule time variables
		// -> modulo slot variables
		// -> time offset variables
		//std::cout << "#q# schedule times" << std::endl;
		for (auto &v : this->g.Vertices()) {
			//std::cout << "#q# v - " << v->getName() << std::endl;
			if (this->vertexIsUnlimited.at(v)) {
				//std::cout << "#q#   unlimited" << std::endl;
				// t variables
				auto &tauMax = this->numUnlimitedScheduleTimeVariables.at(v);
				for (int tau=0; tau<tauMax; tau++) {
					this->unlimitedScheduleTimeLiteralCounter++;
					this->unlimitedScheduleTimeLiterals[{v, tau}] = ++this->literalCounter;
					//std::cout << "#q#     t -> " << tau << " - " << this->unlimitedScheduleTimeLiterals[{v, tau}] << std::endl;
				}
			}
			else {
				//std::cout << "#q#   limited" << std::endl;
				// modulo slot variables
				auto &tauMaxM = this->numModuloSlotVariables.at(v);
				for (int tau=0; tau<tauMaxM; tau++) {
					this->moduloSlotLiteralCounter++;
					this->moduloSlotLiterals[{v, tau}] = ++this->literalCounter;
					//std::cout << "#q#     m -> " << tau << " - " << this->moduloSlotLiterals[{v, tau}] << std::endl;
				}
				// offset variables
				auto &tauMaxY = this->numTimeOffsetVariables.at(v);
				for (int tau=0; tau<tauMaxY; tau++) {
					this->timeOffsetLiteralCounter++;
					this->timeOffsetLiterals[{v, tau}] = ++this->literalCounter;
					//std::cout << "#q#     y -> " << tau << " - " << this->timeOffsetLiterals[{v, tau}] << std::endl;
				}
			}
		}

		// binding variables
		//std::cout << "#q# bindings" << std::endl;
		for (auto &v : this->g.Vertices()) {
			//std::cout << "#q# v - " << v->getName() << std::endl;
			if (this->vertexIsUnlimited.at(v) or this->resourceLimit.at(v)==1) continue;
			for (int l=0; l<this->numBindingVariables.at(v); l++) {
				this->bindingLiteralCounter++;
				this->bindingLiterals[{v, l}] = ++this->literalCounter;
			}
		}

		/*
		if (this->timeOverlapLiterals.empty() or this->bindingOverlapLiterals.empty()) {
			// only create new literals if necessary
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
		}
		 */
	}

	void SATSchedulerBinEnc::setSolverTimeout(unsigned int newTimeoutInSec) {
		this->solverTimeout = newTimeoutInSec;
	}

	bool SATSchedulerBinEnc::computeNewLatencySuccess(const bool &lastSchedulingAttemptSuccessful) {
		if (!this->quiet) {
			std::cout << "SATSchedulerBinEnc: computing new latency for last candidate latency=" << this->candidateLatency
								<< ", min latency=" << this->latencyLowerBound << ", max latency=" << this->latencyUpperBound
								<< " and last attempt success=" << lastSchedulingAttemptSuccessful << std::endl;
		}
		// set a new value for the candidate latency
		this->lastCandidateLatency = this->candidateLatency;
		if (this->candidateLatency == -1) {
			this->candidateLatency = this->latencyUpperBound;
			return true;
		}
		else {
			if (lastSchedulingAttemptSuccessful and this->candidateLatency > this->latencyLowerBound) {
				this->candidateLatency = --this->latencyUpperBound;
				return true;
			}
			return false;
		}
	}

	void SATSchedulerBinEnc::calculateLatestStartTimes() {
		for (auto &v : this->g.Vertices()) {
			try {
				this->lastLatestStartTime[v] = this->latestStartTime.at(v);
			}
			catch (std::out_of_range&) {
				this->lastLatestStartTime[v] = -1;
			}
			this->latestStartTime[v] = this->candidateLatency - this->latestStartTimeDifferences.at(v);
		}
	}

	void SATSchedulerBinEnc::calculateLatestStartTimeDifferences() {

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
			throw Exception("SATSchedulerBinEnc: failed to compute latest start times - that should never happen");
		}
		auto alapSL = alapScheduler.getScheduleLength();
		auto alapStartTimes = alapScheduler.getSchedule();
		if (!this->quiet) {
			std::cout << "SATSchedulerBinEnc: ALAP schedule length = " << alapSL << std::endl;
		}
		for (auto &v : this->g.Vertices()) {
			try {
				auto lstd = this->latestStartTimeDifferences.at(v);
				if (!this->quiet) {
					std::cout << "SATSchedulerBinEnc: vertex '" << v->getName() << "': user max time diff = '"
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

	void SATSchedulerBinEnc::calculateEarliestStartTimes() {

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
			throw Exception("SATSchedulerBinEnc: failed to compute earliest start times - that should never happen");
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

	void SATSchedulerBinEnc::setTargetLatency(const int &newTargetLatency) {
		this->minLatency = newTargetLatency;
		this->maxLatency = newTargetLatency;
		this->enableIIBasedLatencyLowerBound = false;
	}

	void SATSchedulerBinEnc::setEarliestStartTimes(const map<Vertex *, int> &newEarliestStartTimes) {
		bool foundAll = true;
		for (auto &v : this->g.Vertices()) {
			if (newEarliestStartTimes.find(v) == newEarliestStartTimes.end()) {
				std::cout << "SATSchedulerBinEnc::setEarliestStartTimes: Warning: failed to find earliest start time for vertex '" << v->getName() << "' -> skipping assignment!" << std::endl;
				foundAll = false;
			}
		}
		if (!foundAll) return;
		this->earliestStartTime = newEarliestStartTimes;
	}

	void SATSchedulerBinEnc::setLatestStartTimeDifferences(const map<Vertex *, int> &newLatestStartTimeDifferences) {
		bool foundAll = true;
		for (auto &v : this->g.Vertices()) {
			if (newLatestStartTimeDifferences.find(v) == newLatestStartTimeDifferences.end()) {
				std::cout << "SATSchedulerBinEnc::setLatestStartTimeDifferences: Warning: failed to find earliest start time for vertex '" << v->getName() << "' -> skipping assignment!" << std::endl;
				foundAll = false;
			}
		}
		if (!foundAll) return;
		this->latestStartTimeDifferences = newLatestStartTimeDifferences;
	}

	void SATSchedulerBinEnc::simplifyResourceLimits() {
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

	void SATSchedulerBinEnc::restoreResourceLimits() {
		for (auto &r : this->resourceModel.Resources()) {
			if (this->originalResourceLimits.find(r) == this->originalResourceLimits.end()) continue; // limit was not changed
			auto lim = this->originalResourceLimits.at(r);
			r->setLimit(lim, false);
		}
	}

	void SATSchedulerBinEnc::defineEarliestStartTimeOffsets() {
		// only compute offsets once
		if (this->lastCandidateLatency != -1) return;
		// // -> the smallest multiple of II that is smaller than or equal to the actual earliest start time
		for (auto &v : this->g.Vertices()) {
			if (this->vertexIsUnlimited.at(v)) {
				// vertex has only (binary-encoded) schedule time variables t_i that encode the difference between
				// tMin and tMax
				auto numBitsLog2 = std::ceil(std::log2(this->latestStartTime.at(v) - this->earliestStartTime.at(v) + 1));
				auto numBits = std::max(1.0, numBitsLog2);
				this->numUnlimitedScheduleTimeVariables[v] = numBits;
				this->greatestExpressibleNumber[v] = this->earliestStartTime.at(v) + std::pow(2, numBits) - 1;
				/*std::cout << "#q# vertex '" << v->getName() << "':" << std::endl;
				std::cout << "    earliest time = " <<  this->earliestStartTime.at(v) << std::endl;
				std::cout << "    latest time =  " << this->latestStartTime.at(v) << std::endl;
				std::cout << "    range =  " << this->latestStartTime.at(v) - this->earliestStartTime.at(v) + 1 << std::endl;
				std::cout << "    #bits = " << numBits << std::endl;
				std::cout << "    greatest expressible number = " << this->greatestExpressibleNumber[v] << std::endl;*/
			}
			else {
				// t_i = C_i + m_i + II*y_i
				// vertex has its start time split into (binary-encoded) modulo slot variables (m_i)
				// and (binary-encoded) start time offset variables (y_i)
				// C_i is the smalles multiple of II that is smaller than or equal to the actual earliest start time of v_i
				// define C_i
				auto C_i = this->earliestStartTimeOffsets[v] = (int)std::floor(((double)this->earliestStartTime.at(v))/((double)this->candidateII))*this->candidateII;
				// define m_i
				auto tMax = this->latestStartTime.at(v);
				int numBitsM = std::ceil(std::log2(this->candidateII));
				/*
				if (tMax - C_i == 0) {
					numBitsM = 1;
				}
				else if (this->candidateII > tMax - C_i + 1) {
					numBitsM = std::ceil(std::log2(tMax - C_i + 1));
				}
				else {
					numBitsM = std::ceil(std::log2(this->candidateII));
				}
				 */
				this->numModuloSlotVariables[v] = numBitsM;
				// define y_i
				int numBitsY;
				int rem = tMax - C_i + 1 - this->candidateII;
				if (rem <= 0) {
					numBitsY = 0;
				}
				else {
					numBitsY = std::max((int)std::ceil(std::log2(rem)), 1);
				}
				this->numTimeOffsetVariables[v] = numBitsY;
				this->greatestExpressibleNumber[v] = C_i + (this->candidateII - 1) + (this->candidateII * (std::pow(2, numBitsY) - 1));
				/*std::cout << "#q# vertex '" << v->getName() << "':" << std::endl;
				std::cout << "    earliest time = " <<  this->earliestStartTime.at(v) << std::endl;
				std::cout << "    earliest time offset = " << this->earliestStartTimeOffsets.at(v) << std::endl;
				std::cout << "    latest time =  " << this->latestStartTime.at(v) << std::endl;
				std::cout << "    range =  " << this->latestStartTime.at(v) - this->earliestStartTimeOffsets.at(v) + 1 << std::endl;
				std::cout << "    #bits (m) = " << numBitsM << " (II = " << this->candidateII << ")" << std::endl;
				std::cout << "    #bits (y) = " << numBitsY << " (remaining range = " << rem << ")" << std::endl;
				std::cout << "    greatest expressible number = " << this->greatestExpressibleNumber[v] << std::endl;*/
				// define resource limit
				auto rLim = this->resourceLimit.at(v);
				if (rLim == 1) continue;
				this->numBindingVariables[v] = (int)std::ceil(std::log2(rLim));
				//std::cout << "    limit = " << rLim << std::endl;
				//std::cout << "    #bits (b) = " << this->numBindingVariables[v] << std::endl;
			}
		}
	}

	void SATSchedulerBinEnc::defineLatLimits() {
		if (this->minLatency >= 0 and this->maxLatency >= 0) {
			if (!this->quiet) {
				std::cout << "SATSchedulerBinEnc: min & max latency defined by user: " << this->minLatency << " & " << this->maxLatency << std::endl;
			}
			// limits were defined by the user
			// only set min and max times based on SDC schedule
			auto result = Utility::getSDCAsapAndAlapTimes(&this->g, &this->resourceModel, this->candidateII, this->quiet);
			for (auto &v : this->g.Vertices()) {
				try {
					this->earliestStartTime[v] = std::max(this->earliestStartTime.at(v), result.first.at(v));
				}
				catch (std::out_of_range&) {
					this->earliestStartTime[v] = result.first.at(v);
				}
			}
			//this->earliestStartTime = result.first;
			int sdcScheduleLength = 0;
			for (auto &v : this->g.Vertices()) {
				auto t = result.second.at(v) + this->resourceModel.getVertexLatency(v);
				if (t > sdcScheduleLength) sdcScheduleLength = t;
			}
			for (auto &v : this->g.Vertices()) {
				try {
					this->latestStartTimeDifferences[v] = sdcScheduleLength - std::max(this->latestStartTimeDifferences.at(v), result.second.at(v));
				}
				catch (std::out_of_range&) {
					this->latestStartTimeDifferences[v] = sdcScheduleLength - result.second.at(v);
				}
			}
		}
		else if (this->maxLatency >= 0 and this->minLatency < 0) {
			// only max latency defined by the user
			// set min and max times and also min latency
			if (!this->quiet) {
				std::cout << "SATSchedulerBinEnc: max latency defined by user: " << this->maxLatency << std::endl;
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
				std::cout << "SATSchedulerBinEnc: min latency defined by user: " << this->minLatency << std::endl;
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
				std::cout << "SATSchedulerBinEnc: no latency defined by user" << std::endl;
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
		if (!this->quiet) {
			std::cout << "SATSchedulerBinEnc: determined min latency = " << this->minLatency << std::endl;
			std::cout << "SATSchedulerBinEnc: determined max latency = " << this->maxLatency << std::endl;
		}
		// handle max latency constraint
		if (this->maxLatencyConstraint >= 0 and this->maxLatency > this->maxLatencyConstraint) {
			this->maxLatency = this->maxLatencyConstraint;
			std::cout << "SATSchedulerBinEnc: max latency overridden due to user constraint = " << this->maxLatency << std::endl;
		}
		if (!this->quiet) {
			std::cout << "SATSchedulerBinEnc: latency limits: " << this->minLatency << " <= L <= " << this->maxLatency << std::endl;
			// sort vertices by name
			std::set<std::string> vertexNames;
			for (auto &v : this->g.Vertices()) {
				vertexNames.insert(v->getName());
			}
			// print sorted times
			std::cout << "SATSchedulerBinEnc: printing all start time limits" << std::endl;
			for (auto &vn : vertexNames) {
				auto *v = &this->g.getVertexByName(vn);
				std::cout << "  " << v->getName() << " earliest: " << this->earliestStartTime.at(v) << ", latest diff: "
									<< this->latestStartTimeDifferences.at(v) << std::endl;
			}
		}
	}

	void SATSchedulerBinEnc::getNumberRepr(Vertex* v, int t, bool negate, std::vector<int> *literals) {
		//std::cout << "#q# " << (negate?"forbidding":"allowing") << " t=" << t << " for vertex '" << v->getName() << "'" << std::endl;
		int globalMultiplier = negate ? -1 : 1;
		if (this->vertexIsUnlimited.at(v)) {
			// tMin_i + t_i
			int &tMin = this->earliestStartTime.at(v);
			int num = t - tMin;
			int numDigits = this->numUnlimitedScheduleTimeVariables.at(v);
			this->getNumberRepr(&this->unlimitedScheduleTimeLiterals, v, numDigits, num, globalMultiplier, literals);
			/*std::cout << "#q#   -> t = " << tMin << " + " << num << std::endl;
			std::cout << "#q#      literals = ";
			for (auto &l : *literals) {
				std::cout << l << " ";
			}
			std::cout << std::endl;*/
		}
		else {
			// C_i + m_i + II * y_i
			int &earliestOffset = this->earliestStartTimeOffsets.at(v);
			int num = t - earliestOffset;
			int numDigitsM = this->numModuloSlotVariables.at(v);
			int modSlot = num % this->candidateII;
			this->getNumberRepr(&this->moduloSlotLiterals, v, numDigitsM, modSlot, globalMultiplier, literals);
			num -= modSlot;
			num /= this->candidateII;
			auto numDigitsY = this->numTimeOffsetVariables.at(v);
			this->getNumberRepr(&this->timeOffsetLiterals, v, numDigitsY, num, globalMultiplier, literals);
			/*std::cout << "#q#   -> t = " << earliestOffset << " + " << modSlot << " + " << num << " * " << this->candidateII << std::endl;
			std::cout << "#q#      literals = ";
			for (auto &l : *literals) {
				std::cout << l << " ";
			}
			std::cout << std::endl;*/
		}
	}

	void SATSchedulerBinEnc::getNumberRepr(std::map<std::pair<Vertex*, int>, int>* container, Vertex* v, const int &numDigits, const int &num, const int &globalMultiplier, std::vector<int>* literals) {
		for (int i=0; i<numDigits; i++) {
			int bitSet = (num >> i) & 1;
			auto multiplier = (bitSet * 2 - 1) * globalMultiplier;
			literals->emplace_back(multiplier * container->at({v, i}));
		}
	}
}
#endif