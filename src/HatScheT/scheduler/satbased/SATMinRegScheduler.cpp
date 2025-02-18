//
// Created by nfiege on 5/3/22.
//

#include "SATMinRegScheduler.h"
#include <chrono>
#include <iomanip>
#include <cmath>
#include <HatScheT/scheduler/ALAPScheduler.h>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/utility/Utility.h>

#ifdef USE_CADICAL
namespace HatScheT {
#define CADICAL_SAT 10

	SATMinRegScheduler::SATMinRegScheduler(HatScheT::Graph &g, HatScheT::ResourceModel &resourceModel)
		: SchedulerBase(g,resourceModel), solverTimeout(300),
		terminator(0.0),
		numRegs(-1),
		  ros(RegisterOptimizationStrategy::SQRT), skipFirstRegAttempt(true), sqrtJumpLength(-1) {
		this->II = -1;
		this->scheduleFound = false;
		this->optimalResult = false;

		this->solvingTime = -1.0;
		this->candidateNumRegs = -1;
		this->literalCounter = -1;
		this->scheduleTimeLiteralCounter = -1;
		this->bindingLiteralCounter = -1;
		this->variableLiteralCounter = -1;
		this->registerBindingLiteralCounter = -1;
		this->clauseCounter = -1;
		this->dependencyConstraintClauseCounter = -1;
		this->resourceConstraintClauseCounter = -1;
		this->scheduleTimeConstraintClauseCounter = -1;
		this->bindingConstraintClauseCounter = -1;
		this->variableConstraintClauseCounter = -1;
		this->registerBindingConstraintClauseCounter = -1;
		this->registerOverlapConstraintClauseCounter = -1;
	}

	void SATMinRegScheduler::schedule() {
		this->initScheduler();
		if (!this->quiet) {
			auto currentTime0 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			std::cout << "SATMinRegScheduler: trying candidate II=" << this->II << " at time " << 	std::put_time(std::localtime(&currentTime0), "%Y-%m-%d %X") << std::endl;
		}

		bool breakByTimeout = false;
		this->terminator = CaDiCaLTerminator((double)this->solverTimeout);
		bool lastAttemptSuccess = false;
		this->candidateNumRegs = -1;
		double elapsedTime = 0.0;
		//for (this->candidateNumRegs = this->regMax; this->candidateNumRegs >= 0; this->candidateNumRegs--) {
		while (this->computeNewNumRegistersSuccess(lastAttemptSuccess)) {
			//if (!this->quiet) {
				auto currentTime1 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				std::cerr << "SATMinRegScheduler: trying candidate #Regs=" << this->candidateNumRegs << " at time " << 	std::put_time(std::localtime(&currentTime1), "%Y-%m-%d %X") << std::endl;
			//}
			if (!this->quiet) {
				std::cout << "SATMinRegScheduler: resetting containers" << std::endl;
			}
			this->resetContainer();
			if (!this->quiet) {
				std::cout << "SATMinRegScheduler: setting up solver" << std::endl;
			}
			this->setUpSolver();
			if (!this->quiet) {
				std::cout << "SATMinRegScheduler: creating literals" << std::endl;
			}
			this->createLiterals();
			if (!this->quiet) {
				std::cout << "SATMinRegScheduler: creating clauses" << std::endl;
			}
			this->createClauses();
			if (!this->quiet) {
				std::cout << "SATMinRegScheduler: start scheduling for II=" << this->II << " and latency="
					<< this->maxLatencyConstraint << " and #Regs=" << this->candidateNumRegs << " with '"
					<< this->literalCounter << "' literals and '" << this->clauseCounter << "' clauses" << std::endl;
				std::cout << "  '" << this->scheduleTimeLiteralCounter << "' schedule time literals" << std::endl;
				std::cout << "  '" << this->bindingLiteralCounter << "' binding literals" << std::endl;
				std::cout << "  '" << this->variableLiteralCounter << "' variable literals" << std::endl;
				std::cout << "  '" << this->registerBindingLiteralCounter << "' register binding literals" << std::endl;
				std::cout << "  '" << this->dependencyConstraintClauseCounter << "' dependency constraint clauses" << std::endl;
				std::cout << "  '" << this->resourceConstraintClauseCounter << "' resource constraint clauses" << std::endl;
				std::cout << "  '" << this->scheduleTimeConstraintClauseCounter << "' schedule time constraint clauses" << std::endl;
				std::cout << "  '" << this->bindingConstraintClauseCounter << "' binding constraint clauses" << std::endl;
				std::cout << "  '" << this->variableConstraintClauseCounter << "' variable constraint clauses" << std::endl;
				std::cout << "  '" << this->registerBindingConstraintClauseCounter << "' register binding constraint clauses" << std::endl;
				std::cout << "  '" << this->registerOverlapConstraintClauseCounter << "' register overlap constraint clauses" << std::endl;
				auto currentTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				std::cout << "  current time: " << std::put_time(std::localtime(&currentTime2), "%Y-%m-%d %X") << std::endl;
			}
			elapsedTime = this->terminator.getElapsedTime();
			if (!this->quiet) {
				std::cout << "SATMinRegScheduler: time is " << elapsedTime << "sec after constructing the problem" << std::endl;
			}
			if (elapsedTime >= this->solverTimeout) {
				// timeout after problem construction!
				if (!this->quiet) {
					std::cout << "SATMinRegScheduler: encountered timeout after " << elapsedTime << "sec (time budget was "
						<< this->solverTimeout << "sec) - total elapsed time: " << this->solvingTime << "sec" << std::endl;
				}
				breakByTimeout = true;
				break;
			}
			// start solving
			//if (!this->quiet) {
				auto currentTime3 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				std::cerr << "SATMinRegScheduler: start solving at time " << 	std::put_time(std::localtime(&currentTime3), "%Y-%m-%d %X") << std::endl;
			//}
			auto stat = this->solver->solve();
			elapsedTime = this->terminator.getElapsedTime();
			lastAttemptSuccess = stat == CADICAL_SAT;
			if (!this->quiet) {
				std::cout << "SATMinRegScheduler: finished solving with status '" <<
									(lastAttemptSuccess?"SAT":"UNSAT") << "' (code '" << stat << "') after " << elapsedTime
									<< " sec (total: " << this->solvingTime << " sec)" << std::endl;
			}
			if(!lastAttemptSuccess) {
				//if (!this->quiet) {
					auto currentTime4 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
					std::cerr << "SATMinRegScheduler: failed to find solution for II=" << this->II << " and SL="
										<< this->maxLatencyConstraint << " and #Regs=" << this->candidateNumRegs << " at "
										<< std::put_time(std::localtime(&currentTime4), "%Y-%m-%d %X") << std::endl;
				//}
				// check if it was due to a timeout
				if (elapsedTime >= this->solverTimeout) {
					// timeout when solving
					if (!this->quiet) {
						std::cout << "SATMinRegScheduler: encountered timeout after " << elapsedTime << "sec (time budget was "
							<< this->solverTimeout << "sec)" << std::endl;
					}
					breakByTimeout = true;
					break;
				}
				// schedule attempt failed :(
				if (!this->quiet) {
					std::cout << "SATMinRegScheduler: #Regs=" << this->candidateNumRegs
						<< " is infeasible" << std::endl;
				}
				continue;
			}
			this->scheduleFound = true;
			this->numRegs = this->candidateNumRegs;
			this->fillSolutionStructure();
			//if (!this->quiet) {
				auto currentTime5 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				std::cerr << "SATMinRegScheduler: found solution for II=" << this->II << " and SL="
									<< this->maxLatencyConstraint << " and #Regs=" << this->candidateNumRegs << " at "
									<< std::put_time(std::localtime(&currentTime5), "%Y-%m-%d %X") << std::endl;
			//}
		}
		this->solvingTime += elapsedTime;
		this->optimalResult = !breakByTimeout;
	}

	void SATMinRegScheduler::initScheduler() {
		if (!this->quiet) {
			std::cout << "SATMinRegScheduler: start initializing scheduler" << std::endl;
		}
		// solution info
		this->scheduleFound = false;
		this->optimalResult = false;
		// handle max latency constraint
		if (this->maxLatencyConstraint < 0) {
			throw Exception("SATMinRegScheduler: need to specify latency limit!");
		}
		// handle II
		if (this->minII > 0) {
			// maybe min II was set instead of actual II?
			this->II = this->minII;
		}
		if (this->II <= 0.0) {
			throw Exception("SATMinRegScheduler: need to specify II!");
		}
		// handle register limit
		if (this->regMax < 0) {
			throw Exception("SATMinRegScheduler: need to specify an upper bound for the number of registers! (e.g. by calculating the minimum number of registers for another schedule)");
		}
		// handle resource limitations
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
		this->solvingTime = 0.0;
		// check which vertices have outgoing edges
		for (auto &v : this->g.Vertices()) {
			this->hasOutgoingEdges[v] = false;
			this->latestVariableReadTime[v] = -1;
		}
		for (auto &e : this->g.Edges()) {
			this->hasOutgoingEdges.at(&e->getVertexSrc()) = true;
			auto t = (this->maxLatencyConstraint - this->resourceModel.getVertexLatency(&e->getVertexDst())) +
							 (e->getDistance() * (int)this->II);
			if (t > this->latestVariableReadTime.at(&e->getVertexSrc())) {
				this->latestVariableReadTime.at(&e->getVertexSrc()) = t;
			}
		}
		if (!this->quiet) {
			for (auto &v : this->g.Vertices()) {
				std::cout << "  " << v->getName() << std::endl;
				std::cout << "    " << (this->hasOutgoingEdges.at(v)?"has":"doesn't have") << " outgoing edges" << std::endl;
				std::cout << "    latest variable read time = " << this->latestVariableReadTime.at(v) << std::endl;
			}
		}
		// calc earliest and latest start times for all vertices to reduce the number of necessary boolean variables
		this->calculateEarliestStartTimes();
		this->calculateLatestStartTimes();
		if (!this->quiet) {
			std::cout << "SATScheduler: computed earliest/latest start times:" << std::endl;
			for (auto &v : this->g.Vertices()) {
				std::cout << "  " << v->getName() << " - " << this->earliestStartTime.at(v) << " / "
				  << this->latestStartTime.at(v) << std::endl;
			}
		}
		// init num register search space
		this->registerAttempts.clear();
		this->registerUpperBound = this->regMax;
		this->registerLowerBound = 0;
		this->sqrtJumpLength = (int)round(sqrt(this->regMax));
	}

	void SATMinRegScheduler::resetContainer() {
		// just create a new solver lol
#ifdef USE_KISSAT
		this->solver = std::unique_ptr<kissatpp::kissatpp>(new kissatpp::kissatpp(static_cast<double>(this->solverTimeout) - this->terminator.getElapsedTime()));
#else
		this->solver = std::unique_ptr<CaDiCaL::Solver>(new CaDiCaL::Solver);
#endif
		this->scheduleTimeLiterals.clear();
		this->bindingLiterals.clear();
		this->literalCounter = 0;
		this->scheduleTimeLiteralCounter = 0;
		this->bindingLiteralCounter = 0;
		this->variableLiteralCounter = 0;
		this->registerBindingLiteralCounter = 0;
		this->clauseCounter = 0;
		this->dependencyConstraintClauseCounter = 0;
		this->resourceConstraintClauseCounter = 0;
		this->scheduleTimeConstraintClauseCounter = 0;
		this->bindingConstraintClauseCounter = 0;
		this->variableConstraintClauseCounter = 0;
		this->registerBindingConstraintClauseCounter = 0;
		this->registerOverlapConstraintClauseCounter = 0;
	}

	void SATMinRegScheduler::createClauses() {
		// dependency constraints + variable constraints
		if (!this->quiet) {
			std::cout << "SATMinRegScheduler: creating dependency constraints" << std::endl;
		}
		for (auto &e : this->g.Edges()) {
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			if (!this->quiet) {
				std::cout << "SATMinRegScheduler: creating dependency constraint for edge '" << vSrc->getName() << "' -> '" << vDst->getName() << "'" << std::endl;
			}
			auto lSrc = this->resourceModel.getVertexLatency(vSrc);
			auto distance = e->getDistance();
			auto delay = e->getDelay();
			for (int tau1=this->earliestStartTime.at(vSrc); tau1 <= this->latestStartTime.at(vSrc); tau1++) {
				for (int tau2=this->earliestStartTime.at(vDst); tau2 <= this->latestStartTime.at(vDst); tau2++) {
					// dependency constraint
					if (tau2 + (distance * this->II) - tau1 - lSrc - delay < 0) {
						// dependency violated!
						// both literals must not be 1 at the same time!
						this->solver->add(-this->scheduleTimeLiterals.at({vSrc, tau1}));
						this->solver->add(-this->scheduleTimeLiterals.at({vDst, tau2}));
						this->solver->add(0);
						this->dependencyConstraintClauseCounter++;
					}

					// variable constraint
					for (int tau3=tau1+1+lSrc; tau3 <= tau2 + (distance * this->II); tau3++) {
						// variable must be stored in a register
						this->solver->add(-this->scheduleTimeLiterals.at({vSrc, tau1}));
						this->solver->add(-this->scheduleTimeLiterals.at({vDst, tau2}));
						this->solver->add(this->variableLiterals.at({vSrc, tau3}));
						this->solver->add(0);
						this->variableConstraintClauseCounter++;
					}

				}
			}
		}
		// resource constraints
		if (!this->quiet) {
			std::cout << "SATMinRegScheduler: creating resource constraints" << std::endl;
		}
		for (auto &r : this->resourceModel.Resources()) {
			auto limit = r->getLimit();
			auto bindingTrivial = limit == 1;
			if (limit == UNLIMITED) continue;
			for (int l=0; l<limit; l++) {
				if (!this->quiet) {
					std::cout << "SATMinRegScheduler: creating resource constraints for resource '" << r->getName() << "' instance '" << l << "'" << std::endl;
				}
				for (auto &v1 : this->g.Vertices()) {
					if (this->resourceModel.getResource(v1) != r) continue;
					auto b1 = -1;
					if (!bindingTrivial) {
						b1 = this->bindingLiterals.at({v1, l});
					}
					for (auto &v2 : this->g.Vertices()) {
						if (this->resourceModel.getResource(v2) != r) continue;
						if (v1->getId() >= v2->getId()) continue;
						auto b2 = -1;
						if (!bindingTrivial) {
							b2 = this->bindingLiterals.at({v2, l});
						}
						for (int x=0; x<this->II; x++) {
							for (int tau1=this->earliestStartTime.at(v1); tau1<= this->latestStartTime.at(v1); tau1++) {
								if (tau1 % (int)this->II != x) continue;
								auto t1 = this->scheduleTimeLiterals.at({v1, tau1});
								for (int tau2=this->earliestStartTime.at(v2); tau2<= this->latestStartTime.at(v2); tau2++) {
									if (tau2 % (int)this->II != x) continue;
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
		// ensure at least 1 schedule time and at least 1 register and resource binding
		if (!this->quiet) {
			std::cout << "SATMinRegScheduler: creating schedule time and binding constraints" << std::endl;
		}
		for (auto &v : this->g.Vertices()) {
			if (!this->quiet) {
				std::cout << "SATMinRegScheduler: creating schedule time constraint for vertex '" << v->getName() << "'" << std::endl;
			}
			// schedule time
			for (int tau=this->earliestStartTime.at(v); tau<= this->latestStartTime.at(v); tau++) {
				this->solver->add(this->scheduleTimeLiterals.at({v, tau}));
			}
			this->solver->add(0);
			this->scheduleTimeConstraintClauseCounter++;
			// register binding (only needed if the vertex actually has outgoing edges)
			if (this->hasOutgoingEdges.at(v)) {
				if (!this->quiet) {
					std::cout << "SATMinRegScheduler: creating register binding constraints for vertex '" << v->getName() << "'" << std::endl;
				}
				for (int tau=this->earliestStartTime.at(v); tau<= this->latestVariableReadTime.at(v); tau++) {
					this->solver->add(-this->variableLiterals.at({v, tau}));
					for (int reg=0; reg < this->candidateNumRegs; reg++) {
						this->solver->add(this->registerBindingLiterals.at({v, tau, reg}));
					}
					this->solver->add(0);
					this->registerBindingConstraintClauseCounter++;
				}
			}
			// binding (only needed if the resource has a non-trivial limit)
			if (!this->vertexIsUnlimited.at(v) and this->resourceLimit.at(v) != 1) {
				if (!this->quiet) {
					std::cout << "SATMinRegScheduler: creating binding constraint for vertex '" << v->getName() << "'" << std::endl;
				}
				for (int l=0; l<this->resourceLimit.at(v); l++) {
					this->solver->add(this->bindingLiterals.at({v, l}));
				}
				this->solver->add(0);
				this->bindingConstraintClauseCounter++;
			}
		}
		// register overlap constraints
		for (int gamma=0; gamma<this->II; gamma++) {
			for (auto &v1 : this->g.Vertices()) {
				if (!this->hasOutgoingEdges.at(v1)) continue; // skip vertices without outputs
				for (auto &v2 : this->g.Vertices()) {
					if (v1->getId() > v2->getId()) continue; // only create overlap constraints once
					if (!this->hasOutgoingEdges.at(v2)) continue; // skip vertices without outputs
					for (int tau1=this->earliestStartTime.at(v1); tau1<= this->latestVariableReadTime.at(v1); tau1++) {
						if (tau1 % (int)this->II != gamma) continue;
						for (int tau2=this->earliestStartTime.at(v2); tau2<= this->latestVariableReadTime.at(v2); tau2++) {
							if (tau2 % (int)this->II != gamma) continue;
							if (v1 == v2 and tau1 == tau2) continue;
							for (int reg=0; reg<this->candidateNumRegs; reg++) {
								this->solver->add(-this->registerBindingLiterals.at({v1, tau1, reg}));
								this->solver->add(-this->registerBindingLiterals.at({v2, tau2, reg}));
								this->solver->add(0);
								this->registerOverlapConstraintClauseCounter++;
							}
						}
					}
				}
			}
		}
		this->clauseCounter = this->dependencyConstraintClauseCounter + this->resourceConstraintClauseCounter +
													this->scheduleTimeConstraintClauseCounter + this->bindingConstraintClauseCounter +
													this->variableConstraintClauseCounter + this->registerBindingConstraintClauseCounter +
													this->registerOverlapConstraintClauseCounter;
	}

	void SATMinRegScheduler::fillSolutionStructure() {
		// print solution
		if (!this->quiet) {
			std::cout << "SATMinRegScheduler: CaDiCaL solution: " << std::endl;
			for (auto &v : this->g.Vertices()) {
				std::cout << "  vertex " << v->getName() << std::endl;
				for (int tau=this->earliestStartTime.at(v); tau<= this->latestStartTime.at(v); tau++) {
					std::cout << "    t=" << tau << " - " <<
#ifdef USE_KISSAT
					this->solver->value(this->scheduleTimeLiterals.at({v, tau}))
#else
					this->solver->val(this->scheduleTimeLiterals.at({v, tau}))
#endif
					<< std::endl;
				}
				if (!this->vertexIsUnlimited.at(v) and this->resourceLimit.at(v) != 1) {
					for (int l=0; l< this->resourceLimit.at(v); l++) {
						std::cout << "    b=" << l << " - " <<
#ifdef USE_KISSAT
						this->solver->value(this->bindingLiterals.at({v, l}))
#else
						this->solver->val(this->bindingLiterals.at({v, l}))
#endif
						<< std::endl;
					}
				}
				for (int tau=this->earliestStartTime.at(v); tau<this->latestVariableReadTime.at(v); tau++) {
					std::cout << "    z=" << tau << " - " <<
#ifdef USE_KISSAT
					this->solver->value(this->variableLiterals.at({v, tau}))
#else
					this->solver->val(this->variableLiterals.at({v, tau}))
#endif
					<< std::endl;
				}
			}
			for (auto reg=0; reg< this->candidateNumRegs; reg++) {
				std::cout << "  register " << reg << std::endl;
				for (auto &v : this->g.Vertices()) {
					std::cout << "    vertex " << v->getName() << std::endl;
					for (int tau=this->earliestStartTime.at(v); tau<this->latestVariableReadTime.at(v); tau++) {
						std::cout << "      r=" << tau << " - " <<
#ifdef USE_KISSAT
						this->solver->value(this->registerBindingLiterals.at({v, tau, reg}))
#else
						this->solver->val(this->registerBindingLiterals.at({v, tau, reg}))
#endif
						<< std::endl;
					}
				}
			}
		}
		this->startTimesContainer.clear();
		for (auto &v : this->g.Vertices()) {
			// schedule time
			for (int tau=this->earliestStartTime.at(v); tau<= this->latestStartTime.at(v); tau++) {
				if (
#ifdef USE_KISSAT
					this->solver->value(this->scheduleTimeLiterals.at({v, tau}))
#else
					this->solver->val(this->scheduleTimeLiterals.at({v, tau}))
#endif
					< 0) {
					continue;
				}
				this->startTimesContainer[v].insert(tau);
			}
			if (this->startTimesContainer[v].empty()) {
				throw Exception("Failed to find start time for vertex '"+v->getName()+"' - that should never happen!");
			}
		}
		for (auto &v : this->g.Vertices()) {
			this->startTimes[v] = *this->startTimesContainer.at(v).begin();
		}
		// override candidate register limit in case the scheduler found a solution for which the register limit
		// which is smaller than the given one (unlikely I guess, but who knows...)
		this->candidateNumRegs = Utility::calcMinNumRegs(&this->g, &this->resourceModel, this->startTimes, (int)this->II);
	}

	void SATMinRegScheduler::setUpSolver() {
#ifdef USE_KISSAT
		// no need to connect terminator with kissat
#else
		// attach terminator to support timeout
		this->solver->connect_terminator(&this->terminator);
#endif
	}

	void SATMinRegScheduler::createLiterals() {
		for (auto &v : this->g.Vertices()) {
			// schedule time literals
			for (int tau=this->earliestStartTime.at(v); tau<= this->latestStartTime.at(v); tau++) {
				this->scheduleTimeLiteralCounter++;
				this->scheduleTimeLiterals[{v, tau}] = ++this->literalCounter;
			}
			// variable and register binding literals
			if (this->hasOutgoingEdges.at(v)) {
				for (int tau=this->earliestStartTime.at(v); tau<= this->latestVariableReadTime.at(v); tau++) {
					// variable literals
					this->variableLiteralCounter++;
					this->variableLiterals[{v, tau}] = ++this->literalCounter;
					// register binding literals
					for (int reg=0; reg < this->candidateNumRegs; reg++) {
						this->registerBindingLiteralCounter++;
						this->registerBindingLiterals[{v, tau, reg}] = ++this->literalCounter;
					}
				}
			}
			// binding literals
			if (!this->vertexIsUnlimited.at(v) and this->resourceLimit.at(v)>1) {
				for (int l=0; l<this->resourceLimit.at(v); l++) {
					this->bindingLiteralCounter++;
					this->bindingLiterals[{v, l}] = ++this->literalCounter;
				}
			}
		}
	}

	void SATMinRegScheduler::setSolverTimeout(unsigned int newTimeoutInSec) {
		this->solverTimeout = newTimeoutInSec;
	}

	int SATMinRegScheduler::getNumRegs() const {
		return this->numRegs;
	}

	void SATMinRegScheduler::setRegMax(const int &newRegMax) {
		if (newRegMax <= 0) {
			throw Exception("SATMinRegScheduler: specified illegal number of registers -> #Regs must be >= 0 but #Regs = "+std::to_string(newRegMax)+" was given");
		}
		this->regMax = newRegMax;
	}

	void SATMinRegScheduler::calculateLatestStartTimes() {
		// use ALAP scheduler without resource constraints to calc latest start times
		std::map<const Resource*, int> originalLimits;
		for (auto &r : this->resourceModel.Resources()) {
			originalLimits[r] = r->getLimit();
			r->setLimit(UNLIMITED, false); // disable errors during set limit function
		}
		ALAPScheduler alapScheduler(this->g, this->resourceModel);
		alapScheduler.schedule();
		if (!alapScheduler.getScheduleFound()) {
			throw Exception("SATMinRegScheduler: failed to compute latest start times - that should never happen");
		}
		auto alapSL = alapScheduler.getScheduleLength();
		auto alapStartTimes = alapScheduler.getSchedule();
		for (auto &v : this->g.Vertices()) {
			auto diff = alapSL - alapStartTimes.at(v);
			this->latestStartTime[v] = this->maxLatencyConstraint - diff;
		}
		// set resource limits back to original values
		for (auto &r : this->resourceModel.Resources()) {
			r->setLimit(originalLimits.at(r));
		}
	}

	void SATMinRegScheduler::calculateEarliestStartTimes() {
#ifdef USE_SCALP // USE_SCALP
		if (this->sdcSolverWishlist.empty()) {
			this->sdcSolverWishlist = {"Gurobi", "CPLEX", "SCIP", "LPSolve"};
		}
		// use SDC-solver to calculate earliest start time for the given vertex
		// without resource constraints
		ScaLP::Solver s(this->sdcSolverWishlist);
		// variables
		std::map<Vertex*, ScaLP::Variable> vars;
		for (auto &v : this->g.Vertices()) {
			vars[v] = ScaLP::newIntegerVariable(v->getName(), 0, this->maxLatencyConstraint - this->resourceModel.getVertexLatency(v));
		}
		// dependencies
		for (auto &e : this->g.Edges()) {
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			auto c = vars.at(vDst) + (this->II * e->getDistance()) - (vars.at(vSrc) + this->resourceModel.getVertexLatency(vSrc) + e->getDelay()) >= 0;
			s.addConstraint(c);
		}
		for (auto &v : this->g.Vertices()) {
			this->earliestStartTime[v] = this->calculateEarliestStartTime(v, &s, vars);
		}
#else
		// use ASAP scheduler without resource constraints if ScaLP is unavailable
		// this should produce values that might be a bit too small in general
		// especially in presence of recurrences
		// but (and this is a big but), using ASAP is WAY faster than solving multiple SDC problems
		std::map<const Resource*, int> originalLimits;
		for (auto &r : this->resourceModel.Resources()) {
			originalLimits[r] = r->getLimit();
			r->setLimit(UNLIMITED, false); // disable errors during set limit function
		}
		ASAPScheduler asapScheduler(this->g, this->resourceModel);
		asapScheduler.schedule();
		if (!asapScheduler.getScheduleFound()) {
			throw Exception("SATMinRegScheduler: failed to compute earliest start times - that should never happen");
		}
		auto asapSL = asapScheduler.getScheduleLength();
		auto asapStartTimes = asapScheduler.getSchedule();
		for (auto &v : this->g.Vertices()) {
			this->earliestStartTime[v] = asapStartTimes.at(v);
		}
		// set resource limits back to original values
		for (auto &r : this->resourceModel.Resources()) {
			r->setLimit(originalLimits.at(r));
		}
#endif
	}

#ifdef USE_SCALP
	int SATMinRegScheduler::calculateEarliestStartTime(Vertex *v, ScaLP::Solver *s, const std::map<Vertex*, ScaLP::Variable> &vars) {
		// override objective to minimize vertex start time
		s->setObjective(ScaLP::minimize(vars.at(v)));
		// solve
		auto stat = s->solve();
		// check if solution was found
		if (stat != ScaLP::status::OPTIMAL and stat != ScaLP::status::FEASIBLE and stat != ScaLP::status::TIMEOUT_FEASIBLE) {
			return 0;
		}
		// return solution
		return (int)round(s->getResult().objectiveValue);
	}
#endif

	void SATMinRegScheduler::setSolverWishlist(const list<std::string> &newSolverWishlist) {
		this->sdcSolverWishlist = newSolverWishlist;
	}

	void
	SATMinRegScheduler::setLatencyOptimizationStrategy(const SATMinRegScheduler::RegisterOptimizationStrategy &newRos) {
		this->ros = newRos;
	}

	bool SATMinRegScheduler::computeNewNumRegistersSuccess(const bool &lastSchedulingAttemptSuccessful) {
		switch (this->ros) {
			case RegisterOptimizationStrategy::LINEAR: {
				if (this->candidateNumRegs < 0) {
					// first try
					if (this->skipFirstRegAttempt and this->registerUpperBound > this->registerLowerBound) {
						// skip first attempt for the number of registers
						this->candidateNumRegs = this->registerUpperBound - 1;
					} else {
						// do not skip first possible value for the number of registers
						this->candidateNumRegs = this->registerUpperBound;
					}
					return true;
				} else {
					// not first try
					if (lastSchedulingAttemptSuccessful) {
						// last attempt was successful
						if (this->candidateNumRegs == this->registerLowerBound) {
							// reached optimum -> stop searching
							return false;
						} else {
							// set new upper bound
							this->registerUpperBound = this->candidateNumRegs;
							// keep iterating towards lower bound
							this->candidateNumRegs--;
							return true;
						}
					} else {
						// last attempt was not successful
						// -> we either reached the optimum or we encountered a timeout...
						return false;
					}
				}
			}
			case RegisterOptimizationStrategy::SQRT: {
				if (this->candidateNumRegs < 0) {
					// first try
					if (this->skipFirstRegAttempt and this->registerUpperBound > 0) {
						// skip first attempt for the number of registers
						this->candidateNumRegs = this->registerUpperBound - this->sqrtJumpLength;
					} else {
						// do not skip first possible value for the number of registers
						this->candidateNumRegs = this->registerUpperBound;
					}
					return true;
				} else {
					// not first try
					if (lastSchedulingAttemptSuccessful) {
						// last attempt was successful
						if (this->candidateNumRegs == this->registerLowerBound) {
							// reached optimum -> stop searching
							return false;
						} else {
							// set new upper bound
							this->registerUpperBound = this->candidateNumRegs;
							// keep jumping towards lower bound
							this->candidateNumRegs -= this->sqrtJumpLength;
							if (this->candidateNumRegs < this->registerLowerBound) this->candidateNumRegs = this->registerLowerBound;
							return true;
						}
					} else {
						// last attempt was not successful
						if (this->candidateNumRegs == this->registerUpperBound - 1) {
							// reached optimum -> stop searching
							return false;
						} else {
							// update lower bound
							this->registerLowerBound = this->candidateNumRegs + 1;
							// start iterating again from upper bound
							// but set jump length to 1
							this->sqrtJumpLength = 1;
							this->candidateNumRegs = this->registerUpperBound - 1;
							return true;
						}
					}
				}
			}
			case RegisterOptimizationStrategy::LOGARITHMIC: {
				if (this->candidateNumRegs < 0) {
					// first try
					if (this->skipFirstRegAttempt and this->registerUpperBound > 0) {
						// skip first attempt for the number of registers
						this->candidateNumRegs = this->registerUpperBound/2;
					}
					else {
						// do not skip first possible value for the number of registers
						this->candidateNumRegs = this->registerUpperBound;
					}
					this->registerAttempts.insert(this->candidateNumRegs);
					return true;
				}
				else {
					// not first try
					if (lastSchedulingAttemptSuccessful) {
						// last attempt was a success
						this->registerUpperBound = this->candidateNumRegs;
						this->candidateNumRegs = floor(((float)this->candidateNumRegs + (float)this->registerLowerBound) / 2.0);
					}
					else {
						// last attempt was a fail
						this->registerLowerBound = this->candidateNumRegs;
						this->candidateNumRegs = ceil(((float)this->candidateNumRegs + (float)this->registerUpperBound) / 2.0);
					}
					auto successPair = this->registerAttempts.insert(this->candidateNumRegs);
					return successPair.second;
				}
			}
		}
		// whoops, something went wrong... ABORT!
		return false;
	}
}
#endif
