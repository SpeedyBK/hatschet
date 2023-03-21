//
// Created by nfiege on 17/02/21.
//

#include "RationalIIModuloSDCScheduler.h"
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/scheduler/ALAPScheduler.h>
#include <math.h>
#include <cmath>
#include <ctime>
#include <stdlib.h>
#include <sstream>
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/Binding.h"

namespace HatScheT {
	const char *TimeoutException1::what() const noexcept {
		return msg.c_str();
	}

	std::ostream &operator<<(std::ostream &oss, HatScheT::TimeoutException1 &e) {
		return oss << e.msg;
	}

	RationalIIModuloSDCScheduler::RationalIIModuloSDCScheduler(Graph &g, ResourceModel &resourceModel,
																														 std::list<std::string> solverWishlist) :
		RationalIISchedulerLayer(g, resourceModel), ILPSchedulerBase(solverWishlist),
		priorityForSchedQueue(), schedQueue(), scalpVariables(), timeInILPSolvers(0.0), fastObjective(true),
		pType(PriorityHandler1::priorityType::SUBSEQUALAP), budget(-1), uniqueVariableName(""),
		budgetMultiplier(0), outputsEqualScheduleLength(false), vertexHasOutgoingEdges(),
		scheduleLength(-1), scalpStatus(ScaLP::status::UNKNOWN) {
		// do nothing
	}

	void RationalIIModuloSDCScheduler::scheduleIteration() {
		if (!this->quiet) cout << "sample: " << this->samples << " modulo: " << this->modulo << endl;
		this->initiationIntervals = RationalIISchedulerLayer::getOptimalInitiationIntervalSequence(this->samples,
																																															 this->modulo,
																																															 this->quiet);
		this->latencySequence = RationalIISchedulerLayer::getLatencySequenceFromInitiationIntervals(
			this->initiationIntervals, this->modulo);

		if (!this->quiet) {
			std::cout << "Start scheduling Attempt!" << std::endl;
			std::cout << "Latency Sequence: " << std::endl;
			for (auto l : this->latencySequence) std::cout << l << " ";
			std::cout << std::endl;
			std::cout << "Initiation Intervals: " << std::endl;
			for (auto j : this->initiationIntervals) std::cout << j << " ";
			std::cout << std::endl;
		}
		this->calcDeltaMins(); // calculate delta mins
		this->calculatePriorities(); // calculate priorities for scheduling queue
		if (this->budget < 0)
			this->setDefaultBudget(); // set default budget according to paper if no budget was given by the user
		this->initialBudget = this->budget;
		this->timeBudget = (double) this->solverTimeout;
		this->timeTracker = std::chrono::high_resolution_clock::now();
		//finds a valid non-rectangular MRT
		if (this->latencySequence.empty())
			throw HatScheT::Exception("Latency sequence vector is empty after ratII schedule - this should never happen!");
		RationalIIModuloSDCScheduler::setMRT(this->mrt, this->resourceModel, this->initiationIntervals, this->samples,
																				 this->modulo, this->quiet);

		//debugging
		if (!this->quiet) this->mrt.print();


		// reset ScaLP status
		this->scalpStatus = ScaLP::status::UNKNOWN;
		// delete scheduling constraints from previous iteration
		this->clearAllAdditionalConstraints();
		//////////////////////
		// ALGORITHM LINE 1 //
		//////////////////////
		// asap scheduling without resource constraints
		if (!this->createInitialSchedule()) {
			return;
		}
		//////////////////////
		// ALGORITHM LINE 2 //
		//////////////////////
		// create scheduling queue from all resource constrained instructions
		this->createSchedulingQueue(this->getResourceConstrainedVertices());
		this->sdcTimes = this->asapTimes;

		int numIt = 0;
		while ((!this->schedQueue.empty()) && (budget >= 0)) {
			numIt++;
			if (numIt % 1000 == 0) {
				std::cout << "Iteration #" << numIt << std::endl;
			}
			if (!this->quiet) this->mrt.print();
			//////////////////////
			// ALGORITHM LINE 4 //
			//////////////////////
			// pop first element from scheduling queue
			Vertex *I = this->schedQueue.front();
			this->schedQueue.pop_front();
			if (!this->quiet) {
				cout << "Queue" << endl;
				if (this->schedQueue.empty()) {
					cout << "Empty Queue" << endl;
				}
				for (auto it: this->schedQueue) {
					cout << "Vertex in schedQueue: " << it->getName() << endl;
				}
			}
			//////////////////////
			// ALGORITHM LINE 5 //
			//////////////////////
			int time;
			try {
				time = this->sdcTimes.at(I);
			}
			catch (std::out_of_range &) {
				throw HatScheT::Exception("sdc times container corrupt, can't find instruction " + I->getName());
			}
			if (!this->quiet)
			  cout << "Next Vertex: " << I->getName() << " with time=" << time << endl;

			if (time < 0)
				throw HatScheT::Exception("Error: ModuloSDCScheduler::modSDCIteration: invalid time (" + to_string(time) +
																	") found by ILP solver for instruction '" + I->getName() + "'");
			if (this->mrt.insertVertex(I, time)) {
				if (!this->quiet) cout << "No RessourceConflict Next Vertex: " << I->getName() << " Time: " << time << endl;

				////////////////////////
				// ALGORITHM LINE 7-8 //
				////////////////////////
				scheduleInstruction(I, time);
			} else {
				if (!this->quiet) cout << "RessourceConflict found Next Vertex: " << I->getName() << " Time: " << time << endl;

				///////////////////////
				// ALGORITHM LINE 10 //
				///////////////////////
				// add constraint t_I >= time+1 to ilp formulation
				ScaLP::Constraint c(this->scalpVariables.at(I) >= (time + 1));
				this->clearConstraintForVertex(I);
				this->createAdditionalConstraint(I, c);
				///////////////////////
				// ALGORITHM LINE 11 //
				///////////////////////
				bool foundSolution;
				try {
					foundSolution = this->solveSDCProblem();
				}
				catch (HatScheT::TimeoutException1 &e) {
					this->handleTimeout();
					this->scheduleFound = false;
					return;
				}
				if (foundSolution) {
					if (!this->quiet) cout << "No Backtracking needed Next Vertex: " << I->getName() << " Time: " << time << endl;
					///////////////////////
					// ALGORITHM LINE 13 //
					///////////////////////
					PriorityHandler1::putIntoSchedQueue(I, this->pType, &this->priorityForSchedQueue, &this->schedQueue);
				} else {
					if (!this->quiet) cout << "Need Backtracking Next Vertex: " << I->getName() << " Time: " << time << endl;
					///////////////////////
					// ALGORITHM LINE 15 //
					///////////////////////
					this->clearConstraintForVertex(I);
					///////////////////////
					// ALGORITHM LINE 16 //
					///////////////////////
					try {
						this->backtracking(I, time);
					}
					catch (HatScheT::TimeoutException1 &e) {
						this->handleTimeout();
						this->scheduleFound = false;
						return;
					}
					///////////////////////
					// ALGORITHM LINE 17 //
					///////////////////////
					try {
						foundSolution = this->solveSDCProblem();
						if (!foundSolution) {
							cout
								<< "ERROR: ModuloSDCScheduler::modSDCIteration: Pseudocode line 17; solver should always find solution"
								<< endl;
							throw HatScheT::Exception(
								"ModuloSDCScheduler::modSDCIteration: Pseudocode line 17; solver should always find solution");
						}
					}
					catch (HatScheT::TimeoutException1 &e) {
						this->handleTimeout();
						this->scheduleFound = false;
						return;
					}
				}
			}
			///////////////////////
			// ALGORITHM LINE 20 //
			///////////////////////
			budget--;
		}
		// reset timer for next II calculation
		this->timeBudget = this->solverTimeout;
		this->timeTracker = std::chrono::high_resolution_clock::now();
		///////////////////////
		// ALGORITHM LINE 22 //
		///////////////////////
		if (!this->schedQueue.empty()) {
			this->budgedEmptyCounter++;
			if (!this->quiet) {
				cout << "ModuloSDCScheduler::modSDCIteration: empty budged for II " << this->II << endl;
				cout << "ModuloSDCScheduler::modSDCIteration: empty budged counter " << this->budgedEmptyCounter << endl;
			}
		}
		this->scheduleFound = schedQueue.empty();

		// stop function if no schedule was found
		if (!this->scheduleFound) return;

		// looks like we found a schedule! :)
		if (!this->quiet) std::cout << "Found feasible solution!" << std::endl;
		// set start times
		auto solution = this->solver->getResult().values;
		for (auto *v : this->g.Vertices())
			this->startTimes[v] = (int) std::lround(solution.find(this->scalpVariables[v])->second);
		for (auto &late : this->initiationIntervals) {
			std::map<Vertex *, int> additionalStartTimes;
			for (auto startTime : this->startTimes) {
				additionalStartTimes[startTime.first] = startTime.second + late;
			}
			this->startTimesVector.emplace_back(additionalStartTimes);
		}
		//this->II = this->minII;
		bool emptyGraph = this->g.isEmpty();

		if (!this->quiet) {
			std::cout << "Rational II modulo schedule found with:" << std::endl;
			std::cout << "  S=" << this->samples << std::endl;
			std::cout << "  M=" << this->modulo << std::endl;
			std::cout << "  IIs=";
			for (auto i : this->latencySequence) {
				std::cout << i << " ";
			}
			std::cout << std::endl;
			std::cout << "  Latency=" << this->getScheduleLength() << std::endl;
		}
		//return;
		//return this->schedQueue.empty();

	}


	void RationalIIModuloSDCScheduler::constructProblem() {
		// clear old solver settings and create new variables
		this->createScalpVariables();
		this->createDataDependencyConstraints();
		this->createAdditionalConstraints();
	}

	void RationalIIModuloSDCScheduler::setObjective() {
		if (this->fastObjective) {
			// minimize sum of all start times (significantly less constraints, but slightly more complex objective)
			ScaLP::Term o;
			for (auto &it : this->scalpVariables) {
				o += it.second;
			}
			this->solver->setObjective(ScaLP::minimize(o));
		} else {
			// minimize the latest end time
			// do that by creating a new variable with the constraint t_new >= t_start(i) + latency(i)
			this->setUniqueVariableName();
			ScaLP::Variable newVar = ScaLP::newIntegerVariable(this->uniqueVariableName, 0, ScaLP::INF());
			for (auto &it : this->scalpVariables) {
				ScaLP::Constraint c = (newVar - it.second >= this->resourceModel.getResource(it.first)->getLatency());
				*this->solver << c;

				if (this->outputsEqualScheduleLength) {
					if (!this->vertexHasOutgoingEdges[it.first]) {
						ScaLP::Constraint c2 = (it.second + this->resourceModel.getResource(it.first)->getLatency() - newVar == 0);
						*this->solver << c2;
					}
				}
			}
			ScaLP::Term o = 1 * newVar;
			this->solver->setObjective(ScaLP::minimize(o));
		}
	}

	void RationalIIModuloSDCScheduler::resetContainer() {
		//this->rectangularMRT.clearMRT();
		//this->initiationIntervals.clear();
		//this->latencySequence.clear();
		//delta min
		this->asapTimes.clear();
		this->sdcTimes.clear();
		this->prevSched.clear();
		this->schedQueue.clear();
	}

	void RationalIIModuloSDCScheduler::createSchedulingQueue(const std::list<Vertex *> scheduleMe) {
		for (auto it : scheduleMe) {
			if (this->resourceModel.getResource(it)->getLimit() > 0) {
				PriorityHandler1::putIntoSchedQueue(it, this->pType, &this->priorityForSchedQueue, &this->schedQueue);
			}
		}
	}

	void RationalIIModuloSDCScheduler::setDefaultBudget() {
		//this->budget = (int) (this->budgetMultiplier * this->g.getNumberOfVertices());
		if (this->budgetMultiplier > 0) {
			this->budget = (int) (this->budgetMultiplier * this->g.getNumberOfVertices());
		} else {
			this->budget = INT32_MAX;
		}
	}

	void RationalIIModuloSDCScheduler::createScalpVariables() {
		if (this->scalpVariables.empty()) {
			for (auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); it++) {
				auto v = (*it);
				this->scalpVariables[v] = ScaLP::newIntegerVariable(v->getName(), 0, ScaLP::INF());
			}
		}
	}

	void RationalIIModuloSDCScheduler::setUpScalp() {
		this->initSolver();
		this->constructProblem();
		this->setObjective();
	}

	int RationalIIModuloSDCScheduler::getNumberOfConstrainedVertices(Graph &g, ResourceModel &rm) {
		int counter = 0;
		for (auto it = g.verticesBegin(); it != g.verticesEnd(); it++) {
			if (rm.getResource(*it)->getLimit() > 0) counter++;
		}
		return counter;
	}

	int RationalIIModuloSDCScheduler::getPrevSched(Vertex *v) {
		try {
			return this->prevSched.at(v);
		}
		catch (std::out_of_range &) {
			return -1;
		}
	}

	void RationalIIModuloSDCScheduler::createDataDependencyConstraints() {
		for (auto it = this->g.edgesBegin(); it != this->g.edgesEnd(); it++) {
			Edge *edge = (*it);
			Vertex &src = edge->getVertexSrc();
			Vertex &dst = edge->getVertexDst();
			ScaLP::Constraint c = (
				this->scalpVariables.at(&src) - this->scalpVariables.at(&dst) <= int(this->deltaMins[edge->getDistance()]) -
																																				 this->resourceModel.getResource(
																																					 &src)->getLatency() - edge->getDelay());
			*this->solver << c;
		}
	}

	ScaLP::Constraint *RationalIIModuloSDCScheduler::getAdditionalConstraint(Vertex *v) {
		try {
			return this->additionalConstraints.at(v);
		}
		catch (std::out_of_range &) {
			return nullptr;
		}
	}

	void RationalIIModuloSDCScheduler::clearConstraintForVertex(Vertex *v) {
		if (this->getAdditionalConstraint(v) != nullptr) {
			delete this->additionalConstraints.at(v);
			this->additionalConstraints.erase(v);
		}
	}

	void RationalIIModuloSDCScheduler::createAdditionalConstraint(Vertex *v, ScaLP::Constraint &c) {
		this->additionalConstraints[v] = new ScaLP::Constraint(c);
	}

	void RationalIIModuloSDCScheduler::createAdditionalConstraints() {
		for (auto it : this->additionalConstraints) {
			*this->solver << *it.second;
		}
	}

	void RationalIIModuloSDCScheduler::clearAllAdditionalConstraints() {
		for (auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); it++) {
			this->clearConstraintForVertex(*it);
		}
	}

	bool RationalIIModuloSDCScheduler::solveSDCProblem() {
		std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
		this->setUpScalp();
		if (!this->manageTimeBudgetSuccess()) {
			if (!this->quiet) cout << "Timeout!" << endl;
			this->timeouts++;
			throw HatScheT::TimeoutException1("Time budget empty when trying to find solution for II=" + to_string(this->II));
		}

		// solve ilp and track time
		ScaLP::status s = this->solver->solve();
		std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
		std::chrono::nanoseconds timeSpan = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1);
		this->timeInILPSolvers += (((double) timeSpan.count()) / 1000000000.0);

		if (s == ScaLP::status::TIMEOUT_INFEASIBLE) {
			if (!this->quiet) cout << "Timeout!" << endl;
			this->timeouts++;
			throw HatScheT::TimeoutException1("Solver timeout when trying to find solution for II=" + to_string(this->II));
		}
		if ((s != ScaLP::status::FEASIBLE) && (s != ScaLP::status::OPTIMAL) && (s != ScaLP::status::TIMEOUT_FEASIBLE)) {
			return false;
		}

		ScaLP::Result r = this->solver->getResult();
		for (auto &it : r.values) {
			Vertex *v = this->getVertexFromVariable(it.first);
			if (v != nullptr) this->sdcTimes[v] = (int) it.second;
			else this->scheduleLength = (int) it.second;
		}

		return true;
	}

	Vertex *RationalIIModuloSDCScheduler::getVertexFromVariable(const ScaLP::Variable &sv) {
		for (auto &it : this->scalpVariables) {
			if (it.second == sv) return it.first;
		}
		return nullptr;
	}

	void RationalIIModuloSDCScheduler::backtracking(Vertex *I, const int &time) {
		int minTime;
		try {
			minTime = this->asapTimes.at(I);
		}
		catch (std::out_of_range &) {
			throw HatScheT::Exception("ASAP times container corrupt, can't find instruction " + I->getName());
		}
		for (; minTime <= time; minTime++) {
			//////////////////////
			// ALGORITHM LINE 2 //
			//////////////////////
			auto tempConstraint = ScaLP::Constraint(this->scalpVariables.at(I) == minTime);
			this->createAdditionalConstraint(I, tempConstraint);
			bool foundSolution = this->solveSDCProblem();
			//////////////////////
			// ALGORITHM LINE 3 //
			//////////////////////
			this->clearConstraintForVertex(I);
			if (foundSolution) break;
			if (minTime == time) {
				if (!this->quiet) {
					cout << "ERROR: backtracking algorithm (" << I->getName() << "," << time
							 << ") can't find time slot for instruction " << I->getName() << endl;
					cout << "Additional constraints: " << endl;
				}
				for (auto it : this->additionalConstraints) {
					cout << "    " << *it.second << endl;
				}
				throw HatScheT::Exception("backtracking algorithm can't find time slot for instruction " + I->getName());
			}
		}
		//////////////////////
		// ALGORITHM LINE 5 //
		//////////////////////
		int prevSchedTime = this->getPrevSched(I);
		int evictTime;
		if (!this->quiet) {
			cout << "Vertex: " << I->getName() << " min Time: " << minTime << " Prev SchedTime" << prevSchedTime << endl;
		}

		if (minTime > prevSchedTime || prevSchedTime < 0) {
			if (!this->quiet) {
				cout << "In if minTime > prevSchedTime || prevSchedTime < 0" << endl;
			}
			//////////////////////
			// ALGORITHM LINE 7 //
			//////////////////////
			evictTime = minTime;
		} else {
			if (!this->quiet) {
				cout << "Not in if minTime > prevSchedTime || prevSchedTime < 0" << endl;
			}

			//////////////////////
			// ALGORITHM LINE 9 //
			//////////////////////
			evictTime = prevSchedTime + 1;
		}
		if (evictTime < 0)
			throw HatScheT::Exception(
				"Error: ModuloSDCScheduler::backtracking: Invalid evict time (" + to_string(evictTime) + ") for instruction '" +
				I->getName() + "'");
		int timeslot = evictTime % this->modulo;
		std::list<Vertex *> evictInst = this->mrt.getResourceConflicts(I, timeslot);
		///////////////////////////
		// ALGORITHM LINES 11-15 //
		///////////////////////////
		for (auto it : evictInst) {
			this->unscheduleInstruction(it);
			// PUT OPERATION BACK INTO QUEUE EVEN THO IT IS NOT SPECIFIED IN PSEUDOCODE!
			if (!this->quiet) cout << "Back into Queue: " << it->getName() << " Resource Conflict" << endl;
			PriorityHandler1::putIntoSchedQueue(it, this->pType, &this->priorityForSchedQueue, &this->schedQueue);
		}
		///////////////////////
		// ALGORITHM LINE 16 //
		///////////////////////
		if (this->dependencyConflict(I, evictTime)) {
			///////////////////////
			// ALGORITHM LINE 17 //
			///////////////////////
			auto copy = this->additionalConstraints;
			for (auto it : copy) {
				///////////////////////////
				// ALGORITHM LINES 18-19 //
				///////////////////////////
				this->unscheduleInstruction(it.first);
				///////////////////////
				// ALGORITHM LINE 20 //
				///////////////////////
				if (!this->quiet) cout << "Back into Queue: " << it.first->getName() << " Dependency Conflict" << endl;
				PriorityHandler1::putIntoSchedQueue(it.first, this->pType, &this->priorityForSchedQueue, &this->schedQueue);
			}


		}
		//////////////////////////
		// ALGORITHM LINE 23-24 //
		//////////////////////////
		scheduleInstruction(I, evictTime);
	}

	bool RationalIIModuloSDCScheduler::createInitialSchedule() {

		this->setUpScalp();
		ScaLP::status s = this->solver->solve(); // solver should never timeout here...
		if ((s != ScaLP::status::FEASIBLE) && (s != ScaLP::status::OPTIMAL) && (s != ScaLP::status::TIMEOUT_FEASIBLE)) {
			if (!this->quiet) {
				cout << "ScaLP Backend: " << this->solver->getBackendName() << endl;
				cout << "ScaLP Status: " << s << endl;
				cout << "Additional constraints: " << endl;
			}
			this->printAdditionalSolverConstraints();
			if (!this->quiet)
				cout
					<< "ERROR: ModuloSDCScheduler::createInitialSchedule: failed to find schedule without resource constraints (no hatschet exception)"
					<< endl;
			return false;
		}

		ScaLP::Result r = this->solver->getResult();
		for (auto &it : r.values) {
			Vertex *v = this->getVertexFromVariable(it.first);
			if (v != nullptr) this->asapTimes[v] = (int) it.second;
			else this->scheduleLength = (int) it.second;
		}
		return true;
	}

	void RationalIIModuloSDCScheduler::unscheduleInstruction(Vertex *evictInst) {
		this->clearConstraintForVertex(evictInst);
		this->mrt.removeVertex(evictInst);
	}

	bool RationalIIModuloSDCScheduler::dependencyConflict(Vertex *I, const int &evictTime) {
		for (auto it = this->g.edgesBegin(); it != this->g.edgesEnd(); it++) {
			auto e = (*it);
			// I == e.start
			if ((&e->getVertexSrc() == I)) {
				if (this->dependencyConflictForTwoInstructions(I, evictTime, this->sdcTimes.at(&e->getVertexDst()),
																											 e->getDelay(), e->getDistance())) {
					return true;
				}
			}
			// I == e.dst
			if ((&e->getVertexDst() == I)) {
				if (this->dependencyConflictForTwoInstructions(&e->getVertexSrc(), this->sdcTimes.at(&e->getVertexSrc()),
																											 evictTime, e->getDelay(), e->getDistance())) {
					return true;
				}
			}
		}
		return false;
	}

	bool RationalIIModuloSDCScheduler::dependencyConflictForTwoInstructions(Vertex *i, const int &newStartTime_i,
																																					const int &newStartTime_j,
																																					const int &edgeDelay, const int &distance) {
		return ((newStartTime_i + this->resourceModel.getResource(i)->getLatency() + edgeDelay - newStartTime_j) >
						(this->II * distance));
	}

	void RationalIIModuloSDCScheduler::setUniqueVariableName() {
		if (!this->uniqueVariableName.empty()) return;
		this->uniqueVariableName = "you_sexy_creature";
		bool unique = false;
		while (!unique) {
			unique = true;
			for (auto &it : this->scalpVariables) {
				if (this->uniqueVariableName == it.second->getName()) {
					unique = false;
					this->uniqueVariableName += "_I_like_your_choice_of_vertex_names";
					break;
				}
			}
		}
	}

	void RationalIIModuloSDCScheduler::initSolver() {
		this->solver->reset();
		this->solver->quiet = this->solverQuiet;
		this->solver->timeout = (long) this->timeBudget;
		if (!quiet) {
			cout << "RationalIIModuloSDCScheduler: Timeout set to " << solver->timeout << " seconds!" << endl;
		}
		if (this->solver->getBackendName() == "Dynamic: LPSolve") {
			this->solver->presolve = false;
			this->solver->threads = 0;
		} else {
			this->solver->presolve = true;
			if (this->threads > 0) this->solver->threads = (int) this->threads;
		}
	}

	void RationalIIModuloSDCScheduler::calculatePriorities() {
		// Paper: priority of an instruction depends on how many operations depend on the result of this one
		// => different metrics possible; only god knows whats best
		switch (this->pType) {
			case PriorityHandler1::priorityType::ALAP: {
				auto p = this->getALAPScheduleWithoutResourceConstraints();
				for (auto it : p) {
					if (this->resourceModel.getResource(it.first)->getLimit() >= 0) {
						this->priorityForSchedQueue[it.first] = new PriorityHandler1(this->pType, it.second);
					}
				}
				break;
			}
			case PriorityHandler1::priorityType::ASAP: {
				auto p = this->getASAPScheduleWithoutResourceConstraints();
				for (auto it : p) {
					if (this->resourceModel.getResource(it.first)->getLimit() >= 0) {
						this->priorityForSchedQueue[it.first] = new PriorityHandler1(this->pType, it.second);
					}
				}
				break;
			}
			case PriorityHandler1::priorityType::PERTUBATION: {
				for (auto it : this->g.Vertices()) {
					if (this->resourceModel.getResource(it)->getLimit() > 0) {
						auto noOfSubseq = this->getNoOfSubsequentVertices(it);
						this->priorityForSchedQueue[it] = new PriorityHandler1(this->pType, noOfSubseq);
					}
				}
				break;
			}
			case PriorityHandler1::priorityType::MOBILITY_LOW: {
				auto pASAP = this->getASAPScheduleWithoutResourceConstraints();
				auto pALAP = this->getALAPScheduleWithoutResourceConstraints();
				for (auto it : pALAP) {
					try {
						if (this->resourceModel.getResource(it.first)->getLimit() >= 0) {
							this->priorityForSchedQueue[it.first] = new PriorityHandler1(this->pType, it.second - pASAP.at(it.first));
						}
					}
					catch (std::out_of_range &) {
						throw HatScheT::Exception(
							"Can't determine priority for scheduling queue, ASAP or ALAP scheduler might be buggy");
					}
				}
				break;
			}
			case PriorityHandler1::priorityType::MOBILITY_HIGH: {
				auto pASAP = this->getASAPScheduleWithoutResourceConstraints();
				auto pALAP = this->getALAPScheduleWithoutResourceConstraints();
				for (auto it : pALAP) {
					try {
						if (this->resourceModel.getResource(it.first)->getLimit() >= 0) {
							this->priorityForSchedQueue[it.first] = new PriorityHandler1(this->pType, it.second - pASAP.at(it.first));
						}
					}
					catch (std::out_of_range &) {
						throw HatScheT::Exception(
							"Can't determine priority for scheduling queue, ASAP or ALAP scheduler might be buggy");
					}
				}
				break;
			}
			case PriorityHandler1::priorityType::MOBLAP: {
				auto pASAP = this->getASAPScheduleWithoutResourceConstraints();
				auto pALAP = this->getALAPScheduleWithoutResourceConstraints();
				for (auto it : pALAP) {
					try {
						if (this->resourceModel.getResource(it.first)->getLimit() >= 0) {
							this->priorityForSchedQueue[it.first] = new PriorityHandler1(this->pType, it.second - pASAP.at(it.first),
																																					 it.second);
						}
					}
					catch (std::out_of_range &) {
						throw HatScheT::Exception(
							"Can't determine priority for scheduling queue, ASAP or ALAP scheduler might be buggy");
					}
				}
				break;
			}
			case PriorityHandler1::priorityType::ALABILITY: {
				auto pASAP = this->getASAPScheduleWithoutResourceConstraints();
				auto pALAP = this->getALAPScheduleWithoutResourceConstraints();
				for (auto it : pALAP) {
					try {
						if (this->resourceModel.getResource(it.first)->getLimit() >= 0) {
							this->priorityForSchedQueue[it.first] = new PriorityHandler1(this->pType, it.second,
																																					 it.second - pASAP.at(it.first));
						}
					}
					catch (std::out_of_range &) {
						throw HatScheT::Exception(
							"Can't determine priority for scheduling queue, ASAP or ALAP scheduler might be buggy");
					}
				}
				break;
			}
			case PriorityHandler1::priorityType::SUBSEQUALAP: {
				auto pALAP = this->getALAPScheduleWithoutResourceConstraints();
				for (auto it : pALAP) {
					auto noOfSubseq = this->getNoOfSubsequentVertices(it.first);
					this->priorityForSchedQueue[it.first] = new PriorityHandler1(this->pType, noOfSubseq, it.second);
				}
				break;
			}
			case PriorityHandler1::priorityType::ALASUB: {
				auto pALAP = this->getALAPScheduleWithoutResourceConstraints();
				for (auto it : pALAP) {
					auto noOfSubseq = this->getNoOfSubsequentVertices(it.first);
					this->priorityForSchedQueue[it.first] = new PriorityHandler1(this->pType, it.second, noOfSubseq);
				}
				break;
			}
			case PriorityHandler1::priorityType::RANDOM: {
				srand((unsigned int) time(nullptr));
				for (auto it : this->g.Vertices()) {
					if (this->resourceModel.getResource(it)->getLimit() >= 0) {
						this->priorityForSchedQueue[it] = new PriorityHandler1(this->pType, rand());
					}
				}
				break;
			}
			case PriorityHandler1::priorityType::CUSTOM: {
				// just check if all priorities are set
				for (auto it : this->g.Vertices()) {
					if (this->resourceModel.getResource(it)->getLimit() >= 0) {
						try {
							this->priorityForSchedQueue.at(it);
						}
						catch (std::out_of_range &) {
							throw HatScheT::Exception("Priority for vertex '" + it->getName() + "' is not set");
						}
					}
				}
				break;
			}
			default:
				throw HatScheT::Exception("No priority type for scheduling queue order specified");
		}
	}

	RationalIIModuloSDCScheduler::~RationalIIModuloSDCScheduler() {
		for (auto it : this->additionalConstraints) delete it.second;
		for (auto it : this->priorityForSchedQueue) delete it.second;
	}

	void RationalIIModuloSDCScheduler::scheduleInstruction(Vertex *I, int t) {
		ScaLP::Constraint c(this->scalpVariables.at(I) == t);
		this->createAdditionalConstraint(I, c);
		//this->mrt.insertVertex(I,t);
		this->prevSched[I] = t;
	}

	void RationalIIModuloSDCScheduler::handleTimeout() {
		this->timeBudget = this->solverTimeout;
		this->timeTracker = std::chrono::high_resolution_clock::now();
	}

	void RationalIIModuloSDCScheduler::printAdditionalSolverConstraints() {
		for (auto it : this->additionalConstraints) {
			if (!this->quiet) cout << (*it.second) << endl;
		}
	}

	int RationalIIModuloSDCScheduler::getNoOfSubsequentVertices(Vertex *v) {
		int noOfSubseq = 0;
		std::map<Vertex *, bool> visited;
		for (auto it : this->g.Vertices()) {
			visited[it] = false;
		}
		visited[v] = true; //No need to visit myself again

		std::list<Vertex *> queue = {v};

		while (!queue.empty()) {
			Vertex *pop = queue.front();
			queue.pop_front();

			for (auto it : this->g.Edges()) {
				if (&it->getVertexSrc() == pop) {
					Vertex *dst = &it->getVertexDst();
					if (!visited[dst]) {
						visited[dst] = true;
						queue.emplace_back(dst);
						noOfSubseq++;
					}
				}
			}
		}
		if (!this->quiet) cout << "Vertex: " << v->getName() << " Vertex Count: " << noOfSubseq << endl;
		return noOfSubseq;
	}

	map<Vertex *, int> RationalIIModuloSDCScheduler::getASAPScheduleWithoutResourceConstraints() {
		auto resM = this->getUnlimitedResourceModel();
		auto asap = ASAPScheduler(this->g, *resM);
		asap.schedule();
		delete resM;
		return asap.getSchedule();
	}

	map<Vertex *, int> RationalIIModuloSDCScheduler::getALAPScheduleWithoutResourceConstraints() {
		auto resM = this->getUnlimitedResourceModel();
		auto alap = ALAPScheduler(this->g, *resM);
		alap.schedule();
		delete resM;
		return alap.getSchedule();
	}

	ResourceModel *RationalIIModuloSDCScheduler::getUnlimitedResourceModel() {
		auto resM = new ResourceModel();
		for (auto it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); it++) {
			// copy resource but make it unlimited
			auto originalRes = (*it);
			int limit = -1; // unlimited resource
			// special_loop always throws an error if it's not set to 1
			if (originalRes->getName() == "special_loop") limit = 1;
			Resource &res = resM->makeResource(originalRes->getName(), limit, originalRes->getLatency(),
																				 originalRes->getBlockingTime());
			auto vertices = this->resourceModel.getVerticesOfResource(originalRes);
			for (auto it2 : vertices) {
				resM->registerVertex(it2, &res);
			}
		}
		return resM;
	}

	list<Vertex *> RationalIIModuloSDCScheduler::getResourceConstrainedVertices() {
		list<Vertex *> returnMe;
		for (auto it : this->g.Vertices()) {
			if (this->resourceModel.getResource(it)->getLimit() >= 0)
				returnMe.emplace_back(it);
		}
		return returnMe;
	}

	void RationalIIModuloSDCScheduler::setPriority(Vertex *v, PriorityHandler1 p) {
		if (this->pType != PriorityHandler1::priorityType::CUSTOM)
			throw HatScheT::Exception("ModuloSDCScheduler::setPriority: priority type must be CUSTOM but is " +
																PriorityHandler1::getPriorityTypeAsString(this->pType));
		try {
			auto a = this->priorityForSchedQueue.at(v);
			delete a;
		}
		catch (std::out_of_range &) {
			// chill for a second and enjoy the view
		}
		this->priorityForSchedQueue[v] = new PriorityHandler1(p);
	}

	std::map<Edge *, int> RationalIIModuloSDCScheduler::getLifeTimes() {
		if (this->startTimes.empty())
			throw HatScheT::Exception("ModuloSDCScheduler.getLifeTimes: cant return lifetimes! no startTimes determined!");

		std::map<Edge *, int> lifetimes;

		for (auto it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it) {
			Edge *e = *it;
			Vertex *vSrc = &e->getVertexSrc();
			Vertex *vDst = &e->getVertexDst();
			int lifetime = this->startTimes[vDst] - this->startTimes[vSrc] - this->resourceModel.getVertexLatency(vSrc) +
										 e->getDistance() * (int) this->II;
			if (lifetime < 0) throw HatScheT::Exception("ModuloSDCScheduler.getLifeTimes: negative lifetime detected!");
			else lifetimes.insert(make_pair(e, lifetime));
		}
		return lifetimes;
	}

	void RationalIIModuloSDCScheduler::setOutputsOnLatestControlStep() {
		this->fastObjective = false;
		this->outputsEqualScheduleLength = true;
		if (this->vertexHasOutgoingEdges.empty()) {
			// initialize map
			for (auto it : this->g.Vertices()) {
				this->vertexHasOutgoingEdges[it] = false;
			}
			for (auto it : this->g.Edges()) {
				this->vertexHasOutgoingEdges[&it->getVertexSrc()] = true;
			}
		}
	}

	PriorityHandler1::PriorityHandler1(PriorityHandler1::priorityType p, int prio1, int prio2) :
		pType(p), firstPriority(prio1), secondPriority(prio2) {
	}

	void PriorityHandler1::putIntoSchedQueue(Vertex *v, const priorityType &p,
																					 const map<Vertex *, PriorityHandler1 *> *pHandlers,
																					 std::list<Vertex *> *schedQ) {
		PriorityHandler1 *pV;
		try {
			pV = pHandlers->at(v);
		}
		catch (std::out_of_range &) {
			stringstream err;
			err << "PriorityHandler::putIntoSchedQueue: can't find vertex '";
			if (v != nullptr) err << v->getName();
			err << "' (" << v << ") in priority map";
			throw HatScheT::Exception(err.str());
		}
		for (auto it = (*schedQ).begin(); it != (*schedQ).end(); it++) {
			PriorityHandler1 *pI;
			try {
				pI = pHandlers->at(*it);
			}
			catch (std::out_of_range &) {
				stringstream err;
				err << "PriorityHandler::putIntoSchedQueue: can't find vertex '";
				if ((*it) != nullptr) err << (*it)->getName();
				err << "' (" << (*it) << ") in priority map";
				throw HatScheT::Exception(err.str());
			}
			switch (p) {
				case priorityType::ASAP: {
					if (pV->getFirstPriority() < pI->getFirstPriority()) {
						schedQ->emplace(it, v);
						return;
					}
					break;
				}
				case priorityType::ALAP: {
					if (pV->getFirstPriority() < pI->getFirstPriority()) {
						schedQ->emplace(it, v);
						return;
					}
					break;
				}
				case priorityType::PERTUBATION: {
					if (pV->getFirstPriority() > pI->getFirstPriority()) {
						schedQ->emplace(it, v);
						return;
					}
					break;
				}
				case priorityType::MOBLAP: {
					if (pV->getFirstPriority() < pI->getFirstPriority()) {
						schedQ->emplace(it, v);
						return;
					} else if (pV->getFirstPriority() == pI->getFirstPriority() &&
										 pV->getSecondPriority() < pI->getSecondPriority()) {
						schedQ->emplace(it, v);
						return;
					}
					break;
				}
				case priorityType::ALABILITY: {
					if (pV->getFirstPriority() < pI->getFirstPriority()) {
						schedQ->emplace(it, v);
						return;
					} else if (pV->getFirstPriority() == pI->getFirstPriority() &&
										 pV->getSecondPriority() < pI->getSecondPriority()) {
						schedQ->emplace(it, v);
						return;
					}
					break;
				}
				case priorityType::MOBILITY_HIGH: {
					if (pV->getFirstPriority() > pI->getFirstPriority()) {
						schedQ->emplace(it, v);
						return;
					}
					break;
				}
				case priorityType::MOBILITY_LOW: {
					if (pV->getFirstPriority() < pI->getFirstPriority()) {
						schedQ->emplace(it, v);
						return;
					}
					break;
				}
				case priorityType::SUBSEQUALAP: {
					if (pV->getFirstPriority() > pI->getFirstPriority()) {
						schedQ->emplace(it, v);
						return;
					} else if (pV->getFirstPriority() == pI->getFirstPriority() &&
										 pV->getSecondPriority() < pI->getSecondPriority()) {
						schedQ->emplace(it, v);
						return;
					}
					break;
				}
				case priorityType::ALASUB: {
					if (pV->getFirstPriority() < pI->getFirstPriority()) {
						schedQ->emplace(it, v);
						return;
					} else if (pV->getFirstPriority() == pI->getFirstPriority() &&
										 pV->getSecondPriority() > pI->getSecondPriority()) {
						schedQ->emplace(it, v);
						return;
					}
					break;
				}
				case priorityType::RANDOM: {
					if (pV->getFirstPriority() > pI->getFirstPriority()) {
						schedQ->emplace(it, v);
						return;
					}
					break;
				}
				case priorityType::CUSTOM: {
					if (pV->getFirstPriority() > pI->getFirstPriority()) {
						schedQ->emplace(it, v);
						return;
					}
					break;
				}
				default: {
					throw HatScheT::Exception("PriorityHandler::putIntoSchedQueue: unknown priority type (" +
																		PriorityHandler1::getPriorityTypeAsString(p) + ")");
				}
			}
		}
		schedQ->emplace_back(v);
	}

	std::string PriorityHandler1::getPriorityTypeAsString(const priorityType &p) {
		switch (p) {
			case priorityType::ASAP: {
				return "ASAP";
			}
			case priorityType::ALAP: {
				return "ALAP";
			}
			case priorityType::PERTUBATION: {
				return "PERTUBATION";
			}
			case priorityType::MOBLAP: {
				return "MOBLAP";
			}
			case priorityType::ALABILITY: {
				return "ALABILITY";
			}
			case priorityType::MOBILITY_HIGH: {
				return "MOBILITY_HIGH";
			}
			case priorityType::MOBILITY_LOW: {
				return "MOBILITY_LOW";
			}
			case priorityType::RANDOM: {
				return "RANDOM";
			}
			case priorityType::CUSTOM: {
				return "CUSTOM";
			}
			case priorityType::SUBSEQUALAP: {
				return "SUBSEQUALAP";
			}
			case priorityType::ALASUB: {
				return "ALASUB";
			}
			case priorityType::NONE: {
				return "NONE";
			}
		}
		return "UNKNOWN";
	}

	PriorityHandler1::priorityType PriorityHandler1::getPriorityTypeFromString(std::string priorityTypeStr) {
		if (priorityTypeStr == "ASAP") return priorityType::ASAP;
		if (priorityTypeStr == "ALAP") return priorityType::ALAP;
		if (priorityTypeStr == "PERTUBATION") return priorityType::PERTUBATION;
		if (priorityTypeStr == "MOBLAP") return priorityType::MOBLAP;
		if (priorityTypeStr == "ALABILITY") return priorityType::ALABILITY;
		if (priorityTypeStr == "MOBILITY_HIGH") return priorityType::MOBILITY_HIGH;
		if (priorityTypeStr == "MOBILITY_LOW") return priorityType::MOBILITY_LOW;
		if (priorityTypeStr == "RANDOM") return priorityType::RANDOM;
		if (priorityTypeStr == "CUSTOM") return priorityType::CUSTOM;
		if (priorityTypeStr == "SUBSEQUALAP") return priorityType::SUBSEQUALAP;
		if (priorityTypeStr == "ALASUB") return priorityType::ALASUB;
		return priorityType::NONE;
	}

	bool RationalIIModuloSDCScheduler::manageTimeBudgetSuccess() {
		std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();
		std::chrono::milliseconds timeSpan = std::chrono::duration_cast<std::chrono::milliseconds>(tp - this->timeTracker);
		double elapsedTime = ((double) timeSpan.count()) / 1000.0;
		//std::cout << "elapsed time: " << elapsedTime<<endl;
		//std::cout << "time budget before minus elapsed time: " << this->solverTimeout<<endl;
		this->timeTracker = tp;
		this->timeBudget -= elapsedTime;
		//std::cout << "time budget after minus elapsed time: " << this->solverTimeout<<endl;
		if (this->timeBudget < 0) this->scalpStatus = ScaLP::status::TIMEOUT_INFEASIBLE;
		return this->timeBudget >= 0.0;
	}

	void RationalIIModuloSDCScheduler::calcDeltaMins() {
		if (this->initiationIntervals.empty() or this->latencySequence.empty())
			throw HatScheT::Exception(
				"UniformRationalIISchedulerNew::calcDeltaMins: need to specify initiation intervals/latency sequence");
		if (this->samples <= 0 or this->modulo <= 0)
			throw HatScheT::Exception("UniformRationalIISchedulerNew::calcDeltaMins: need to specify samples and modulo");
		// distance 0 is trivial
		this->deltaMins[0] = 0;
		if (!this->quiet) std::cout << "set min delta (0) = " << 0 << std::endl;
		for (auto &e : this->g.Edges()) {
			auto edgeDistance = e->getDistance();
			// check if delta for this distance was already calculated
			if (edgeDistance == 0) continue;
			if (this->deltaMins.find(edgeDistance) != this->deltaMins.end()) continue;
			// calc minimum delta
			unsigned int minDelta = 10000000; // 'infinity'
			for (auto offset = 0; offset < this->samples; ++offset) {
				unsigned int delta = 0;
				for (auto d = 0; d < edgeDistance; ++d) {
					delta += this->latencySequence[(offset + d) % this->samples];
				}
				if (delta < minDelta) minDelta = delta;
			}
			this->deltaMins[edgeDistance] = minDelta;
			if (!this->quiet)
				std::cout << "set min delta (" << edgeDistance << ") = " << this->deltaMins[edgeDistance] << std::endl;
		}
	}

	void RationalIIModuloSDCScheduler::setMRT(RationalIIModuloSDCSchedulerMRT &mrt, ResourceModel &resourceModel,
																						std::vector<int> &initiationIntervals, int samples, int modulo,
																						bool quiet) {
		if (!quiet) {
			std::cout << "setting MRT for Initiation Intervals '";
			for (auto l : initiationIntervals) {
				std::cout << l << " ";
			}
			std::cout << "'" << std::endl;
		}
		mrt.setResourceModelAndII(resourceModel, modulo);
		if (samples == 1) {
			// construct trivial MRT
			for (auto res : resourceModel.Resources()) {
				auto limit = res->getLimit();
				bool limited = limit > 0;
				int height = limit;
				if (!limited)
					height = samples * resourceModel.getVerticesOfResource(res).size();
				for (int i = 0; i < modulo; ++i) {
					mrt.specifyColumnHeight(res, i, height);
				}
			}
			return;
		}
		// "copy" resource model (only relevant info)
		ResourceModel rm;
		std::map<const Resource *, const Resource *> resourceMap;
		for (auto res : resourceModel.Resources()) {
			auto *copiedResource = &rm.makeResource(res->getName(), res->getLimit(), res->getLatency(),
																							res->getBlockingTime());
			resourceMap[copiedResource] = res;
			if (!quiet) {
				std::cout << "Created resource copy from '" << res->getName() << "' -> '" << copiedResource->getName() << "'"
									<< std::endl;
			}
		}
		// construct rectangular dummy MRT
		if (!quiet) {
			std::cout << "Constructing dummy MRT now" << std::endl;
		}
		RationalIIModuloSDCSchedulerMRT dummyMRT;
		dummyMRT.setResourceModelAndII(rm, modulo);
		for (auto res : rm.Resources()) {
			auto limit = res->getLimit();
			auto numberOfVertices = (unsigned int) resourceModel.getVerticesOfResource(resourceMap[res]).size();
			if (limit < 0) {
				auto noOfVertices = resourceModel.getVerticesOfResource(res).size();
				for (int i = 0; i < modulo; ++i) {
					mrt.specifyColumnHeight(resourceMap[res], i, noOfVertices * samples);
				}
				continue;
			}
			if (!quiet) {
				std::cout << "Finished constructing dummy MRT now" << std::endl;
			}
			// handle case - vertices for the resource is 1 => build trivial MRT with height=limit
			if (numberOfVertices == 1) {
				if (!quiet)
					std::cout << "Found limited resource (" << res->getName()
										<< ") with only one vertex registered to it - build trivial MRT for this resource" << std::endl;
				for (unsigned int i = 0; i < modulo; ++i) {
					mrt.specifyColumnHeight(resourceMap[res], i, resourceMap[res]->getLimit());
					if (!quiet)
						std::cout << "specified MRT column height1 = " << mrt.getHeight(resourceMap[res], i) << " for resource "
											<< resourceMap[res]->getName() << " at column " << i << std::endl;
				}
				continue;
			}
			// handle case - resource limit modulo #samples == 0 => build trivial MRT with height=limit/#samples
			if (!quiet) std::cout << "samples value: " << samples << endl;
			if (limit % samples == 0) {
				if (!quiet)
					std::cout << "Found limited resource with limit modulo #samples == 0 - build trivial MRT for this resource"
										<< std::endl;
				for (unsigned int i = 0; i < modulo; ++i) {
					mrt.specifyColumnHeight(resourceMap[res], i, resourceMap[res]->getLimit() / samples);
					if (!quiet)
						std::cout << "specified MRT column height2 = " << mrt.getHeight(resourceMap[res], i) << " for resource "
											<< resourceMap[res]->getName() << " at column " << i << std::endl;
				}
				continue;
			}
			if (!quiet) std::cout << "vertices counter: " << numberOfVertices << endl;
			for (unsigned int i = 0; i < modulo; ++i) {
				if (!quiet) std::cout << "i=" << i << ", limit = " << limit << std::endl;
				mrt.specifyColumnHeight(resourceMap[res], i, Utility::hFunction(numberOfVertices, modulo, i));
				if (!quiet)
					std::cout << "specified MRT column height3= " << mrt.getHeight(resourceMap[res], i) << " for resource "
										<< resourceMap[res]->getName() << " at column " << i << std::endl;
			}
		}
	}

	RationalIIModuloSDCSchedulerMRT::RationalIIModuloSDCSchedulerMRT() : quiet(true), rm(nullptr), II(0), mrt() {}

	RationalIIModuloSDCSchedulerMRT::RationalIIModuloSDCSchedulerMRT(ResourceModel &rm, unsigned int II) : quiet(true),
																																																				 rm(&rm),
																																																				 II(II) {
		for (auto resIt = this->rm->resourcesBegin(); resIt != this->rm->resourcesEnd(); ++resIt) {
			auto res = *resIt;
			// insert matrix for each resource
			std::vector<std::vector<Vertex *>> matrix;
			for (int i = 0; i < res->getLimit(); ++i) {
				std::vector<Vertex *> row(this->II, nullptr);
				matrix.emplace_back(row);
			}
			if (res->getLimit() > 0) this->mrt[const_cast<const Resource *>(res)] = matrix;
		}
	}

	bool RationalIIModuloSDCSchedulerMRT::insertVertex(Vertex *v, unsigned int moduloSlot) {
		if (!this->quiet) cout << "moduloslot: " << moduloSlot << endl;
		if (!this->quiet) cout << "II: " << this->II << endl;
		moduloSlot = moduloSlot % this->II;
		if (!this->quiet) cout << "moduloslot: " << moduloSlot << endl;
		//int limit = this->rm->getResource(v)->getLimit();
		if (moduloSlot >= this->II)
			throw HatScheT::Exception(
				"Invalid modulo slot requested: " + to_string(moduloSlot) + ", II=" + to_string(this->II));
		auto *res = this->rm->getResource(v);
		if (!this->quiet) cout << "resource: " << this->rm->getResource(v)->getName() << endl;
		if (this->mrt.find(res) == this->mrt.end())
			throw HatScheT::Exception("Invalid vertex provided, its resource type doesn't exist in MRT");
		auto &column = this->mrt[res][moduloSlot];
		for (auto &it : column) {
			if (!this->quiet) cout << "it: " << it << endl;
			if (it == nullptr) {
				it = v;
				return true;
			}
		}
		return false;
	}

	bool RationalIIModuloSDCSchedulerMRT::removeVertex(Vertex *v) {
		bool removed = false;
		try {
			for (auto &column : this->mrt.at(this->rm->getResource(v))) {
				for (auto &it : column) {
					if (it == v) {
						removed = true;
						it = nullptr;
					}
				}
			}
		}
		catch (std::out_of_range &) {
			// resource does not exist in MRT
			return false;
		}

		return removed;
	}

	void RationalIIModuloSDCSchedulerMRT::print() const {
		std::cout << "MRT" << std::endl;
		for (auto &it : this->mrt) {
			std::cout << "  Resource " << it.first->getName() << std::endl;
			unsigned int maxHeight = 0;
			for (auto &it2 : it.second) {
				if (it2.size() > maxHeight) maxHeight = (unsigned int) it2.size();
			}
			for (unsigned int row = 0; row < maxHeight; ++row) {
				for (unsigned int column = 0; column < this->II; ++column) {
					if (it.second.at(column).size() <= row) {
						std::cout << "---------- ";
						continue;
					}
					if (it.second.at(column).at(row) == nullptr) {
						std::cout << "0000000000 ";
						continue;
					}
					auto name = it.second.at(column).at(row)->getName();
					std::cout << name << " ";
					for (auto i = 0; i < (10 - name.size()); ++i) {
						std::cout << " ";
					}
				}
				std::cout << std::endl;
			}
		}
	}

	std::vector<int> RationalIIModuloSDCSchedulerMRT::getModuloSlots(Vertex *v) const {
		vector<int> slots;
		if (v == nullptr)
			throw HatScheT::Exception("ModuloQMRT::getModuloSlots: can't request nullptr");
		for (auto &row : this->mrt.at(this->rm->getResource(v))) {
			int slot = 0;
			for (auto vertex : row) {
				if (vertex == v) slots.emplace_back(slot);
				++slot;
			}
		}
		return slots;
	}

	int RationalIIModuloSDCSchedulerMRT::getHeight(const Resource *res, int column) const {
		try {
			return (int) this->mrt.at(res).at(column).size();
		}
		catch (...) {
			throw HatScheT::Exception("Invalid resource or MRT column requested");
		}
	}

	void
	RationalIIModuloSDCSchedulerMRT::specifyColumnHeight(const Resource *res, unsigned int column, unsigned int height) {
		if (this->mrt.find(res) == this->mrt.end())
			throw HatScheT::Exception("Invalid resource '" + res->getName() + "' provided - it does not exist in MRT");
		if (column >= this->II)
			throw HatScheT::Exception("Invalid column requested: " + to_string(column) + ", II=" + to_string(this->II));
		this->mrt.at(res).at(column).resize(height);
	}

	void RationalIIModuloSDCSchedulerMRT::setResourceModelAndII(ResourceModel &rm, unsigned int II) {
		this->II = II;
		this->rm = &rm;
		this->mrt.clear();
		for (auto resIt = this->rm->resourcesBegin(); resIt != this->rm->resourcesEnd(); ++resIt) {
			auto res = *resIt;
			auto limit = res->getLimit();
			//if(limit<0) continue;
			this->rotations[res] = 0;
			for (unsigned int i = 0; i < II; ++i) {
				this->mrt[res].emplace_back(std::vector<Vertex *>());
			}
		}
	}

	void RationalIIModuloSDCSchedulerMRT::rotateLeft() {
		bool newRotation = true;
		for (auto &innerMrt : this->mrt) {
			if (newRotation) {
				// some resources in this MRT might have rectangular matrices that do not have to be rotated
				auto rotatable = false;
				auto height = innerMrt.second.front().size();
				for (auto &column : innerMrt.second) {
					if (column.size() != height) {
						rotatable = true;
						break;
					}
				}
				if (!rotatable) {
					if (!this->quiet)
						std::cout << "resource '" << innerMrt.first->getName() << "' is not rotatable - skip it" << std::endl;
					continue;
				}

				// THE MATRIX OF THIS RESOURCE CAN BE ROTATED!
				++this->rotations[innerMrt.first];
				if (this->rotations[innerMrt.first] == this->II) {
					this->rotations[innerMrt.first] = 0;
				} else {
					newRotation = false;
				}
				if (!this->quiet)
					std::cout << "rotate resource '" << innerMrt.first->getName() << "'; rotation counter: "
										<< this->rotations[innerMrt.first] << std::endl;
				std::vector<Vertex *> backup = innerMrt.second[0];
				for (unsigned int i = 0; i < innerMrt.second.size() - 1; ++i) {
					innerMrt.second[i] = innerMrt.second[i + 1];
				}
				innerMrt.second[innerMrt.second.size() - 1] = backup;
			}
		}
	}

	unsigned long RationalIIModuloSDCSchedulerMRT::getMaxNumberOfRotations() {
		unsigned long rot = 1;
		for (auto &it : this->mrt) {
			auto matrix = it.second;
			auto height = matrix.front().size();
			for (auto &column : matrix) {
				if (column.size() != height) {
					rot *= this->II;
					break;
				}
			}
		}
		return rot;
	}

	std::list<Vertex *> RationalIIModuloSDCSchedulerMRT::getVerticesInModuloSlot(int m, const Resource *res) {
		if (m >= this->mrt[res].size()) {
			throw Exception("ModuloQMRT::getVerticesInModuloSlot: invalid modulo slot requested");
		}
		list<Vertex *> l;
		for (auto v : this->mrt[res][m]) {
			if (v != nullptr) l.emplace_back(v);
		}
		return l;
	}

	void RationalIIModuloSDCSchedulerMRT::clearMRT() {
		this->mrt.clear();
	};

	std::map<const Resource *, std::vector<std::vector<Vertex *>>> RationalIIModuloSDCSchedulerMRT::getMRT() const {
		return this->mrt;
	};

	std::list<Vertex *> RationalIIModuloSDCSchedulerMRT::getResourceConflicts(Vertex *I, const int &evictTime) {

		std::list<Vertex *> l;
		if (evictTime >= this->II)
			throw HatScheT::Exception(
				"Invalid modulo slot requestedtest: " + to_string(evictTime) + ", II=" + to_string(this->II));
		auto *res = this->rm->getResource(I);
		if (this->mrt.find(res) == this->mrt.end())
			throw HatScheT::Exception("Invalid vertex provided, its resource type doesn't exist in MRT");
		auto &column = this->mrt[res][evictTime];
		for (auto &it : column) {
			l.emplace_back(it);
			if (it == nullptr) {
				//return empty list because no resource conflict
				return std::list<Vertex *>();
			}
		}
		return l;
	};
}
