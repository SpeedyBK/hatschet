//
// Created by nfiege on 17/02/21.
//

#include "RationalIIModuloSDCScheduler.h"
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/scheduler/ALAPScheduler.h>
#include <math.h>

namespace HatScheT {
	RationalIIModuloSDCScheduler::RationalIIModuloSDCScheduler(Graph &g, ResourceModel &resourceModel,
																														 std::list<std::string> solverWishlist) :
		RationalIISchedulerLayer(g, resourceModel), ILPSchedulerBase(solverWishlist), initiationIntervals(), budget(0),
		timeBudget(0.0) {
		this->calculatePriorities(); // calculate priorities for scheduling queue
	}

	void RationalIIModuloSDCScheduler::scheduleIteration() {
		this->timeTracker = std::chrono::high_resolution_clock::now();
		if(!this->quiet) {
			std::cout << "RationalIIModuloSDCScheduler::scheduleIteration start scheduling for II = M/S = " << this->modulo
				<< "/" << this->samples << std::endl;
		}
		// set time budget
		this->timeBudget = this->solverTimeout;
		// delete scheduling constraints from previous iteration
		this->clearAllAdditionalConstraints();
		// calc budget according to ModuloSDC paper
		this->budget = (int) 6 * this->g.getNumberOfVertices();
		//clear up and reset
		this->solver->reset();
		this->resetContainer();

		//init latency sequence, init intervals, deltaMin containers
		this->initiationIntervals = RationalIISchedulerLayer::getOptimalInitiationIntervalSequence(this->samples,
																																															 this->modulo,
																																															 this->quiet);
		this->latencySequence = RationalIISchedulerLayer::getLatencySequenceFromInitiationIntervals(
			this->initiationIntervals, this->modulo);
		if(!this->quiet) {
			std::cout << "RationalIIModuloSDCScheduler::scheduleIteration: insertion times:" << std::endl;
			for(auto it : this->initiationIntervals) {
				std::cout << "  " << it << std::endl;
			}
			std::cout << "RationalIIModuloSDCScheduler::scheduleIteration: latency sequence:" << std::endl;
			for(auto it : this->latencySequence) {
				std::cout << "  " << it << std::endl;
			}
		}

		if(!this->quiet) {
			std::cout << "RationalIIModuloSDCScheduler::scheduleIteration: calculating delta mins now" << std::endl;
		}
		this->calcDeltaMins();

		// init MRT for the given M
		if(!this->quiet) {
			std::cout << "RationalIIModuloSDCScheduler::scheduleIteration: initializing MRT for M=" << this->modulo
				<< std::endl;
		}
		this->mrt.setResourceModelAndII(this->resourceModel, this->modulo);
		for (auto res : this->resourceModel.Resources()) {
			if(!this->quiet) {
				std::cout << "RationalIIModuloSDCScheduler::scheduleIteration: initializing MRT for resource '"
					<< res->getName() << "'" << std::endl;
			}
			if(res->isUnlimited()) continue;
			for (auto m = 0; m < this->modulo; ++m) {
				this->mrt.specifyColumnHeight(res, m, res->getLimit());
			}
		}

		// LINE 1 - initial schedule
		if(!this->quiet) {
			std::cout << "RationalIIModuloSDCScheduler::scheduleIteration: creating initial schedule now" << std::endl;
		}
		this->createInitialSchedule();
		this->sdcTimes = this->asapTimes;

		// LINE 2 - create queue
		if(!this->quiet) {
			std::cout << "RationalIIModuloSDCScheduler::scheduleIteration: creating scheduling queue" << std::endl;
		}
		this->createSchedulingQueue(this->getResourceConstrainedVertices());

		while (!this->schedQueue.empty() and budget >= 0) {
			if(!this->quiet) {
				std::cout << "RationalIIModuloSDCScheduler::scheduleIteration: scheduling queue:" << std::endl;
				for (auto v : this->schedQueue) {
					std::cout << "  " << v->getName() << std::endl;
				}
			}
			// LINE 4 - pop first element from scheduling queue
			Vertex *I = this->schedQueue.front();
			this->schedQueue.pop_front();

			// LINE 5 - get control step
			int time;
			try {
				time = this->sdcTimes.at(I);
			}
			catch (std::out_of_range &) {
				throw HatScheT::Exception("sdc times container corrupt, can't find instruction " + I->getName());
			}
			if (time < 0) {
				throw HatScheT::Exception("Error: RationalIIModuloSDCScheduler::scheduleIteration: invalid time (" +
																	to_string(time) + ") found by ILP solver for instruction '" + I->getName() + "'");
			}
			if(!this->quiet) {
				std::cout << "RationalIIModuloSDCScheduler::scheduleIteration: try scheduling vertex '" << I->getName() << "'"
									<< " at time " << time << std::endl;
			}

			if (!this->hasResourceConflict(I, time)) {
				// LINE 7-8 - schedule instruction
				this->scheduleInstruction(I, time);
			} else {
				if(!this->quiet) {
					std::cout
						<< "RationalIIModuloSDCScheduler::scheduleIteration: detected resource conflict when trying to schedule '"
						<< I->getName() << "' in time slot " << time << std::endl;
				}
				// LINE 10 - add constraint t_I >= time+offset to ilp formulation
				this->clearConstraintForVertex(I);
				auto offsetTime = getNextFreeTimeSlot(I, time);
				// add constraint t_I >= time+1 to ilp formulation
				this->additionalConstraints[I] = new ScaLP::Constraint(this->tVariables[I] >= time+offsetTime);
				if(!this->quiet) {
					std::cout
						<< "RationalIIModuloSDCScheduler::scheduleIteration: offset for instruction '" << I->getName() << "': "
						<< offsetTime << std::endl;
				}

				if (offsetTime < 1) {
					if(!this->quiet) {
						std::cout
							<< "RationalIIModuloSDCScheduler::scheduleIteration: can not find available modulo slot for instruction '"
							<< I->getName() << "'; need backtracking" << std::endl;
					}
					// LINES 15 clear constraint
					this->clearConstraintForVertex(I);
					// LINE 16: backtracking
					try {
						//neededBacktrackingForOffset = true;
						this->backtracking(I, time, true);
					}
					catch (HatScheT::TimeoutException &e) {
						this->handleTimeout();
						this->failSchedulingAttempt();
						return;
					}
					// LINE 17: solve sdc again
					try {
						bool foundSolution = this->solveSDCProblem();
						if (!foundSolution) {
							cout << "ERROR: ModSDC::modSDCIteration: Pseudocode line 17; solver should always find solution" << endl;
							throw HatScheT::Exception(
								"ModSDC::modSDCIteration: Pseudocode line 17; solver should always find solution");
						}
					}
					catch (HatScheT::TimeoutException &e) {
						this->handleTimeout();
						this->failSchedulingAttempt();
						return;
					}
					this->budget--;
					continue;
				}

				// LINE 11: solve sdc
				bool foundSolution;
				try {
					foundSolution = this->solveSDCProblem();
				}
				catch (HatScheT::TimeoutException &e) {
					this->handleTimeout();
					this->failSchedulingAttempt();
					return;
				}
				if (foundSolution) {
					// LINE 13: put into queue
					PriorityHandler::putIntoSchedQueue(I, PriorityHandler::priorityType::SUBSEQUALAP, &this->priorityForSchedQueue, &this->schedQueue);
				} else {
					// LINES 15 clear constraint
					this->clearConstraintForVertex(I);
					// LINE 16: backtracking
					try {
						this->backtracking(I, time);
					}
					catch (HatScheT::TimeoutException &e) {
						this->handleTimeout();
						this->failSchedulingAttempt();
						return;
					}
					// LINE 17: solve sdc again
					try {
						foundSolution = this->solveSDCProblem();
						if (!foundSolution) {
							cout << "ERROR: ModSDC::modSDCIteration: Pseudocode line 17; solver should always find solution" << endl;
							throw HatScheT::Exception(
								"ModSDC::modSDCIteration: Pseudocode line 17; solver should always find solution");
						}
					}
					catch (HatScheT::TimeoutException &e) {
						this->handleTimeout();
						this->failSchedulingAttempt();
						return;
					}
				}
			}
			// LINE 20 - decrement budget
			this->budget--;
		}
		// check if scheduling queue is empty
		// success if it is
		// fail if not
		if (this->schedQueue.empty()) {
			// success
			this->startTimes = this->sdcTimes; // last result from ILP solver is a valid schedule!
			this->startTimesVector.resize(this->samples);
			for (auto i = 0; i < this->samples; ++i) {
				for (auto v : this->g.Vertices()) {
					this->startTimesVector[i][v] = this->sdcTimes[v] + this->initiationIntervals[i];
				}
			}
			this->scheduleFound = true;
		} else {
			// fail
			this->failSchedulingAttempt();
		}
	}

	void RationalIIModuloSDCScheduler::constructProblem() {
		if(!this->quiet)
			std::cout << "RationalIIModuloSDCScheduler::constructProblem: start" << std::endl;
		// create scalp variables
		this->fillTContainer();
		// dependency constraints
		for (auto *edge : this->g.Edges()) {
			auto &src = edge->getVertexSrc();
			auto &dst = edge->getVertexDst();
			ScaLP::Constraint c =
				this->tVariables.at(&src) - this->tVariables.at(&dst) <=
					(int)(this->deltaMins[edge->getDistance()] - edge->getDelay() -
					this->resourceModel.getResource(&src)->getLatency());
			if(!this->quiet)
				std::cout << "RationalIIModuloSDCScheduler::constructProblem: add constraint '" << c << "'" << std::endl;
			this->solver->addConstraint(c);
		}
		// additional constraints to handle resource constraints
		for (auto it : this->additionalConstraints) {
			this->solver->addConstraint(*it.second);
			if(!this->quiet)
				std::cout << "RationalIIModuloSDCScheduler::constructProblem: add constraint '" << *it.second << "'" << std::endl;
		}
	}

	void RationalIIModuloSDCScheduler::setObjective() {
		//supersink latency objective
		ScaLP::Variable supersink = ScaLP::newIntegerVariable("supersink");

		for (auto &v : this->g.Vertices())
			this->solver->addConstraint(supersink - this->tVariables[v] - this->resourceModel.getVertexLatency(v) >= 0);

		if (this->maxLatencyConstraint > 0)
			this->solver->addConstraint(supersink <= this->maxLatencyConstraint);

		this->solver->setObjective(ScaLP::minimize(supersink));
	}

	void RationalIIModuloSDCScheduler::resetContainer() {
		this->asapTimes.clear();
		this->sdcTimes.clear();
		this->prevSched.clear();
		this->schedQueue.clear();
		this->tVariables.clear();
		this->additionalConstraints.clear();
	}

	void RationalIIModuloSDCScheduler::fillTContainer() {
		if (!this->tVariables.empty()) return;
		// create one time variable for each vertex in the graph
		for (auto &v : this->g.Vertices()) {
			auto var = ScaLP::newIntegerVariable(v->getName(),0,ScaLP::INF());
			this->tVariables[v] = var;
		}
	}

	void RationalIIModuloSDCScheduler::calcDeltaMins() {
		if (this->initiationIntervals.empty() or this->latencySequence.empty())
			throw HatScheT::Exception(
				"RationalIIModuloSDCScheduler::calcDeltaMins: need to specify initiation intervals/latency sequence");
		if (this->samples <= 0 or this->modulo <= 0)
			throw HatScheT::Exception("RationalIIModuloSDCScheduler::calcDeltaMins: need to specify samples and modulo");
		// distance 0 is trivial
		this->deltaMins[0] = 0;
		if (!this->quiet)
			std::cout << "set min delta (0) = " << 0 << std::endl;
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

	void RationalIIModuloSDCScheduler::calculatePriorities() {
		auto pALAP = this->getALAPScheduleWithoutResourceConstraints();
		for (auto it : pALAP) {
			auto noOfSubseq = this->getNoOfSubsequentVertices(it.first);
			this->priorityForSchedQueue[it.first] = new PriorityHandler(PriorityHandler::priorityType::SUBSEQUALAP,
																																	noOfSubseq, it.second);
		}
	}

	map<Vertex *, int> RationalIIModuloSDCScheduler::getALAPScheduleWithoutResourceConstraints() {
		auto resM = this->getUnlimitedResourceModel();
		auto alap = ALAPScheduler(this->g, *resM);
		alap.schedule();
		delete resM;
		return alap.getSchedule();
	}

	map<Vertex *, int> RationalIIModuloSDCScheduler::getASAPScheduleWithoutResourceConstraints() {
		auto resM = this->getUnlimitedResourceModel();
		auto alap = ASAPScheduler(this->g, *resM);
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

	int RationalIIModuloSDCScheduler::getNoOfSubsequentVertices(Vertex *v) {
		int noOfSubseq = 0;
		std::map<Vertex *, bool> visited;
		for (auto it : this->g.Vertices()) {
			visited[it] = false;
		}

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
		return noOfSubseq;
	}

	void RationalIIModuloSDCScheduler::createInitialSchedule() {
		this->setUpScalp();
		ScaLP::status s = this->solver->solve(); // solver should never timeout here...
		if ((s != ScaLP::status::FEASIBLE) and (s != ScaLP::status::OPTIMAL) and (s != ScaLP::status::TIMEOUT_FEASIBLE)) {
			if (!this->quiet) {
				cout << "ScaLP Backend: " << this->solver->getBackendName() << endl;
				cout << "ScaLP Status: " << s << endl;
			}
			if (!this->quiet)
				cout
					<< "RationalIIModuloSDCScheduler::createInitialSchedule: failed to find schedule without resource constraints"
					<< endl;
			throw HatScheT::Exception(
				"RationalIIModuloSDCScheduler::createInitialSchedule: failed to find schedule without resource constraints");
		}

		auto r = this->solver->getResult().values;
		for (auto v : this->g.Vertices()) {
			this->asapTimes[v] = (int)round(r[this->tVariables[v]]);
			if(!this->quiet) {
				std::cout << "RationalIIModuloSDCScheduler::createInitialSchedule: ASAP time for '" << v->getName() << "': "
					<< this->asapTimes[v] << std::endl;
			}
		}
	}

	void RationalIIModuloSDCScheduler::setUpScalp() {
		this->initSolver();
		this->constructProblem();
		this->setObjective();
	}

	void RationalIIModuloSDCScheduler::initSolver() {
		this->solver->reset();
		this->solver->quiet = this->solverQuiet;
		this->solver->timeout = (long) this->timeBudget;
		if (this->solver->getBackendName() == "Dynamic: LPSolve") {
			this->solver->presolve = false;
			this->solver->threads = 0;
		} else {
			this->solver->presolve = true;
			if (this->threads > 0) this->solver->threads = this->threads;
		}

	}

	void RationalIIModuloSDCScheduler::clearConstraintForVertex(Vertex *v) {
		if (this->getAdditionalConstraint(v) != nullptr) {
			delete this->additionalConstraints.at(v);
			this->additionalConstraints.erase(v);
		}
	}

	void RationalIIModuloSDCScheduler::clearAllAdditionalConstraints() {
		for (auto it : this->g.Vertices()) {
			this->clearConstraintForVertex(it);
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

	void RationalIIModuloSDCScheduler::handleTimeout() {
		this->timeBudget = this->solverTimeout;
		this->timeTracker = std::chrono::high_resolution_clock::now();
	}

	bool RationalIIModuloSDCScheduler::manageTimeBudgetSuccess() {
		std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();
		std::chrono::milliseconds timeSpan = std::chrono::duration_cast<std::chrono::milliseconds>(tp - this->timeTracker);
		double elapsedTime = ((double) timeSpan.count()) / 1000.0;
		this->timeTracker = tp;
		this->timeBudget -= elapsedTime;
		return this->timeBudget >= 0.0;
	}

	RationalIIModuloSDCScheduler::~RationalIIModuloSDCScheduler() {
		for (auto it : this->additionalConstraints) delete it.second;
		for (auto it : this->priorityForSchedQueue) delete it.second;
	}

	list<Vertex *> RationalIIModuloSDCScheduler::getResourceConstrainedVertices() {
		list<Vertex *> returnMe;
		for (auto it : this->g.Vertices()) {
			if (this->resourceModel.getResource(it)->getLimit() >= 0)
				returnMe.emplace_back(it);
		}
		return returnMe;
	}

	void RationalIIModuloSDCScheduler::createSchedulingQueue(std::list<Vertex *> scheduleMe) {
		for (auto it : scheduleMe) {
			if (this->resourceModel.getResource(it)->getLimit() > 0) {
				PriorityHandler::putIntoSchedQueue(it, PriorityHandler::priorityType::SUBSEQUALAP, &this->priorityForSchedQueue,
																					 &this->schedQueue);
			}
		}
	}

	bool RationalIIModuloSDCScheduler::hasResourceConflict(Vertex *I, int t) {
		for (auto i : this->initiationIntervals) {
			bool valid = this->mrt.insertVertex(I, (t + i) % this->modulo);
			if (!valid) {
				this->mrt.removeVertex(I);
				if(!this->quiet) {
					std::cout << "RationalIIModuloSDCScheduler::hasResourceConflict: detected resource conflict scheduling '"
						<< I->getName() << "' in time " << t << "!" << std::endl;
					this->mrt.print();
				}
				return true;
			}
		}
		this->mrt.removeVertex(I);
		if(!this->quiet) {
			std::cout << "RationalIIModuloSDCScheduler::hasResourceConflict: detected NO resource conflict scheduling '"
								<< I->getName() << "' in time " << t << "!" << std::endl;
			//this->mrt.print();
		}
		return false;
	}

	void RationalIIModuloSDCScheduler::scheduleInstruction(Vertex *I, int t) {
		this->additionalConstraints[I] = new ScaLP::Constraint(this->tVariables.at(I) == t);
		for (auto i : this->initiationIntervals) {
			this->mrt.insertVertex(I, (t + i) % this->modulo);
		}
		this->prevSched[I] = t;
		if(!this->quiet) {
			std::cout << "RationalIIModuloSDCScheduler::scheduleInstruction: MRT after scheduling '" << I->getName()
				<< "' in time " << t << std::endl;
			this->mrt.print();
		}
	}

	int RationalIIModuloSDCScheduler::getNextFreeTimeSlot(Vertex *I, int time) {
		for (auto offset = 1; offset < this->modulo; ++offset) {
			if (!this->hasResourceConflict(I, time + offset)) {
				return offset;
			}
		}
		return -1;
	}

	void RationalIIModuloSDCScheduler::backtracking(Vertex *I, int time, bool mrtBacktracking) {
		if (!this->quiet) {
			std::cout << "RationalIIModuloSDCScheduler::backtracking: start for '" << I->getName() << "' and time " << time
				<< std::endl;
		}
		int minTime;
		try {
			minTime = this->asapTimes.at(I);
		}
		catch (std::out_of_range &) {
			throw HatScheT::Exception("ASAP times container corrupt, can't find instruction " + I->getName());
		}
		for (; minTime <= time; minTime++) {
			// LINE 2 - find time slot
			this->additionalConstraints[I] = new ScaLP::Constraint(this->tVariables.at(I) == minTime);
			bool foundSolution = this->solveSDCProblem();
			// LINE 3 - clear constraint when found
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
		// LINE 5 - get previously scheduled time
		int prevSchedTime = this->getPrevSched(I);
		int evictTime;
		if ((minTime >= prevSchedTime and !mrtBacktracking) or (minTime > prevSchedTime and mrtBacktracking) or prevSchedTime < 0) {
			// LINE 7 - set evict time
			evictTime = minTime;
		} else {
			// LINE 9 - set evict time
			evictTime = prevSchedTime + 1;
		}
		if (evictTime < 0) {
			throw HatScheT::Exception(
				"Error: RationalIIModuloSDCScheduler::backtracking: Invalid evict time (" + to_string(evictTime) +
				") for instruction '" + I->getName() + "'");
		}
		if (!this->quiet) {
			std::cout << "RationalIIModuloSDCScheduler::backtracking: min time = " << minTime << std::endl;
			std::cout << "RationalIIModuloSDCScheduler::backtracking: prev sched = " << prevSchedTime << std::endl;
			std::cout << "RationalIIModuloSDCScheduler::backtracking: evict time = " << evictTime << std::endl;
		}
		std::list<Vertex *> evictInst = this->getResourceConflicts(I, evictTime);
		// LINES 11-15 - unschedule instructions with resource conflicts
		for (auto it : evictInst) {
			if(!this->quiet) {
				std::cout << "RationalIIModuloSDCScheduler::backtracking: resource conflict detected for '" << it->getName()
					<< "'" << std::endl;
			}
			this->unscheduleInstruction(it);
			// PUT OPERATION BACK INTO QUEUE EVEN THO IT IS NOT SPECIFIED IN PSEUDOCODE!
			PriorityHandler::putIntoSchedQueue(it, PriorityHandler::priorityType::SUBSEQUALAP, &this->priorityForSchedQueue, &this->schedQueue);
		}
		// LINE 16 - check for dependency violation
		if (this->dependencyConflict(I, evictTime)) {
			if(!this->quiet) {
				std::cout << "RationalIIModuloSDCScheduler::backtracking: dependency conflict detected" << std::endl;
			}
			auto copy = this->additionalConstraints;
			for (auto it : copy) {
				// LINES 18-19 - unschedule all instructions
				this->unscheduleInstruction(it.first);
				// LINE 20 - put them back into queue
				PriorityHandler::putIntoSchedQueue(it.first, PriorityHandler::priorityType::SUBSEQUALAP, &this->priorityForSchedQueue, &this->schedQueue);
			}
		}
		// LINES 23-24 - schedule instruction
		this->scheduleInstruction(I, evictTime);
	}

	bool RationalIIModuloSDCScheduler::solveSDCProblem() {
		if(!this->quiet) {
			std::cout << "RationalIIModuloSDCScheduler::solveSDCProblem: START" << std::endl;
		}
		std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
		this->setUpScalp();
		if (!this->manageTimeBudgetSuccess()) {
			if (!this->quiet) cout << "Timeout!" << endl;
			this->timeouts++;
			throw HatScheT::TimeoutException(
				"Time budget empty when trying to find solution for II=" + to_string(this->II));
		}

		// solve ilp and track time
		ScaLP::status s = this->solver->solve();
		std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
		std::chrono::nanoseconds timeSpan = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1);

		if (s == ScaLP::status::TIMEOUT_INFEASIBLE) {
			if (!this->quiet) cout << "Timeout!" << endl;
			this->timeouts++;
			throw HatScheT::TimeoutException("Solver timeout when trying to find solution for II=" + to_string(this->II));
		}
		if ((s != ScaLP::status::FEASIBLE) && (s != ScaLP::status::OPTIMAL) && (s != ScaLP::status::TIMEOUT_FEASIBLE)) {
			return false;
		}

		auto r = this->solver->getResult().values;
		for (auto v : this->g.Vertices()) {
			this->sdcTimes[v] = (int) round(r[this->tVariables[v]]);
		}

		if (!this->quiet) {
			std::cout << "RationalIIModuloSDCScheduler::solveSDCProblem: found schedule:" << std::endl;
			for(auto it : this->sdcTimes) {
				std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
			}
		}

		return true;
	}

	void RationalIIModuloSDCScheduler::failSchedulingAttempt() {
		// reset timer for next II calculation
		this->timeBudget = this->solverTimeout;
		this->timeTracker = std::chrono::high_resolution_clock::now();
		this->scheduleFound = false;
	}

	int RationalIIModuloSDCScheduler::getPrevSched(Vertex *v) {
		try {
			return this->prevSched.at(v);
		}
		catch (std::out_of_range &) {
			return -1;
		}
	}

	std::list<Vertex *> RationalIIModuloSDCScheduler::getResourceConflicts(Vertex *I, int evictTime) {
		std::list<Vertex*> l;
		for(auto i : this->initiationIntervals) {
			auto vertices = this->mrt.getVerticesInModuloSlot((evictTime+i) % this->modulo,this->resourceModel.getResource(I));
			for(auto v : vertices) {
				// check if it's already in the list
				bool alreadyIn = false;
				for(auto it : l) {
					if(it == v) {
						alreadyIn = true;
						break;
					}
				}
				// put into it if not
				if(!alreadyIn) {
					l.emplace_back(v);
				}
			}
		}
		return l;
	}

	void RationalIIModuloSDCScheduler::unscheduleInstruction(Vertex *evictInst) {
		this->clearConstraintForVertex(evictInst);
		this->mrt.removeVertex(evictInst);
	}

	bool RationalIIModuloSDCScheduler::dependencyConflict(Vertex *I, int evictTime) {
		for (auto& e : this->g.Edges()) {
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

	bool
	RationalIIModuloSDCScheduler::dependencyConflictForTwoInstructions(Vertex *i, int newStartTime_i, int newStartTime_j,
																																		 int edgeDelay, int edgeDistance) {
		return ((newStartTime_i + this->resourceModel.getResource(i)->getLatency() + edgeDelay - newStartTime_j) >
						(this->deltaMins[edgeDistance]));
	}

}
