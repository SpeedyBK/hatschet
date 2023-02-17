//
// Created by nfiege on 12/20/22.
//

#include "SATSchedulerRes.h"
#include <HatScheT/utility/Utility.h>
#include <iomanip>

namespace HatScheT {

	SATSchedulerRes::SATSchedulerRes(Graph &g, ResourceModel &resourceModel, int II) :
		SATSchedulerBase(g, resourceModel, II) {
		this->los = LatencyOptimizationStrategy::REVERSE_LINEAR;
	}

	void SATSchedulerRes::scheduleIteration() {
		// call parent's method for some basic stuff
		SATSchedulerBase::scheduleIteration();

		this->scheduleTimeVariables.clear();
		//this->moduloSlotVariables.clear();
		this->constOneVar = 0;
		this->constZeroVar = 0;
		this->resourceConstraintLiteralCounter = 0;
		this->scheduleTimeLiteralCounter = 0;
		//this->moduloSlotLiteralCounter = 0;

		this->resourceConstraintClauseCounter = 0;
		this->dependencyConstraintClauseCounter = 0;
		this->scheduleTimeConstraintCounter = 0;

		//if (!this->quiet) {
		auto currentTime1 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
		std::cerr << "SATSchedulerRes: trying candidate II=" << this->candidateII << " at time " << 	std::put_time(std::localtime(&currentTime1), "%Y-%m-%d %X") << std::endl;
		//}

		// compute min/max latency
		this->defineLatLimits();
		if (!this->quiet) {
			std::cout << "SATSchedulerRes: defined SL limits: " << this->minLatency << " <= SL <= " << this->maxLatency << std::endl;
		}

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
		// create solver and connect terminator
		this->terminator = CaDiCaLTerminator((double)this->solverTimeout);
		//this->solver = std::unique_ptr<CaDiCaL::Solver>(new CaDiCaL::Solver);
		//this->solver->connect_terminator(&this->terminator);
		if (!this->quiet) {
			std::cout << "SATSchedulerRes: candidate II=" << this->candidateII << " with min latency="
								<< this->latencyLowerBound << " and max latency=" << this->latencyUpperBound << std::endl;
		}
		while (this->computeNewLatencySuccess(lastAttemptSuccess)) {
			if (!this->quiet) {
				std::cout << "SATSchedulerRes: new latency limit = " << this->candidateLatency << std::endl;
				std::cout << "SATSchedulerRes: resetting containers" << std::endl;
			}
			this->resetContainer();
			if (!this->quiet) {
				std::cout << "SATSchedulerRes: setting up solver" << std::endl;
			}
			this->setUpSolver();
			if (!this->quiet) {
				std::cout << "SATSchedulerRes: creating literals" << std::endl;
			}
			this->createLiterals();
			if (!this->quiet) {
				std::cout << "SATSchedulerRes: creating clauses" << std::endl;
			}
			this->createClauses();
			if (!this->quiet) {
				std::cout << "SATSchedulerRes: time is " << this->terminator.getElapsedTime() << "sec after constructing the problem" << std::endl;
			}
			if (this->terminator.terminate()) {
				// timeout during problem construction :(
				if (!this->quiet) {
					std::cout << "SATSchedulerRes: encountered timeout during problem construction after " << this->terminator.getElapsedTime() << "sec (time budget was " << this->solverTimeout << "sec) - total elapsed time: " << this->solvingTimeTotal+this->terminator.getElapsedTime() << "sec" << std::endl;
				}
				breakByTimeout = true;
				break;
			}
			if (!this->quiet) {
				std::cout << "SATSchedulerRes: start scheduling for II=" << this->candidateII << " and latency=" << this->candidateLatency << " with '" << this->literalCounter << "' literals and '"
									<< this->clauseCounter << "' clauses" << std::endl;
				std::cout << "  '" << this->scheduleTimeLiteralCounter << "' time slot literals" << std::endl;
				//std::cout << "  '" << this->moduloSlotLiteralCounter << "' modulo slot literals" << std::endl;
				std::cout << "  '" << this->resourceConstraintLiteralCounter << "' resource constraint literals" << std::endl;
				std::cout << "  '" << this->scheduleTimeConstraintCounter << "' time slot constraint clauses" << std::endl;
				std::cout << "  '" << this->resourceConstraintClauseCounter << "' resource constraint clauses" << std::endl;
				std::cout << "  '" << this->dependencyConstraintClauseCounter << "' dependency constraint clauses" << std::endl;

				auto currentTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				std::cout << "  current time: " << std::put_time(std::localtime(&currentTime2), "%Y-%m-%d %X") << std::endl;
			}
			if (this->terminator.getElapsedTime() >= this->solverTimeout) {
				// timeout after problem construction!
				if (!this->quiet) {
					std::cout << "SATSchedulerRes: encountered timeout after " << this->terminator.getElapsedTime() << "sec (time budget was " << this->solverTimeout << "sec) - total elapsed time: " << this->solvingTimeTotal+this->terminator.getElapsedTime() << "sec" << std::endl;
				}
				breakByTimeout = true;
				break;
			}
			// start solving
			//if (!this->quiet) {
			auto currentTime3 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			std::cerr << "SATSchedulerRes: start solving at time " << 	std::put_time(std::localtime(&currentTime3), "%Y-%m-%d %X") << std::endl;
			//}
			auto stat = this->solver->solve();
			lastAttemptSuccess = stat == CADICAL_SAT;
			if (!this->quiet) {
				std::cout << "SATSchedulerRes: finished solving with status '" <<
									(lastAttemptSuccess?"SAT":"UNSAT") << "' (code '" << stat << "') after " << this->terminator.getElapsedTime()
									<< " sec (total: " << this->solvingTimeTotal << " sec)" << std::endl;
			}
			if(!lastAttemptSuccess) {
				//if (!this->quiet) {
				auto currentTime4 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				std::cerr << "SATSchedulerRes: failed to find solution for II=" << this->candidateII << " and SL=" << this->candidateLatency << " at " << std::put_time(std::localtime(&currentTime4), "%Y-%m-%d %X") << std::endl;
				//}
				// check if it was due to a timeout
				if (this->terminator.getElapsedTime() >= this->solverTimeout) {
					// timeout when solving
					if (!this->quiet) {
						std::cout << "SATSchedulerRes: encountered timeout after " << this->terminator.getElapsedTime() << "sec (time budget was " << this->solverTimeout << "sec)" << std::endl;
					}
					breakByTimeout = true;
					break;
				}
				// schedule attempt failed :(
				// let's try again for the next latency :)
				if (!this->quiet) {
					std::cout << "SATSchedulerRes: latency " << this->candidateLatency << " is infeasible - trying again with modified latency limit" << std::endl;
				}
				continue;
			}
			this->scheduleFound = true;
			this->fillSolutionStructure();
			//if (!this->quiet) {
			auto currentTime5 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			std::cerr << "SATSchedulerRes: found solution for II=" << this->candidateII << " and SL=" << this->candidateLatency << " at " << std::put_time(std::localtime(&currentTime5), "%Y-%m-%d %X") << std::endl;
			//}
		}
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
			this->solvingTimePerIteration = this->terminator.getElapsedTime();
			this->timeRemaining = std::max(0.0, this->solverTimeout - this->solvingTimePerIteration);
			return;
		}
		this->solvingTimePerIteration = this->terminator.getElapsedTime();
		this->timeRemaining = std::max(0.0, this->solverTimeout - this->solvingTimePerIteration);
	}

	void SATSchedulerRes::defineLatLimits() {
#if 1
		if (this->targetSLUserDef) {
			// limits were defined by the user
			// only set min and max times based on SDC schedule
			if (!this->scheduleLengthEstimation->minSLEstimationFound()) {
				this->scheduleLengthEstimation->estimateMinSL(this->candidateII, (int) this->solverTimeout);
			}
			this->earliestStartTime = this->scheduleLengthEstimation->getASAPTimesSDC();
			this->latestStartTimeDifferences = this->scheduleLengthEstimation->getALAPTimeDiffsSDC();
		}
		else {
			// set max latency if required...
			if (this->maxSL >= 0) {
				this->maxLatency = this->maxSL;
			}
			else {
				this->scheduleLengthEstimation->estimateMaxSL(this->candidateII, (int) this->solverTimeout);
				this->maxLatency = this->scheduleLengthEstimation->getMaxSLEstimation();
			}
			// ... and set min latency if required ...
			if (this->minSL >= 0) {
				this->minLatency = this->minSL;
			}
			else {
				this->scheduleLengthEstimation->estimateMinSL(this->candidateII, (int) this->solverTimeout);
				this->minLatency = this->scheduleLengthEstimation->getMinSLEstimation();
			}
			// ... and always set min and max times
			this->earliestStartTime = this->scheduleLengthEstimation->getASAPTimesSDC();
			this->latestStartTimeDifferences = this->scheduleLengthEstimation->getALAPTimeDiffsSDC();
		}
#else
		if (this->minLatencyUserDef and this->maxLatencyUserDef) {
			// limits were defined by the user
			// only set min and max times based on SDC schedule
			if (!this->scheduleLengthEstimation->minSLEstimationFound()) {
				this->scheduleLengthEstimation->estimateMinSL(this->candidateII, (int) this->solverTimeout);
			}
			this->earliestStartTime = this->scheduleLengthEstimation->getASAPTimesSDC();
			this->latestStartTimeDifferences = this->scheduleLengthEstimation->getALAPTimeDiffsSDC();
		}
		else if (this->maxLatencyUserDef and !this->minLatencyUserDef) {
			// only max latency defined by the user
			// set min latency ...
			this->scheduleLengthEstimation->estimateMinSL(this->candidateII, (int) this->solverTimeout);
			this->minLatency = this->scheduleLengthEstimation->getMinSLEstimation();
			// ... and also min and max times
			this->earliestStartTime = this->scheduleLengthEstimation->getASAPTimesSDC();
			this->latestStartTimeDifferences = this->scheduleLengthEstimation->getALAPTimeDiffsSDC();
		}
		else if (!this->maxLatencyUserDef and this->minLatencyUserDef) {
			// only min latency defined by the user
			// set max latency...
			this->scheduleLengthEstimation->estimateMaxSL(this->candidateII, (int) this->solverTimeout);
			this->maxLatency = this->scheduleLengthEstimation->getMaxSLEstimation();
			// ... and also min and max times
			if (!this->scheduleLengthEstimation->minSLEstimationFound()) {
				this->scheduleLengthEstimation->estimateMinSL(this->candidateII, (int) this->solverTimeout);
			}
			this->earliestStartTime = this->scheduleLengthEstimation->getASAPTimesSDC();
			this->latestStartTimeDifferences = this->scheduleLengthEstimation->getALAPTimeDiffsSDC();
		}
		else {
			// set max latency ...
			this->scheduleLengthEstimation->estimateMaxSL(this->candidateII, (int) this->solverTimeout);
			this->maxLatency = this->scheduleLengthEstimation->getMaxSLEstimation();
			// ... and set min latency ...
			this->scheduleLengthEstimation->estimateMinSL(this->candidateII, (int) this->solverTimeout);
			this->minLatency = this->scheduleLengthEstimation->getMinSLEstimation();
			// ... and also min and max times
			this->earliestStartTime = this->scheduleLengthEstimation->getASAPTimesSDC();
			this->latestStartTimeDifferences = this->scheduleLengthEstimation->getALAPTimeDiffsSDC();
		}
		if (this->minSL >= 0) {
			this->minLatency = std::max(this->minLatency, this->minSL);
		}
		if (this->maxSL >= 0) {
			this->maxLatency = std::min(this->maxLatency, this->maxSL);
		}
#endif
	}

	void SATSchedulerRes::resetContainer() {
		// reset literal containers
		//this->scheduleTimeVariables.clear();
		//this->moduloSlotVariables.clear();
		/*
		this->scheduleTimeLiterals.clear();
		this->bindingLiterals.clear();
		 */
		// reset literal counter(s)
		//this->literalCounter = 0;
		//this->moduloSlotLiteralCounter = 0;
		//this->resourceConstraintLiteralCounter = 0;
		//this->dependencyConstraintLiteralCounter = 0;
		/*
		this->scheduleTimeLiteralCounter = 0;
		this->bindingLiteralCounter = 0;
		this->timeOverlapLiteralCounter = 0;
		this->bindingOverlapLiteralCounter = 0;
		 */
		// reset clause counter(s)
		//this->clauseCounter = 0;
		//this->dependencyConstraintClauseCounter = 0;
		/*
		this->dependencyConstraintClauseCounter = 0;
		this->resourceConstraintClauseCounter = 0;
		this->scheduleTimeConstraintClauseCounter = 0;
		this->bindingConstraintClauseCounter = 0;
		this->timeOverlapClauseCounter = 0;
		this->bindingOverlapClauseCounter = 0;
		this->calculateLatestStartTimes();
		 */
	}

	void SATSchedulerRes::setUpSolver() {
		// reset solver
		//this->terminator = CaDiCaLTerminator(this->solverTimeout);
		this->solver = std::unique_ptr<CaDiCaL::Solver>(new CaDiCaL::Solver);
		this->solver->connect_terminator(&this->terminator);
		// update latest start times based on candidate latency
		for (auto &v : this->g.Vertices()) {
			this->latestStartTime[v] = this->candidateLatency - this->latestStartTimeDifferences.at(v);
		}
	}

	void SATSchedulerRes::createLiterals() {
		// const zero/one
		this->constOneVar = ++this->literalCounter;
		this->constZeroVar = ++this->literalCounter;
		// create schedule time literals
		for (auto &v : this->g.Vertices()) {
			auto tMin = this->earliestStartTime.at(v);
			auto tMax = this->latestStartTime.at(v);
			for (int t=tMin; t<=tMax; t++) {
				this->scheduleTimeVariables[{v, t}] = ++this->literalCounter;
			}
		}
	}

	void SATSchedulerRes::createClauses() {
		// set const zero/one variables
		if (!this->quiet) std::cout << "SATSchedulerRes::createClauses: start" << std::endl;
		this->create_arbitrary_clause({{this->constOneVar, false}});
		this->create_arbitrary_clause({{this->constZeroVar, true}});
		if (!this->quiet) std::cout << "SATSchedulerRes::createClauses: creating clauses to enforce at least one schedule time assignment" << std::endl;
		this->createScheduleTimeClauses();
		if (!this->quiet) std::cout << "SATSchedulerRes::createClauses: creating clauses for dependency constraints" << std::endl;
		this->createDependencyClauses();
		// resource constraints
		if (!this->quiet) std::cout << "SATSchedulerRes::createClauses: creating clauses for resource constraints" << std::endl;
		this->createResourceLimitationClauses();
		if (!this->quiet) std::cout << "SATSchedulerRes::createClauses: finished" << std::endl;
	}

	void SATSchedulerRes::createScheduleTimeClauses() {
		for (auto &v : this->g.Vertices()) {
			auto tMin = this->earliestStartTime.at(v);
			auto tMax = this->latestStartTime.at(v);
			std::vector<std::pair<int, bool>> clause;
			for (int t=tMin; t<=tMax; t++) {
				clause.emplace_back(std::make_pair(this->scheduleTimeVariables.at({v, t}), false));
			}
			this->create_arbitrary_clause(clause);
		}
	}

	void SATSchedulerRes::createDependencyClauses() {
		for (auto &e : this->g.Edges()) {
			auto vSrc = &e->getVertexSrc();
			auto vDst = &e->getVertexDst();
			auto lSrc = this->resourceModel.getVertexLatency(vSrc);
			auto dist = e->getDistance();
			auto delay = e->getDelay();
			auto tSrcMin = this->earliestStartTime.at(vSrc);
			auto tDstMin = this->earliestStartTime.at(vDst);
			auto tSrcMax = this->latestStartTime.at(vSrc);
			auto tDstMax = this->latestStartTime.at(vDst);
			for (auto tSrc = tSrcMin; tSrc <= tSrcMax; tSrc++) {
				for (auto tDst = tDstMin; tDst <= tDstMax; tDst++) {
					// t_j - t_i >= L_i + delay - distance*II
					if (tDst - tSrc >= lSrc + delay - (dist * this->candidateII)) continue; // constraint satisfied
					this->create_arbitrary_clause({
																					{this->scheduleTimeVariables.at({vSrc, tSrc}), true},
																					{this->scheduleTimeVariables.at({vDst, tDst}), true}
																				});
				}
			}
		}
	}

	void SATSchedulerRes::createResourceLimitationClauses() {
#if 1
		std::map<std::pair<const Resource*, int>, int> curMaxVal;
		std::map<std::pair<const Resource*, int>, vector<int>> partialSumVars;
		// build partial sums for all resource-limited vertices
		for (auto &v : this->g.Vertices()) {
			// check if vertex can be skipped
			auto *r = this->resourceModel.getResource(v);
			if (r->isUnlimited()) continue;
			auto resLim = r->getLimit();
			auto resLimWordSize = (int)std::ceil(std::log2(resLim+2));
			// schedule time info
			auto &tMin = this->earliestStartTime.at(v);
			auto &tMax = this->latestStartTime.at(v);
			// create implications for the adder input
			/*for (int t=tMin; t<=tMax; t++) {
				auto modSlot = t % this->candidateII;
				int addInput;
				try {
					addInput = this->moduloSlotVariables.at({v, modSlot});
				}
				catch (std::out_of_range&) {
					addInput = this->moduloSlotVariables[{v, modSlot}] = ++this->literalCounter;
					this->moduloSlotLiteralCounter++;
				}
				// create implication
				std::vector<std::pair<int, bool>> clause = {
					{this->scheduleTimeVariables.at({v, t}), true},
					{addInput, false}
				};
				this->create_arbitrary_clause(clause);
			}*/
			// create adders for all possible modulo slots where this vertex can be scheduled
			// and forbid that the adder output exceeds the resource limit
			for (int t = tMin; t <= tMax; t++) {
				auto &addInputBit = this->scheduleTimeVariables[{v, t}];
				auto m = t % this->candidateII;
				// max value for this modulo slot increases by 1
				curMaxVal[{r, m}] = curMaxVal[{r, m}]+1;
				auto &addInputVector = partialSumVars[{r, m}];
				if (addInputVector.empty()) {
					// the first operation that can be scheduled in this modulo slot
					// -> no need to calculate something, just pass the bit to the next stage
					partialSumVars[{r, m}] = {addInputBit};
					continue;
				}
				// create adder
				auto inputWordSize = addInputVector.size();
				auto outputWordSize = (int)std::ceil(std::log2(curMaxVal[{r, m}]+1));
				// calculating until the resource limit is enough
				outputWordSize = std::min(outputWordSize, resLimWordSize);
				int carryIn = addInputBit;
				std::vector<int> partialSumOutput(outputWordSize);
				for (int w = 0; w < inputWordSize; w++) {
					auto sumBit = partialSumOutput[w] = ++this->literalCounter;
					this->resourceConstraintLiteralCounter++;
					if (w < inputWordSize-1) {
						int carryOut = ++this->literalCounter;
						this->resourceConstraintLiteralCounter++;
						// normal half adder
						this->create_half_adder_clauses({carryIn, false}, {addInputVector.at(w), false}, {sumBit, false}, {carryOut, false});
						// pass carry output to next stage
						carryIn = carryOut;
					}
					else if (w == inputWordSize-1 and outputWordSize > inputWordSize) {
						int carryOut = ++this->literalCounter;
						this->resourceConstraintLiteralCounter++;
						// normal half adder
						this->create_half_adder_clauses({carryIn, false}, {addInputVector.at(w), false}, {sumBit, false}, {carryOut, false});
						// the carry output is the result MSB
						partialSumOutput[w+1] = carryOut;
					}
					else {
						// half adder without carry output
						this->create_half_adder_clauses({carryIn, false}, {addInputVector.at(w), false}, {sumBit, false});
					}
				}
				// pass adder output to next stage
				partialSumVars[{r, m}] = partialSumOutput;
				// forbid that the adder output exceeds the resource limit
				if (curMaxVal[{r, m}] <= resLim) {
					// limit cannot be exceeded because the partial sum is too small
					continue;
				}
				std::vector<std::pair<int, bool>> clause;
				auto resLimViolated = resLim + 1;
				for (int w = 0; w < outputWordSize; w++) {
					auto resLimBit = (bool)((resLimViolated >> w) & 1);
					clause.emplace_back(partialSumOutput.at(w), resLimBit);
				}
				this->create_arbitrary_clause(clause);
			}
		}
#else
		std::map<std::pair<const Resource*, int>, std::vector<int>> moduloSlotVariables;
		for (auto &v : this->g.Vertices()) {
			auto *r = this->resourceModel.getResource(v);
			if (r->isUnlimited()) continue;
			auto tMin = this->earliestStartTime.at(v);
			auto tMax = this->latestStartTime.at(v);
			for (int t=tMin; t<=tMax; t++) {
				auto m = t % this->candidateII;
				moduloSlotVariables[{r, m}].emplace_back(this->scheduleTimeVariables.at({v, t}));
			}
		}
		for (auto &it : moduloSlotVariables) {
			auto *r = it.first.first;
			auto variables = it.second;
			auto limit = r->getLimit();
			if (variables.size() <= limit) continue; // trivially satisfied :)
			// sum up all variables
			std::vector<std::pair<std::vector<int>, bool>> bitheapInput(variables.size(), std::make_pair(std::vector<int>(1), false));
			for (int i=0; i<variables.size(); i++) {
				bitheapInput[i].first[0] = variables.at(i);
			}
			auto sum = this->create_bitheap(bitheapInput);
			// enforce constraint: sum <= limit
			auto forbidden = limit;
			auto sumWordSize = (int)sum.size();
			std::vector<std::pair<int, bool>> clauseBase;
			for (int idx = sumWordSize-1; idx >= 0; idx--) {
				if ((forbidden >> idx) & 1) {
					clauseBase.emplace_back(sum.at(idx), true);
					continue;
				}
				auto clause = clauseBase;
				clause.emplace_back(sum.at(idx), true);
				this->create_arbitrary_clause(clause);
			}
		}
#endif
	}

	void SATSchedulerRes::fillSolutionStructure() {
		// define schedule times from schedule time literals
		std::cout << "#q# SATSchedulerRes: found schedule!" << std::endl;
		for (auto &v : this->g.Vertices()) {
			auto tMin = this->earliestStartTime.at(v);
			auto tMax = this->latestStartTime.at(v);
			bool assignedStartTime = false;
			for (int t=tMin; t<=tMax; t++) {
				auto bitSet = this->solver->val(this->scheduleTimeVariables.at({v, t})) > 0;
				if (bitSet) {
					this->startTimes[v] = t;
					assignedStartTime = true;
					std::cout << "  '" << v->getName() << "' t=" << t << std::endl;
					break;
				}
			}
			if (not assignedStartTime) {
				throw Exception("SATScheduelerRes: failed to assign start time to vertex '"+v->getName()+"'");
			}
		}
		// override candidate latency in case the scheduler found a solution with a schedule length
		// which is smaller than the given candidate latency (unlikely I guess, but who knows...)
		auto actualScheduleLength = this->getScheduleLength();
		if (actualScheduleLength > this->candidateLatency) {
			// print vertices that violate the requested limit
			for (auto &it : this->startTimes) {
				if (it.second + this->resourceModel.getVertexLatency(it.first) > this->candidateLatency) {
					// start time is too late!
					std::cout << "SATSchedulerRes: vertex '" << it.first->getName() << "' (t='" << it.second << "', lat='" << this->resourceModel.getVertexLatency(it.first) << "') violates requested schedule length of '" << this->candidateLatency << "'" << std::endl;
				}
			}
			throw Exception("SATSchedulerRes: Found invalid schedule! Requested candidate schedule length was '"+std::to_string(this->candidateLatency)+"' but scheduler found schedule length '"+std::to_string(actualScheduleLength)+"'");
		}
		this->candidateLatency = actualScheduleLength;
		// debugging
		/*
		for (auto &e : this->g.Edges()) {
			auto vSrc = &e->getVertexSrc();
			auto vDst = &e->getVertexDst();
			auto lSrc = this->resourceModel.getVertexLatency(vSrc);
			auto dist = e->getDistance();
			auto delay = e->getDelay();
			auto tSrcMin = this->earliestStartTime.at(vSrc);
			auto tDstMin = this->earliestStartTime.at(vDst);
			auto tSrcDiff = this->latestStartTime.at(vSrc) - this->earliestStartTime.at(vSrc);
			auto tDstDiff = this->latestStartTime.at(vDst) - this->earliestStartTime.at(vSrc);
			auto edgeConst = -(lSrc + delay - (dist * this->candidateII) + tSrcMin - tDstMin);
			auto wordSizeSrc = this->scheduleTimeWordSize.at(vSrc);
			auto wordSizeDst = this->scheduleTimeWordSize.at(vDst);
			auto wordSizeDiff = std::max(wordSizeSrc, wordSizeDst) + 2;
			auto solverTimeSrc = 0;
			auto solverTimeDst = 0;
			auto solverEdgeDiff = 0;
			auto worstCaseDiff = tSrcDiff;
			if (worstCaseDiff <= edgeConst) {
				continue;
			}
			std::string srcString;
			for (int w = 0; w < wordSizeSrc; w++) {
				auto bitResult = this->solver->val(this->scheduleTimeVariables.at({vSrc, w})) > 0 ? 1 : 0;
				srcString = std::to_string(bitResult) + srcString;
				solverTimeSrc += (bitResult << w);
			}
			std::string dstString;
			for (int w = 0; w < wordSizeDst; w++) {
				auto bitResult = this->solver->val(this->scheduleTimeVariables.at({vDst, w})) > 0 ? 1 : 0;
				dstString = std::to_string(bitResult) + dstString;
				solverTimeDst += (bitResult << w);
			}
			std::string edgeDiffString;
			for (int w = 0; w < wordSizeDiff; w++) {
				auto bitResult = this->solver->val(this->diffVariables.at({e, w})) > 0 ? 1 : 0;
				edgeDiffString = std::to_string(bitResult) + edgeDiffString;
				solverEdgeDiff += (bitResult << w);
			}
			auto numBitsDiffInt = sizeof(solverEdgeDiff) * 8;
			for (int w = wordSizeDiff; w < numBitsDiffInt; w++) {
				auto signBit = (solverEdgeDiff >> (wordSizeDiff-1)) & 1;
				solverEdgeDiff |= (signBit << w);
			}
		}
		 */
	}

	int SATSchedulerRes::create_arbitrary_clause(const vector<std::pair<int, bool>> &a) {
		this->clauseCounter++;
		for (auto &it : a) {
			if (it.second) {
				// negated literal
				this->solver->add(-it.first);
			}
			else {
				// non-negated literal
				this->solver->add(it.first);
			}
		}
		this->solver->add(0);
		return 1;
	}

	int SATSchedulerRes::create_arbitrary_clause(const vector<std::pair<int, bool>> &a, bool quiet) {
		this->clauseCounter++;
		std::string s = "creating clause";
		for (auto &it : a) {
			if (it.second) {
				// negated literal
				this->solver->add(-it.first);
				s += " "+std::to_string(-it.first);
			}
			else {
				// non-negated literal
				this->solver->add(it.first);
				s += " "+std::to_string(it.first);
			}
		}
		this->solver->add(0);
		if (!quiet) {
			std::cout << s << std::endl;
		}
		return 1;
	}

	int SATSchedulerRes::create_full_adder_clauses(std::pair<int, bool> a, std::pair<int, bool> b,
																										std::pair<int, bool> c_i, std::pair<int, bool> sum,
																										std::pair<int, bool> c_o) {
		// 1)
		this->create_arbitrary_clause({{a.first, a.second}, {b.first, not b.second}, {c_i.first, c_i.second}, {sum.first, sum.second}});
		// 2)
		this->create_arbitrary_clause({{a.first, not a.second}, {b.first, b.second}, {c_i.first, c_i.second}, {sum.first, sum.second}});
		// 3)
		this->create_arbitrary_clause({{a.first, a.second}, {b.first, b.second}, {c_i.first, c_i.second}, {sum.first, not sum.second}});
		// 4)
		this->create_arbitrary_clause({{a.first, not a.second}, {b.first, not b.second}, {c_i.first, c_i.second}, {sum.first, not sum.second}});
		// 5)
		this->create_arbitrary_clause({{a.first, a.second}, {b.first, not b.second}, {c_i.first, not c_i.second}, {sum.first, not sum.second}});
		// 6)
		this->create_arbitrary_clause({{a.first, not a.second}, {b.first, b.second}, {c_i.first, not c_i.second}, {sum.first, not sum.second}});
		// 7)
		this->create_arbitrary_clause({{a.first, a.second}, {b.first, b.second}, {c_i.first, not c_i.second}, {sum.first, sum.second}});
		// 8)
		this->create_arbitrary_clause({{a.first, not a.second}, {b.first, not b.second}, {c_i.first, not c_i.second}, {sum.first, sum.second}});

		if (c_o.first <= 0) return 8;
		// 1)
		this->create_arbitrary_clause({{a.first, not a.second}, {b.first, not b.second}, {c_o.first, c_o.second}});
		// 2)
		this->create_arbitrary_clause({{a.first, a.second}, {c_i.first, c_i.second}, {c_o.first, not c_o.second}});
		// 3)
		this->create_arbitrary_clause({{b.first, b.second}, {c_i.first, c_i.second}, {c_o.first, not c_o.second}});
		// 4)
		this->create_arbitrary_clause({{a.first, a.second}, {b.first, b.second}, {c_o.first, not c_o.second}});
		// 5)
		this->create_arbitrary_clause({{b.first, not b.second}, {c_i.first, not c_i.second}, {c_o.first, c_o.second}});
		// 6)
		this->create_arbitrary_clause({{a.first, not a.second}, {c_i.first, not c_i.second}, {c_o.first, c_o.second}});
		return 14;
	}

	int SATSchedulerRes::create_half_adder_clauses(std::pair<int, bool> a, std::pair<int, bool> b,
																										std::pair<int, bool> sum, std::pair<int, bool> c_o) {
		// 1) a b -sum
		this->create_arbitrary_clause({{a.first, a.second}, {b.first, b.second}, {sum.first, not sum.second}});
		// 2) a -b sum
		this->create_arbitrary_clause({{a.first, a.second}, {b.first, not b.second}, {sum.first, sum.second}});
		// 3) -a b sum
		this->create_arbitrary_clause({{a.first, not a.second}, {b.first, b.second}, {sum.first, sum.second}});
		// 4) -a -b -sum
		this->create_arbitrary_clause({{a.first, not a.second}, {b.first, not b.second}, {sum.first, not sum.second}});

		if (c_o.first <= 0) return 4;
		// 1) -a -b c_o
		this->create_arbitrary_clause({{a.first, not a.second},{b.first, not b.second},{c_o.first, c_o.second}});
		// 2) a -c_o
		this->create_arbitrary_clause({{a.first, a.second},{c_o.first, not c_o.second}});
		// 3) b -c_o
		this->create_arbitrary_clause({{b.first, b.second},{c_o.first, not c_o.second}});
		return 7;
	}

	int SATSchedulerRes::create_not(int x, int not_x) {
		// 1) -x -not_x
		this->create_arbitrary_clause({{x, true}, {not_x, true}});
		// 2) x not_x
		this->create_arbitrary_clause({{x, false}, {not_x, false}});
		return 2;
	}

	int SATSchedulerRes::create_2x1_and(int a, int b, int y) {
		// 1) -a -b  y
		this->create_arbitrary_clause({{a, true},{b, true},{y, false}});
		// 2)  a -y
		this->create_arbitrary_clause({{a, false},{y, true}});
		// 3)  b -y
		this->create_arbitrary_clause({{b, false},{y, true}});
		return 3;
	}

	int SATSchedulerRes::create_2x1_or(int a, int b, int y) {
		// 1)  a  b  -y
		this->create_arbitrary_clause({{a, false},{b, false},{y, true}});
		// 2) -a  y
		this->create_arbitrary_clause({{a, true},{y, false}});
		// 3) -b  y
		this->create_arbitrary_clause({{b, true},{y, false}});
		return 3;
	}

	int SATSchedulerRes::create_2x1_mux(int a, int b, int s, int y) {
		// 1) -a  s  y
		this->create_arbitrary_clause({{a, true}, {s, false}, {y, false}});
		// 2)  a  s -y
		this->create_arbitrary_clause({{a, false}, {s, false}, {y, true}});
		// 3) -b -s  y
		this->create_arbitrary_clause({{b, true}, {s, true}, {y, false}});
		// 4)  b -s -y
		this->create_arbitrary_clause({{b, false}, {s, true}, {y, true}});
		// 5) -a -b  y (REDUNDANT but good for unit propagation)
		this->create_arbitrary_clause({{a, true}, {b, true}, {y, false}});
		// 6)  a  b -y (REDUNDANT but good for unit propagation)
		this->create_arbitrary_clause({{a, false}, {b, false}, {y, true}});
		return 6;
	}

	std::vector<int> SATSchedulerRes::create_bitheap(const vector<std::pair<std::vector<int>, bool>> &x) {
		std::vector<int> result_variables;
		std::map<int, std::vector<std::pair<int, bool>>> y;
		int num_bits = 0;
		for (auto &it : x) {
			auto bits = it.first;
			auto sub = it.second;
			if (bits.size() > num_bits) num_bits = bits.size();
			if (sub) {
				// add 1 for 2k inversion
				y[0].emplace_back(this->constOneVar, false);
				// add inverted bits
				for (int bit_pos=0; bit_pos<bits.size(); bit_pos++) {
					y[bit_pos].emplace_back(bits[bit_pos], true);
				}
			}
			else {
				// add bits
				for (int bit_pos=0; bit_pos<bits.size(); bit_pos++) {
					y[bit_pos].emplace_back(bits[bit_pos], false);
				}
			}
		}
		int i = 0;
		while (i < num_bits) {
			while (y[i].size() > 1) {
				if (y[i].size() == 2) {
					// half adder
					// create new literals for sum and carry
					auto sum = ++this->literalCounter;
					auto carry = ++this->literalCounter;
					// get bits to add from container
					auto a = y[i].back();
					y[i].pop_back();
					auto b = y[i].back();
					y[i].pop_back();
					// create clauses
					this->create_half_adder_clauses(a, b, {sum, false}, {carry, false});
					// add new bits to bitheap
					y[i].emplace_back(sum, false);
					y[i+1].emplace_back(carry, false);
					if (i+2 > num_bits) num_bits = i+2;
				}
				else {
					// full adder
					// create new literals for sum and carry
					auto sum = ++this->literalCounter;
					auto carry = ++this->literalCounter;
					// get bits to add from container
					auto a = y[i].back();
					y[i].pop_back();
					auto b = y[i].back();
					y[i].pop_back();
					auto c = y[i].back();
					y[i].pop_back();
					// create clauses
					this->create_full_adder_clauses(a, b, c, {sum, false}, {carry, false});
					y[i].emplace_back(sum, false);
					y[i+1].emplace_back(carry, false);
					if (i+2 > num_bits) num_bits = i+2;
				}
			}
			if (y[i].size() != 1) {
				std::cerr << "Failed compressing bits at position " << i << " -> " << y[i].size() << " bits are left instead of 1" << std::endl;
				throw std::runtime_error("error during bitheap clause generation");
			}
			if (y[i][0].second) {
				std::cerr << "Failed compressing bits at position " << i << " -> the output bit is inverted..." << std::endl;
				throw std::runtime_error("error during bitheap clause generation");
			}
			result_variables.emplace_back(y[i][0].first);
			// advance to next bit position
			i++;
		}
		return result_variables;
	}
}