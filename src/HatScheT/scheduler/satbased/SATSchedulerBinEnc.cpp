//
// Created by nfiege on 12/20/22.
//

#include "SATSchedulerBinEnc.h"
#include <HatScheT/utility/Utility.h>
#include <iomanip>

namespace HatScheT {

	SATSchedulerBinEnc::SATSchedulerBinEnc(Graph &g, ResourceModel &resourceModel, int II) :
		SATSchedulerBase(g, resourceModel, II) {
		if (this->recMinII > 1.0) {
			this->los = LatencyOptimizationStrategy::REVERSE_LINEAR; // reverse linear for graphs with cycles
		}
		else {
			this->los = LatencyOptimizationStrategy::LINEAR_JUMP; // linear jump for graphs without cycles
		}
	}

	void SATSchedulerBinEnc::scheduleIteration() {
		// call parent's method for some basic stuff
		SATSchedulerBase::scheduleIteration();

		this->scheduleTimeVariables.clear();
		this->scheduleTimeWordSize.clear();
		this->diffVariables.clear();
		this->moduloSlotVariables.clear();
		this->lastForbiddenTime.clear();
		this->constOneVar = 0;
		this->constZeroVar = 0;
		this->resourceConstraintLiteralCounter = 0;
		this->moduloSlotLiteralCounter = 0;
		this->dependencyConstraintLiteralCounter = 0;

		this->dependencyConstraintClauseCounter = 0;

		//if (!this->quiet) {
		auto currentTime1 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
		std::cerr << "SATSchedulerBinEnc: trying candidate II=" << this->candidateII << " at time " << 	std::put_time(std::localtime(&currentTime1), "%Y-%m-%d %X") << std::endl;
		//}

		// compute min/max latency
		this->defineLatLimits();
		if (!this->quiet) {
			std::cout << "SATSchedulerBinEnc: defined SL limits: " << this->minLatency << " <= SL <= " << this->maxLatency << std::endl;
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
			std::cout << "SATSchedulerBinEnc: candidate II=" << this->candidateII << " with min latency="
								<< this->latencyLowerBound << " and max latency=" << this->latencyUpperBound << std::endl;
		}
		while (this->computeNewLatencySuccess(lastAttemptSuccess)) {
			if (!this->quiet) {
				std::cout << "SATSchedulerBinEnc: new latency limit = " << this->candidateLatency << std::endl;
				std::cout << "SATSchedulerBinEnc: resetting containers" << std::endl;
			}
			this->resetContainer();
			if (!this->quiet) {
				std::cout << "SATSchedulerBinEnc: setting up solver" << std::endl;
			}
			auto trivialDecisionSetup = this->setUpSolver();
			if (trivialDecisionSetup == triviallyUNSAT) {
				// scheduling for this II is impossible because the maxLatencyConstraint is too tight
				// terminate early and try with larger II
				this->scheduleFound = false;
				this->solvingTimePerIteration = this->terminator.getElapsedTime();
				this->timeRemaining = std::max(0.0, this->solverTimeout - this->solvingTimePerIteration);
				return;
			}
			if (!this->quiet) {
				std::cout << "SATSchedulerBinEnc: creating literals" << std::endl;
			}
			this->createLiterals();
			if (!this->quiet) {
				std::cout << "SATSchedulerBinEnc: creating clauses" << std::endl;
			}
			auto trivialDecisionClauses = this->createClauses();
			if (trivialDecisionClauses == triviallyUNSAT) {
				// some dependency constraints are impossible to satisfy because the maxLatencyConstraint is too tight
				// terminate early and try with larger II
				this->scheduleFound = false;
				this->solvingTimePerIteration = this->terminator.getElapsedTime();
				this->timeRemaining = std::max(0.0, this->solverTimeout - this->solvingTimePerIteration);
				return;
			}
			if (!this->quiet) {
				std::cout << "SATSchedulerBinEnc: time is " << this->terminator.getElapsedTime() << "sec after constructing the problem" << std::endl;
			}
			if (this->terminator.terminate()) {
				// timeout during problem construction :(
				if (!this->quiet) {
					std::cout << "SATSchedulerBinEnc: encountered timeout during problem construction after " << this->terminator.getElapsedTime() << "sec (time budget was " << this->solverTimeout << "sec) - total elapsed time: " << this->solvingTimeTotal+this->terminator.getElapsedTime() << "sec" << std::endl;
				}
				breakByTimeout = true;
				break;
			}
			if (!this->quiet) {
				std::cout << "SATSchedulerBinEnc: start scheduling for II=" << this->candidateII << " and latency=" << this->candidateLatency << " with '" << this->literalCounter << "' literals and '"
									<< this->clauseCounter << "' clauses" << std::endl;
				std::cout << "  '" << this->moduloSlotLiteralCounter << "' modulo slot literals" << std::endl;
				std::cout << "  '" << this->resourceConstraintLiteralCounter << "' resource constraint literals" << std::endl;
				std::cout << "  '" << this->dependencyConstraintLiteralCounter << "' dependency constraint literals" << std::endl;
				std::cout << "  '" << this->dependencyConstraintClauseCounter << "' dependency constraint clauses" << std::endl;

				/*
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
				 */
				auto currentTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				std::cout << "  current time: " << std::put_time(std::localtime(&currentTime2), "%Y-%m-%d %X") << std::endl;
			}
			if (this->terminator.getElapsedTime() >= this->solverTimeout) {
				// timeout after problem construction!
				if (!this->quiet) {
					std::cout << "SATSchedulerBinEnc: encountered timeout after " << this->terminator.getElapsedTime() << "sec (time budget was " << this->solverTimeout << "sec) - total elapsed time: " << this->solvingTimeTotal+this->terminator.getElapsedTime() << "sec" << std::endl;
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
			lastAttemptSuccess = stat == CADICAL_SAT;
			if (!this->quiet) {
				std::cout << "SATSchedulerBinEnc: finished solving with status '" <<
									(lastAttemptSuccess?"SAT":"UNSAT") << "' (code '" << stat << "') after " << this->terminator.getElapsedTime()
									<< " sec (total: " << this->solvingTimeTotal << " sec)" << std::endl;
			}
			if(!lastAttemptSuccess) {
				//if (!this->quiet) {
				auto currentTime4 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				std::cerr << "SATSchedulerBinEnc: failed to find solution for II=" << this->candidateII << " and SL=" << this->candidateLatency << " at " << std::put_time(std::localtime(&currentTime4), "%Y-%m-%d %X") << std::endl;
				//}
				// check if it was due to a timeout
				if (this->terminator.getElapsedTime() >= this->solverTimeout) {
					// timeout when solving
					if (!this->quiet) {
						std::cout << "SATSchedulerBinEnc: encountered timeout after " << this->terminator.getElapsedTime() << "sec (time budget was " << this->solverTimeout << "sec)" << std::endl;
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
			this->fillSolutionStructure();
			//if (!this->quiet) {
			auto currentTime5 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			std::cerr << "SATSchedulerBinEnc: found solution for II=" << this->candidateII << " and SL=" << this->candidateLatency << " at " << std::put_time(std::localtime(&currentTime5), "%Y-%m-%d %X") << std::endl;
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

	void SATSchedulerBinEnc::defineLatLimits() {
#if 1
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
#else
		if (this->minLatencyUserDef and this->maxLatencyUserDef) {
			if (!this->quiet) {
				std::cout << "SATSchedulerBinEnc: min & max latency defined by user: " << this->minLatency << " & " << this->maxLatency << std::endl;
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
		else if (this->maxLatencyUserDef and !this->minLatencyUserDef) {
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
		else if (this->minLatencyUserDef and !this->maxLatencyUserDef) {
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
		// DEBUG
		/*for (auto &v : this->g.Vertices()) {
			this->earliestStartTime[v] = 0;
			this->latestStartTimeDifferences[v] = this->resourceModel.getVertexLatency(v);
		}*/
		// DEBUG
		// user feedback
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
#endif
	}

	void SATSchedulerBinEnc::resetContainer() {
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

	SATSchedulerBinEnc::trivialDecision SATSchedulerBinEnc::setUpSolver() {
		// reset solver
		//this->terminator = CaDiCaLTerminator(this->solverTimeout);
		if (!this->inIncrementalMode()) {
			this->solver = std::unique_ptr<CaDiCaL::Solver>(new CaDiCaL::Solver);
			this->solver->connect_terminator(&this->terminator);
		}
		// update latest start times based on candidate latency
		trivialDecision td = noTrivialDecisionPossible;
		for (auto &v : this->g.Vertices()) {
			this->latestStartTime[v] = this->candidateLatency - this->latestStartTimeDifferences.at(v);
			auto diff = this->latestStartTime[v] - this->earliestStartTime.at(v);
			if (diff < 0) {
				// maxLatencyConstraint is too tight! scheduling for this II is trivially impossible
				td = triviallyUNSAT;
				if (!this->quiet) {
					std::cout << "SATSchedulerBinEnc: maxLatencyConstraint is too tight! scheduling for this II is trivially impossible" << std::endl;
				}
				continue;
			}
			if (this->inIncrementalMode()) continue;
			auto wSize = (int)std::ceil(std::log2(diff+1));
			if (wSize < 1) wSize = 1;
			this->scheduleTimeWordSize[v] = wSize;
		}
		return td;
	}

	void SATSchedulerBinEnc::createLiterals() {
		if (this->inIncrementalMode()) return;
		// const zero/one
		this->constOneVar = ++this->literalCounter;
		this->constZeroVar = ++this->literalCounter;
		// create schedule time literals
		for (auto &v : this->g.Vertices()) {
			auto wordSize = this->scheduleTimeWordSize.at(v);
			for (int w=0; w<wordSize; w++) {
				this->scheduleTimeVariables[{v, w}] = ++this->literalCounter;
			}
		}
	}

	SATSchedulerBinEnc::trivialDecision SATSchedulerBinEnc::createClauses() {
		// set const zero/one variables
		if (!this->quiet) std::cout << "SATSchedulerBinEnc::createClauses: start" << std::endl;
		// check if we only need to create incremental clauses
		if (this->inIncrementalMode()) {
			this->createIncrementalClauses();
			return noTrivialDecisionPossible;
		}
		// create all base clauses
		this->create_arbitrary_clause({{this->constOneVar, false}});
		this->create_arbitrary_clause({{this->constZeroVar, true}});
		// upper limit for time slots
		if (!this->quiet) std::cout << "SATSchedulerBinEnc::createClauses: creating clauses for start time limitations" << std::endl;
		this->createTimeSlotLimitationClauses();
		// dependency constraints based on t_i - t_j <= C_ij
		if (!this->quiet) std::cout << "SATSchedulerBinEnc::createClauses: creating clauses for dependency constraints" << std::endl;
		auto tdDep = this->createDependencyClauses();
		if (tdDep == triviallyUNSAT) {
			if (!this->quiet) std::cout << "SATSchedulerBinEnc::createClauses: dependency constraints are impossible to satisfy" << std::endl;
			return triviallyUNSAT;
		}
		// resource constraints
		if (!this->quiet) std::cout << "SATSchedulerBinEnc::createClauses: creating clauses for resource constraints" << std::endl;
		this->createResourceLimitationClauses();
		if (!this->quiet) std::cout << "SATSchedulerBinEnc::createClauses: finished" << std::endl;
		return noTrivialDecisionPossible;
	}

	void SATSchedulerBinEnc::createTimeSlotLimitationClauses() {
		for (auto &v : this->g.Vertices()) {
			auto wordSize = this->scheduleTimeWordSize.at(v);
			if (wordSize < 1) {
				throw HatScheT::Exception("SATSchedulerBinEnc::createClauses: detected wordSize = '"+std::to_string(wordSize)+"' for vertex '"+v->getName()+"' -> this should never happen!");
			}
			auto maxValue = (1 << wordSize)-1;
			auto upperLimit = this->latestStartTime.at(v) - this->earliestStartTime.at(v);
			this->lastForbiddenTime[v] = upperLimit+1;
#if 1
			std::vector<std::pair<int, bool>> clauseBase;
			for (int idx = wordSize-1; idx >= 0; idx--) {
				if ((upperLimit >> idx) & 1) {
					clauseBase.emplace_back(this->scheduleTimeVariables.at({v, idx}), true);
					continue;
				}
				auto clause = clauseBase;
				clause.emplace_back(this->scheduleTimeVariables.at({v, idx}), true);
				this->create_arbitrary_clause(clause);
			}
#else
			for (int t = maxValue; t > upperLimit; t--) {
				std::vector<std::pair<int, bool>> clause;
				bool foundTrailingOne = false;
				for (int w = 0; w < wordSize; w++) {
					auto &var = this->scheduleTimeVariables.at({v, w});
					auto tBitVal = (bool)((t >> w) & 1);
					if (tBitVal) foundTrailingOne = true;
					if (foundTrailingOne) clause.emplace_back(var, tBitVal);
					//clause.emplace_back(var, tBitVal);
				}
				if (clause.empty()) continue;
				this->create_arbitrary_clause(clause);
			}
#endif
		}
	}

	SATSchedulerBinEnc::trivialDecision SATSchedulerBinEnc::createDependencyClauses() {
		trivialDecision td = triviallySAT; // optimistic
		for (auto &e : this->g.Edges()) {
			auto vSrc = &e->getVertexSrc();
			auto vDst = &e->getVertexDst();
			auto lSrc = this->resourceModel.getVertexLatency(vSrc);
			auto dist = e->getDistance();
			auto delay = e->getDelay();
			auto tSrcMin = this->earliestStartTime.at(vSrc);
			auto tDstMin = this->earliestStartTime.at(vDst);
			auto tSrcWidth = this->scheduleTimeWordSize.at(vSrc);
			auto tDstWidth = this->scheduleTimeWordSize.at(vDst);
			auto tSrcMax = this->latestStartTime.at(vSrc);
			auto tDstMax = this->latestStartTime.at(vDst);
			auto tSrcDiff = tSrcMax - tSrcMin;
			auto tDstDiff = tDstMax - tDstMin;
			auto edgeConst = -(lSrc + delay - (dist * this->candidateII) + tSrcMin - tDstMin);
			// enforce diff_ij = t_i - t_j <= edgeConst
			// check if edge is trivially satisfied
			auto worstCaseDiff = tSrcDiff; // t_j = 0 => diff_ij = t_i
			if (worstCaseDiff <= edgeConst) {
				continue;
			}
			// check if edge is trivially violated
			auto bestCaseDiff = -tDstDiff; // t_i = 0 => diff_ij = -t_j
			if (bestCaseDiff > edgeConst) {
				if (!this->quiet) {
					std::cout << "Edge '" << vSrc->getName() << "' -(" << dist << ")-> '" << vDst->getName() << "' is impossible to satisfy" << std::endl;
					std::cout << "  C = " << edgeConst << " = " << dist << "*" << this->candidateII << " + " << tDstMin << " - " << tSrcMin << " - " << lSrc << " - " << delay << std::endl;
					std::cout << "  tSrcDiff = " << tSrcDiff << std::endl;
					std::cout << "  tDstDiff = " << tDstDiff << std::endl;
				}
				return triviallyUNSAT;
			}
			// compute diff_ij = t_i - t_j
			td = noTrivialDecisionPossible;
			auto wordSizeSrc = this->scheduleTimeWordSize.at(vSrc);
			auto wordSizeDst = this->scheduleTimeWordSize.at(vDst);
			auto wordSizeDiff = std::max(wordSizeSrc, wordSizeDst) + 2;
			int litCarry = this->constOneVar;
			for (int w=0; w < wordSizeDiff; w++) {
				int litSrc;
				int litDst;
				if (w >= wordSizeSrc) {
					litSrc = this->constZeroVar;
				}
				else {
					litSrc = this->scheduleTimeVariables.at({vSrc, w});
				}
				if (w >= wordSizeDst) {
					litDst = this->constZeroVar;
				}
				else {
					litDst = this->scheduleTimeVariables.at({vDst, w});
				}
				int carryOut;
				if (w == wordSizeDiff - 1) {
					carryOut = -1;
				}
				else {
					carryOut = ++this->literalCounter;
					this->dependencyConstraintLiteralCounter++;
				}
				int litSum = this->diffVariables[{e, w}] = ++this->literalCounter;
				this->dependencyConstraintLiteralCounter++;
				this->dependencyConstraintClauseCounter += this->create_full_adder_clauses({litSrc, false}, {litDst, true}, {litCarry, false}, {litSum, false}, {carryOut, false});
				litCarry = carryOut;
			}
			// enforce diff_ij <= edgeConst
#if 1
			// build a hardware comparator
			int ok_last = -1;
			int carry_last = -1;
			for (int w = wordSizeDiff-1; w >= 0; w--) {
				auto c = (int)((edgeConst >> w) & 1);
				int x = this->diffVariables.at({e, w});
				int ok_new = -1;
				int carry_new = -1;
				if (w == wordSizeDiff-1) {
					// sign bit
					if (c) {
						ok_new = this->diffVariables.at({e, w});
						if (w != 0) {
							carry_new = this->diffVariables.at({e, w});
						}
					}
					else {
						ok_new = this->constOneVar;
						if (w != 0) {
							carry_new = ++this->literalCounter;
							this->dependencyConstraintLiteralCounter++;
							// carry_new = not x (via negated implications)
							this->dependencyConstraintClauseCounter += this->create_not(x, carry_new);
						}
					}
				}
				else {
					// regular bit
					ok_new = ++this->literalCounter;
					this->dependencyConstraintLiteralCounter++;
					if (w != 0) {
						carry_new = ++this->literalCounter;
						this->dependencyConstraintLiteralCounter++;
					}
					if (c) {
						this->dependencyConstraintClauseCounter += this->create_2x1_or(ok_last, carry_last, ok_new);
						if (w != 0) {
							this->dependencyConstraintClauseCounter += this->create_2x1_and(x, carry_last, carry_new);
						}
					}
					else {
						auto not_x = ++this->literalCounter;
						this->dependencyConstraintLiteralCounter++;
						this->dependencyConstraintClauseCounter += this->create_not(x, not_x);
						this->dependencyConstraintClauseCounter += this->create_2x1_mux(ok_last, not_x, carry_last, ok_new);
						if (w != 0) {
							this->dependencyConstraintClauseCounter += this->create_2x1_and(carry_last, not_x, carry_new);
						}
					}
				}
				// force ok bit of this stage to 1
				this->dependencyConstraintClauseCounter += this->create_arbitrary_clause({{ok_new, false}});
				// pass ok and carry bits to next stage
				ok_last = ok_new;
				carry_last = carry_new;
			}
#else
			for (int w1 = wordSizeDiff-1; w1 >= 0; w1--) {
				// check: is it the sign bit and must it be set to 1?
				if (w1 == wordSizeDiff-1 and edgeConst < 0) {
					this->dependencyConstraintClauseCounter += this->create_arbitrary_clause({{this->diffVariables.at({e, w1}), false}});
					continue;
				}
				// check: is it not the sign bit and must it be set to 0?
				if (w1 != wordSizeDiff-1 and (1 << w1) > edgeConst) {
					this->dependencyConstraintClauseCounter += this->create_arbitrary_clause({{this->diffVariables.at({e, w1}), true}});
					continue;
				}
				for (int w2 = w1-1; w2 >= 0; w2--) {
					int num = (1 << w2);
					std::vector<std::pair<int, bool>> clause(2);
					if (w1 == wordSizeDiff-1) {
						num -= (1 << w1);
						clause[0] = {this->diffVariables.at({e, w1}), false};
					}
					else {
						num += (1 << w1);
						clause[0] = {this->diffVariables.at({e, w1}), true};
					}
					if (num > edgeConst) {
						clause[1] = {this->diffVariables.at({e, w2}), true};
						this->dependencyConstraintClauseCounter += this->create_arbitrary_clause(clause);
					}
				}
			}
#endif
		}
		return td;
	}

	void SATSchedulerBinEnc::createResourceLimitationClauses2() {
		std::map<std::pair<const Vertex*, int>, int> bindingVariables;
		for (auto &v : this->g.Vertices()) {
			auto *r = this->resourceModel.getResource(v);
			auto rLim = r->getLimit();
			if (rLim == UNLIMITED or rLim == 1) continue;
			auto wLim = (int)std::ceil(std::log2(rLim));
			for (int w=0; w<wLim; w++) {
				bindingVariables[{v, w}] = ++this->literalCounter;
			}
			//
		}
	}

	void SATSchedulerBinEnc::createResourceLimitationClauses() {
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
			auto &tOffset = this->earliestStartTime.at(v);
			auto tDiff = this->latestStartTime.at(v) - this->earliestStartTime.at(v);
			auto &tWordSize = this->scheduleTimeWordSize.at(v);
			// create implications for the adder input
			for (int t=0; t<=tDiff; t++) {
				auto tSched = t + tOffset;
				auto modSlot = tSched % this->candidateII;
				auto addInput = this->moduloSlotVariables[{v, modSlot}];
				if (addInput == 0) {
					addInput = this->moduloSlotVariables[{v, modSlot}] = ++this->literalCounter;
					this->moduloSlotLiteralCounter++;
				}
				// create implication
				std::vector<std::pair<int, bool>> clause(tWordSize+1);
				for (int w = 0; w < tWordSize; w++) {
					auto bitVal = (bool)((t >> w) & 1);
					clause[w] = {this->scheduleTimeVariables.at({v, w}), bitVal};
				}
				clause[tWordSize] = {addInput, false};
				this->create_arbitrary_clause(clause);
			}
			// create adders for all possible modulo slots where this vertex can be scheduled
			// and forbid that the adder output exceeds the resource limit
			for (int m = 0; m < this->candidateII; m++) {
				auto &addInputBit = this->moduloSlotVariables[{v, m}];
				if (addInputBit == 0) {
					// vertex cannot be scheduled in this modulo slot -> no need to account for that
					continue;
				}
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
	}

	void SATSchedulerBinEnc::fillSolutionStructure() {
		// define schedule times from schedule time literals
		for (auto &v : this->g.Vertices()) {
			int t = 0;
			auto wordSize = this->scheduleTimeWordSize.at(v);
			for (int w = 0; w < wordSize; w++) {
				auto bitResult = this->solver->val(this->scheduleTimeVariables.at({v, w})) > 0 ? 1 : 0;
				t += (bitResult << w);
			}
			this->startTimes[v] = t + this->earliestStartTime.at(v);
			for (int m = 0; m < this->candidateII; m++) {
				if (this->moduloSlotVariables[{v, m}] == 0) continue;
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
					std::cout << "SATSchedulerBinEnc: vertex '" << it.first->getName() << "' (t='" << it.second << "', lat='" << this->resourceModel.getVertexLatency(it.first) << "') violates requested schedule length of '" << this->candidateLatency << "'" << std::endl;
				}
			}
			throw Exception("SATSchedulerBinEnc: Found invalid schedule! Requested candidate schedule length was '"+std::to_string(this->candidateLatency)+"' but scheduler found schedule length '"+std::to_string(actualScheduleLength)+"'");
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

	int SATSchedulerBinEnc::create_arbitrary_clause(const vector<std::pair<int, bool>> &a) {
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

	int SATSchedulerBinEnc::create_arbitrary_clause(const vector<std::pair<int, bool>> &a, bool quiet) {
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

	int SATSchedulerBinEnc::create_full_adder_clauses(std::pair<int, bool> a, std::pair<int, bool> b,
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

	int SATSchedulerBinEnc::create_half_adder_clauses(std::pair<int, bool> a, std::pair<int, bool> b,
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

	int SATSchedulerBinEnc::create_not(int x, int not_x) {
		// 1) -x -not_x
		this->create_arbitrary_clause({{x, true}, {not_x, true}});
		// 2) x not_x
		this->create_arbitrary_clause({{x, false}, {not_x, false}});
		return 2;
	}

	int SATSchedulerBinEnc::create_2x1_and(int a, int b, int y) {
		// 1) -a -b  y
		this->create_arbitrary_clause({{a, true},{b, true},{y, false}});
		// 2)  a -y
		this->create_arbitrary_clause({{a, false},{y, true}});
		// 3)  b -y
		this->create_arbitrary_clause({{b, false},{y, true}});
		return 3;
	}

	int SATSchedulerBinEnc::create_2x1_or(int a, int b, int y) {
		// 1)  a  b  -y
		this->create_arbitrary_clause({{a, false},{b, false},{y, true}});
		// 2) -a  y
		this->create_arbitrary_clause({{a, true},{y, false}});
		// 3) -b  y
		this->create_arbitrary_clause({{b, true},{y, false}});
		return 3;
	}

	int SATSchedulerBinEnc::create_2x1_mux(int a, int b, int s, int y) {
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

	void SATSchedulerBinEnc::createIncrementalClauses() {
		for (auto &v : this->g.Vertices()) {
			auto wordSize = this->scheduleTimeWordSize.at(v);
			auto tForbidden = this->latestStartTime.at(v) - this->earliestStartTime.at(v) + 1;

			auto tForbiddenLast = this->lastForbiddenTime.at(v);
			for (auto t = tForbidden; t < tForbiddenLast; t++) {
#if 1
				std::vector<std::pair<int, bool>> clause;
				for (int w=0; w<wordSize; w++) {
					if ((t >> w) & 1) clause.emplace_back(this->scheduleTimeVariables.at({v, w}), true);
				}
				if (clause.empty()) continue;
				this->create_arbitrary_clause(clause);
#else
				std::vector<std::pair<int, bool>> clause;
				bool foundTrailingOne = false;
				for (int w=0; w<wordSize; w++) {
					auto bitSet = (bool)((t >> w) & 1);
					if (bitSet) foundTrailingOne = true;
					if (foundTrailingOne) clause.emplace_back(this->scheduleTimeVariables.at({v, w}), bitSet);
					//clause.emplace_back(this->scheduleTimeVariables.at({v, w}), bitSet);
				}
				if (clause.empty()) continue;
				this->create_arbitrary_clause(clause);
#endif
			}
			this->lastForbiddenTime[v] = tForbidden;

			/*
			int breakpointWordSize = -1;
			for (int w=wordSize-1; w>=0; w--) {
				auto num = (1 << w);
				if (num >= tForbidden) {
					this->create_arbitrary_clause({{this->scheduleTimeVariables.at({v, w}), true}});
					continue;
				}
				breakpointWordSize = w;
				break;
			}
			if (breakpointWordSize < 0) continue;
			//std::cerr << "#q# breakpointWordSize = " << breakpointWordSize << std::endl;
			for (int w=breakpointWordSize; w>=0; w--) {
				if (((tForbidden >> w) & 1) != 0) {
					continue;
				}
				std::vector<std::pair<int, bool>> clause;
				// add clause to forbid that shit
				for (int w2=breakpointWordSize; w2>w; w2--) {
					if (((tForbidden >> w2) & 1) == 1) {
						clause.emplace_back(this->scheduleTimeVariables.at({v, w2}), true);
					}
				}
				clause.emplace_back(this->scheduleTimeVariables.at({v, w}), true);
				this->create_arbitrary_clause(clause);
			}
			*/
		}
	}

	bool SATSchedulerBinEnc::inIncrementalMode() {
		return this->scheduleFound and (this->los == LINEAR_JUMP or this->los == REVERSE_LINEAR);
	}
}