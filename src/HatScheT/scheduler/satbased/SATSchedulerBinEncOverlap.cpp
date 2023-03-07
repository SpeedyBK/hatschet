//
// Created by nfiege on 3/6/23.
//

#include "SATSchedulerBinEncOverlap.h"
#include <HatScheT/utility/Utility.h>
#include <HatScheT/utility/Verifier.h>
#include <iomanip>

namespace HatScheT {

	SATSchedulerBinEncOverlap::SATSchedulerBinEncOverlap(Graph &g, ResourceModel &resourceModel, int II) :
		SATSchedulerBase(g, resourceModel, II) {
		this->los = LatencyOptimizationStrategy::LINEAR_JUMP;
	}

	void SATSchedulerBinEncOverlap::scheduleIteration() {
		// call parent's method for some basic stuff
		SATSchedulerBase::scheduleIteration();

		this->scheduleTimeVariables.clear();
		this->scheduleTimeWordSize.clear();
		this->diffVariables.clear();
		this->moduloSlotVariables.clear();
		this->lastForbiddenTime.clear();
		this->constOneVar = 0;
		this->constZeroVar = 0;

		this->timeSlotLiteralCounter = 0;
		this->bindingLiteralCounter = 0;
		this->overlapLiteralCounter = 0;
		this->resourceConstraintLiteralCounter = 0;
		this->moduloSlotLiteralCounter = 0;
		this->dependencyConstraintSubLiteralCounter = 0;
		this->dependencyConstraintCompLiteralCounter = 0;

		this->moduloComputationClauseCounter = 0;
		this->overlapClauseCounter = 0;
		this->bindingConstraintClauseCounter = 0;
		this->timeSlotConstraintClauseCounter = 0;
		this->dependencyConstraintSubClauseCounter = 0;
		this->dependencyConstraintCompClauseCounter = 0;
		this->resourceConstraintClauseCounter = 0;

		//if (!this->quiet) {
		auto currentTime1 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
		std::cerr << "SATSchedulerBinEncOverlap: trying candidate II=" << this->candidateII << " at time " << 	std::put_time(std::localtime(&currentTime1), "%Y-%m-%d %X") << std::endl;
		//}

		// compute min/max latency
		this->defineLatLimits();
		if (!this->quiet) {
			std::cout << "SATSchedulerBinEncOverlap: defined SL limits: " << this->minLatency << " <= SL <= " << this->maxLatency << std::endl;
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
			std::cout << "SATSchedulerBinEncOverlap: candidate II=" << this->candidateII << " with min latency="
								<< this->latencyLowerBound << " and max latency=" << this->latencyUpperBound << std::endl;
		}
		while (this->computeNewLatencySuccess(lastAttemptSuccess)) {
			if (!this->quiet) {
				std::cout << "SATSchedulerBinEncOverlap: new latency limit = " << this->candidateLatency << std::endl;
				std::cout << "SATSchedulerBinEncOverlap: resetting containers" << std::endl;
			}
			this->resetContainer();
			if (!this->quiet) {
				std::cout << "SATSchedulerBinEncOverlap: setting up solver" << std::endl;
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
				std::cout << "SATSchedulerBinEncOverlap: creating literals" << std::endl;
			}
			this->createLiterals();
			if (!this->quiet) {
				std::cout << "SATSchedulerBinEncOverlap: creating clauses" << std::endl;
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
				std::cout << "SATSchedulerBinEncOverlap: time is " << this->terminator.getElapsedTime() << "sec after constructing the problem" << std::endl;
			}
			if (this->terminator.terminate()) {
				// timeout during problem construction :(
				if (!this->quiet) {
					std::cout << "SATSchedulerBinEncOverlap: encountered timeout during problem construction after " << this->terminator.getElapsedTime() << "sec (time budget was " << this->solverTimeout << "sec) - total elapsed time: " << this->solvingTimeTotal+this->terminator.getElapsedTime() << "sec" << std::endl;
				}
				breakByTimeout = true;
				break;
			}
			if (!this->quiet) {
				std::cout << "SATSchedulerBinEncOverlap: start scheduling for II=" << this->candidateII << " and latency=" << this->candidateLatency << " with '" << this->literalCounter << "' literals and '"
									<< this->clauseCounter << "' clauses" << std::endl;
				std::cout << "  '" << this->timeSlotLiteralCounter << "' time slot literals" << std::endl;
				std::cout << "  '" << this->bindingLiteralCounter << "' binding literals" << std::endl;
				std::cout << "  '" << this->moduloSlotLiteralCounter << "' modulo slot literals" << std::endl;
				std::cout << "  '" << this->resourceConstraintLiteralCounter << "' resource constraint literals" << std::endl;
				std::cout << "  '" << this->overlapLiteralCounter << "' overlap literals" << std::endl;
				std::cout << "  '" << this->dependencyConstraintSubLiteralCounter << "' dependency constraint subtract literals" << std::endl;
				std::cout << "  '" << this->dependencyConstraintCompLiteralCounter << "' dependency constraint comparator literals" << std::endl;

				std::cout << "  '" << this->timeSlotConstraintClauseCounter << "' time slot constraint clauses" << std::endl;
				std::cout << "  '" << this->bindingConstraintClauseCounter << "' binding constraint constraint clauses" << std::endl;
				std::cout << "  '" << this->dependencyConstraintSubClauseCounter << "' dependency constraint subtract clauses" << std::endl;
				std::cout << "  '" << this->dependencyConstraintCompClauseCounter << "' dependency constraint comparator clauses" << std::endl;
				std::cout << "  '" << this->moduloComputationClauseCounter << "' modulo computation constraint clauses" << std::endl;
				std::cout << "  '" << this->overlapClauseCounter << "' overlap constraint clauses" << std::endl;
				std::cout << "  '" << this->resourceConstraintClauseCounter << "' resource constraint clauses" << std::endl;

				auto currentTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				std::cout << "  current time: " << std::put_time(std::localtime(&currentTime2), "%Y-%m-%d %X") << std::endl;
			}
			if (this->terminator.getElapsedTime() >= this->solverTimeout) {
				// timeout after problem construction!
				if (!this->quiet) {
					std::cout << "SATSchedulerBinEncOverlap: encountered timeout after " << this->terminator.getElapsedTime() << "sec (time budget was " << this->solverTimeout << "sec) - total elapsed time: " << this->solvingTimeTotal+this->terminator.getElapsedTime() << "sec" << std::endl;
				}
				breakByTimeout = true;
				break;
			}
			// start solving
			//if (!this->quiet) {
			auto currentTime3 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			std::cerr << "SATSchedulerBinEncOverlap: start solving at time " << 	std::put_time(std::localtime(&currentTime3), "%Y-%m-%d %X") << std::endl;
			//}
			auto stat = this->solver->solve();
			lastAttemptSuccess = stat == CADICAL_SAT;
			auto unsat = stat == CADICAL_UNSAT;
			std::string statusStr;
			if (lastAttemptSuccess) {
				statusStr = "SAT";
			}
			else if (unsat) {
				statusStr = "UNSAT";
			}
			else {
				statusStr = "TIMEOUT";
			}
			if (!this->quiet) {
				std::cout << "SATSchedulerBinEncOverlap: finished solving with status '" << statusStr
									<< "' (code '" << stat << "') after " << this->terminator.getElapsedTime()
									<< " sec (total: " << this->solvingTimeTotal << " sec)" << std::endl;
			}
			if(!lastAttemptSuccess) {
				//if (!this->quiet) {
				auto currentTime4 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				std::cerr << "SATSchedulerBinEncOverlap: failed to find solution for II=" << this->candidateII << " and SL=" << this->candidateLatency << " at " << std::put_time(std::localtime(&currentTime4), "%Y-%m-%d %X") << " (" << statusStr << ")" << std::endl;
				//}
				// check if it was due to a timeout
				if (this->terminator.getElapsedTime() >= this->solverTimeout) {
					// timeout when solving
					if (!this->quiet) {
						std::cout << "SATSchedulerBinEncOverlap: encountered timeout after " << this->terminator.getElapsedTime() << "sec (time budget was " << this->solverTimeout << "sec)" << std::endl;
					}
					breakByTimeout = true;
					break;
				}
				// schedule attempt failed :(
				// let's try again for the next latency :)
				if (!this->quiet) {
					std::cout << "SATSchedulerBinEncOverlap: latency " << this->candidateLatency << " is infeasible - trying again with modified latency limit" << std::endl;
				}
				continue;
			}
			this->scheduleFound = true;
			this->fillSolutionStructure();
			//if (!this->quiet) {
			auto currentTime5 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			std::cerr << "SATSchedulerBinEncOverlap: found solution for II=" << this->candidateII << " and SL=" << this->candidateLatency << " at " << std::put_time(std::localtime(&currentTime5), "%Y-%m-%d %X") << std::endl;
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
		//throw Exception("#q# EXCEPT!!");
	}

	void SATSchedulerBinEncOverlap::defineLatLimits() {
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
				this->maxLatency = std::min(this->maxLatency, this->maxLatencyConstraint);
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
	}

	void SATSchedulerBinEncOverlap::resetContainer() {
		if (this->inIncrementalMode()) return;
		// reset literal containers
		this->scheduleTimeVariables.clear();
		this->moduloSlotVariables.clear();
		this->bindingVariables.clear();
		this->diffVariables.clear();
		this->bindingOverlapVariables.clear();
		this->moduloOverlapVariables.clear();
		// reset literal counter(s)
		this->literalCounter = 0;
		this->timeSlotLiteralCounter = 0;
		this->moduloSlotLiteralCounter = 0;
		this->resourceConstraintLiteralCounter = 0;
		this->dependencyConstraintSubClauseCounter = 0;
		this->dependencyConstraintCompClauseCounter = 0;
		this->bindingLiteralCounter = 0;
		this->overlapLiteralCounter = 0;
		// reset clause counter(s)
		this->clauseCounter = 0;
		this->timeSlotConstraintClauseCounter = 0;
		this->bindingConstraintClauseCounter = 0;
		this->dependencyConstraintSubClauseCounter = 0;
		this->dependencyConstraintCompClauseCounter = 0;
		this->moduloComputationClauseCounter = 0;
		this->overlapClauseCounter = 0;
		this->resourceConstraintClauseCounter = 0;
	}

	SATSchedulerBinEncOverlap::trivialDecision SATSchedulerBinEncOverlap::setUpSolver() {
		// reset solver
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
					std::cout << "SATSchedulerBinEncOverlap: maxLatencyConstraint is too tight! scheduling for this II is trivially impossible" << std::endl;
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

	void SATSchedulerBinEncOverlap::createLiterals() {
		if (this->inIncrementalMode()) return;
		// const zero/one
		this->constOneVar = ++this->literalCounter;
		this->constZeroVar = ++this->literalCounter;
		// create schedule time literals
		for (auto &v : this->g.Vertices()) {
			auto wordSize = this->scheduleTimeWordSize.at(v);
			for (int w=0; w<wordSize; w++) {
				this->scheduleTimeVariables[{v, w}] = ++this->literalCounter;
				this->timeSlotLiteralCounter++;
			}
		}
		// create modulo slot literals
		auto modSlotWordSize = (int)std::ceil(std::log2(this->candidateII));
		for (auto &v : this->g.Vertices()) {
			auto r = this->resourceModel.getResource(v);
			auto limit = r->getLimit();
			if (limit == UNLIMITED) continue;
			for (int w=0; w<modSlotWordSize; w++) {
				this->moduloSlotVariables[{v, w}] = ++this->literalCounter;
				this->moduloSlotLiteralCounter++;
			}
		}
		// create binding literals
		for (auto &v : this->g.Vertices()) {
			auto r = this->resourceModel.getResource(v);
			auto limit = r->getLimit();
			if (limit == UNLIMITED or limit == 1) continue;
			auto wordSize = (int)std::ceil(std::log2(limit));
			for (int w=0; w<wordSize; w++) {
				this->bindingVariables[{v, w}] = ++this->literalCounter;
				this->bindingLiteralCounter++;
			}
		}
		// create overlap variables
		for (auto &r : this->resourceModel.Resources()) {
			auto limit = r->getLimit();
			if (limit == UNLIMITED) continue;
			auto vertices = this->resourceModel.getVerticesOfResource(r);
			for (auto &i : vertices) {
				for (auto &j : vertices) {
					if (i->getId() >= j->getId()) continue;
					auto wordSizeModulo = (int)std::ceil(std::log2(this->candidateII));
					for (int w=0; w<wordSizeModulo; w++) {
						this->moduloOverlapVariables[{i, j, w}] = ++this->literalCounter;
						this->overlapLiteralCounter++;
					}
					if (limit == 1) continue;
					auto wordSizeBind = (int)std::ceil(std::log2(limit));
					for (int w=0; w<wordSizeBind; w++) {
						this->bindingOverlapVariables[{i, j, w}] = ++this->literalCounter;
						this->overlapLiteralCounter++;
					}
				}
			}
		}
	}

	SATSchedulerBinEncOverlap::trivialDecision SATSchedulerBinEncOverlap::createClauses() {
		// set const zero/one variables
		if (!this->quiet) std::cout << "SATSchedulerBinEncOverlap::createClauses: start" << std::endl;
		// check if we only need to create incremental clauses
		if (this->inIncrementalMode()) {
			this->createIncrementalClauses();
			return noTrivialDecisionPossible;
		}
		// create all base clauses
		this->create_arbitrary_clause({{this->constOneVar, false}});
		this->create_arbitrary_clause({{this->constZeroVar, true}});
		// upper limit for time slots
		if (!this->quiet) std::cout << "SATSchedulerBinEncOverlap::createClauses: creating clauses for start time limitations" << std::endl;
		this->createTimeSlotLimitationClauses();
		if (!this->quiet) std::cout << "SATSchedulerBinEncOverlap::createClauses: creating clauses for binding value limitations" << std::endl;
		this->createBindingLimitationClauses();
		// dependency constraints based on t_i - t_j <= C_ij
		if (!this->quiet) std::cout << "SATSchedulerBinEncOverlap::createClauses: creating clauses for dependency constraints" << std::endl;
		auto tdDep = this->createDependencyClauses();
		if (tdDep == triviallyUNSAT) {
			if (!this->quiet) std::cout << "SATSchedulerBinEncOverlap::createClauses: dependency constraints are impossible to satisfy" << std::endl;
			return triviallyUNSAT;
		}
		// resource constraints
		if (!this->quiet) std::cout << "SATSchedulerBinEncOverlap::createClauses: creating clauses for modulo computation (m_i = t_i mod " << this->candidateII << ")" << std::endl;
		this->createModuloComputationClauses();
		if (!this->quiet) std::cout << "SATSchedulerBinEncOverlap::createClauses: creating overlap clauses for resource constraints" << std::endl;
		this->createOverlapClauses();
		if (!this->quiet) std::cout << "SATSchedulerBinEncOverlap::createClauses: finished" << std::endl;
		return noTrivialDecisionPossible;
	}

	void SATSchedulerBinEncOverlap::createTimeSlotLimitationClauses() {
		for (auto &v : this->g.Vertices()) {
			auto wordSize = this->scheduleTimeWordSize.at(v);
			if (wordSize < 1) {
				throw HatScheT::Exception("SATSchedulerBinEncOverlap::createClauses: detected wordSize = '"+std::to_string(wordSize)+"' for vertex '"+v->getName()+"' -> this should never happen!");
			}
			auto maxValue = (1 << wordSize)-1;
			auto upperLimit = this->latestStartTime.at(v) - this->earliestStartTime.at(v);
			this->lastForbiddenTime[v] = upperLimit+1;
			std::vector<std::pair<int, bool>> clauseBase;
			for (int idx = wordSize-1; idx >= 0; idx--) {
				if ((upperLimit >> idx) & 1) {
					clauseBase.emplace_back(this->scheduleTimeVariables.at({v, idx}), true);
					continue;
				}
				auto clause = clauseBase;
				clause.emplace_back(this->scheduleTimeVariables.at({v, idx}), true);
				this->timeSlotConstraintClauseCounter += this->create_arbitrary_clause(clause);
			}
		}
	}

	void SATSchedulerBinEncOverlap::createBindingLimitationClauses() {
		// forbid too large binding values
		for (auto &v : this->g.Vertices()) {
			auto r = this->resourceModel.getResource(v);
			auto limit = r->getLimit();
			if (limit == UNLIMITED or limit == 1) continue;
			auto upperLimit = limit-1;
			auto wordSize = (int)std::ceil(std::log2(limit));
			auto maxValue = (1 << wordSize) - 1;
			if (upperLimit == maxValue) continue; // limit is a power of 2
			std::vector<std::pair<int, bool>> clauseBase;
			for (int idx = wordSize-1; idx >= 0; idx--) {
				if ((upperLimit >> idx) & 1) {
					clauseBase.emplace_back(this->bindingVariables.at({v, idx}), true);
					continue;
				}
				auto clause = clauseBase;
				clause.emplace_back(this->bindingVariables.at({v, idx}), true);
				this->bindingConstraintClauseCounter += this->create_arbitrary_clause(clause);
			}
		}
	}

	SATSchedulerBinEncOverlap::trivialDecision SATSchedulerBinEncOverlap::createDependencyClauses() {
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
					this->dependencyConstraintSubLiteralCounter++;
				}
				int litSum = this->diffVariables[{e, w}] = ++this->literalCounter;
				this->dependencyConstraintSubLiteralCounter++;
				int clauseMode = 0;
				if (w == wordSizeDiff - 1) {
					clauseMode = -1;
				}
				else {
					clauseMode = 1;
				}
				this->dependencyConstraintSubClauseCounter += this->create_full_adder_clauses({litSrc, false}, {litDst, true}, {litCarry, false}, {litSum, false}, {carryOut, false});
				litCarry = carryOut;
			}
			// enforce diff_ij <= edgeConst
			std::vector<std::pair<int, bool>> clauseBase;
			for (int idx = wordSizeDiff-1; idx >= 0; idx--) {
				auto bitSet = (bool)((edgeConst >> idx) & 1);
				if (idx == wordSizeDiff-1 and !bitSet) {
					clauseBase.emplace_back(this->diffVariables.at({e, idx}), false);
					continue;
				}
				if (idx != wordSizeDiff-1 and bitSet) {
					clauseBase.emplace_back(this->diffVariables.at({e, idx}), true);
					continue;
				}
				auto clause = clauseBase;
				clause.emplace_back(this->diffVariables.at({e, idx}), true);
				this->dependencyConstraintCompClauseCounter += this->create_arbitrary_clause(clause);
			}
		}
		return td;
	}

	void SATSchedulerBinEncOverlap::createModuloComputationClauses() {
		// compute m_i = t_i mod II for all resource-limited vertices v_i
		auto mWordSize = (int)std::ceil(std::log2(this->candidateII));
		for (auto &v : this->g.Vertices()) {
			if (this->resourceModel.getResource(v)->isUnlimited()) continue;
			auto tMin = this->earliestStartTime.at(v);
			auto tMax = this->latestStartTime.at(v);
			auto tWordSize = this->scheduleTimeWordSize.at(v);
			for (int t=tMin; t<=tMax; t++) {
				auto m = t % this->candidateII;
				std::vector<std::pair<int, bool>> clauseBase;
				for (int w=0; w<tWordSize; w++) {
					auto bitSet = (bool)(((t-tMin) >> w) & 1);
					clauseBase.emplace_back(this->scheduleTimeVariables.at({v, w}), bitSet);
				}
				for (int w=0; w<mWordSize; w++) {
					auto clause = clauseBase;
					auto bitSet = (bool)((m >> w) & 1);
					clause.emplace_back(this->moduloSlotVariables.at({v, w}), not bitSet);
					this->moduloComputationClauseCounter += this->create_arbitrary_clause(clause);
				}
			}
		}
	}

	void SATSchedulerBinEncOverlap::createOverlapClauses() {
		// ensure that all pairs of vertices of the same limited operator type are either
		//   a) scheduled in different modulo slots
		//   b) bound to different operators
		for (auto &r : this->resourceModel.Resources()) {
			auto limit = r->getLimit();
			if (limit == UNLIMITED) continue;
			auto trivialBinding = limit == 1;
			auto vertices = this->resourceModel.getVerticesOfResource(r);
			for (auto &i : vertices) {
				for (auto &j : vertices) {
					if (i->getId() >= j->getId()) continue;
					auto moduloSlotWordSize = (int)std::ceil(std::log2(this->candidateII));
					// clauses: M_ji -> m_i != m_j
					// => for each bit w:
					//   1) -M_ijw or -m_iw or -m_jw
					//   2) -M_ijw or  m_iw or  m_jw
					std::vector<std::pair<int, bool>> overlapClause;
					for (auto w=0; w<moduloSlotWordSize; w++) {
						auto M_ijw = this->moduloOverlapVariables.at({i, j, w});
						auto m_iw = this->moduloSlotVariables.at({i, w});
						auto m_jw = this->moduloSlotVariables.at({j, w});
						this->overlapClauseCounter += this->create_arbitrary_clause({{M_ijw, true}, {m_iw, true}, {m_jw, true}});
						this->overlapClauseCounter += this->create_arbitrary_clause({{M_ijw, true}, {m_iw, false}, {m_jw, false}});
						overlapClause.emplace_back(M_ijw, false);
					}
					if (trivialBinding) {
						// number of FUs = 1 => modulo slots MUST be different!
						this->overlapClauseCounter += this->create_arbitrary_clause(overlapClause);
						continue;
					}
					auto bindingWordSize = (int)std::ceil(std::log2(limit));
					// clauses: B_ji -> b_i != b_j
					// => for each bit w:
					//   1) -B_ij or -b_iw or -b_jw
					//   2) -B_ij or  b_iw or  b_jw
					for (auto w=0; w<bindingWordSize; w++) {
						auto B_ijw = this->bindingOverlapVariables.at({i, j, w});
						auto b_iw = this->bindingVariables.at({i, w});
						auto b_jw = this->bindingVariables.at({j, w});
						this->overlapClauseCounter += this->create_arbitrary_clause({{B_ijw, true}, {b_iw, true}, {b_jw, true}});
						this->overlapClauseCounter += this->create_arbitrary_clause({{B_ijw, true}, {b_iw, false}, {b_jw, false}});
						overlapClause.emplace_back(B_ijw, false);
					}
					// modulo slot or binding must be different!
					this->overlapClauseCounter += this->create_arbitrary_clause(overlapClause);
				}
			}
		}
	}

	void SATSchedulerBinEncOverlap::fillSolutionStructure() {
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
					std::cout << "SATSchedulerBinEncOverlap: vertex '" << it.first->getName() << "' (t='" << it.second << "', lat='" << this->resourceModel.getVertexLatency(it.first) << "', wSize='" << this->scheduleTimeWordSize.at(it.first) << "', offset='" << this->earliestStartTime.at(it.first) << "', diff='" << this->latestStartTimeDifferences.at(it.first) << "') violates requested schedule length of '" << this->candidateLatency << "'" << std::endl;
				}
			}
			verifyModuloSchedule(this->g, this->resourceModel, this->startTimes, this->candidateII);
			throw Exception("SATSchedulerBinEncOverlap: Found invalid schedule! Requested candidate schedule length was '"+std::to_string(this->candidateLatency)+"' but scheduler found schedule length '"+std::to_string(actualScheduleLength)+"'");
		}
		this->candidateLatency = actualScheduleLength;
	}

	int SATSchedulerBinEncOverlap::create_arbitrary_clause(const vector<std::pair<int, bool>> &a) {
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

	int SATSchedulerBinEncOverlap::create_arbitrary_clause(const vector<std::pair<int, bool>> &a, bool quiet) {
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

	int SATSchedulerBinEncOverlap::create_full_adder_clauses(std::pair<int, bool> a, std::pair<int, bool> b,
																										std::pair<int, bool> c_i, std::pair<int, bool> sum,
																										std::pair<int, bool> c_o, int clauseMode) {
		// clause mode =  0: create all clauses
		// clause mode =  1: create all clauses leading to true sum/c_o values
		// clause mode = -1: create all clauses leading to false sum/c_o values
		if (clauseMode > 1 or clauseMode < -1) {
			throw Exception("SATSchedulerBinEncOverlap::create_full_adder_clauses: invalid clause mode");
		}
		int clause_counter = 0;
		if (c_o.first <= 0) {
			// build only sum bit
			// 1)
			if (clauseMode == 0 or (not sum.second and clauseMode == 1) or (sum.second and clauseMode == -1))
				clause_counter += this->create_arbitrary_clause({{a.first, a.second}, {b.first, not b.second}, {c_i.first, c_i.second}, {sum.first, sum.second}});
			// 2)
			if (clauseMode == 0 or (not sum.second and clauseMode == 1) or (sum.second and clauseMode == -1))
				clause_counter += this->create_arbitrary_clause({{a.first, not a.second}, {b.first, b.second}, {c_i.first, c_i.second}, {sum.first, sum.second}});
			// 3)
			if (clauseMode == 0 or (not sum.second and clauseMode == 1) or (sum.second and clauseMode == -1))
				clause_counter += this->create_arbitrary_clause({{a.first, a.second}, {b.first, b.second}, {c_i.first, not c_i.second}, {sum.first, sum.second}});
			// 4)
			if (clauseMode == 0 or (sum.second and clauseMode == 1) or (not sum.second and clauseMode == -1))
				clause_counter += this->create_arbitrary_clause({{a.first, not a.second}, {b.first, not b.second}, {c_i.first, not c_i.second}, {sum.first, sum.second}});
			// 5)
			if (clauseMode == 0 or (sum.second and clauseMode == 1) or (not sum.second and clauseMode == -1))
				clause_counter += this->create_arbitrary_clause({{a.first, a.second}, {b.first, b.second}, {c_i.first, c_i.second}, {sum.first, not sum.second}});
			// 6)
			if (clauseMode == 0 or (sum.second and clauseMode == 1) or (not sum.second and clauseMode == -1))
				clause_counter += this->create_arbitrary_clause({{a.first, not a.second}, {b.first, not b.second}, {c_i.first, c_i.second}, {sum.first, not sum.second}});
			// 7)
			if (clauseMode == 0 or (sum.second and clauseMode == 1) or (not sum.second and clauseMode == -1))
				clause_counter += this->create_arbitrary_clause({{a.first, a.second}, {b.first, not b.second}, {c_i.first, not c_i.second}, {sum.first, not sum.second}});
			// 8)
			if (clauseMode == 0 or (sum.second and clauseMode == 1) or (not sum.second and clauseMode == -1))
				clause_counter += this->create_arbitrary_clause({{a.first, not a.second}, {b.first, b.second}, {c_i.first, not c_i.second}, {sum.first, not sum.second}});
			return clause_counter;
		}
		// build carry and use it to build sum
		// 1)
		if (clauseMode == 0 or (not c_o.second and clauseMode == 1) or (c_o.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{a.first, not a.second}, {b.first, not b.second}, {c_o.first, c_o.second}});
		// 2)
		if (clauseMode == 0 or (not c_o.second and clauseMode == 1) or (c_o.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{b.first, not b.second}, {c_i.first, not c_i.second}, {c_o.first, c_o.second}});
		// 3)
		if (clauseMode == 0 or (not c_o.second and clauseMode == 1) or (c_o.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{a.first, not a.second}, {c_i.first, not c_i.second}, {c_o.first, c_o.second}});
		// 4)
		if (clauseMode == 0 or (c_o.second and clauseMode == 1) or (not c_o.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{a.first, a.second}, {c_i.first, c_i.second}, {c_o.first, not c_o.second}});
		// 5)
		if (clauseMode == 0 or (c_o.second and clauseMode == 1) or (not c_o.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{b.first, b.second}, {c_i.first, c_i.second}, {c_o.first, not c_o.second}});
		// 6)
		if (clauseMode == 0 or (c_o.second and clauseMode == 1) or (not c_o.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{a.first, a.second}, {b.first, b.second}, {c_o.first, not c_o.second}});

		// 1)    a -c_o -sum
		if (clauseMode == 0 or (sum.second and clauseMode == 1) or (not sum.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{a.first, a.second}, {c_o.first, not c_o.second}, {sum.first, not sum.second}});
		// 2)    b -c_o -sum
		if (clauseMode == 0 or (sum.second and clauseMode == 1) or (not sum.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{b.first, b.second}, {c_o.first, not c_o.second}, {sum.first, not sum.second}});
		// 3)  c_i -c_o -sum
		if (clauseMode == 0 or (sum.second and clauseMode == 1) or (not sum.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{c_i.first, c_i.second}, {c_o.first, not c_o.second}, {sum.first, not sum.second}});
		// 4)    a    b  c_i -sum
		if (clauseMode == 0 or (sum.second and clauseMode == 1) or (not sum.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{a.first, a.second}, {b.first, b.second}, {c_i.first, c_i.second}, {sum.first, not sum.second}});
		// 5)   -a  c_o  sum
		if (clauseMode == 0 or (not sum.second and clauseMode == 1) or (sum.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{a.first, not a.second}, {c_o.first, c_o.second}, {sum.first, sum.second}});
		// 6)   -b  c_o  sum
		if (clauseMode == 0 or (not sum.second and clauseMode == 1) or (sum.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{b.first, not b.second}, {c_o.first, c_o.second}, {sum.first, sum.second}});
		// 7) -c_i  c_o  sum
		if (clauseMode == 0 or (not sum.second and clauseMode == 1) or (sum.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{c_i.first, not c_i.second}, {c_o.first, c_o.second}, {sum.first, sum.second}});
		// 8)   -a   -b -c_i  sum
		if (clauseMode == 0 or (not sum.second and clauseMode == 1) or (sum.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{a.first, not a.second}, {b.first, not b.second}, {c_i.first, not c_i.second}, {sum.first, sum.second}});
		return clause_counter;
	}

	int SATSchedulerBinEncOverlap::create_half_adder_clauses(std::pair<int, bool> a, std::pair<int, bool> b,
																										std::pair<int, bool> sum, std::pair<int, bool> c_o,
																										int clauseMode) {
		// clause mode =  0: create all clauses
		// clause mode =  1: create all clauses leading to true sum/c_o values
		// clause mode = -1: create all clauses leading to false sum/c_o values
		// clause mode =  1 and c_o = -1: sum = a or b (for resource constraints)
		if (clauseMode > 1 or clauseMode < -1) throw Exception("SATSchedulerBinEncOverlap::create_half_adder_clauses: invalid clause mode");
		int clause_counter = 0;
		if (clauseMode != 0 and c_o.first == -1) {
			throw Exception("SATSchedulerBinEncOverlap::create_half_adder_clauses: cannot create half adder in clause mode 1/-1 without carry output");
		}
		if (c_o.first <= 0) {
			// only build sum
			// 1) a b -sum
			if (clauseMode == 0 or (sum.second and clauseMode == 1) or (not sum.second and clauseMode == -1))
				clause_counter += this->create_arbitrary_clause({{a.first, a.second}, {b.first, b.second}, {sum.first, not sum.second}});
			// 2) -a -b -sum
			if (clauseMode == 0 or (sum.second and clauseMode == 1) or (not sum.second and clauseMode == -1))
				clause_counter += this->create_arbitrary_clause({{a.first, not a.second}, {b.first, not b.second}, {sum.first, not sum.second}});
			// 3) a -b sum
			if (clauseMode == 0 or (not sum.second and clauseMode == 1) or (sum.second and clauseMode == -1))
				clause_counter += this->create_arbitrary_clause({{a.first, a.second}, {b.first, not b.second}, {sum.first, sum.second}});
			// 4) -a b sum
			if (clauseMode == 0 or (not sum.second and clauseMode == 1) or (sum.second and clauseMode == -1))
				clause_counter += this->create_arbitrary_clause({{a.first, not a.second}, {b.first, b.second}, {sum.first, sum.second}});
			return clause_counter;
		}
		// build carry and use it to build sum
		// 1) -a -b c_o
		if (clauseMode == 0 or (not c_o.second and clauseMode == 1) or (c_o.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{a.first, not a.second},{b.first, not b.second},{c_o.first, c_o.second}});
		// 2) a -c_o
		if (clauseMode == 0 or (c_o.second and clauseMode == 1) or (not c_o.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{a.first, a.second},{c_o.first, not c_o.second}});
		// 3) b -c_o
		if (clauseMode == 0 or (c_o.second and clauseMode == 1) or (not c_o.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{b.first, b.second},{c_o.first, not c_o.second}});

		// 1) -c_o -sum
		if (clauseMode == 0 or (sum.second and clauseMode == 1) or (not sum.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{c_o.first, not c_o.second}, {sum.first, not sum.second}});
		// 2) -a  c_o  sum
		if (clauseMode == 0 or (not sum.second and clauseMode == 1) or (sum.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{a.first, not a.second}, {c_o.first, c_o.second}, {sum.first, sum.second}});
		// 3) -b  c_o  sum
		if (clauseMode == 0 or (not sum.second and clauseMode == 1) or (sum.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{b.first, not b.second}, {c_o.first, c_o.second}, {sum.first, sum.second}});
		return clause_counter;
	}

	int SATSchedulerBinEncOverlap::create_not(int x, int not_x) {
		// 1) -x -not_x
		this->create_arbitrary_clause({{x, true}, {not_x, true}});
		// 2) x not_x
		this->create_arbitrary_clause({{x, false}, {not_x, false}});
		return 2;
	}

	int SATSchedulerBinEncOverlap::create_2x1_and(int a, int b, int y) {
		// 1) -a -b  y
		this->create_arbitrary_clause({{a, true},{b, true},{y, false}});
		// 2)  a -y
		this->create_arbitrary_clause({{a, false},{y, true}});
		// 3)  b -y
		this->create_arbitrary_clause({{b, false},{y, true}});
		return 3;
	}

	int SATSchedulerBinEncOverlap::create_2x1_or(int a, int b, int y) {
		// 1)  a  b  -y
		this->create_arbitrary_clause({{a, false},{b, false},{y, true}});
		// 2) -a  y
		this->create_arbitrary_clause({{a, true},{y, false}});
		// 3) -b  y
		this->create_arbitrary_clause({{b, true},{y, false}});
		return 3;
	}

	int SATSchedulerBinEncOverlap::create_2x1_mux(int a, int b, int s, int y) {
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

	int SATSchedulerBinEncOverlap::create_nand(int a, int b, int y) {
		// 1)
		this->create_arbitrary_clause({{a, false}, {y, false}});
		// 2)
		this->create_arbitrary_clause({{b, false}, {y, false}});
		// 3)
		this->create_arbitrary_clause({{a, true}, {b, true}, {y, true}});
		return 3;
	}

	void SATSchedulerBinEncOverlap::createIncrementalClauses() {
		for (auto &v : this->g.Vertices()) {
			//std::cout << "#q# creating incremental clauses for '" << v->getName() << "'" << std::endl;
			auto wordSize = this->scheduleTimeWordSize.at(v);
			auto tForbidden = this->latestStartTime.at(v) - this->earliestStartTime.at(v) + 1;

			auto tForbiddenLast = this->lastForbiddenTime.at(v);
			for (auto t = tForbidden; t < tForbiddenLast; t++) {
				std::vector<std::pair<int, bool>> clause;
				for (int w=0; w<wordSize; w++) {
					if ((t >> w) & 1) clause.emplace_back(this->scheduleTimeVariables.at({v, w}), true);
				}
				if (clause.empty()) continue;
				this->create_arbitrary_clause(clause);
			}
			this->lastForbiddenTime[v] = tForbidden;
		}
	}

	bool SATSchedulerBinEncOverlap::inIncrementalMode() {
		return this->scheduleFound and (this->los == LINEAR_JUMP or this->los == REVERSE_LINEAR);
	}
}