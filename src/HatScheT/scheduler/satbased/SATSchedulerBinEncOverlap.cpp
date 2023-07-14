//
// Created by nfiege on 3/6/23.
//

#include "SATSchedulerBinEncOverlap.h"
#include <HatScheT/utility/Utility.h>
#include <HatScheT/utility/Verifier.h>
#include <iomanip>
#include <algorithm>

#ifdef USE_SATSCM
#include <scm_cadical.h>
#include <scm_syrup.h>
#include <scm_z3.h>
#endif

#define DEBUG_MODE 0

namespace HatScheT {

	SATSchedulerBinEncOverlap::SATSchedulerBinEncOverlap(Graph &g, ResourceModel &resourceModel, int II) :
		SATSchedulerBase(g, resourceModel, II) {
		this->los = LatencyOptimizationStrategy::LINEAR_JUMP;
#ifdef USE_SATSCM
		this->satFormulationMode = constMult;
		//this->satFormulationMode = implication; // DEBUG
#else
		this->satFormulationMode = implication;
#endif
	}

	void SATSchedulerBinEncOverlap::scheduleIteration() {
		// call parent's method for some basic stuff
		SATSchedulerBase::scheduleIteration();

		// clear word sizes
		this->scheduleTimeWordSize.clear();
		this->offsetIIWordSize.clear();
		this->offsetIIMultWordSize.clear();

		// clear variable containers
		this->scheduleTimeVariables.clear();
		this->offsetIIVariables.clear();
		this->offsetIIMultVariables.clear();
		this->moduloSlotVariables.clear();
		this->bindingVariables.clear();
		this->moduloOverlapVariables.clear();
		this->bindingOverlapVariables.clear();
		this->adderGraphVariables.clear();
		this->diffVariables.clear();
		this->constOneVar = 0;
		this->constZeroVar = 0;

		// clear forbidden time container
		this->lastForbiddenTime.clear();

		// reset variable counters
		this->timeSlotLiteralCounter = 0;
		this->bindingLiteralCounter = 0;
		this->overlapLiteralCounter = 0;
		this->moduloComputationLiteralCounter = 0;
		this->resourceConstraintLiteralCounter = 0;
		this->moduloSlotLiteralCounter = 0;
		this->dependencyConstraintSubLiteralCounter = 0;
		this->dependencyConstraintCompLiteralCounter = 0;

		// reset clause counters
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

		// compute adder graph for const mult mode
		if (this->satFormulationMode == constMult) {
			this->computeAdderGraph();
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
				std::cout << "  '" << this->moduloComputationLiteralCounter << "' modulo computation literals" << std::endl;
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
		// the const mult mode needs earliest start times to be a multiple of the II
		//if (this->satFormulationMode == constMult) { //DEBUG
			/*for (auto &it : this->earliestStartTime) {
				// we need the floor anyways so just directly divide the integers instead of casting to float
				it.second = this->candidateII * (it.second / this->candidateII);
				//it.second = 0;
				//this->earliestStartTime.at(it.first) = 0;
			}*/
			for (auto &v : this->g.Vertices()) {
				auto *val = &this->earliestStartTime[v];
				*val = this->candidateII * ((*val) / this->candidateII);
			}
		//} //DEBUG
	}

	void SATSchedulerBinEncOverlap::resetContainer() {
		if (this->inIncrementalMode()) return;
		// clear word sizes
		this->scheduleTimeWordSize.clear();
		this->offsetIIWordSize.clear();
		this->offsetIIMultWordSize.clear();
		// reset literal containers
		this->scheduleTimeVariables.clear();
		this->offsetIIVariables.clear();
		this->offsetIIMultVariables.clear();
		this->moduloSlotVariables.clear();
		this->bindingVariables.clear();
		this->moduloOverlapVariables.clear();
		this->bindingOverlapVariables.clear();
		this->adderGraphVariables.clear();
		this->diffVariables.clear();
		this->constOneVar = 0;
		this->constZeroVar = 0;
		// reset literal counter(s)
		this->timeSlotLiteralCounter = 0;
		this->moduloSlotLiteralCounter = 0;
		this->bindingLiteralCounter = 0;
		this->overlapLiteralCounter = 0;
		this->resourceConstraintLiteralCounter = 0;
		this->moduloComputationLiteralCounter = 0;
		this->dependencyConstraintSubLiteralCounter = 0;
		this->dependencyConstraintCompLiteralCounter = 0;
		// reset clause counter(s)
		this->timeSlotConstraintClauseCounter = 0;
		this->resourceConstraintClauseCounter = 0;
		this->overlapClauseCounter = 0;
		this->bindingConstraintClauseCounter = 0;
		this->moduloComputationClauseCounter = 0;
		this->dependencyConstraintSubClauseCounter = 0;
		this->dependencyConstraintCompClauseCounter = 0;
	}

	SATSchedulerBinEncOverlap::trivialDecision SATSchedulerBinEncOverlap::setUpSolver() {
		// reset solver
		if (!this->inIncrementalMode()) {
#ifdef USE_KISSAT
			this->solver = std::unique_ptr<kissatpp::kissatpp>(new kissatpp::kissatpp(static_cast<int>(std::ceil(this->solverTimeout - this->terminator.getElapsedTime()))));
#else
			this->solver = std::unique_ptr<CaDiCaL::Solver>(new CaDiCaL::Solver);
			this->solver->connect_terminator(&this->terminator);
#endif
		}
		// update latest start times based on candidate latency
		trivialDecision td = noTrivialDecisionPossible;
		for (auto &v : this->g.Vertices()) {
			this->latestStartTime[v] = this->candidateLatency - this->latestStartTimeDifferences.at(v);
			auto diff = this->latestStartTime.at(v) - this->earliestStartTime.at(v);
			if (diff < 0) {
				// maxLatencyConstraint is too tight! scheduling for this II is trivially impossible
				td = triviallyUNSAT;
				if (!this->quiet) {
					std::cout << "SATSchedulerBinEncOverlap: maxLatencyConstraint is too tight! scheduling for this II is trivially impossible" << std::endl;
				}
				continue;
			}
			if (this->inIncrementalMode()) continue;
			if (this->satFormulationMode == constMult and !this->resourceModel.getResource(v)->isUnlimited()) continue;
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
			// in const mult mode, we create schedule time variables when setting up clauses for modulo slot computation
			if (this->satFormulationMode == constMult and !this->resourceModel.getResource(v)->isUnlimited()) continue;
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
		// create offset II literals
		if (this->satFormulationMode == constMult) {
			for (auto &v : this->g.Vertices()) {
				auto r = this->resourceModel.getResource(v);
				auto limit = r->getLimit();
				if (limit == UNLIMITED) continue;
				// todo: check if floor also works
				auto maxVal = static_cast<int>(std::ceil(
					static_cast<double>(this->latestStartTime.at(v) - this->earliestStartTime.at(v)) /
					static_cast<double>(this->candidateII)));
				int wordSize;
				if (maxVal > 1) {
					wordSize = static_cast<int>(std::ceil(std::log2(maxVal + 1)));
				} else {
					wordSize = 1;
				}
				this->offsetIIWordSize[v] = wordSize;
				for (int w = 0; w < wordSize; w++) {
					this->offsetIIVariables[{v, w}] = ++this->literalCounter;
					this->moduloComputationLiteralCounter++;
				}
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
		// first of all, create clauses to compute modulo slots for resource-limited vertices
		if (!this->quiet) std::cout << "SATSchedulerBinEncOverlap::createClauses: creating clauses for modulo computation (m_i = t_i mod " << this->candidateII << ")" << std::endl;
		this->createModuloComputationClauses();
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
			//auto maxValue = (1 << wordSize)-1;
			auto upperLimit = this->latestStartTime.at(v) - this->earliestStartTime.at(v);
			this->lastForbiddenTime[v] = upperLimit+1;
			std::vector<std::pair<int, bool>> clauseBase;
			/*std::cout << "#q# time slot limitation clauses for '" << v->getName() << "' with upper limit " << upperLimit << " and word size " << wordSize << std::endl;
			std::cout << "      ->";
			for (int idx = wordSize-1; idx >= 0; idx--) {
				std::cout << " " << this->scheduleTimeVariables.at({v, idx});
			}
			std::cout << std::endl;*/
			for (int idx = wordSize-1; idx >= 0; idx--) {
				if ((upperLimit >> idx) & 1) {
					clauseBase.emplace_back(this->scheduleTimeVariables.at({v, idx}), true);
					continue;
				}
				auto clause = clauseBase;
				clause.emplace_back(this->scheduleTimeVariables.at({v, idx}), true);
				this->timeSlotConstraintClauseCounter += this->create_arbitrary_clause(clause);
			}

			// also limit modulo slots if they exist
			if (this->satFormulationMode != constMult or this->resourceModel.getResource(v)->isUnlimited()) continue;
			upperLimit = this->candidateII-1;
			wordSize = static_cast<int>(std::ceil(std::log2(this->candidateII)));
			clauseBase.clear();
			for (int idx = wordSize-1; idx >= 0; idx--) {
				if ((upperLimit >> idx) & 1) {
					clauseBase.emplace_back(this->moduloSlotVariables.at({v, idx}), true);
					continue;
				}
				auto clause = clauseBase;
				clause.emplace_back(this->moduloSlotVariables.at({v, idx}), true);
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
			//auto edgeConst = -(lSrc + delay - (dist * this->candidateII) + tSrcMin - tDstMin);
			auto edgeConst = (dist * this->candidateII) - lSrc - delay - tSrcMin + tDstMin;
			// enforce diff_ij = t_i - t_j <= edgeConst
			// check if edge is trivially satisfied
			auto worstCaseDiff = tSrcDiff; // t_j = 0 => diff_ij = t_i
			if (worstCaseDiff <= edgeConst) {
				//std::cout << "#q# edge '" << vSrc->getName() << "' -> '" << vDst->getName() << "' is trivially satisfied" << std::endl;
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
				auto polarity = idx != wordSizeDiff-1;
				if (idx == wordSizeDiff-1 and !bitSet) {
					clauseBase.emplace_back(this->diffVariables.at({e, idx}), polarity);
					continue;
				}
				if (idx != wordSizeDiff-1 and bitSet) {
					clauseBase.emplace_back(this->diffVariables.at({e, idx}), polarity);
					continue;
				}
				auto clause = clauseBase;
				clause.emplace_back(this->diffVariables.at({e, idx}), polarity);
				this->dependencyConstraintCompClauseCounter += this->create_arbitrary_clause(clause);
			}
		}
		return td;
	}

	void SATSchedulerBinEncOverlap::createModuloComputationClauses() {
		switch (this->satFormulationMode) {
			case implication: {
				this->createModuloComputationImplicationClauses();
				break;
			}
			case constMult: {
				this->createModuloComputationConstMultClauses();
				break;
			}
			default: {
				throw Exception("SATSchedulerBinEncOverlap: invalid SAT formulation mode");
			}
		}
	}

	void SATSchedulerBinEncOverlap::createModuloComputationImplicationClauses() {
		// compute m_i = t_i mod II for all resource-limited vertices v_i using implication
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

	void SATSchedulerBinEncOverlap::createModuloComputationConstMultClauses() {
		auto moduloSlotWordSize = static_cast<int>(std::ceil(std::log2(this->candidateII)));
		for (auto &v : this->g.Vertices()) {
			auto r = this->resourceModel.getResource(v);
			auto limit = r->getLimit();
			if (limit == UNLIMITED) continue;
			// we represent the II as an odd number multiplied by a power of 2:
			// II = II_odd << II_shift = II_odd * 2^II_shift
			// compute z_i = y_i * II_odd based on the adder graph that we computed earlier
			std::vector<int> z_i;
			std::map<int, std::vector<int>> &adderGraphVars = this->adderGraphVariables[v];
			int yWordSize = this->offsetIIWordSize.at(v);
			for (auto w=0; w<yWordSize; w++) {
				adderGraphVars[1].emplace_back(this->offsetIIVariables.at({v, w}));
			}
			if (this->scmAdderGraphVertexOrder.empty()) {
				z_i = adderGraphVars[1];
			}
			else {
				for (auto &c : this->scmAdderGraphVertexOrder) {
					auto nodeValue = c->getId();
					auto incomingEdges = this->scmAdderGraph->getIncomingEdges(c);
					if (incomingEdges.size() != 2) {
						throw Exception("SATSchedulerBinEncOverlap: invalid adder graph computed (incoming edges != 2)");
					}
					auto firstInputEdge = *incomingEdges.begin();
					auto secondInputEdge = *incomingEdges.rbegin();
					int leftInputValue;
					int rightInputValue;
					int leftShift;
					int rightShift;
					bool sub = c->getName() == "-";
					if (firstInputEdge->getDelay() == 0) {
						// first input is left input
						// second input is right input
						leftInputValue = firstInputEdge->getVertexSrc().getId();
						leftShift = firstInputEdge->getDistance();
						rightInputValue = secondInputEdge->getVertexSrc().getId();
						rightShift = secondInputEdge->getDistance();
					}
					else {
						// second input is left input
						// first input is right input
						leftInputValue = secondInputEdge->getVertexSrc().getId();
						leftShift = secondInputEdge->getDistance();
						rightInputValue = firstInputEdge->getVertexSrc().getId();
						rightShift = firstInputEdge->getDistance();
					}
					auto leftInputVars = adderGraphVars.at(leftInputValue);
					auto rightInputVars = adderGraphVars.at(rightInputValue);
					auto wLeft = leftInputVars.size() + leftShift;
					auto wRight = rightInputVars.size() + rightShift;
					std::vector<std::tuple<std::vector<int>, bool, int>> bitheapInput = {
						{leftInputVars, false, leftShift},
						{rightInputVars, sub, rightShift}
					};
#if DEBUG_MODE
					std::cout << "creating bitheap for '" << v->getName() << "': " << nodeValue << " = (" << leftInputValue << " << " << leftShift << ") " << (sub?"-":"+") << " (" << rightInputValue << " << " << rightShift << ")" << std::endl;
					std::cout << "  -> left input vars:";
					for (int i=leftInputVars.size()-1; i>=0; i--) {
						std::cout << " " << leftInputVars.at(i);
					}
					std::cout << std::endl;
					std::cout << "  -> right input vars:";
					for (int i=rightInputVars.size()-1; i>=0; i--) {
						std::cout << " " << rightInputVars.at(i)*(sub?-1:1);
					}
					std::cout << std::endl;
#endif
					auto bitheapOutput = this->create_unsigned_result_bitheap(bitheapInput,
																																		&this->moduloComputationLiteralCounter,
																																		&this->moduloComputationClauseCounter);
#if DEBUG_MODE
					std::cout << "  -> output vars:";
					for (int i=bitheapOutput.size()-1; i>=0; i--) {
						std::cout << " " << bitheapOutput.at(i);
					}
					std::cout << std::endl << std::endl;
#endif
					auto constMultMax = ((1 << yWordSize)-1) * nodeValue;
					auto wConstMult = static_cast<int>(std::ceil(std::log2(constMultMax+1)));
					if (wConstMult > bitheapOutput.size()) {
						// word size is not enough -> something went wrong :(
						throw Exception("SATSchedulerBinEncOverlap: invalid bitheap compression (output word size too small)");
					}
					bitheapOutput.resize(wConstMult);
					adderGraphVars[nodeValue] = bitheapOutput;
				}
				z_i = adderGraphVars.at(this->scmOutputConst);
			}
			this->offsetIIMultWordSize[v] = static_cast<int>(z_i.size());
			for (auto w=0; w<z_i.size(); w++) {
				this->offsetIIMultVariables[{v, w}] = z_i.at(w);
			}
			// compute t_i = (z_i << II_shift) + m_i
			std::vector<int> m_i(moduloSlotWordSize);
			for (auto w=0; w<moduloSlotWordSize; w++) {
				m_i[w] = this->moduloSlotVariables.at({v, w});
			}
			std::vector<std::tuple<std::vector<int>, bool, int>> bitheapInput = {
				{z_i, false, this->scmOutputShift}, // z_i << II_shift
				{m_i, false, 0}  // m_i
			};
			auto tMinVal = this->earliestStartTime.at(v);
			auto tMinValWordSize = static_cast<int>(std::ceil(std::log2(tMinVal+1)));
			if (tMinVal % this->candidateII != 0) {
				throw Exception("Earliest start time must always be a multiple of the II for const mult mode");
				/*std::vector<int> tMin_i(tMinValWordSize);
				for (auto w=0; w<tMinValWordSize; w++) {
					auto bitSet = static_cast<bool>((tMinVal >> w) & 1);
					if (bitSet) {
						tMin_i[w] = this->constOneVar;
					}
					else {
						tMin_i[w] = this->constZeroVar;
					}
				}
				bitheapInput.emplace_back(tMin_i, false, 0);*/
			}
			auto bitheapOutput = this->create_unsigned_result_bitheap(bitheapInput, &this->moduloComputationLiteralCounter,
																																&this->moduloComputationClauseCounter);
#if DEBUG_MODE
			std::cout << "#q# sched time sum for '" << v->getName() << "':" << std::endl;
			std::cout << "      -> z_i << " << this->scmOutputShift << " with " << z_i.size() << " bits:";
			for (int w=(int)z_i.size()-1; w>=0; w--) {
				auto it = z_i.at(w);
				std::cout << " " << it;
			}
			std::cout << std::endl;
			std::cout << "      -> m_i with " << m_i.size() << " bits:";
			for (int w=(int)m_i.size()-1; w>=0; w--) {
				auto it = m_i.at(w);
				std::cout << " " << it;
			}
			std::cout << std::endl;
			std::cout << "      -> t_i with " << bitheapOutput.size() << " bits:";
			for (int w=(int)bitheapOutput.size()-1; w>=0; w--) {
				auto it = bitheapOutput.at(w);
				std::cout << " " << it;
			}
			std::cout << std::endl << std::endl;
#endif
			this->scheduleTimeWordSize[v] = bitheapOutput.size();
			for (auto w=0; w<bitheapOutput.size(); w++) {
				this->scheduleTimeVariables[{v, w}] = bitheapOutput.at(w);
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
		bool foundError = false;
		for (auto &v : this->g.Vertices()) {
			int t = 0;
			auto wordSize = this->scheduleTimeWordSize.at(v);
			for (int w = 0; w < wordSize; w++) {
				auto bitResult =
#ifdef USE_KISSAT
					this->solver->value(this->scheduleTimeVariables.at({v, w}))
#else
					this->solver->val(this->scheduleTimeVariables.at({v, w}))
#endif
					> 0 ? 1 : 0;
				t += (bitResult << w);
			}
			this->startTimes[v] = t + this->earliestStartTime.at(v);
			// DEBUG START
			if (this->satFormulationMode != constMult or this->resourceModel.getResource(v)->isUnlimited()) continue;
			int moduloSlot = 0;
			int offsetII = 0;
			int offsetIIMult = 0;
			wordSize = static_cast<int>(std::ceil(std::log2(this->candidateII)));
			for (int w = 0; w < wordSize; w++) {
				auto bitResult =
#ifdef USE_KISSAT
					this->solver->value(this->moduloSlotVariables.at({v, w}))
#else
					this->solver->val(this->moduloSlotVariables.at({v, w}))
#endif
					> 0 ? 1 : 0;
				moduloSlot += (bitResult << w);
			}
			wordSize = this->offsetIIWordSize.at(v);
			for (int w = 0; w < wordSize; w++) {
				auto bitResult =
#ifdef USE_KISSAT
					this->solver->value(this->offsetIIVariables.at({v, w}))
#else
					this->solver->val(this->offsetIIVariables.at({v, w}))
#endif
					> 0 ? 1 : 0;
				offsetII += (bitResult << w);
			}
			wordSize = this->offsetIIMultWordSize.at(v);
			for (int w = 0; w < wordSize; w++) {
				auto bitResult =
#ifdef USE_KISSAT
					this->solver->value(this->offsetIIMultVariables.at({v, w}))
#else
					this->solver->val(this->offsetIIMultVariables.at({v, w}))
#endif
					> 0 ? 1 : 0;
				offsetIIMult += (bitResult << w);
			}
			if (t % this->candidateII != moduloSlot
			 or t != moduloSlot + (this->candidateII * offsetII)
			 or offsetIIMult != this->scmOutputConst * offsetII) {
				std::cout << "Vertex '" << v->getName() << "'" << std::endl;
				std::cout << "  start time = " << this->startTimes.at(v) << std::endl;
				std::cout << "  t_i = " << t << " :";
				for (int w=this->scheduleTimeWordSize.at(v)-1; w>=0; w--) {
					std::cout << " " << this->scheduleTimeVariables.at({v, w});
				}
				std::cout << std::endl;
				std::cout << "  m_i = " << moduloSlot << " :";
				for (int w=static_cast<int>(std::ceil(std::log2(this->candidateII)))-1; w>=0; w--) {
					std::cout << " " << this->moduloSlotVariables.at({v, w});
				}
				std::cout << std::endl;
				std::cout << "  y_i = " << offsetII << " :";
				for (int w=this->offsetIIWordSize.at(v)-1; w>=0; w--) {
					std::cout << " " << this->offsetIIVariables.at({v, w});
				}
				std::cout << std::endl;
				std::cout << "  yMult_i = " << offsetIIMult << " :";
				for (int w=this->offsetIIMultWordSize.at(v)-1; w>=0; w--) {
					std::cout << " " << this->offsetIIMultVariables.at({v, w});
				}
				std::cout << std::endl;
				std::cout << "  t_min = " << this->earliestStartTime.at(v) << std::endl;
				foundError = true;
			}
			// adder graph
			auto &adderGraphVars = this->adderGraphVariables.at(v);
			for (auto &it : adderGraphVars) {
				int val = 0;
				int w = 0;
				for (auto &var : it.second) {
					auto bitResult =
#ifdef USE_KISSAT
						this->solver->value(var)
#else
						this->solver->val(var)
#endif
						> 0 ? 1 : 0;
					val += (bitResult << w);
					w++;
				}
				if (val != (offsetII * it.first)) {
					std::cout << "  y_i * " << it.first << " = " << val << std::endl;
					foundError = true;
				}
			}
			// DEBUG END
		}
		if (foundError) {
			throw Exception("Wrong modulo slot computation");
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
		std::stringstream ss;
		ss << "  -> clause";
		for (auto &it : a) {
			if (it.second) {
				// negated literal
				this->solver->add(-it.first);
				ss << " " << -it.first;
			}
			else {
				// non-negated literal
				this->solver->add(it.first);
				ss << " " << it.first;
			}
		}
		this->solver->add(0);
		if (!this->quiet) std::cout << ss.str() << std::endl;
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
		// 1) -a -b c_o
		if (clauseMode == 0 or (not c_o.second and clauseMode == 1) or (c_o.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{a.first, not a.second}, {b.first, not b.second}, {c_o.first, c_o.second}});
		// 2) -b -c_i c_o
		if (clauseMode == 0 or (not c_o.second and clauseMode == 1) or (c_o.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{b.first, not b.second}, {c_i.first, not c_i.second}, {c_o.first, c_o.second}});
		// 3) -a -c_i c_o
		if (clauseMode == 0 or (not c_o.second and clauseMode == 1) or (c_o.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{a.first, not a.second}, {c_i.first, not c_i.second}, {c_o.first, c_o.second}});
		// 4)  a  c_i -c_o
		if (clauseMode == 0 or (c_o.second and clauseMode == 1) or (not c_o.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{a.first, a.second}, {c_i.first, c_i.second}, {c_o.first, not c_o.second}});
		// 5)  b  c_i -c_o
		if (clauseMode == 0 or (c_o.second and clauseMode == 1) or (not c_o.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{b.first, b.second}, {c_i.first, c_i.second}, {c_o.first, not c_o.second}});
		// 6)  a  b -c_o
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
		// 2)  a  b -sum
		if (clauseMode == 0 or (sum.second and clauseMode == 1) or (not sum.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{a.first, a.second}, {b.first, b.second}, {sum.first, not sum.second}});
		// 3) -a  c_o  sum
		if (clauseMode == 0 or (not sum.second and clauseMode == 1) or (sum.second and clauseMode == -1))
			clause_counter += this->create_arbitrary_clause({{a.first, not a.second}, {c_o.first, c_o.second}, {sum.first, sum.second}});
		// 4) -b  c_o  sum
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

	void SATSchedulerBinEncOverlap::computeAdderGraph() {
#ifdef USE_SATSCM
		// init adder graph
		this->scmAdderGraph = std::unique_ptr<Graph>(new Graph);
		this->scmAdderGraphVertexOrder.clear();
		auto &adderGraphInputNode = this->scmAdderGraph->createVertex(1);
		adderGraphInputNode.setName("src");
		// solve scm problem
		this->scmOutputShift = 0;
		this->scmOutputConst = this->candidateII;
		if (this->scmOutputConst < 1) {
			// oh well, something went terribly wrong here :/
			throw Exception("SATSchedulerBinEncOverlap: invalid II requested (II="+std::to_string(this->scmOutputConst)+")");
		}
		while ((this->scmOutputConst & 1) == 0) {
			// it's still odd
			this->scmOutputShift++;
			this->scmOutputConst = this->scmOutputConst >> 1;
		}
		// no negative numbers allowed (it's easier and negative numbers should not be necessary here)
		// don't write cnf (also not needed)
		scm_cadical scm
									(
									{this->scmOutputConst},
									static_cast<int>(std::round(this->solverTimeout-this->terminator.getElapsedTime())),
									(this->quiet?scm::verbosity_mode::quiet_mode:scm::verbosity_mode::normal_mode),
									false,
									false
									);
		// minimize the number of bit computations we need to do in SAT
		scm.also_minimize_full_adders();
		// solve it
		scm.solve();
		// get result and build graph from that
		auto adderGraph = scm.get_adder_graph_description();
		if (adderGraph.size() < 2) {
			throw Exception("SATSchedulerBinEncOverlap: got invalid adder graph for C="+std::to_string(this->scmOutputConst)+" (wrong size)");
		}
		// build hatschet graph from adder graph string representation
		// remove all '{', '}', '[' and ']'
		adderGraph.erase(std::remove(adderGraph.begin(), adderGraph.end(), '{'), adderGraph.end());
		adderGraph.erase(std::remove(adderGraph.begin(), adderGraph.end(), '}'), adderGraph.end());
		adderGraph.erase(std::remove(adderGraph.begin(), adderGraph.end(), '['), adderGraph.end());
		adderGraph.erase(std::remove(adderGraph.begin(), adderGraph.end(), ']'), adderGraph.end());
		if (adderGraph.empty()) {
			return;
		}
		std::stringstream adderGraphStr(adderGraph);
		std::string buffer;
		int nodeValue = -1;
		int srcLeft = -1;
		int srcRight = -1;
		int shiftLeft = -1;
		int shiftRight = -1;
		bool sub = false;
		int cnt = 0;
		while(std::getline(adderGraphStr, buffer, ',')) {
			if (buffer == "'A'") {
				nodeValue = -1;
				srcLeft = -1;
				srcRight = -1;
				shiftLeft = -1;
				shiftRight = -1;
				sub = false;
				cnt = 0;
				continue;
			}
			switch (cnt) {
				case 0: {
					nodeValue = std::stoi(buffer);
				}
				case 1: {
					// stage is irrelevant
					break;
				}
				case 2: {
					srcLeft = std::stoi(buffer);
					if (srcLeft < 0) {
						throw Exception("SATSchedulerBinEncOverlap: got invalid adder graph for C="+std::to_string(this->scmOutputConst)+" (wrong subtraction)");
					}
					break;
				}
				case 3: {
					// stage is irrelevant
					break;
				}
				case 4: {
					shiftLeft = std::stoi(buffer);
					if (shiftLeft < 0) {
						throw Exception("SATSchedulerBinEncOverlap: got invalid adder graph for C="+std::to_string(this->scmOutputConst)+" (wrong shift)");
					}
					break;
				}
				case 5: {
					srcRight = std::stoi(buffer);
					if (srcRight < 0) {
						srcRight = -srcRight;
						sub = true;
					}
					break;
				}
				case 6: {
					// stage is irrelevant
					break;
				}
				case 7: {
					shiftRight = std::stoi(buffer);
					if (shiftRight < 0) {
						throw Exception("SATSchedulerBinEncOverlap: got invalid adder graph for C="+std::to_string(this->scmOutputConst)+" (wrong shift)");
					}
					// create vertex
					auto &vertex = this->scmAdderGraph->createVertex(nodeValue);
					if (sub) {
						vertex.setName("-");
					}
					else {
						vertex.setName("+");
					}
					this->scmAdderGraphVertexOrder.emplace_back(&vertex);
					// create left src edge
					// distance represents shift length
					// set delay = 0 to represent left input
					auto &lSrc = this->scmAdderGraph->getVertexById(srcLeft);
					auto &edgeLeft = this->scmAdderGraph->createEdge(lSrc, vertex, shiftLeft, Edge::Data);
					edgeLeft.setDelay(0);
					// create right src edge
					// distance represents shift length
					// set delay = 1 to represent right input
					auto &rSrc = this->scmAdderGraph->getVertexById(srcRight);
					auto &edgeRight = this->scmAdderGraph->createEdge(rSrc, vertex, shiftRight, Edge::Data);
					edgeRight.setDelay(1);
					break;
				}
				default: {
					throw Exception("SATSchedulerBinEncOverlap: got invalid adder graph for C="+std::to_string(this->scmOutputConst)+" (wrong format)");
				}
			}
			cnt++;
		}
#else
		throw Exception("SATSchedulerBinEncOverlap: Link SAT SCM library to use constMult mode");
#endif
	}

	std::vector<int> SATSchedulerBinEncOverlap::create_unsigned_result_bitheap(const std::vector<std::tuple<std::vector<int>, bool, int>> &x, int* literalCounterPtr, int* clauseCounterPtr) {
		std::vector<int> result_variables;
		std::map<int, std::vector<std::pair<int, bool>>> y;
		int num_bits = 0;
		for (auto &it : x) {
			auto &bits = std::get<0>(it);
			auto &shift = std::get<2>(it);
			num_bits = std::max(num_bits, static_cast<int>(bits.size()+shift));
		}
		for (auto &it : x) {
			auto &bits = std::get<0>(it);
			auto sub = std::get<1>(it);
			auto shift = std::get<2>(it);
#if DEBUG_MODE
			std::cout << "bitheap input: w=" << bits.size() << " shift=" << shift << " and op=" << (sub?"sub:":"add:");
			for (int i=bits.size()-1; i>=0; i--) {
				auto var = bits.at(i);
				std::cout << " " << var;
			}
			std::cout << std::endl;
#endif
			//if (bits.size()+shift > num_bits) num_bits = static_cast<int>(bits.size())+shift;
			if (sub) {
				// add 1 for 2k inversion
				y[shift].emplace_back(this->constOneVar, false);
				// add inverted bits
				for (int bit_pos=0; bit_pos<bits.size(); bit_pos++) {
					y[bit_pos+shift].emplace_back(bits[bit_pos], true);
				}
				// sign extension
				for (int bit_pos=static_cast<int>(bits.size()+shift); bit_pos < num_bits; bit_pos++) {
					y[bit_pos].emplace_back(this->constOneVar, false);
				}
			}
			else {
				// add bits
				for (int bit_pos=0; bit_pos<bits.size(); bit_pos++) {
					y[bit_pos+shift].emplace_back(bits[bit_pos], false);
				}
			}
		}
#if DEBUG_MODE
		for (auto &it : y) {
			std::cout << "y[" << it.first << "] has " << it.second.size() << " bits:";
			for (int i=it.second.size()-1; i>=0; i--) {
				auto var = it.second.at(i);
				std::cout << " " << (var.second?-var.first:var.first);
			}
			std::cout << std::endl;
		}
#endif
		int i = 0;
		while (i < num_bits) {
			while (y[i].size() > 1) {
				if (y[i].size() == 2) {
					// half adder
					// create new literals for sum and carry
					auto sum = ++this->literalCounter;
					auto carry = ++this->literalCounter;
					if (literalCounterPtr != nullptr) {
						*literalCounterPtr += 2;
					}
					// get bits to add from container
					auto a = y[i].back();
					y[i].pop_back();
					auto b = y[i].back();
					y[i].pop_back();
					// create clauses
#if DEBUG_MODE
					std::cout << "i=" << i << ": half adder with a=" << (a.second?-a.first:a.first) << ", b=" << (b.second?-b.first:b.first) << " sum=" << sum << ", carry=" << carry << std::endl;
#endif
					auto numClauses = this->create_half_adder_clauses(a, b, {sum, false}, {carry, false});
					if (clauseCounterPtr != nullptr) {
						*clauseCounterPtr += numClauses;
					}
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
					if (literalCounterPtr != nullptr) {
						*literalCounterPtr += 2;
					}
					// get bits to add from container
					auto a = y[i].back();
					y[i].pop_back();
					auto b = y[i].back();
					y[i].pop_back();
					auto c = y[i].back();
					y[i].pop_back();
					// create clauses
#if DEBUG_MODE
					std::cout << "i=" << i << ": full adder with a=" << (a.second?-a.first:a.first) << ", b=" << (b.second?-b.first:b.first) << ", c=" << (c.second?-c.first:c.first) << " sum=" << sum << ", carry=" << carry << std::endl;
#endif
					auto numClauses = this->create_full_adder_clauses(a, b, c, {sum, false}, {carry, false});
					if (clauseCounterPtr != nullptr) {
						*clauseCounterPtr += numClauses;
					}
					y[i].emplace_back(sum, false);
					y[i+1].emplace_back(carry, false);
					if (i+2 > num_bits) num_bits = i+2;
				}
			}
			if (y[i].empty()) {
				// add constant zero to empty positions
				y[i].emplace_back(this->constZeroVar, false);
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