//
// Created by bkessler on 18/10/19.
//

//UNDER DEVELOPEMENT

#ifdef USE_CADICAL
#include "SDSScheduler.h"
#include "HatScheT/scheduler/ASAPScheduler.h"
#include "HatScheT/scheduler/ULScheduler.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/Verifier.h"
#include "HatScheT/utility/SDCSolver.h"
#include <cassert>
#include <iomanip>
#include <climits>
#include <cmath>
#include <chrono>


#if 1
namespace HatScheT {
	SDSScheduler::SDSScheduler(Graph &g, ResourceModel &resourceModel, int II) :
		SATSchedulerBase(g, resourceModel, II) {
		this->los = LatencyOptimizationStrategy::LOGARITHMIC; // binary search as stated in the paper
	}

	void SDSScheduler::scheduleIteration() {
		// set up the scheduler
		SATSchedulerBase::scheduleIteration();
		if (!this->quiet) std::cout << "SDSScheduler::scheduleIteration: start for II = " << this->candidateII << std::endl;
		this->computeLatencyBounds();
		if (!this->quiet) std::cout << "SDSScheduler::scheduleIteration: computed SL <= " << this->maxLatency << std::endl;
		this->initSATSolver();
		if (!this->quiet) std::cout << "SDSScheduler::scheduleIteration: successfully initialized SAT solver" << std::endl;
		std::list<std::tuple<const Vertex*, const Vertex*, int, int>> additionalSDCConstraints;
		this->initSDCSolver();
		if (!this->quiet) std::cout << "SDSScheduler::scheduleIteration: successfully initialized SDC solver" << std::endl;
		int tries = 0;
		int maxTries = INT_MAX;
		double timeSATSolver = 0.0;
		double timeSDCSolver = 0.0;
		double currentTime;
		while (!this->scheduleFound) {
			tries++;
			if(tries % 100 == 99) {
				std::cout << "#q# iteration #" << tries + 1 << std::endl;
				std::cout << "    -> total time: " << this->terminator.getElapsedTime() << "sec" << std::endl;
				std::cout << "    -> SAT solver time: " << timeSATSolver << "sec" << std::endl;
				std::cout << "    -> SDC solver time: " << timeSDCSolver << "sec" << std::endl;
			}
			if (tries > maxTries) {
				std::cout << "#q# ITERATION LIMIT (" << maxTries << ") REACHED!" << std::endl;
				return;
			}
			if (!this->quiet) std::cout << "SDSScheduler::scheduleIteration: start new iteration" << std::endl;
			// solve SDC
			if (!this->quiet) std::cout << "SDSScheduler::scheduleIteration:     start SDC solving" << std::endl;
			currentTime = this->terminator.getElapsedTime();
			auto conflictingResourceConstraints = this->solveSDC(additionalSDCConstraints);
			timeSDCSolver += this->terminator.getElapsedTime() - currentTime;
			if (conflictingResourceConstraints.empty()) {
				if (!this->quiet) std::cout << "SDSScheduler::scheduleIteration:     did not encounter resource constraint conflicts in SDC solver" << std::endl;
				// SDC solver found solution
				// check if SDC solution meets resource constraints
				this->scheduleFound = verifyModuloSchedule(this->g, this->resourceModel, this->startTimes, this->candidateII, this->quiet);
				if (not this->scheduleFound) {
					if (!this->quiet) std::cout << "SDSScheduler::scheduleIteration:     computed schedule is invalid" << std::endl;
					// if not: start SAT solver to get additional SDC constraints
					currentTime = this->terminator.getElapsedTime();
					auto additionalSDCConstraintsPair = this->getAdditionalSDCConstraints();
					timeSATSolver += this->terminator.getElapsedTime() - currentTime;
					if (!additionalSDCConstraintsPair.first) {
						// SAT search space is exhausted or we ran into a timeout
						if (!this->quiet) std::cout << "SDSScheduler::scheduleIteration:     SAT solver failed to find solution (search space exhausted or timeout)" << std::endl;
						return;
					}
					additionalSDCConstraints = additionalSDCConstraintsPair.second;
				}
			}
			else {
				if (!this->quiet) std::cout << "SDSScheduler::scheduleIteration:     encountered resource constraint conflicts" << std::endl;
				// SDC solver found invalid SAT modulo ordering proposal
				// forbid proposal for conflicting edges and compute new additional SDC constraints
				this->forbidConflictingResourceConstraints(conflictingResourceConstraints);
				if (!this->quiet) std::cout << "SDSScheduler::scheduleIteration:     start computing new SDC constraints" << std::endl;
				currentTime = this->terminator.getElapsedTime();
				auto additionalSDCConstraintsPair = this->getAdditionalSDCConstraints();
				timeSATSolver += this->terminator.getElapsedTime() - currentTime;
				if (!additionalSDCConstraintsPair.first) {
					// SAT search space is exhausted
					if (!this->quiet) std::cout << "SDSScheduler::scheduleIteration:     SAT solver failed to find solution (search space exhausted or timeout)" << std::endl;
					return;
				}
				if (!this->quiet) std::cout << "SDSScheduler::scheduleIteration:     successfully computed new set of SDC constraints" << std::endl;
				additionalSDCConstraints = additionalSDCConstraintsPair.second;
			}
		}
#if USE_BELLMAN_FORD
		// we already have the shortest path solution
#else
		// incremental SDC solver only produces a feasible solution but not the shortest path solution
		// => run bellman ford to get shortest path solution
		this->computeShortestPathSolution();
#endif
		if (!this->quiet) std::cout << "SDSScheduler::scheduleIteration: finished for II = " << this->candidateII << std::endl;
	}

	void SDSScheduler::computeLatencyBounds() {
		// max latency = latency of heuristic non-modulo schedule
		//ASAPScheduler scheduler(this->g, this->resourceModel);
		ULScheduler scheduler(this->g, this->resourceModel);
		scheduler.schedule();
		this->latencyUpperBound = scheduler.getScheduleLength();
		/*
		if (this->maxLatencyUserDef) {
			this->latencyUpperBound = std::min(this->maxLatency, this->latencyUpperBound);
		}
		 */
		if (this->maxLatencyConstraint > 0) {
			this->latencyUpperBound = std::min(this->maxLatencyConstraint, this->latencyUpperBound);
		}
		this->latencyLowerBound = this->minLatency = 0;
		this->computeNewLatencySuccess(false);
	}

	void SDSScheduler::initSATSolver() {
		// init solver
		this->terminator = CaDiCaLTerminator(this->solverTimeout);
#ifdef USE_KISSAT
		this->solver = std::unique_ptr<kissatpp::kissatpp>(new kissatpp::kissatpp(this->solverTimeout));
#else
		this->solver = std::unique_ptr<CaDiCaL::Solver>(new CaDiCaL::Solver);
		this->solver->connect_terminator(&this->terminator);
#endif
		// clear containers
		this->learnedClauses.clear();
		this->B_ir.clear();
		this->R_ij.clear();
		this->O_ij.clear();
#if USE_OIJK
		this->O_ijk.clear();
#endif
		// create variables
#if USE_OIJK
		//this->kMax = (int)std::ceil(((double)this->latencyUpperBound) / ((double)this->candidateII));
		//this->kMin = 0;
		this->kMax = 0;
		this->kMin = 0;
		if (!this->quiet) std::cout << "kMin=" << this->kMin << " and kMax=" << this->kMax << std::endl;
#endif
		for (auto &i : this->g.Vertices()) {
			auto res = this->resourceModel.getResource(i);
			auto rLim = res->getLimit();
			if (rLim == UNLIMITED) continue;
			// B_ir
			for (int r=0; r<rLim; r++) {
				this->B_ir[{i, r}] = ++this->literalCounter;
			}
			for (auto &j : this->g.Vertices()) {
				if (i == j) continue;
				if (this->resourceModel.getResource(j) != res) continue;
				// R_ij
				this->R_ij[{i, j}] = ++this->literalCounter;
				// O_ij
				this->O_ij[{i, j}] = ++this->literalCounter;
#if USE_OIJK
				// O_ijk
				for (int k=this->kMin; k<=this->kMax; k++) {
					this->O_ijk[{i, j}][k] = ++this->literalCounter;
				}
#endif
			}
		}
		// create base clauses
		for (auto &i : this->g.Vertices()) {
			auto res = this->resourceModel.getResource(i);
			auto rLim = res->getLimit();
			if (rLim == UNLIMITED) continue;

			// Exactly 1 binding is set per vertex (paragraph over Eq. (4) in the paper)
			for (int r=0; r<rLim; r++) {
				this->solver->add(this->B_ir.at({i, r}));
			}
			this->solver->add(0);
			for (int r1=0; r1<rLim; r1++) {
				for (int r2=0; r2<rLim; r2++) {
					if (r1 == r2) continue;
					this->solver->add(-this->B_ir.at({i, r1}));
					this->solver->add(-this->B_ir.at({i, r2}));
					this->solver->add(0);
				}
			}

			for (auto &j : this->g.Vertices()) {
				if (i == j) continue;
				if (this->resourceModel.getResource(j) != res) continue;

				// Eq. (4) from the paper
				for (int r=0; r<rLim; r++) {
					this->solver->add(-this->B_ir.at({i, r}));
					this->solver->add(-this->B_ir.at({j, r}));
					this->solver->add(this->R_ij.at({i, j}));
					this->solver->add(0);
				}

#if USE_OIJK
				// Eq. (7) from the paper
				// part 1: sum over all O_ijk <= 1
				for (int k1=this->kMin; k1<=this->kMax; k1++) {
					for (int k2=this->kMin; k2<=this->kMax; k2++) {
						if (k1 == k2) continue;
						this->solver->add(-this->O_ijk.at({i, j}).at(k1));
						this->solver->add(-this->O_ijk.at({i, j}).at(k2));
						this->solver->add(0);
					}
				}
				// part 2: O_ijk implies O_ij
				for (int k=this->kMin; k<=this->kMax; k++) {
					this->solver->add(-this->O_ijk.at({i, j}).at(k));
					this->solver->add(this->O_ij.at({i, j}));
					this->solver->add(0);
				}
				// part 3: O_ij implies that at least one O_ijk is true
				this->solver->add(-this->O_ij.at({i, j}));
				for (int k=this->kMin; k<=this->kMax; k++) {
					this->solver->add(this->O_ijk.at({i, j}).at(k));
				}
				this->solver->add(0);
#endif

				// Eq. (8) from the paper
				this->solver->add(-this->R_ij.at({i, j}));
				this->solver->add(this->O_ij.at({i, j}));
				this->solver->add(this->O_ij.at({j, i}));
				this->solver->add(0);

				// Eq. (9) from the paper
				this->solver->add(-this->O_ij.at({i, j}));
				this->solver->add(-this->O_ij.at({j, i}));
				this->solver->add(0);
			}
		}
	}

	std::pair<bool, std::list<std::tuple<const Vertex *, const Vertex *, int, int>>> SDSScheduler::getAdditionalSDCConstraints() {
		auto status = this->solver->solve();
		if (!this->quiet) std::cout << "    SDSScheduler::getAdditionalSDCConstraints: SAT solver returned with status '" << status << "'" << std::endl;
		if (status != CADICAL_SAT) {
			return {false, {}};
		}
		// PRINT SOLUTION START (DEBUG)
		/*
		for (auto &it : this->B_ir) {
			std::cout << "B_" << it.first.first->getName() << "_" << it.first.second << " = " << this->solver->val(it.second) << std::endl;
		}
		for (auto &it : this->R_ij) {
			std::cout << "R_" << it.first.first->getName() << "_" << it.first.second->getName() << " = " << this->solver->val(it.second) << std::endl;
		}
		for (auto &it : this->O_ij) {
			std::cout << "O_" << it.first.first->getName() << "_" << it.first.second->getName() << " = " << this->solver->val(it.second) << std::endl;
		}
		for (auto &it : this->O_ijk) {
			for (auto &it2 : it.second) {
				std::cout << "O_" << it.first.first->getId() << "_" << it.first.second->getId() << "_" << it2.first << " = " << (this->solver->val(it2.second)>0) << std::endl;
			}
		}
		 */
		// PRINT SOLUTION END (DEBUG)
		std::pair<bool, std::list<std::tuple<const Vertex *, const Vertex *, int, int>>> returnMe;
		returnMe.first = true;

#if USE_OIJK
		for (auto &it : this->O_ijk) {
			auto i = it.first.first;
			auto j = it.first.second;
			for (auto &it2 : it.second) {
				auto k = it2.first;
				auto satVar = it2.second;
				if (
#ifdef USE_KISSAT
					this->solver->value(satVar)
#else
					this->solver->val(satVar)
#endif
					> 0) {
					// add additional SDC constraints
					auto rhs1 = k*this->candidateII-1;
					//std::cout << "#q# O_" << i->getId() << "_" << j->getId() << "_" << k << std::endl;
					returnMe.second.emplace_back(i, j, rhs1, satVar);
					//std::cout << "     => resource constraint t_" << i->getId() << " - t_" << j->getId() << " <= " << rhs1 << " = " << k << "*" << this->candidateII << "-1" << std::endl;
					auto rhs2 = (-(k-1)*this->candidateII)-1;
					returnMe.second.emplace_back(j, i, rhs2, satVar);
					//std::cout << "     => resource constraint t_" << j->getId() << " - t_" << i->getId() << " <= " << rhs2 << " = (-(" << k << "-1)*" << this->candidateII << ")-1" << std::endl;
				}
			}
		}
#else
		for (auto &it : this->O_ij) {
			auto i = it.first.first;
			auto j = it.first.second;
			auto satVar = it.second;
			if (this->solver->val(satVar) > 0) {
				// add additional SDC constraints
				auto rhs1 = this->candidateII-1;
				//std::cout << "#q# O_" << i->getId() << "_" << j->getId() << "_" << k << std::endl;
				returnMe.second.emplace_back(i, j, rhs1, satVar);
				//std::cout << "     => resource constraint t_" << i->getId() << " - t_" << j->getId() << " <= " << rhs1 << " = " << k << "*" << this->candidateII << "-1" << std::endl;
				auto rhs2 = -1;
				returnMe.second.emplace_back(j, i, rhs2, satVar);
				//std::cout << "     => resource constraint t_" << j->getId() << " - t_" << i->getId() << " <= " << rhs2 << " = (-(" << k << "-1)*" << this->candidateII << ")-1" << std::endl;
			}
		}
#endif
		return returnMe;
	}

	void SDSScheduler::initSDCSolver() {
#if USE_BELLMAN_FORD
		this->sdcSolver = SDCSolverBellmanFord();
		this->sdcSolver.setQuiet(true);
#else
		this->sdcSolver = SDCSolverIncremental();
		this->sdcSolver.setQuiet(this->quiet);
#endif
		for (auto &e : this->g.Edges()) {
			auto vSrc = &e->getVertexSrc();
			auto vDst = &e->getVertexDst();
			auto c = (this->candidateII * e->getDistance()) - e->getDelay() - this->resourceModel.getVertexLatency(vSrc);
			auto u = vSrc->getId();
			auto v = vDst->getId();
			this->sdcSolver.addBaseConstraint(u, v, c);
			this->dependencyConstraints[{vSrc, vDst}] = c;
		}
#if USE_BELLMAN_FORD
		// do nothing I guess?!
#else
		this->sdcSolver.computeInitialSolution();
#endif
		for (auto &r : this->resourceModel.Resources()) {
			auto vertices = this->resourceModel.getVerticesOfResource(r);
			for (auto &v1 : vertices) {
				for (auto &v2 : vertices) {
					try {
						// check if a dependency constraint between these vertices exists
						this->dependencyConstraints.at({v1, v2});
					}
					catch (std::out_of_range&) {
						// create dependency constraint "infinity" just so accessing that pair works without problems
						this->dependencyConstraints[{v1, v2}] = INT32_MAX;
					}
				}
			}
		}
	}

	std::set<int> SDSScheduler::solveSDC(const list<std::tuple<const Vertex *, const Vertex *, int, int>> &additionalSDCConstraints) {
		// DEBUG START
		/*
		if (this->sdcSolver.getSolutionFound()) {
		std::cout << "#q# Current SDC solver solution:" << std::endl;
		std::map<Vertex*, int> sdcSchedule;
		for (auto &it : this->sdcSolver.getSolution()) {
			std::cout << "  -> t_" << it.first << " = " << it.second << std::endl;
			sdcSchedule[&this->g.getVertexById(it.first)] = it.second;
		}
		auto currentSDCConstraints = this->sdcSolver.getCurrentConstraints();
		for (auto &it : currentSDCConstraints) {
			auto i = std::get<0>(it);
			auto j = std::get<1>(it);
			if (i < 0 or j < 0) continue; // auxiliary constraint added by the solver
			auto v_i = &this->g.getVertexById(i);
			auto v_j = &this->g.getVertexById(j);
			auto t_i = sdcSchedule.at(v_i);
			auto t_j = sdcSchedule.at(v_j);
			auto c = std::get<2>(it);
			auto valid = t_i - t_j <= c;
			std::cout << "SDC constraint 't_" << v_i->getId() << " - t_" << v_j->getId() << " <= " << c << "' is " << (valid?"valid":"invalid") << " (v_" << v_i->getId() << " = '" << v_i->getName() << "', v_" << v_j->getId() << " = '" << v_j->getName() << "')" << std::endl;
		}
		for (auto &e : this->g.Edges()) {
			auto v_i = &e->getVertexSrc();
			auto v_j = &e->getVertexDst();
			auto t_i = sdcSchedule.at(v_i);
			auto t_j = sdcSchedule.at(v_j);
			auto valid = t_j - (t_i + this->resourceModel.getVertexLatency(v_i) + e->getDelay()) + (e->getDistance() * this->candidateII) >= 0;
			if (!valid) {
				throw HatScheT::Exception("Edge '"+v_i->getName()+"' -("+std::to_string(e->getDistance())+")-> '"+v_j->getName()+"' invalid (t_i="+std::to_string(t_i)+" and t_j="+std::to_string(t_j)+")");
			}
		}
	}
	else {
		std::cout << "#q# SDC solver has no solution (yet)" << std::endl;
	}
		 */
		// DEBUG END
		bool valid = true;
		std::list<std::tuple<const Vertex *, const Vertex *, int, int>> addedConstraints;
		std::set<int> conflictResourceConstraints;
#if USE_BELLMAN_FORD
		//std::cout << "#q# ADDITIONAL CONSTRAINTS:" << std::endl;
		for (auto &it : additionalSDCConstraints) {
			//std::cout << "#q#   additional SDC constraint: t_" << std::get<0>(it)->getId() << " - t_" << std::get<1>(it)->getId() << " <= " << std::get<2>(it) << std::endl;
			this->sdcSolver.addAdditionalConstraint(std::get<0>(it)->getId(), std::get<1>(it)->getId(), std::get<2>(it));
		}
		this->sdcSolver.solve();
		valid = this->sdcSolver.getSolutionFound();
		auto conflictConstraints = this->sdcSolver.getAdditionalConflicts();
		//std::cout << "#q# CONFLICT CONSTRAINTS:" << std::endl;
		for (auto &conflictConstraint : conflictConstraints) {
			//std::cout << "#q#   conflict constraint: t_" << std::get<0>(conflictConstraint) << " - t_" << std::get<1>(conflictConstraint) << " <= " << std::get<2>(conflictConstraint) << std::endl;
			for (auto &addedConstraint : additionalSDCConstraints) {
				if (std::get<0>(addedConstraint)->getId() != std::get<0>(conflictConstraint)) continue;
				if (std::get<1>(addedConstraint)->getId() != std::get<1>(conflictConstraint)) continue;
				if (std::get<2>(addedConstraint) != std::get<2>(conflictConstraint)) continue;
				//std::cout << "#q# ADDED CONFLICT RESOURCE CONSTRAINT!" << std::endl;
				conflictResourceConstraints.insert(std::get<3>(addedConstraint));
			}
		}
#else
		for (auto &it : additionalSDCConstraints) {
			// check if we can skip that constraint
			auto &vSrc = std::get<0>(it);
			auto &vDst = std::get<1>(it);
			auto &c = std::get<2>(it);
			if (c >= this->dependencyConstraints.at({vSrc, vDst})) continue;
			//std::cout << "#q# addAdditionalConstraint START" << std::endl;
			valid = this->sdcSolver.addAdditionalConstraint(vSrc->getId(), vDst->getId(), c);
			//std::cout << "#q# addAdditionalConstraint END" << std::endl;
			//addedConstraints.emplace_back(it);
			addedConstraints.emplace_front(it);
			if (valid) {
				// check if solution is still valid
				auto sdcSolution = this->sdcSolver.getSolution();
				if (!this->quiet) {
					std::cout << "SDSScheduler::solveSDC: adding constraint DID NOT cause infeasibility" << std::endl;
					std::cout << "SDSScheduler::solveSDC: current solution:" << std::endl;
					for (auto &itSDCSol : sdcSolution) {
						std::cout << "    " << itSDCSol.first << " = " << itSDCSol.second << std::endl;
					}
				}
				bool allOk = true;
				for (auto &e : this->g.Edges()) {
					auto *u = &e->getVertexSrc();
					auto *v = &e->getVertexDst();
					auto dist = e->getDistance();
					auto delay = e->getDelay();
					auto lu = this->resourceModel.getVertexLatency(u);
					auto rhs = -(lu + delay - (this->candidateII * dist));
					auto tu = sdcSolution.at(u->getId());
					auto tv = sdcSolution.at(v->getId());
					auto ok = tu - tv <= rhs;
					if (not ok) {
						allOk = false;
						std::cerr << "Constraint 't_" << u->getId() << " - t_" << v->getId() << " <= " << rhs << "' violated!" << std::endl;
						std::cerr << "    u = " << u->getName() << std::endl;
						std::cerr << "    v = " << v->getName() << std::endl;
						std::cerr << "    t_u = " << tu << std::endl;
						std::cerr << "    t_v = " << tv << std::endl;
					}
				}
				if (not allOk) {
					throw Exception("SDSScheduler::solveSDC: Found bug in SDC solver");
				}
			}
			else {
				// adding constraint caused infeasibility
#if USE_OIJK
				//std::cout << "#q# CONFLICT CONSTRAINTS 1" << std::endl;
				auto conflictConstraints = this->sdcSolver.getConflicts();
				//std::cout << "#q# CONFLICT CONSTRAINTS 2" << std::endl;
				for (auto &conflictConstraint : conflictConstraints) {
					//std::cout << "#q#   conflict constraint: t_" << std::get<0>(conflictConstraint) << " - t_" << std::get<1>(conflictConstraint) << " <= " << std::get<2>(conflictConstraint) << std::endl;
					for (auto &addedConstraint : addedConstraints) {
						if (std::get<0>(addedConstraint)->getId() != std::get<0>(conflictConstraint)) continue;
						if (std::get<1>(addedConstraint)->getId() != std::get<1>(conflictConstraint)) continue;
						if (std::get<2>(addedConstraint) != std::get<2>(conflictConstraint)) continue;
						//conflictResourceConstraints.insert({std::get<0>(addedConstraint), std::get<1>(addedConstraint), std::get<3>(addedConstraint)});
						conflictResourceConstraints.insert(std::get<3>(addedConstraint));
					}
				}
#else
				for (auto &addedConstraint : addedConstraints) {
					conflictResourceConstraints.insert(std::get<3>(addedConstraint));
				}
#endif
				break;
			}
		}
#endif
		if (valid) {
			// we have a solution!
			auto solution = this->sdcSolver.getSolution();
			int minTime = 0;
			for (auto &it : solution) {
				if (it.first == -1) continue; // helper vertex
				this->startTimes[&this->g.getVertexById(it.first)] = it.second;
				if (it.second < minTime) minTime = it.second;
			}
			for (auto &it : this->startTimes) {
				this->startTimes[it.first] = it.second - minTime;
			}
			if (!this->quiet) {
				std::cout << "SDSScheduler::solveSDC: found solution!" << std::endl;
				for (auto &it : this->startTimes) {
					std::cout << "    " << it.first->getName() << " - " << it.second << std::endl;
				}
			}
		}
		if (not valid) {
#if USE_BELLMAN_FORD
			this->sdcSolver.clearAdditionalConstraints();
#else
			// remove added constraints again in case of an invalid solution
			for (auto &it : addedConstraints) {
				this->sdcSolver.removeAdditionalConstraint(std::get<0>(it)->getId(), std::get<1>(it)->getId(), std::get<2>(it));
			}
#endif
			if (!this->quiet) std::cout << "SDSScheduler::solveSDC: cleared additional constraints" << std::endl;
		}
		// validation
		if (this->sdcSolver.getSolutionFound()) {
			auto sdcSolution = this->sdcSolver.getSolution();
			if (!this->quiet) {
				std::cout << "SDSScheduler::solveSDC: current solution:" << std::endl;
				for (auto &it : sdcSolution) {
					std::cout << "    " << it.first << " = " << it.second << std::endl;
				}
			}
			bool allOk = true;
			for (auto &e : this->g.Edges()) {
				auto *u = &e->getVertexSrc();
				auto *v = &e->getVertexDst();
				auto dist = e->getDistance();
				auto delay = e->getDelay();
				auto lu = this->resourceModel.getVertexLatency(u);
				auto rhs = -(lu + delay - (this->candidateII * dist));
				auto tu = sdcSolution.at(u->getId());
				auto tv = sdcSolution.at(v->getId());
				auto ok = tu - tv <= rhs;
				if (not ok) {
					allOk = false;
					std::cerr << "Constraint 't_" << u->getId() << " - t_" << v->getId() << " <= " << rhs << "' violated!" << std::endl;
					std::cerr << "    u = " << u->getName() << std::endl;
					std::cerr << "    v = " << v->getName() << std::endl;
					std::cerr << "    t_u = " << tu << std::endl;
					std::cerr << "    t_v = " << tv << std::endl;
				}
			}
			if (not allOk) {
				throw Exception("SDSScheduler::solveSDC: Found bug in SDC solver -> it computed an infeasible solution!");
			}
		}
		return conflictResourceConstraints;
	}

	void SDSScheduler::forbidConflictingResourceConstraints(const std::set<int> &conflictingResourceConstraints) {
		if (conflictingResourceConstraints.empty()) {
			throw Exception("SDSScheduler::forbidConflictingResourceConstraints: no resource constraints given -> that should never happen!");
		}
		std::stringstream clause;
		bool begin = true;
		std::set<int> learnedClause;
		if (!this->quiet) {
			std::cout << "Add clause";
		}
		for (auto &oijk : conflictingResourceConstraints) {
			if (begin) {
				begin = false;
				if (!this->quiet) {
					std::cout << " ";
				}
			}
			else {
				if (!this->quiet) {
					std::cout << " OR ";
				}
			}
			this->solver->add(-oijk);
			learnedClause.insert(-oijk);
			if (!this->quiet) {
				std::cout << -oijk;
			}
		}
		this->solver->add(0);
		// FOR DEBUGGING ONLY BEGIN
		/*auto ret = this->learnedClauses.insert(learnedClause);
		if (!ret.second) {
			throw HatScheT::Exception("Learned clause twice - this should never happen!!!");
		}*/
		// FOR DEBUGGING ONLY END

		if (!this->quiet) {
			std::cout << std::endl;
		}
	}

	void SDSScheduler::computeShortestPathSolution() {
		if (!this->scheduleFound) return; // can only compute the shortest path solution for feasible constraints
		if (!this->quiet) {
			std::cout
				<< "SDSScheduler::computeShortestPathSolution: Schedule before refining:" << std::endl;
			for (auto &it : this->startTimes) {
				std::cout << "  '" << it.first->getName() << "' : t = " << it.second << std::endl;
			}
		}
		SDCSolverBellmanFord s;
		s.setQuiet(this->quiet);
		auto constraints = this->sdcSolver.getCurrentConstraints();
		if (!this->quiet) {
			std::cout
				<< "SDSScheduler::computeShortestPathSolution: refine final schedule with bellman ford algorithm and following constraints:"
				<< std::endl;
		}
		for (auto &it : constraints) {
			auto &u = std::get<0>(it);
			auto &v = std::get<1>(it);
			auto &c = std::get<2>(it);
			if (u < 0 or v < 0) continue; // skip constraints that belong to the virtual source node
			if (!this->quiet) {
				std::cout << "  -> '" << this->g.getVertexById(u).getName() << "' - '" << this->g.getVertexById(v).getName() << "' <= " << c << std::endl;
			}
			s.addBaseConstraint(u, v, c);
		}
		s.solve();
		if (!s.getSolutionFound()) {
			// that should never happen!
			throw Exception("SDSScheduler::computeShortestPathSolution: Applying Bellman Ford to a feasible SDC should never fail!");
		}
		auto solution = s.getNormalizedSolution();
		for (auto &it : solution) {
			this->startTimes[&this->g.getVertexById(it.first)] = it.second;
		}
	}
}

#else

namespace HatScheT {
	HatScheT::SDSScheduler::SDSScheduler(HatScheT::Graph &g, HatScheT::ResourceModel &resourceModel,
																			 std::list<std::string> &sw) : SchedulerBase(g, resourceModel),
																																		 ILPSchedulerBase(sw) {

		this->unsatisiable = false;
		computeMinII(&this->g, &resourceModel);
		this->minII = ceil(this->minII);
		computeMaxII(&g, &resourceModel);
		if (minII >= maxII) maxII = (int) minII + 1;
		this->II = minII;
		this->swishlist = sw;
		this->bindingType = 'S';
		this->numOfLimitedResources = 0;

		//ToDo: Has to be removed
		cout << "Constructor SDS:" << endl;
		cout << "Graph generated with " << g.getNumberOfVertices() << " Vertices." << endl;
		cout << "MinII is: " << minII << endl;
		cout << "MaxII is: " << maxII << endl;
		cout << "II is: " << II << endl;
	}

	void HatScheT::SDSScheduler::schedule() {

		if (sdcS == BellmanFord) {

			double timer = 0;
			std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

			/*!
			 * Setup of Variables.
			 */
			//Create Binding Variables.
			createBindingVariables();

			//Set Values for Binding Variables.
			setBindingVariables();

			//Create Sharing Variables.
			if (bindingType == 'S') {
				sharingVariables = createShVarsMaxSpeed();
			} else if (bindingType == 'R') {
				sharingVariables = createShVarsMinRes();
			}

			//Create Data Dependency Constraints.
			createDependencyConstraints();

			//Print Depandency Constraints for Debugging.
			cout << "Data Dependency Constraints:" << endl;
			for (auto &it : dependencyConstraintsSDC) {
				static int i = 0;
				cout << ++i << ". " << it.first.first->getName() << " - " << it.first.second->getName() << " < " << it.second
						 << endl;
			}
			cout << endl;

			std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
			std::chrono::nanoseconds timeSpan = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1);
			timer += (((double) timeSpan.count()) / 1000000000.0);
			cout << "Creating Variables Done after " << timer << " Seconds" << endl << endl;

			/*!
			 * Calculate a Solution for the SDC-Problem without Ressource Constraints.
			 */
			//Setup an SDC-Solver
			auto *s = new SDCSolver;

			//Add Dependency Constraints to the SDC-Solver
			for (auto &it : dependencyConstraintsSDC) {
				s->add_sdc_constraint(
					s->create_sdc_constraint((Vertex *) it.first.second, (Vertex *) it.first.first, it.second));
			}

			//Calculate a Shedule with Data Dependencys only.
			s->compute_inital_solution();

			//Check is SDC-System is feasible. (3)
			if (s->get_solver_status() == 10) {
				//If feasible save it.
				startTimes = map_SDC_solution_to_Graph(s->get_solution());
			} else {
				//Else terminate with error.
				throw HatScheT::Exception("1. System of Constraints not feasible.");
			}
			//Delete SDC-Solver
			delete s;

			std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
			timeSpan = std::chrono::duration_cast<std::chrono::nanoseconds>(t3 - t2);
			timer += (((double) timeSpan.count()) / 1000000000.0);
			cout << "Initial SDC-Problem solved after " << timer << " Seconds" << endl;

			//Calculate Rssource Constraints with SAT, from Sharing Variable, conflictClauses is an empty Vector for now.
			passToSATSolver(sharingVariables, conflictClauses);

			//Print Resource Constraints for Debugging.
			cout << endl << "Resource Constraints: " << endl;
			for (auto &it : resourceConstraints) {
				static int i = 0;
				cout << ++i << ". " << it.constraintOneVertices.first->getName() << " - "
						 << it.constraintOneVertices.second->getName() << " < " << it.constraintOne << endl;
			}
			cout << endl;

			std::chrono::high_resolution_clock::time_point t4 = std::chrono::high_resolution_clock::now();
			timeSpan = std::chrono::duration_cast<std::chrono::nanoseconds>(t4 - t3);
			timer += (((double) timeSpan.count()) / 1000000000.0);
			cout << "Initial SAT-Problem solved after " << timer << " Seconds" << endl;

			//Add Dependency Constraints to an SDC-Solver.
			s = new SDCSolver;
			for (auto &it : dependencyConstraintsSDC) {
				s->add_sdc_constraint(
					s->create_sdc_constraint((Vertex *) it.first.second, (Vertex *) it.first.first, it.second));
			}
			//Add Resource Constraints to SDC-Solver
			for (auto &it : resourceConstraints) {
				static int counter = 0;
				//s->add_sdc_constraint(s->create_sdc_constraint((Vertex*)it.constraintOneVertices.first, (Vertex*)it.constraintOneVertices.second, it.constraintOne));
				//s->compute_inital_solution();
				s->add_Constraint(s->create_sdc_constraint((Vertex *) it.constraintOneVertices.first,
																									 (Vertex *) it.constraintOneVertices.second, it.constraintOne));
				//if (s->get_solver_status() != 10){
				if (s->get_solver_status() != 30) {
					cout << counter << endl;
					std::chrono::high_resolution_clock::time_point t6 = std::chrono::high_resolution_clock::now();
					timeSpan = std::chrono::duration_cast<std::chrono::nanoseconds>(t6 - t4);
					timer += (((double) timeSpan.count()) / 1000000000.0);
					cout << "SDC-Problem with Resource Constraints done after " << timer << " Seconds" << endl;
					throw HatScheT::Exception("2. System of Constraints not feasible.");
				} else {
					startTimes = map_SDC_solution_to_Graph(s->get_solution());
				}
				counter++;
			}

			std::chrono::high_resolution_clock::time_point t5 = std::chrono::high_resolution_clock::now();
			timeSpan = std::chrono::duration_cast<std::chrono::nanoseconds>(t5 - t4);
			timer += (((double) timeSpan.count()) / 1000000000.0);
			cout << "SDC-Problem with Resource Constraints solved after " << timer << " Seconds" << endl;

			//Print Constraint Graph for Debugging.
			cout << "Constraint Graph: " << endl;
			s->print_Constraint_Graph();

			//Delete SDC-Solver
			delete s;
		}
	}

	void HatScheT::SDSScheduler::createBindingVariables() {

		list<Resource *> localResources;
		for (auto it = resourceModel.resourcesBegin(); it != resourceModel.resourcesEnd(); ++it) {
			if (!(*it)->isUnlimited()) {
				localResources.push_back(*it);
			}
		}

		this->numOfLimitedResources = localResources.size();

		bindingVariable bv;
		int idCount = 0;
		int index = 0;

		for (auto &it:localResources) {
			set<const Vertex *> verticiesOfResource = resourceModel.getVerticesOfResource(it);
			for (auto &vORIt:verticiesOfResource) {
				for (int i = 0; i < it->getLimit(); i++) {
					bv.index = index;
					bv.resource = it;
					bv.resourceID = idCount;
					bv.vertex = vORIt;
					bv.resourceInstance = i;
					bv.binding = false;
					bindingVariables.push_back(bv);
					index++;
				}
			}
			idCount++;
		}
	}

	void SDSScheduler::setBindingVariables() {

		int lastVertexId = -1;
		int count = 0;

		for (int i = 0; i < numOfLimitedResources; i++) {
			for (auto &it : bindingVariables) {
				if (it.resourceID == i) {
					if (it.vertex->getId() != lastVertexId) {
						count++;
						lastVertexId = it.vertex->getId();
					}
					if ((count % it.resource->getLimit()) == it.resourceInstance) {
						it.binding = true;
					}
				}
			}
			count = 0;
		}
	}

	map<pair<const Vertex *, const Vertex *>, bool> SDSScheduler::createShVarsMaxSpeed() {

		map<pair<const Vertex *, const Vertex *>, bool> shared;
		list<bindingVariable *> tempBinVars;
		list<list<bindingVariable *>> templists;

		for (int i = 0; i < numOfLimitedResources; i++) {
			for (auto &it : bindingVariables) {
				if (it.resourceID == i) {
					tempBinVars.push_back(&it);
				}
			}
			templists.push_back(tempBinVars);
			tempBinVars.clear();
		}

		for (auto &lit : templists) {
			int loops = 1;
			for (auto &ibvIt : lit) {
				if (ibvIt->binding) {
					auto jbvIt = lit.begin();
					for (advance(jbvIt, loops); jbvIt != lit.end(); ++jbvIt) {
						if (ibvIt->vertex->getId() != (*jbvIt)->vertex->getId() &&
								(ibvIt->resourceInstance == (*jbvIt)->resourceInstance)) {
							if (ibvIt->binding && (*jbvIt)->binding) {
								auto vpair = make_pair(ibvIt->vertex, (*jbvIt)->vertex);
								shared.insert(make_pair(vpair, true));
							} else {
								auto vpair = make_pair(ibvIt->vertex, (*jbvIt)->vertex);
								shared.insert(make_pair(vpair, false));
							}
						}
					}
				}
				loops++;
			}
		}
		return shared;
	}

	map<pair<const Vertex *, const Vertex *>, bool> SDSScheduler::createShVarsMinRes() {

		map<pair<const Vertex *, const Vertex *>, bool> shared;
		list<bindingVariable *> tempBinVars;
		list<list<bindingVariable *>> templists;

		for (int i = 0; i < numOfLimitedResources; i++) {
			for (auto &it : bindingVariables) {
				if (it.resourceID == i) {
					tempBinVars.push_back(&it);
				}
			}
			templists.push_back(tempBinVars);
			tempBinVars.clear();
		}

		for (auto &lit : templists) {
			int loops = 1;
			for (auto &ibvIt : lit) {
				if (ibvIt->binding) {
					auto jbvIt = lit.begin();
					for (advance(jbvIt, loops); jbvIt != lit.end(); ++jbvIt) {
						if (ibvIt->vertex->getId() != (*jbvIt)->vertex->getId() &&
								(ibvIt->resourceInstance == (*jbvIt)->resourceInstance)) {
							auto vpair = make_pair(ibvIt->vertex, (*jbvIt)->vertex);
							shared.insert(make_pair(vpair, true));
						}
					}
				}
				loops++;
			}
		}
		return shared;
	}


	map<pair<const Vertex *, const Vertex *>, int>
	SDSScheduler::passToSATSolver(map<pair<const Vertex *, const Vertex *>, bool> &shareVars,
																vector<vector<int>> &confClauses) {

		/*!
		 * Instance of SAT-Solver
		 */
		CaDiCaL::Solver *solver = new CaDiCaL::Solver;

		/*!
		 * Converting Sharing Variables to SAT-Literals and pass to SAT-Solver.
		 */
		int litCounter = 1;
		if (!this->quiet) {
			cout << "CaDiCaL: Problem: " << "p cnf " << shareVars.size() * 2 << " "
					 << shareVars.size() * 2 + confClauses.size() << endl;
		}
		for (auto &It : shareVars) {
			if (It.second) {
				solver->add(litCounter);
				solver->add(litCounter + 1);
				solver->add(0);
				if (!this->quiet) { cout << setw(3) << litCounter << setw(3) << litCounter + 1 << setw(3) << " 0" << endl; }
				solver->add(litCounter * -1);
				solver->add((litCounter + 1) * -1);
				solver->add(0);
				if (!this->quiet) {
					cout << setw(3) << litCounter * -1 << setw(3) << (litCounter + 1) * -1 << setw(3) << " 0" << endl;
				}
				litCounter += 2;
			}
		}

		/*!
		 * Adding Conflict Clauses
		 */
		for (auto &It : confClauses) {
			for (auto &Itr : It) {
				solver->add(Itr);
				if (!this->quiet) {
					cout << setw(3) << Itr;
				}
			}
			if (!It.empty()) {
				solver->add(0);
				if (!this->quiet) {
					cout << setw(3) << "0";
				}
			}
			if (!this->quiet) {
				cout << endl;
			}
		}

		/*!
		 * Solve Problem
		 */
		int res = solver->solve();

		/*!
		 * Check if satisfiable:
		 */
		if (res == 10) {
			if (!quiet) {
				cout << endl << "CaDiCaL: Problem Satisfiable" << endl;
			}
		} else if (res == 0) {
			if (!quiet) {
				cout << endl << "CaDiCaL: Problem Unsolved" << endl;
			}
		} else if (res == 20) {
			if (!quiet) {
				cout << endl << "CaDiCaL: Problem Unsatisfiable" << endl;
			}
			unsatisiable = true;
		}

		/*!
		 * Getting Solution
		 * Solver variable starts at 1, since negated variables are shown as negative numbers.
		 * The Solution from SAT is than mapped to an SDC-Format like (srcVertex - dstVertex <= -1)
		 */
		map<pair<const Vertex *, const Vertex *>, int> solutionMap;
		resourceConstraints.clear();
		auto mIt = sharingVariables.begin();
		if (res == 10) {
			for (int i = 0; i < litCounter - 1; i++) {
				//if (!this->quiet) {
				//cout << solver->val(i + 1) << " ";
				//}
				if (i % 2 == 0) {
					if (solver->val(i + 1) > 0) {
						orderingVariabletoSDCMapping mapping;
						mapping.satVariable = i + 1;
						mapping.constraintOneVertices = mIt->first;
						mapping.constraintOne = -resourceModel.getResource(mIt->first.first)->getBlockingTime();
						//mapping.createConstraintTwo((int) this->II);
						resourceConstraints.push_back(mapping);
					}
				} else {
					if (solver->val(i + 1) > 0) {
						orderingVariabletoSDCMapping mapping;
						mapping.satVariable = i + 1;
						mapping.constraintOneVertices = swapPair(mIt->first);
						mapping.constraintOne = -resourceModel.getResource(mIt->first.first)->getBlockingTime();
						//mapping.createConstraintTwo((int) this->II);
						resourceConstraints.push_back(mapping);
					}
					++mIt;
				}
			}
		}

		/*!
		 * Delete the solver.
		 */
		delete solver;

		return solutionMap;

	}

	pair<const Vertex *, const Vertex *> SDSScheduler::swapPair(pair<const Vertex *, const Vertex *> inPair) {
		return make_pair(inPair.second, inPair.first);
	}

	void SDSScheduler::createDependencyConstraints() {

		map<pair<const Vertex *, const Vertex *>, int> dependencyConstraints;

		for (auto &it : this->g.Edges()) {
			auto sVertex = &it->getVertexSrc();
			auto dVertex = &it->getVertexDst();

			auto constVertexPair = make_pair((const Vertex *) sVertex, (const Vertex *) dVertex);
			//int distance = ((int)this->II*it->getDistance() - this->resourceModel.getVertexLatency(&it->getVertexSrc())+it->getDelay());
			int distance;
			if (it->getDistance() == 0) {
				distance = (it->getDelay() - this->resourceModel.getVertexLatency(&it->getVertexSrc()));

				if (sdcS == BellmanFord) {
					dependencyConstraints.insert(make_pair(constVertexPair, distance));
				} else if (sdcS == ScaLP) {
					this->solver->addConstraint(this->scalpVariables[sVertex] - this->scalpVariables[dVertex] <= distance);
				}
			}
		}

		if (sdcS == BellmanFord) {
			dependencyConstraintsSDC = dependencyConstraints;
		}
	}

	bool SDSScheduler::doesVertexExist(Graph *gr, int vertexID) {
		for (auto &it : gr->Vertices()) {
			if (it->getId() == vertexID) {
				return true;
			}
		}
		return false;
	}


	void SDSScheduler::createScaLPVariables() {

		if (this->scalpVariables.empty()) {
			for (auto &it : g.Vertices()) {
				auto v = it;
				this->scalpVariables[v] = ScaLP::newIntegerVariable(v->getName(), 0, ScaLP::INF());
			}
		}

	}

	pair<ScaLP::Constraint, ScaLP::Constraint>
	SDSScheduler::createadditionalScaLPConstraits(orderingVariabletoSDCMapping order) {
		ScaLP::Constraint cA, cB;
		cA = (this->scalpVariables[order.constraintOneVertices.first] -
					this->scalpVariables[order.constraintOneVertices.second] <= order.constraintOne);
		cB = (this->scalpVariables[order.constraintTwoVertices.first] -
					this->scalpVariables[order.constraintTwoVertices.second] <= order.constraintTwo);
		return make_pair(cA, cB);
	}

	bool SDSScheduler::checkfeasibilityScaLP(vector<pair<ScaLP::Constraint, ScaLP::Constraint>> scstr, int maxLatConstr) {

		cout << "Checking feasibility" << endl;

		this->solver->reset();
		createDependencyConstraints();

		for (auto &it : this->scalpVariables) {
			ScaLP::Constraint c = (this->newVar - it.second >= this->resourceModel.getResource(it.first)->getLatency());
			this->solver->addConstraint(c);
		}

		for (auto &it : scstr) {
			this->solver->addConstraint(it.first);
			this->solver->addConstraint(it.second);
		}

		if (maxLatConstr > -1) {
			ScaLP::Constraint cs = (this->newVar <= maxLatConstr); //ToDo nochmal sauber aufschreiben und verstehen.
			this->solver->addConstraint(cs);
		}

		ScaLP::Term o = 1 * newVar;
		this->solver->setObjective(ScaLP::minimize(o));

		solver->writeLP("scalpLOG.txt");

		auto status = this->solver->solve();
		cout << "ScaLP Status: " << status << endl;

		return (status == ScaLP::status::FEASIBLE or status == ScaLP::status::OPTIMAL or
						status == ScaLP::status::TIMEOUT_FEASIBLE);
	}

	map<Vertex *, int> SDSScheduler::findstarttimes(ScaLP::Result &r) {

		map<Vertex *, int> times;

		for (auto &it : r.values) {
			auto *v = (Vertex *) this->getVertexFromVariable(it.first);
			if (v != nullptr) times[v] = (int) it.second;
		}

		return times;
	}

	const Vertex *SDSScheduler::getVertexFromVariable(const ScaLP::Variable &sv) {
		for (auto &it : this->scalpVariables) {
			if (it.second == sv) return it.first;
		}
		return nullptr;
	}

	vector<int> SDSScheduler::getSATClausesfromScaLP(list<orderingVariabletoSDCMapping> &conflicts) {
		vector<int> conflictSAT;
		conflictSAT.reserve(conflicts.size());
		for (auto &it : conflicts) {
			conflictSAT.push_back(it.satVariable * -1);
		}
		return conflictSAT;
	}

	map<Vertex *, int> SDSScheduler::map_SDC_solution_to_Graph(map<Vertex *, int> solution) {

		int min = INT_MAX;
		for (auto &it : solution) {
			it.second < min ? min = it.second : min = min;
		}

		map<Vertex *, int> mapped_solution;
		for (auto &it : solution) {
			if (it.first->getId() != -1) {
				mapped_solution[&g.getVertexById(it.first->getId())] = it.second + abs(min);
			}
		}
		return mapped_solution;
	}
}

#endif

#endif //USE_CADICAL