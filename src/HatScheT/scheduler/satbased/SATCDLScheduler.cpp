//
// Created by nfiege on 1/26/23.
//

#include "SATCDLScheduler.h"
#include <cmath>
#include <HatScheT/utility/SDCSolverBellmanFord.h>

namespace HatScheT {
	SATCDLScheduler::SATCDLScheduler(HatScheT::Graph &g, HatScheT::ResourceModel &resourceModel, int II)
		: SATSchedulerBase(g, resourceModel, II) {}

	void SATCDLScheduler::scheduleIteration() {
		SATSchedulerBase::scheduleIteration();
		this->SATSolverTime = 0.0;
		this->SDCSolverTime = 0.0;

		// set up SAT solver
		if (!this->quiet) std::cout << "SATCDLScheduler::scheduleIteration: Initializing SAT solver now" << std::endl;
		this->initSATSolver();

		this->scheduleFound = false;
		int iterationCounter = 0;
		while (!this->scheduleFound) {
			if (iterationCounter % 100 == 99) {
				std::cout << "SATCDLScheduler::scheduleIteration: Start new iteration #" << iterationCounter+1 << std::endl;
				std::cout << "  -> SAT solver time = " << this->SATSolverTime << "s" << std::endl;
				std::cout << "  -> SDC solver time = " << this->SDCSolverTime << "s" << std::endl;
				std::cout << "  -> total time = " << this->terminator.getElapsedTime() << "s" << std::endl;
			}
			iterationCounter++;
			if (!this->quiet) std::cout << "SATCDLScheduler::scheduleIteration: Start new iteration -> computing modulo slots now" << std::endl;
			auto moduloSlots = this->computeModuloSlots();
			if (!this->quiet) std::cout << "SATCDLScheduler::scheduleIteration: Start solving SDC now" << std::endl;
			auto conflicts = this->solveSDC(moduloSlots);
			if (!conflicts.empty()) {
				if (!this->quiet) std::cout << "SATCDLScheduler::scheduleIteration: Adding conflicts now" << std::endl;
				this->addConflicts(conflicts);
			}
		}
		if (!this->quiet) std::cout << "SATCDLScheduler::scheduleIteration: Finished scheduling" << std::endl;
	}

	void SATCDLScheduler::initSATSolver() {
		// init solver
		this->solver = std::unique_ptr<CaDiCaL::Solver>(new CaDiCaL::Solver);
		this->terminator = CaDiCaLTerminator(this->solverTimeout);
		this->solver->connect_terminator(&this->terminator);
		// clear containers
		this->mrtVariables.clear();
		// create variables
		std::cout << "SATCDLScheduler::initSATSolver: Create variables" << std::endl;
		for (auto &v : this->g.Vertices()) {
			for (int m=0; m<this->candidateII; m++) {
				this->mrtVariables[{v, m}] = ++this->literalCounter;
			}
		}
		// create clauses
		// assign exactly 1 MRT slot to each vertex
		std::cout << "SATCDLScheduler::initSATSolver: Clauses - 'assign exactly 1 MRT slot to each vertex'" << std::endl;
		for (auto &v : this->g.Vertices()) {
			std::vector<int> vars(this->candidateII);
			for (int m=0; m<this->candidateII; m++) {
				vars[m] = this->mrtVariables.at({v, m});
			}
			this->createExactly1Constraint(vars);
		}
		// assign max FU vertices to each MRT slot
		std::cout << "SATCDLScheduler::initSATSolver: Clauses - 'assign max FU vertices to each MRT slot'" << std::endl;
		for (auto &r : this->resourceModel.Resources()) {
			if (r->isUnlimited()) continue;
			std::cout << "  -> Resource '" << r->getName() << "'" << std::endl;
			auto lim = r->getLimit();
			auto vertices = this->resourceModel.getVerticesOfResource(r);
			std::vector<int> vars(vertices.size());
			for (int m=0; m<this->candidateII; m++) {
				int cnt = 0;
				for (auto &v : vertices) {
					vars[cnt] = this->mrtVariables.at({v, m});
					cnt++;
				}
				this->createAtMostConstraint(vars, lim);
			}
		}
	}

	map<const Vertex *, int> SATCDLScheduler::computeModuloSlots() {
		auto startTime = this->terminator.getElapsedTime();
		auto status = this->solver->solve();
		auto endTime = this->terminator.getElapsedTime();
		this->SATSolverTime += (endTime - startTime);
		auto sat = status == CADICAL_SAT;
		auto unsat = status == CADICAL_UNSAT;
		auto timeout = not (sat or unsat);
		if (timeout or unsat) {
			return map<const Vertex *, int>();
		}
		map<const Vertex *, int> moduloSlots;
		for (auto &it : this->mrtVariables) {
			auto *v = it.first.first;
			auto m = it.first.second;
			auto satVar = it.second;
			if (this->solver->val(satVar) < 0) continue; // variable is set to false
			moduloSlots[v] = m;
		}
		return moduloSlots;
	}

	set<std::pair<const Vertex *, int>> SATCDLScheduler::solveSDC(map<const Vertex *, int> &moduloSlots) {
		SDCSolverBellmanFord s;
		s.setQuiet(this->quiet);
		for (auto &e : this->g.Edges()) {
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			auto mSrc = moduloSlots.at(vSrc);
			auto mDst = moduloSlots.at(vDst);
			auto distance = e->getDistance();
			auto delay = e->getDelay();
			auto lSrc = this->resourceModel.getVertexLatency(vSrc);
			auto numerator = (double)(lSrc + delay + mSrc - mDst);
			auto denominator = (double)(this->candidateII);
			auto c = - (int)std::ceil(numerator / denominator) + distance;
			s.addAdditionalConstraint(vSrc->getId(), vDst->getId(), c);
		}
		if (!this->quiet) std::cout << "SATCDLScheduler::solveSDC: start Bellman Ford algorithm" << std::endl;
		auto startTime = this->terminator.getElapsedTime();
		s.solve();
		auto endTime = this->terminator.getElapsedTime();
		this->SDCSolverTime += (endTime - startTime);
		if (!this->quiet) std::cout << "SATCDLScheduler::solveSDC: Bellman Ford algorithm finished" << std::endl;
		if (s.getSolutionFound()) {
			if (!this->quiet) std::cout << "SATCDLScheduler::solveSDC: valid solution found!" << std::endl;
			// yay, no conflict!
			auto solution = s.getNormalizedSolution();
			for (auto &it : solution) {
				auto *v = &this->g.getVertexById(it.first);
				this->startTimes[v] = moduloSlots.at(v) + (this->candidateII * it.second);
			}
			this->scheduleFound = true;
			return set<std::pair<const Vertex *, int>>();
		}
		if (!this->quiet) std::cout << "SATCDLScheduler::solveSDC: failed to find solution :(" << std::endl;
		auto conflicts = s.getAdditionalConflicts();
		set<std::pair<const Vertex *, int>> conflictSlots;
		for (auto &it : conflicts) {
			auto *v = &this->g.getVertexById(std::get<0>(it));
			conflictSlots.insert({v, moduloSlots.at(v)});
		}
		return conflictSlots;
	}

	void SATCDLScheduler::createAtMostConstraint(vector<int> &vars, int rhs) {
		if (!this->quiet) std::cout << "SATCDLScheduler::createAtMostConstraint: for " << vars.size() << " variables and rhs=" << rhs << std::endl;
		// sum all variables up: "result = sum over all vars"
		std::vector<std::vector<int>> bitheapInputs(vars.size(), std::vector<int>(1));
		for (size_t i=0; i<vars.size(); i++) {
			bitheapInputs[i][0] = vars.at(i);
		}
		auto sumResult = this->create_bitheap(bitheapInputs);
		if (!this->quiet) std::cout << "SATCDLScheduler::createAtMostConstraint: created bitheap with " << sumResult.size() << " outputs" << std::endl;
		// "result <= rhs"
		auto wordSize = (int)sumResult.size();
		if (wordSize < 1) {
			throw HatScheT::Exception("SATCDLScheduler::createAtMostConstraint: detected wordSize = "+std::to_string(wordSize)+" -> this should never happen!");
		}
		std::vector<std::pair<int, bool>> clauseBase;
		for (int idx = wordSize-1; idx >= 0; idx--) {
			if ((rhs >> idx) & 1) {
				clauseBase.emplace_back(sumResult.at(idx), true);
				continue;
			}
			auto clause = clauseBase;
			clause.emplace_back(sumResult.at(idx), true);
			this->create_arbitrary_clause(clause);
		}
	}

	void SATCDLScheduler::createExactly1Constraint(vector<int> &vars) {
		// at most 1
		this->createAtMostConstraint(vars, 1);
		// at least 1 (OR-relation of all vars)
		for (auto &var : vars) {
			this->solver->add(var);
		}
		this->solver->add(0);
	}

	std::vector<int> SATCDLScheduler::create_bitheap(const vector<std::vector<int>> &x) {
		std::vector<int> result_variables;
		std::map<int, std::vector<std::pair<int, bool>>> y;
		int num_bits = 0;
		for (auto &it : x) {
			auto bits = it;
			if (bits.size() > num_bits) num_bits = bits.size();
			// add bits
			for (int bit_pos=0; bit_pos<bits.size(); bit_pos++) {
				y[bit_pos].emplace_back(bits[bit_pos], false);
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
					this->create_half_adder(a, b, {sum, false}, {carry, false});
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
					this->create_full_adder(a, b, c, {sum, false}, {carry, false});
					y[i].emplace_back(sum, false);
					y[i+1].emplace_back(carry, false);
					if (i+2 > num_bits) num_bits = i+2;
				}
			}
			if (y[i].size() != 1) {
				std::cerr << "SATCDLScheduler::create_bitheap: Failed compressing bits at position " << i << " -> " << y[i].size() << " bits are left instead of 1" << std::endl;
				throw std::runtime_error("error during bitheap clause generation");
			}
			if (y[i][0].second) {
				std::cerr << "SATCDLScheduler::create_bitheap: Failed compressing bits at position " << i << " -> the output bit is inverted..." << std::endl;
				throw std::runtime_error("error during bitheap clause generation");
			}
			result_variables.emplace_back(y[i][0].first);
			// advance to next bit position
			i++;
		}
		return result_variables;
	}

	void SATCDLScheduler::create_half_adder(std::pair<int, bool> a, std::pair<int, bool> b, std::pair<int, bool> sum,
																					std::pair<int, bool> c_o) {
		//this->create_2x1_xor(a, b, sum);
		// 1) a b -sum
		this->create_arbitrary_clause({{a.first, a.second}, {b.first, b.second}, {sum.first, not sum.second}});
		// 2) a -b sum
		this->create_arbitrary_clause({{a.first, a.second}, {b.first, not b.second}, {sum.first, sum.second}});
		// 3) -a b sum
		this->create_arbitrary_clause({{a.first, not a.second}, {b.first, b.second}, {sum.first, sum.second}});
		// 4) -a -b -sum
		this->create_arbitrary_clause({{a.first, not a.second}, {b.first, not b.second}, {sum.first, not sum.second}});

		if (c_o.first <= 0) return;
		//this->create_2x1_and(a, b, c_o);
		// 1) -a -b c_o
		this->create_arbitrary_clause({{a.first, not a.second},{b.first, not b.second},{c_o.first, c_o.second}});
		// 2) a -c_o
		this->create_arbitrary_clause({{a.first, a.second},{c_o.first, not c_o.second}});
		// 3) b -c_o
		this->create_arbitrary_clause({{b.first, b.second},{c_o.first, not c_o.second}});
	}

	void SATCDLScheduler::create_full_adder(std::pair<int, bool> a, std::pair<int, bool> b, std::pair<int, bool> c_i,
																					std::pair<int, bool> sum, std::pair<int, bool> c_o) {
		//this->create_add_sum(a, b, c_i, sum);
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

		if (c_o.first <= 0) return;
		//this->create_add_carry(a, b, c_i, c_o);
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

	}

	void SATCDLScheduler::create_arbitrary_clause(const vector<std::pair<int, bool>> &a) {
		//std::cout << "SATCDLScheduler::create_arbitrary_clause:";
		for (auto &it : a) {
			if (it.second) {
				// negate
				this->solver->add(-it.first);
				//std::cout << " '" << -it.first << "'";
			}
			else {
				// don't negate
				this->solver->add(it.first);
				//std::cout << " '" << it.first << "'";
			}
		}
		this->solver->add(0);
		//std::cout << std::endl;
	}

	void SATCDLScheduler::addConflicts(set<std::pair<const Vertex *, int>> &conflicts) {
		for (auto &it : conflicts) {
			this->solver->add(-this->mrtVariables.at(it));
		}
		this->solver->add(0);
	}
}