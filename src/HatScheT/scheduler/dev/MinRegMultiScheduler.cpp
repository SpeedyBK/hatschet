//
// Created by nfiege on 4/13/22.
//

#include "MinRegMultiScheduler.h"
#include <cmath>
#include <sstream>
#include <chrono>

namespace HatScheT {

	MinRegMultiScheduler::MinRegMultiScheduler(Graph &g, ResourceModel &resourceModel,
		std::list<std::string> solverWishlist) : IterativeModuloSchedulerLayer(g, resourceModel), ILPSchedulerBase(solverWishlist),
		numLifetimeRegs(-1), bigM(0.0), multipleStartTimesAllowed(true), candII(-1), ilpConstraintCounter(0), ilpVariableCounter(0) {
		this->optimalResult = false;
	}

	void MinRegMultiScheduler::scheduleOLD() {
/*		this->initScheduler();
		this->timeouts = 0;
		this->solvingTime = 0.0;
		for (this->candII = (int)this->minII; this->candII <= (int)this->maxII; ++this->candII) {
			if (!this->quiet) {
				std::cout << "MinRegMultiScheduler: candidate II=" << this->candII << std::endl;
			}
			this->resetContainer();
			this->setUpSolver();
			this->createScaLPVariables();
			this->constructProblem();
			this->setObjective();
			if (!this->quiet) {
				std::cout << "MinRegMultiScheduler: start scheduling with '" << this->ilpVariableCounter << "' variables and '"
				  << this->ilpConstraintCounter << "' constraints" << std::endl;
				std::cout << "MinRegMultiScheduler: start scheduling with ILP formulation:" << std::endl;
				std::cout << this->solver->showLP() << std::endl;
			}
			auto timerStart = std::chrono::steady_clock::now();
			this->stat = this->solver->solve();
			auto timerEnd = std::chrono::steady_clock::now();
			auto elapsedTime = ((double)std::chrono::duration_cast<std::chrono::milliseconds>(timerEnd - timerStart).count()) / 1000.0;
			this->solvingTime += elapsedTime;
			if (!this->quiet) {
				std::cout << "MinRegMultiScheduler: finished solving with status '" << ScaLP::showStatus(this->stat)
				  << "' after " << elapsedTime << " sec (total: " << this->solvingTime << " sec)" << std::endl;
			}
			if(this->stat == ScaLP::status::TIMEOUT_INFEASIBLE or this->stat == ScaLP::status::INFEASIBLE) {
				this->timeouts++;
			}
			this->scheduleFound = stat == ScaLP::status::OPTIMAL | stat == ScaLP::status::FEASIBLE | stat == ScaLP::status::TIMEOUT_FEASIBLE;
			this->optimalResult = stat == ScaLP::status::OPTIMAL;
			if (not this->scheduleFound) {
				// schedule attempt failed :(
				// let's try again for the next II :)
				continue;
			}
			this->II = this->candII;
			this->fillSolutionStructure();
			this->printScheduleAndBinding();
			break;
		}*/
	}

	void MinRegMultiScheduler::createScaLPVariables() {
		if (!this->quiet) {
			std::cout << "MinRegMultiScheduler: start creating ScaLP variables" << std::endl;
		}
		// std::map<Vertex*, std::vector<ScaLP::Variable>> t;
		// std::map<Vertex*, std::vector<ScaLP::Variable>> y;
		// std::map<std::pair<Vertex*,int>, std::vector<ScaLP::Variable>> b;
		// std::map<std::pair<Vertex*,int>, std::vector<ScaLP::Variable>> bHat;
		// std::map<std::pair<Vertex*, int>, std::vector<ScaLP::Variable>> rho;
		// std::map<std::pair<Vertex*, int>, std::vector<ScaLP::Variable>> rhoHat;
		for (auto &v : this->g.Vertices()) {
			auto *res = this->resourceModel.getResource(v);
			auto lim = res->getLimit();
			if (res->isUnlimited()) {
				lim = this->resourceModel.getNumVerticesRegisteredToResource(res);
			}
			for (int x=0; x<this->numberOfPossibleStartTimes.at(v); x++) {
				// t
				std::stringstream name;
				name << "t_" << v->getName() << "_" << x;
				this->t[v].emplace_back(ScaLP::newIntegerVariable(name.str(), 0, ScaLP::INF()));
				this->ilpVariableCounter++;
				name.str("");
				// z
				name << "z_" << v->getName() << "_" << x;
				this->z[v].emplace_back(ScaLP::newIntegerVariable(name.str(), 0, ScaLP::INF()));
				this->ilpVariableCounter++;
				name.str("");
				for (int tau=0; tau<(int)this->candII; tau++) {
					// b
					name << "b_" << v->getName() << "_" << tau << "_" << x;
					this->b[{v, tau}].emplace_back(ScaLP::newBinaryVariable(name.str()));
					this->ilpVariableCounter++;
					name.str("");
					if (this->numberOfPossibleStartTimes.at(v) == 1) {
						// multi scheduling disabled for this operation
						continue;
					}
					// multi scheduling enabled for this operation
					// bHat
					name << "bHat_" << v->getName() << "_" << tau << "_" << x;
					this->bHat[{v, tau}].emplace_back(ScaLP::newBinaryVariable(name.str()));
					this->ilpVariableCounter++;
					name.str("");
				}
				for (int l=0; l<lim; l++) {
					// rho
					name << "rho_" << v->getName() << "_" << l << "_" << x;
					this->rho[{v, l}].emplace_back(ScaLP::newBinaryVariable(name.str()));
					this->ilpVariableCounter++;
					name.str("");
					if (this->numberOfPossibleStartTimes.at(v) == 1) {
						// multi scheduling disabled for this operation
						continue;
					}
					// multi scheduling enabled for this operation
					// rhoHat
					name << "rhoHat_" << v->getName() << "_" << l << "_" << x;
					this->rhoHat[{v, l}].emplace_back(ScaLP::newBinaryVariable(name.str()));
					this->ilpVariableCounter++;
					name.str("");
				}
			}
		}
		// std::map<Edge*, std::vector<ScaLP::Variable>> n;
		// std::map<Edge*, std::vector<ScaLP::Variable>> s;
		for (auto &e : this->g.Edges()) {
			if (not e->isDataEdge()) {
				continue;
			}
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			for (auto x=0; x<this->numberOfPossibleStartTimes.at(vSrc); x++) {
				// n
				std::stringstream name;
				name << "n_" << vSrc->getName() << "_" << vDst->getName() << "_" << x;
				this->n[e].emplace_back(ScaLP::newIntegerVariable(name.str(), -ScaLP::INF(), this->bigM));
				this->ilpVariableCounter++;
				name.str("");
				if (this->numberOfPossibleStartTimes.at(vSrc) == 1) {
					// multi scheduling disabled for this operation
					continue;
				}
				// multi scheduling enabled for this operation
				// s
				name << "s_" << vSrc->getName() << "_" << vDst->getName() << "_" << x;
				this->s[e].emplace_back(ScaLP::newBinaryVariable(name.str()));
				this->ilpVariableCounter++;
				name.str("");
			}
		}
		// std::map<std::pair<Resource*, int>, ScaLP::Variable> xi;
		for (auto &res : this->resourceModel.Resources()) {
			auto lim = res->getLimit();
			if (res->isUnlimited()) {
				lim = this->resourceModel.getNumVerticesRegisteredToResource(res);
			}
			for (int l=0; l<lim; l++) {
				// xi
				std::stringstream name;
				name << "xi_" << res->getName() << "_" << l;
				this->xi[{res, l}] = ScaLP::newIntegerVariable(name.str(), 0, ScaLP::INF());
				this->ilpVariableCounter++;
				name.str("");
			}
		}
	}

	void MinRegMultiScheduler::constructProblem() {
		if (!this->quiet) {
			std::cout << "MinRegMultiScheduler: start constructing problem" << std::endl;
		}
		// max latency
		// link t and y/b variables
		// assign exactly 1 congruence class
		// congruence class selection 1
		// congruence class selection 2
		// assign exactly 1 FU binding
		// binding selection 1
		// binding selection 2
		for (auto &v : this->g.Vertices()) {
			auto *res = this->resourceModel.getResource(v);
			auto lat = res->getLatency();
			auto lim = res->getLimit();
			if (res->isUnlimited()) {
				lim = this->resourceModel.getNumVerticesRegisteredToResource(res);
			}
			for (int x = 0; x < this->numberOfPossibleStartTimes.at(v); x++) {
				// max latency
				if (this->maxLatencyConstraint >= 0) {
					this->solver->addConstraint(this->t.at(v)[x] <= this->maxLatencyConstraint - lat);
					this->ilpConstraintCounter++;
				}
				ScaLP::Term bSum;
				ScaLP::Term bWeightedSum;
				for (int tau=0; tau<(int)this->candII; tau++) {
					bSum += this->b.at({v, tau})[x];
					bWeightedSum += (tau * this->b.at({v, tau})[x]);
					if (this->numberOfPossibleStartTimes.at(v) == 1) {
						// multi scheduling disabled for this operation
						continue;
					}
					// multi scheduling enabled for this operation
					for (auto &e : this->outgoingEdges.at(v)) {
						// congruence class selection 1
						this->solver->addConstraint(this->s.at(e)[x] + this->b.at({v, tau})[x] - (2*this->bHat.at({v, tau})[x]) >= 0);
						this->ilpConstraintCounter++;
						// congruence class selection 2
						this->solver->addConstraint(this->s.at(e)[x] + this->b.at({v, tau})[x] - (2*this->bHat.at({v, tau})[x]) <= 1);
						this->ilpConstraintCounter++;
					}
				}
				// link t and y/b variables
				this->solver->addConstraint(this->t.at(v)[x] - bWeightedSum - this->candII * this->z.at(v)[x] == 0);
				this->ilpConstraintCounter++;
				// assign exactly 1 congruence class
				this->solver->addConstraint(bSum == 1);
				this->ilpConstraintCounter++;
				ScaLP::Term rhoSum;
				for (int l=0; l<lim; l++) {
					rhoSum += this->rho.at({v, l})[x];
					if (this->numberOfPossibleStartTimes.at(v) == 1) {
						// multi scheduling disabled for this operation
						continue;
					}
					// multi scheduling enabled for this operation
					for (auto &e : this->outgoingEdges.at(v)) {
						// binding selection 1
						this->solver->addConstraint(this->s.at(e)[x] + this->rho.at({v, l})[x] - (2*this->rhoHat.at({v, l})[x]) >= 0);
						this->ilpConstraintCounter++;
						// binding selection 2
						this->solver->addConstraint(this->s.at(e)[x] + this->rho.at({v, l})[x] - (2*this->rhoHat.at({v, l})[x]) <= 1);
						this->ilpConstraintCounter++;
					}
				}
				// assign exactly 1 FU binding
				this->solver->addConstraint(rhoSum == 1);
				this->ilpConstraintCounter++;
			}
		}
		// do not exceed resource limits
		for (auto &res : this->resourceModel.Resources()) {
			auto lim = res->getLimit();
			if (res->isUnlimited()) {
				lim = this->resourceModel.getNumVerticesRegisteredToResource(res);
			}
			for (int tau=0; tau<(int)this->candII; tau++) {
				ScaLP::Term resSum;
				for (auto &v : this->g.Vertices()) {
					if (this->resourceModel.getResource(v) != res) {
						continue;
					}
					for (int x=0; x<this->numberOfPossibleStartTimes.at(v); x++) {
						resSum += this->b.at({v, tau})[x];
					}
				}
				this->solver->addConstraint(resSum <= lim);
				this->ilpConstraintCounter++;
			}
		}
		// each FU can execute at most 1 operation per time step
		for (auto &res : this->resourceModel.Resources()) {
			auto lim = res->getLimit();
			if (res->isUnlimited()) {
				lim = this->resourceModel.getNumVerticesRegisteredToResource(res);
			}
			for (auto &v1 : this->g.Vertices()) {
				if (this->resourceModel.getResource(v1) != res) {
					continue;
				}
				for (auto &v2 : this->g.Vertices()) {
					if (v1 == v2) {
						continue;
					}
					if (this->resourceModel.getResource(v2) != res) {
						continue;
					}
					for (int l=0; l<lim; l++) {
						for (int x = 0; x < this->numberOfPossibleStartTimes.at(v1); x++) {
							for (int y = 0; y < this->numberOfPossibleStartTimes.at(v2); y++) {
								for (int tau=0; tau<(int)this->candII; tau++) {
									ScaLP::Variable b1;
									ScaLP::Variable b2;
									ScaLP::Variable rho1;
									ScaLP::Variable rho2;
									if (this->numberOfPossibleStartTimes.at(v1) == 1) {
										// multi scheduling disabled for this operation
										b1 = this->b.at({v1, tau})[x];
										rho1 = this->rho.at({v1, l})[x];
									}
									else {
										// multi scheduling enabled for this operation
										b1 = this->bHat.at({v1, tau})[x];
										rho1 = this->rhoHat.at({v1, l})[x];
									}
									if (this->numberOfPossibleStartTimes.at(v2) == 1) {
										// multi scheduling disabled for this operation
										b2 = this->b.at({v2, tau})[y];
										rho2 = this->rho.at({v2, l})[y];
									}
									else {
										// multi scheduling enabled for this operation
										b2 = this->bHat.at({v2, tau})[y];
										rho2 = this->rhoHat.at({v2, l})[y];
									}
									// each FU can execute at most 1 operation per time step
									this->solver->addConstraint(rho1 + rho2 + b1 + b2 <= 3);
									this->ilpConstraintCounter++;
								}
							}
						}
					}
				}
			}
		}
		// assign exactly 1 source selection
		// edge lifetimes
		for (auto &e : this->g.Edges()) {
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			ScaLP::Term sourceSelectionSum;
			for (int x = 0; x < this->numberOfPossibleStartTimes.at(vSrc); x++) {
				if (e->isDataEdge() and (this->numberOfPossibleStartTimes.at(vSrc) > 1)) {
					sourceSelectionSum += this->s.at(e)[x];
				}
				for (int y = 0; y < this->numberOfPossibleStartTimes.at(vDst); y++) {
					// edge lifetimes
					if (e->isDataEdge()) {
						if (this->numberOfPossibleStartTimes.at(vSrc) == 1) {
							this->solver->addConstraint(this->n.at(e)[x] - this->t.at(vDst)[y] + this->t.at(vSrc)[x] == (e->getDistance() * this->candII) + this->resourceModel.getVertexLatency(vSrc));
							this->ilpConstraintCounter++;
							this->solver->addConstraint(this->n.at(e)[x] >= 0);
							this->ilpConstraintCounter++;
						}
						else {
							this->solver->addConstraint(this->n.at(e)[x] - this->t.at(vDst)[y] + this->t.at(vSrc)[x] - (1-this->s.at(e)[x])*this->bigM == (e->getDistance() * this->candII) + this->resourceModel.getVertexLatency(vSrc));
							this->ilpConstraintCounter++;
							this->solver->addConstraint(this->n.at(e)[x] + (1-this->s.at(e)[x])*this->bigM >= 0);
							this->ilpConstraintCounter++;
						}
					}
					else {
						if (this->numberOfPossibleStartTimes.at(vSrc) == 1) {
							this->solver->addConstraint(this->t.at(vDst)[y] - this->t.at(vSrc)[x] >= e->getDelay() + this->resourceModel.getVertexLatency(vSrc));
							this->ilpConstraintCounter++;
						}
						else {
							this->solver->addConstraint(this->t.at(vDst)[y] - this->t.at(vSrc)[x] + (1-this->s.at(e)[x])*this->bigM >= e->getDelay() + this->resourceModel.getVertexLatency(vSrc));
							this->ilpConstraintCounter++;
						}
					}
				}
			}
			// assign exactly 1 source selection
			if (e->isDataEdge() and this->numberOfPossibleStartTimes.at(vSrc) > 1) {
				this->solver->addConstraint(sourceSelectionSum == 1);
				this->ilpConstraintCounter++;
			}
		}
		// number of lifetime registers
		for (auto &res : this->resourceModel.Resources()) {
			auto lim = res->getLimit();
			if (res->isUnlimited()) {
				lim = this->resourceModel.getNumVerticesRegisteredToResource(res);
			}
			for (int l=0; l<lim; l++) {
				for (auto &e : this->g.Edges()) {
					if (not e->isDataEdge()) {
						continue;
					}
					auto *vSrc = &e->getVertexSrc();
					if (this->resourceModel.getResource(vSrc) != res) {
						continue;
					}
					for (int x=0; x<this->numberOfPossibleStartTimes.at(vSrc); x++) {
						// number of lifetime registers
						if (this->numberOfPossibleStartTimes.at(vSrc) == 1) {
							this->solver->addConstraint(this->xi.at({res, l}) - this->n.at(e)[x] + ((1-this->rho.at({vSrc, l})[x])*2*this->bigM) >= 0);
							this->ilpConstraintCounter++;
						}
						else {
							this->solver->addConstraint(this->xi.at({res, l}) - this->n.at(e)[x] + ((1-this->rho.at({vSrc, l})[x])*2*this->bigM) + ((1-this->s.at(e)[x])*2*this->bigM) >= 0);
							this->ilpConstraintCounter++;
						}
					}
				}
			}
		}
	}

	void MinRegMultiScheduler::setObjective() {
		if (!this->quiet) {
			std::cout << "MinRegMultiScheduler: start setting objective" << std::endl;
		}
		ScaLP::Term obj;
		for (auto &it : this->xi) {
			obj += it.second;
		}
		this->solver->setObjective(ScaLP::minimize(obj));
	}

	void MinRegMultiScheduler::resetContainer() {
		if (!this->quiet) {
			std::cout << "MinRegMultiScheduler: start resetting containers" << std::endl;
		}
		// reset counters
		this->ilpConstraintCounter = 0;
		this->ilpVariableCounter = 0;
		// clear ScaLP variable containers
		this->t.clear();
		this->z.clear();
		this->b.clear();
		this->bHat.clear();
		this->n.clear();
		this->xi.clear();
		this->rho.clear();
		this->rhoHat.clear();
		this->s.clear();
		// re-compute containers that depend on the candidate II
		// compute number of possible start times
		for (auto &v : this->g.Vertices()) {
			this->numberOfPossibleStartTimes[v] = this->computeNumberOfPossibleStartTimes(v);
		}
		// compute value for bigM variable
		this->bigM = 0.0;
		for (auto &v : this->g.Vertices()) {
			this->bigM += (this->resourceModel.getVertexLatency(v) + this->candII);
		}
	}

	void MinRegMultiScheduler::setUpSolver() {
		if (!this->quiet) {
			std::cout << "MinRegMultiScheduler: start setting up solver" << std::endl;
		}
		this->solver->reset();
		if (this->solverTimeout > 0) {
			this->solver->timeout = this->solverTimeout;
		}
		this->solver->threads = (int)this->threads;
		this->solver->quiet = this->solverQuiet;
	}

	void MinRegMultiScheduler::disableMultipleStartTimes() {
		this->multipleStartTimesAllowed = false;
	}

	void MinRegMultiScheduler::enableMultipleStartTimes() {
		this->multipleStartTimesAllowed = true;
	}

	int MinRegMultiScheduler::computeNumberOfPossibleStartTimes(Vertex *v) {
		auto *r = this->resourceModel.getResource(v);
		if (r->isUnlimited()) {
			// forbid multi-computations for unlimited resources
			return 1;
		}
		if (r->getLimit() * this->candII == this->resourceModel.getNumVerticesRegisteredToResource(r)) {
			// multi-computations are impossible if the MRT is already at its capacity limit
			return 1;
		}
		if (this->outgoingEdges[v].empty()) {
			// output nodes should only get assigned exactly once
			return 1;
		}
		return (int)this->outgoingEdges[v].size();
	}

	void MinRegMultiScheduler::initScheduler() {
		if (!this->quiet) {
			std::cout << "MinRegMultiScheduler: start initializing scheduler" << std::endl;
		}
		// II bounds
		if(this->maxRuns > 0) {
			int runs = (int)(this->maxII - this->minII);
			if(runs > this->maxRuns) {
				this->maxII = ((int)this->minII + this->maxRuns - 1);
			}
			if(!this->quiet) {
				std::cout << "MinRegMultiScheduler: maxII changed due to maxRuns value set by user!" << endl;
				std::cout << "MinRegMultiScheduler: min/maxII = " << this->minII << " " << this->maxII << std::endl;
			}
		}
		if (this->minII > this->maxII) {
			throw Exception("Inconsistent II bounds");
		}
		// compute incoming/outgoing edges
		for (auto &e : this->g.Edges()) {
			if (not e->isDataEdge()) {
				// skip chaining edges for this
				continue;
			}
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			this->outgoingEdges[vSrc].insert(e);
			this->incomingEdges[vDst].insert(e);
		}
	}

	void MinRegMultiScheduler::fillSolutionStructure() {
		if (!this->quiet) {
			std::cout << "MinRegMultiScheduler: start filling solution structure" << std::endl;
		}
		// reset solution containers
		this->startTimes.clear();
		this->bindings.clear();
		this->numLifetimeRegs = 0;
		// get result values from solver
		auto results = this->solver->getResult().values;
		if (!this->quiet) {
			std::cout << "ILP solver results:" << std::endl;
			for (auto &it : results) {
				std::cout << "  " << it.first->getName() << " - " << it.second << " (" << round(it.second) << ")" << std::endl;
			}
		}
		// retrieve schedule times
		for (auto &v : this->g.Vertices()) {
			for (int x=0; x<this->numberOfPossibleStartTimes.at(v); x++) {
				// check if any source selection for this x was set (if there are any)
				bool srcSelectionSet = this->outgoingEdges.at(v).empty();
				for (auto &e : this->outgoingEdges.at(v)) {
					if (this->numberOfPossibleStartTimes.at(v) > 1) {
						if (round(results.at(this->s.at(e)[x])) != 1) {
							continue;
						}
					}
					srcSelectionSet = true;
				}
				if (not srcSelectionSet) {
					continue;
				}
				// startTimeMapping[{v, x}] = this->startTimes[v].size();
				this->startTimes[v].emplace_back((int)round(results.at(this->t.at(v)[x])));
			}
			if (this->startTimes[v].empty()) {
				std::cout << "MinRegMultiScheduler: didn't assign vertex '" << v->getName() << "' a start time - this should never happen" << std::endl;
			}
		}
		// retrieve binding info
		for (auto &v : this->g.Vertices()) {
			auto *res = this->resourceModel.getResource(v);
			auto lim = res->getLimit();
			if (res->isUnlimited()) {
				lim = this->resourceModel.getNumVerticesRegisteredToResource(res);
			}
			for (int x=0; x<this->numberOfPossibleStartTimes.at(v); x++) {
				// check if any source selection for this x was set (if there are any)
				bool srcSelectionSet = this->outgoingEdges.at(v).empty();
				for (auto &e : this->outgoingEdges.at(v)) {
					if (this->numberOfPossibleStartTimes.at(v) > 1) {
						if (round(results.at(this->s.at(e)[x])) != 1) {
							continue;
						}
					}
					srcSelectionSet = true;
				}
				if (not srcSelectionSet) {
					continue;
				}
				for (int l=0; l<lim; l++) {
					// bindingMapping[{v, x}] = this->bindings[v].size();
					if ((int)round(results.at(this->rho.at({v, l})[x])) != 1) {
						continue;
					}
					this->bindings[v].emplace_back(l);
				}
			}
			if (this->bindings[v].empty()) {
				std::cout << "MinRegMultiScheduler: didn't assign vertex '" << v->getName() << "' a binding - this should never happen" << std::endl;
			}
		}
		// retrieve number of lifetime registers
		for (auto &res : this->resourceModel.Resources()) {
			auto lim = res->getLimit();
			if (res->isUnlimited()) {
				lim = this->resourceModel.getNumVerticesRegisteredToResource(res);
			}
			for (int l=0; l<lim; l++) {
				this->numLifetimeRegs += (int)results.at(this->xi.at({res,l}));
			}
		}
	}

	int MinRegMultiScheduler::getNumLifetimeRegs() const {
		return this->numLifetimeRegs;
	}

	bool MinRegMultiScheduler::validateScheduleAndBinding() const {
		if (!this->quiet) {
			std::cout << "MinRegMultiScheduler: start validating schedule & binding" << std::endl;
		}
		// let's be optimistic
		bool valid = true;
		// check resource occupations
		// check if each vertex has at least one schedule time and binding associated
		std::map<std::tuple<const Resource*, int, int>, std::set<Vertex*>> isOccupied;
		for (auto &v : this->g.Vertices()) {
			std::vector<int> vertexStartTimes;
			std::vector<int> vertexBindings;
			try {
				vertexStartTimes = this->startTimes.at(v);
				vertexBindings = this->bindings.at(v);
			}
			catch (std::out_of_range&) {
				std::cout << "start times or bindings for vertex '" << v->getName() << "' not set" << std::endl;
				return false;
			}
			if (vertexStartTimes.empty()) {
				std::cout << "start times container for vertex '" << v->getName() << "' is empty" << std::endl;
				return false;
			}
			if (vertexBindings.empty()) {
				std::cout << "bindings container for vertex '" << v->getName() << "' is empty" << std::endl;
				return false;
			}
			if (vertexStartTimes.size() != vertexBindings.size()) {
				std::cout << "start times and bindings container for vertex '" << v->getName() << "' have different sizes: '"
				  << vertexStartTimes.size() << "' != '" << vertexBindings.size() << "'" << std::endl;
				return false;
			}
			auto res = this->resourceModel.getResource(v);
			auto numScheduleTimes = (int) vertexStartTimes.size();
			for (int x=0; x<numScheduleTimes; x++) {
				auto vertexStartTime = vertexStartTimes[x];
				auto vertexBinding = vertexBindings[x];
				auto moduloSlot = vertexStartTime % (int)this->II;
				std::tuple<const Resource*, int, int> keyTuple = {res, vertexBinding, moduloSlot};
				isOccupied[keyTuple].insert(v);
			}
		}
		for (auto &it : isOccupied) {
			auto res = std::get<0>(it.first);
			auto fuIdx = std::get<1>(it.first);
			auto moduloSlot = std::get<2>(it.first);
			if (it.second.size() > 1) {
				std::cout << "resource '" << res->getName() << "' (" << fuIdx
									<< ") has multiple assignments in modulo slot " << moduloSlot << ":" << std::endl;
				for (auto &v : it.second) {
					std::cout << "  " << v->getName() << std::endl;
				}
				valid = false;
			}
		}
		// check data dependencies
		// count minimum number of needed lifetime regs
		std::map<std::tuple<const Resource*, int>, int> minLifetimeRegs;
		for (auto &e : this->g.Edges()) {
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			auto tSrcs = this->startTimes.at(vSrc);
			auto tDsts = this->startTimes.at(vDst);
			auto bSrcs = this->bindings.at(vSrc);
			auto bDsts = this->bindings.at(vDst);
			auto numSrc = (int) tSrcs.size();
			auto numDst = (int) tDsts.size();
			auto lSrc = this->resourceModel.getVertexLatency(vSrc);
			for (int y=0; y<numDst; y++) {
				auto tDst = tDsts.at(y);
				auto latestSrcTime = -1;
				auto latestSrcTimeIdx = -1;
				bool chainingValid = false;
				for (int x=0; x<numSrc; x++) {
					auto tSrc = tSrcs.at(x);
					if (e->isDataEdge()) {
						if (tDst + e->getDistance()*this->II - (tSrc + lSrc) < 0) {
							// dependency violated
							continue;
						}
						if (tSrc <= latestSrcTime) {
							// not latest valid start time
							continue;
						}
						// update latest valid start time
						latestSrcTime = tSrc;
						latestSrcTimeIdx = x;
					}
					else {
						if (tDst - (tSrc + lSrc + e->getDelay()) < 0) {
							// dependency violated
							continue;
						}
						chainingValid = true;
					}
				}
				if ((latestSrcTime < 0) and e->isDataEdge()) {
					std::cout << "data dependency violated for edge '" << vSrc->getName() << "' -(" << e->getDistance() << ")-> '"
										<< vDst->getName() << "'" << std::endl;
					valid = false;
				}
				if ((not chainingValid) and (not e->isDataEdge())) {
					std::cout << "chaining dependency violated for edge '" << vSrc->getName() << "' -(" << e->getDelay()
					  << ")-> '" << vDst->getName() << "'" << std::endl;
					valid = false;
				}
				if (!e->isDataEdge()) {
					continue;
				}
				auto life = tDst + e->getDistance()*this->II - (latestSrcTime + lSrc);
				auto fuSrcIdx = this->bindings.at(vSrc)[latestSrcTimeIdx];
				std::pair<const Resource*, int> p = {this->resourceModel.getResource(vSrc), fuSrcIdx};
				if (minLifetimeRegs[p] < life) minLifetimeRegs[p] = life;
			}
		}
		int minNumLifetimeRegs = 0;
		for (auto &it : minLifetimeRegs) {
			minNumLifetimeRegs += it.second;
		}
		if (minNumLifetimeRegs != this->numLifetimeRegs) {
			std::cout << "counted lifetime regs = " << minNumLifetimeRegs << " but ILP solver reported "
			  << this->numLifetimeRegs << std::endl;
			valid = false;
		}
		return valid;
	}

	void MinRegMultiScheduler::printScheduleAndBinding() const {
		std::cout << "MinRegMultiScheduler: start printing schedule times and bindings" << std::endl;
		for (auto &v : this->g.Vertices()) {
			std::cout << "  Vertex '" << v->getName() << "' of resource type '" << this->resourceModel.getResource(v)->getName() << "'" << std::endl;
			try {
				auto numStartTimes = (int)this->startTimes.at(v).size();
				if (numStartTimes == 0) {
					std::cout << "    *no start time/binding assigned*" << std::endl;
					continue;
				}
				for (int i=0; i<numStartTimes; i++) {
					std::cout << "    t=" << this->startTimes.at(v).at(i) << " b=" << this->bindings.at(v).at(i) << std::endl;
				}
			}
			catch (std::out_of_range&) {
				std::cout << "    *no start time/binding assigned*" << std::endl;
				continue;
			}
		}
	}

  void MinRegMultiScheduler::scheduleInit() {
      this->initScheduler();
      this->timeouts = 0;
      this->solvingTimeTotal = 0.0;
  }

  void MinRegMultiScheduler::scheduleIteration() {
      if (!this->quiet) {
          std::cout << "MinRegMultiScheduler: candidate II=" << this->candII << std::endl;
      }
      this->resetContainer();
      this->setUpSolver();
      this->createScaLPVariables();
      this->constructProblem();
      this->setObjective();
      if (!this->quiet) {
          std::cout << "MinRegMultiScheduler: start scheduling with '" << this->ilpVariableCounter
                    << "' variables and '"
                    << this->ilpConstraintCounter << "' constraints" << std::endl;
          std::cout << "MinRegMultiScheduler: start scheduling with ILP formulation:" << std::endl;
          std::cout << this->solver->showLP() << std::endl;
      }
      startTimeTracking();
      this->stat = this->solver->solve();
      endTimeTracking();
      if (!this->quiet) {
          std::cout << "MinRegMultiScheduler: finished solving with status '" << ScaLP::showStatus(this->stat)
                    << "' after " << solvingTimePerIteration << " sec (total: " << this->solvingTimeTotal << " sec)" << std::endl;
      }
      if(this->stat == ScaLP::status::TIMEOUT_INFEASIBLE or this->stat == ScaLP::status::INFEASIBLE) {
          this->timeouts++;
      }
      this->scheduleFound = stat == ScaLP::status::OPTIMAL | stat == ScaLP::status::FEASIBLE | stat == ScaLP::status::TIMEOUT_FEASIBLE;
      this->optimalResult = stat == ScaLP::status::OPTIMAL;
      if (not this->scheduleFound) {
          // schedule attempt failed :(
          // let's try again for the next II :)
          return;
      }
      this->II = this->candII;
      this->fillSolutionStructure();
      this->printScheduleAndBinding();
  }

  void MinRegMultiScheduler::setSolverTimeout(double timeoutInSeconds) {
      this->solverTimeout = timeoutInSeconds;
      solver->timeout = (long)timeoutInSeconds;
      if (!this->quiet)
      {
          cout << "Solver Timeout set to " << this->solver->timeout << " seconds." << endl;
      }
  }

}