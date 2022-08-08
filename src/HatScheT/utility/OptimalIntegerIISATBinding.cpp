//
// Created by nfiege on 7/1/22.
//


#ifdef USE_CADICAL

#include "OptimalIntegerIISATBinding.h"
#include <sstream>
#include <cmath>
#include <algorithm>
#include <HatScheT/utility/Verifier.h>

namespace HatScheT {
	OptimalIntegerIISATBinding::OptimalIntegerIISATBinding(Graph *g, ResourceModel *rm,
																												 const std::map<Vertex *, int> &sched, const int &II,
																												 const std::map<Edge *, int> &portAssignments,
																												 const std::set<const Resource *> &commutativeOps) :
	BindingBase(g, rm, sched, II, commutativeOps), portAssignments(portAssignments),
	firstObjective(firstObjectiveMuxMin_t), terminator(0.0), elapsedTime(0.0), variableCounter(0),
	clauseCounter(0), operationBindingVarCounter(0), connectionBindingVarCounter(0), lifetimeRegBindingVarCounter(0),
	edgePortBindingVarCounter(0), operationBindingClauseCounter(0), noBindingConflictsClauseCounter(0),
	registerBindingClauseCounter(0), noRegisterConflictsClauseCounter(0), interconnectBindingClauseCounter(0),
	noInterconnectConflictsClauseCounter(0), portBindingClauseCounter(0), edgeBindingClauseCounter(0),
	noPortBindingConflictsClauseCounter(0) {
		// check if port assignments are complete
		for(auto e : this->g->Edges()) {
			try {
				// chaining edges do not need a port assignment
				if (!e->isDataEdge()) continue;
				// check if port assignment is set
				this->portAssignments.at(e);
			}
			catch (std::out_of_range&) {
				// throw error for missing port assignment
				auto vSrc = e->getVertexSrcName();
				auto vDst = e->getVertexDstName();
				throw HatScheT::Exception("Missing port assignment for edge '"+vSrc+"' -> '"+vDst+"'");
			}
		}
	}

	void OptimalIntegerIISATBinding::bind() {
		//////////
		// init //
		//////////
		this->terminator.reset(this->timeBudget);
		this->elapsedTime = 0.0;
		this->solver = std::unique_ptr<CaDiCaL::Solver>(new CaDiCaL::Solver);
		this->solver->connect_terminator(&this->terminator);

		//////////////////////
		// check for errors //
		//////////////////////
		// edges are missing in port assignment container
		for(auto e : this->g->Edges()) {
			try {
				this->portAssignments.at(e);
			}
			catch(...) {
				throw HatScheT::Exception("Binding::OptimalIntegerIISATBinding: Edge '"+e->getVertexSrcName()+"' -> '"+e->getVertexDstName()+"' missing in port assignment container");
			}
		}

		/////////////////////////
		// container to return //
		/////////////////////////
		this->binding = Binding::RegChainBindingContainer();

		/////////////////////////////////////////////////
		// count if less FUs are needed than allocated //
		/////////////////////////////////////////////////
		this->resourceLimits.clear();
		for(auto &r : this->rm->Resources()) {
			resourceLimits[r] = 0;
			if(r->isUnlimited()) {
				resourceLimits[r] = this->rm->getVerticesOfResource(r).size();
				if(!this->quiet) std::cout << "Resource limits for resource '" << r->getName() << "': " << resourceLimits[r] << std::endl;
				continue;
			}
			std::map<int,int> modSlotHeight;
			for(int i=0; i<this->II; i++) {
				modSlotHeight[i] = 0;
			}
			for(auto &it : this->sched) {
				auto v = it.first;
				if(this->rm->getResource(v) != r) continue;
				auto modSlot = it.second % this->II;
				modSlotHeight[modSlot]++;
				if(!this->quiet) {
					std::cout << "Mod slot for vertex '" << v->getName() << "' = " << modSlot << " = " << it.second << " mod "
										<< this->II << std::endl;
					std::cout << "  mod slot height = " << modSlotHeight[modSlot] << std::endl;
				}
			}
			for(auto it : modSlotHeight) {
				if(it.second > resourceLimits[r]) resourceLimits[r] = it.second;
			}
			if(!this->quiet) std::cout << "Resource limits for resource '" << r->getName() << "': " << resourceLimits[r] << std::endl;
		}

		/////////////////////////////////////////////
		// count number of ports for each resource //
		/////////////////////////////////////////////
		this->numResourcePorts.clear();
		for(auto &r : this->rm->Resources()) {
			auto vertices = this->rm->getVerticesOfResource(r);
			int numPorts = 0;
			for(auto &v : vertices) {
				int numInputs = 0;
				for(auto &e : this->g->Edges()) {
					if(!e->isDataEdge()) continue; // skip chaining edges
					if(&e->getVertexDst() != v) continue;
					numInputs++;
				}
				if(numInputs > numPorts) numPorts = numInputs;
			}
			numResourcePorts[r] = numPorts;
		}
		if(!this->quiet) std::cout << "counted number of ports for each resource" << std::endl;

		//////////////////////////////////
		// check if ports are left open //
		//////////////////////////////////
		// this might happen if the portAssignment container is not filled properly
		// this is not supported here
		for(auto vj : this->g->Vertices()) {
			auto rj = this->rm->getResource(vj);
			std::set<int> foundPorts;
			if (!this->quiet) {
				std::cout << "checking ports of vertex '" << vj->getName() << "'" << std::endl;
			}
			for(auto it : this->portAssignments) {
				auto e = it.first;
				auto port = it.second;
				if(&e->getVertexDst() != vj) continue;
				if (!this->quiet) {
					std::cout << "  found connection from '" << e->getVertexSrcName() << "' (" << &e->getVertexSrc() << ") to '" << e->getVertexDstName() << "' (" << &e->getVertexDst() << ") port no. " << port << " (edge " << e << ")" << std::endl;
				}
				if(port >= numResourcePorts[rj]) {
					std::stringstream exc;
					exc << "Binding::getILPBasedIntIIBinding: Illegal port number (" << port << ") provided for edge from "
							<< e->getVertexSrc().getName() << " to " << e->getVertexDst().getName() << " - resource " << rj->getName()
							<< " only has " << numResourcePorts[rj] << " ports";
					throw HatScheT::Exception(exc.str());
				}
				foundPorts.insert(port);
			}
			if(foundPorts.size() != numResourcePorts[rj]) {
				std::stringstream exc;
				exc << "Binding::getILPBasedIntIIBinding: not all ports are occupied for vertex " << vj->getName()
						<< ", found ports: " << foundPorts.size() << ", required ports: " << numResourcePorts[rj]
						<< " - open ports are not supported yet";
				throw HatScheT::Exception(exc.str());
			}
		}
		if(!this->quiet) std::cout << "checked if ports are left open" << std::endl;

		///////////////////////////////////////////////////
		// assign each FU of each resource type a number //
		///////////////////////////////////////////////////
		this->fuIndexMap.clear();
		this->indexFuMap.clear();
		int fuIndexCounter = 0;
		for(auto &r : this->rm->Resources()) {
			auto verticesOfResource = this->rm->getVerticesOfResource(r);
			auto lim = resourceLimits[r];
			for(auto i=0; i<lim; i++) {
				auto p = make_pair(r,i);
				fuIndexMap[p] = fuIndexCounter;
				indexFuMap[fuIndexCounter] = p;
				if (!r->isUnlimited()) {
					// store possible resource bindings in case of a limited resource
					// bindings for unlimited FUs are computed later
					for (auto &v : verticesOfResource) {
						this->possibleResourceBindings[v].insert(fuIndexCounter);
						this->operationsInModSlots[{r, i, this->sched[const_cast<Vertex*>(v)] % this->II}].insert(const_cast<Vertex*>(v));
					}
				}
				fuIndexCounter++;
			}
		}
		if(!this->quiet) std::cout << "assigned all fus a number" << std::endl;

		///////////////////////////////////////////////////////////////
		// check possible edgeLifetimes for each fu -> fu connection //
		///////////////////////////////////////////////////////////////
		this->possibleLifetimes.clear();
		this->edgeLifetimes.clear();
		this->variableLifetimes.clear();
		for(auto &e : this->g->Edges()) {
			auto &vi = e->getVertexSrc();
			auto &vj = e->getVertexDst();
			auto lifetime = getEdgeLifetime(e, &this->edgeLifetimes, this->sched, this->II, this->rm);
			if (variableLifetimes[&vi] < lifetime) variableLifetimes[&vi] = lifetime;
			auto *ri = this->rm->getResource(&vi);
			auto *rj= this->rm->getResource(&vj);
			auto limi = resourceLimits[ri];
			for(int a=0; a<limi; a++) {
				auto m = fuIndexMap[{ri,a}];
				auto limj = resourceLimits[rj];
				for(int b=0; b<limj; b++) {
					auto n = fuIndexMap[{rj,b}];
					possibleLifetimes[{m,n}].insert(lifetime);
				}
			}
		}
		if(!this->quiet) std::cout << "checked possible edgeLifetimes" << std::endl;

		//////////////////////////////////////////////////////////////////
		// check possible port connections for each fu -> fu connection //
		//////////////////////////////////////////////////////////////////
		this->possiblePortConnections.clear();
		for(auto &e : this->g->Edges()) {
			auto &vi = e->getVertexSrc();
			auto &vj = e->getVertexDst();
			auto *ri = this->rm->getResource(&vi);
			auto *rj= this->rm->getResource(&vj);
			auto limi = resourceLimits[ri];
			for(int a=0; a<limi; a++) {
				auto m = fuIndexMap[{ri,a}];
				auto limj = resourceLimits[rj];
				for(int b=0; b<limj; b++) {
					auto n = fuIndexMap[{rj,b}];
					if(this->commutativeOps.find(rj) == this->commutativeOps.end()) {
						// non-commutative operation
						// only the user-given port assignment is allowed
						possiblePortConnections[{m,n}].insert(this->portAssignments[e]);
					}
					else {
						// commutative operations
						// all ports are allowed
						auto ports = numResourcePorts[rj];
						for(int p=0; p<ports; p++) {
							possiblePortConnections[{m,n}].insert(p);
						}
					}
				}
			}
		}
		if(!this->quiet) std::cout << "checked possible port connections" << std::endl;

		//////////////////////////////////////////////////
		// unlimited operations are bound to unique fus //
		//////////////////////////////////////////////////
		this->unlimitedOpFUs.clear();
		for(auto &r : this->rm->Resources()) {
			if(!r->isUnlimited()) continue;
			auto vertices = this->rm->getVerticesOfResource(r);
			int fuCounter = 0;
			for(auto v : vertices) {
				auto vNoConst = const_cast<Vertex*>(v);
				unlimitedOpFUs[vNoConst] = fuCounter;
				this->possibleResourceBindings[v] = {this->fuIndexMap[{r, fuCounter}]};
				this->operationsInModSlots[{r, fuCounter, this->sched[vNoConst] % this->II}] = {vNoConst};
				fuCounter++;
			}
		}

		///////////////////////////////////////////
		// start by computing an initial binding //
		///////////////////////////////////////////
		if (!this->quiet) {
			std::cout << "OptimalIntegerIISATBinding: computing initial binding now" << std::endl;
		}
		this->computeInitialBinding();
		this->solutionStatus = "TIMEOUT_FEASIBLE";

		/////////////////////////
		// define upper bounds //
		/////////////////////////
		if (!this->quiet) {
			std::cout << "OptimalIntegerIISATBinding: defining upper bounds now" << std::endl;
		}
		this->interconnectBounds.second = this->binding.multiplexerCosts;
		this->interconnectCommutativeBounds.second = this->binding.multiplexerCosts;
		this->registerBounds.second = this->binding.registerCosts;

		/////////////////////////
		// define lower bounds //
		/////////////////////////
		if (!this->quiet) {
			std::cout << "OptimalIntegerIISATBinding: defining lower bounds now" << std::endl;
		}
		this->defineLowerBounds();
		if (!this->quiet) {
			std::cout << "  " << this->registerBounds.first << " <= #Regs <= " << this->registerBounds.second << std::endl;
			std::cout << "  " << this->interconnectBounds.first << " <= #Connections <= " << this->interconnectBounds.second << std::endl;
			std::cout << "  " << this->interconnectCommutativeBounds.first << " <= #Connections(commutative) <= " << this->interconnectCommutativeBounds.second << std::endl;
		}

		//////////////////////////////////////////////////
		// calculate maximum lifetimes of all resources //
		//////////////////////////////////////////////////
		if (!this->quiet) {
			std::cout << "OptimalIntegerIISATBinding: calculating the maximum lifetimes of all resources now" << std::endl;
		}
		this->calcResourceLifetimes();

		///////////////////////////////////
		// define optimization procedure //
		///////////////////////////////////
		if (!this->quiet) {
			std::cout << "OptimalIntegerIISATBinding: creating optimization procedure now" << std::endl;
		}
		// each tuple consists of:
		//  - the formulation type
		//  - whether registers are limited
		//  - whether multiplexers are limited
		std::list<SATFormulationType> optimizationProcedure;
		if (this->firstObjective == firstObjectiveMuxMin_t) {
			// first objective: minimize MUX costs
			if (this->commutativeOps.empty()) {
				// there are NO commutative operations in binding problem
				optimizationProcedure = {muxMin_t, regAndMuxMin_t};
			}
			else {
				// there are commutative operations in binding problem
				optimizationProcedure = {muxMin_t, muxMinCommutative_t, regAndMuxMinCommutative_t};
			}
		}
		else {
			// first objective: minimize lifetime register costs
			if (this->commutativeOps.empty()) {
				// there are NO commutative operations in binding problem
				optimizationProcedure = {regMin_t, regAndMuxMin_t};
			}
			else {
				// there are commutative operations in binding problem
				optimizationProcedure = {regMin_t, muxMinCommutative_t, regAndMuxMinCommutative_t};
			}
		}

		////////////////////////////////////////////////////
		// go through optimization procedure and do magic //
		////////////////////////////////////////////////////
		if (!this->quiet) {
			std::cout << "OptimalIntegerIISATBinding: start optimizing now" << std::endl;
		}
		for (auto &opt : optimizationProcedure) {
			while (true) {
				/////////////////////////////
				// terminate if time is up //
				/////////////////////////////
				if (this->terminator.terminate()) break;
				if (!this->quiet) {
					std::cout << std::endl;
				}

				////////////////////
				// define problem //
				////////////////////
				int regMax;
				int connectionsMax;
				bool considerCommutativity;
				bool finishedCurrentOptimizationStep;
				if (!this->quiet) {
					std::cout << "OptimalIntegerIISATBinding: bounds for current optimization step:" << std::endl;
					std::cout << "  " << this->registerBounds.first << " <= #Regs <= " << this->registerBounds.second << std::endl;
					std::cout << "  " << this->interconnectBounds.first << " <= #Connections <= " << this->interconnectBounds.second << std::endl;
					std::cout << "  " << this->interconnectCommutativeBounds.first << " <= #Connections(commutative) <= " << this->interconnectCommutativeBounds.second << std::endl;
				}
				if (this->registerBounds.first > this->registerBounds.second) {
					throw Exception("OptimalIntegerIISATBinding: inconsistent register bounds");
				}
				if (this->interconnectBounds.first > this->interconnectBounds.second) {
					throw Exception("OptimalIntegerIISATBinding: inconsistent interconnect bounds");
				}
				if (this->interconnectCommutativeBounds.first > this->interconnectCommutativeBounds.second) {
					throw Exception("OptimalIntegerIISATBinding: inconsistent interconnect bounds for commutativity");
				}
				switch (opt) {
					case regMin_t: {
						connectionsMax = UNLIMITED;
						regMax = (int)std::floor((((double)this->registerBounds.first) + ((double)this->registerBounds.second)) * 0.5);
						considerCommutativity = false;
						finishedCurrentOptimizationStep = this->registerBounds.first == this->registerBounds.second;
						if (!this->quiet) {
							std::cout << "OptimalIntegerIISATBinding: register minimization with" << std::endl;
						}
						break;
					}
					case muxMin_t: {
						connectionsMax = (int)std::floor((((double)this->interconnectBounds.first) + ((double)this->interconnectBounds.second)) * 0.5);
						regMax = UNLIMITED;
						considerCommutativity = false;
						finishedCurrentOptimizationStep = this->interconnectBounds.first == this->interconnectBounds.second;
						if (!this->quiet) {
							std::cout << "OptimalIntegerIISATBinding: interconnect minimization with" << std::endl;
						}
						break;
					}
					case muxMinCommutative_t: {
						connectionsMax = (int)std::floor((((double)this->interconnectCommutativeBounds.first) + ((double)this->interconnectCommutativeBounds.second)) * 0.5);
						regMax = UNLIMITED;
						considerCommutativity = !this->commutativeOps.empty();
						finishedCurrentOptimizationStep = this->interconnectCommutativeBounds.first == this->interconnectCommutativeBounds.second;
						if (!this->quiet) {
							std::cout << "OptimalIntegerIISATBinding: commutative interconnect minimization with" << std::endl;
						}
						break;
					}
					case regAndMuxMin_t: {
						connectionsMax = (int)std::floor((((double)this->interconnectBounds.first) + ((double)this->interconnectBounds.second)) * 0.5);
						regMax = (int)std::floor((((double)this->registerBounds.first) + ((double)this->registerBounds.second)) * 0.5);
						considerCommutativity = false;
						finishedCurrentOptimizationStep = this->interconnectBounds.first == this->interconnectBounds.second and this->registerBounds.first == this->registerBounds.second;
						if (!this->quiet) {
							std::cout << "OptimalIntegerIISATBinding: register and interconnect minimization with" << std::endl;
						}
						break;
					}
					case regAndMuxMinCommutative_t: {
						connectionsMax = (int)std::floor((((double)this->interconnectCommutativeBounds.first) + ((double)this->interconnectCommutativeBounds.second)) * 0.5);
						regMax = (int)std::floor((((double)this->registerBounds.first) + ((double)this->registerBounds.second)) * 0.5);
						considerCommutativity = !this->commutativeOps.empty();
						finishedCurrentOptimizationStep = this->interconnectCommutativeBounds.first == this->interconnectCommutativeBounds.second and this->registerBounds.first == this->registerBounds.second;
						if (!this->quiet) {
							std::cout << "OptimalIntegerIISATBinding: register and commutative interconnect minimization with" << std::endl;
						}
						break;
					}
				}
				if (!this->quiet) {
					std::cout << "  #connections <= " << connectionsMax << std::endl;
					std::cout << "  #registers <= " << regMax << std::endl;
					std::cout << "  consider commutativity = " << considerCommutativity << std::endl;
					std::cout << "  finished = " << finishedCurrentOptimizationStep << std::endl;
				}

				////////////////////////////////////////////////
				// check if we are done with the current step //
				////////////////////////////////////////////////
				if (finishedCurrentOptimizationStep) break;

				//////////////////////
				// create variables //
				//////////////////////
				if (!this->quiet) {
					std::cout << "OptimalIntegerIISATBinding: creating variables now" << std::endl;
				}
				this->createVariables(regMax, connectionsMax, considerCommutativity);

				////////////////////
				// create clauses //
				////////////////////
				if (!this->quiet) {
					std::cout << "OptimalIntegerIISATBinding: creating clauses now" << std::endl;
				}
				this->createClauses(regMax, connectionsMax, considerCommutativity);

				///////////
				// solve //
				///////////
				if (!this->quiet) {
					std::cout << "OptimalIntegerIISATBinding: start solving now with timeout = " << this->timeBudget - this->terminator.getElapsedTime() << std::endl;
					std::cout << "  #variables = " << this->variableCounter << std::endl;
					std::cout << "    #lifetimeRegBindingVarCounter = " << this->lifetimeRegBindingVarCounter << std::endl;
					std::cout << "    #edgePortBindingVarCounter = " << this->edgePortBindingVarCounter << std::endl;
					std::cout << "    #connectionBindingVarCounter = " << this->connectionBindingVarCounter << std::endl;
					std::cout << "    #operationBindingVarCounter = " << this->operationBindingVarCounter << std::endl;
					std::cout << "  #clauses = " << this->clauseCounter << std::endl;
					std::cout << "    #noPortBindingConflictsClauseCounter = " << this->noPortBindingConflictsClauseCounter << std::endl;
					std::cout << "    #portBindingClauseCounter = " << this->portBindingClauseCounter << std::endl;
					std::cout << "    #noInterconnectConflictsClauseCounter = " << this->noInterconnectConflictsClauseCounter << std::endl;
					std::cout << "    #interconnectBindingClauseCounter = " << this->interconnectBindingClauseCounter << std::endl;
					std::cout << "    #noRegisterConflictsClauseCounter = " << this->noRegisterConflictsClauseCounter << std::endl;
					std::cout << "    #registerBindingClauseCounter = " << this->registerBindingClauseCounter << std::endl;
					std::cout << "    #noBindingConflictsClauseCounter = " << this->noBindingConflictsClauseCounter << std::endl;
					std::cout << "    #operationBindingClauseCounter = " << this->operationBindingClauseCounter << std::endl;
					std::cout << "    #edgeBindingClauseCounter = " << this->edgeBindingClauseCounter << std::endl;
				}
				auto status = this->solver->solve();
				if (!this->quiet) {
					std::cout << "Solver returned with status '" << status << "'" << std::endl;
				}

				if (status == CADICAL_SAT) {
					if (!this->quiet) {
						std::cout << "OptimalIntegerIISATBinding: SAT solver proved feasibility" << std::endl;
					}
					// fill solution structure (operator bindings)
					for (auto &v : this->g->Vertices()) {
						int b = -1;
						for (auto idx : this->possibleResourceBindings[v]) {
							if (this->solver->val(this->operationBindingVars[{v, idx}]) > 0) {
								if (b >= 0) {
									throw Exception("Determined duplicate binding for vertex '"+v->getName()+"'");
								}
								b = this->indexFuMap[idx].second;
							}
						}
						if (b == -1) {
							throw Exception("Determined no binding for vertex '"+v->getName()+"'");
						}
						this->binding.resourceBindings[v->getName()] = b;
					}
					// fill solution structure (port assignments)
					if (considerCommutativity) {
						for (auto &v : this->g->Vertices()) {
							auto *r = this->rm->getResource(v);
							if (this->commutativeOps.find(r) == this->commutativeOps.end()) continue;
							auto &numPorts = this->numResourcePorts[r];
							auto &incomingEdges = this->g->getIncomingEdges(v);
							std::set<int> foundPorts;
							for (auto &e : incomingEdges) {
								int b = -1;
								for (int p=0; p<numPorts; p++) {
									if (this->solver->val(this->edgePortBindingVars[{e, p}]) > 0) {
										if (b >= 0) {
											throw Exception("Duplicate port assignment for vertex '"+v->getName()+"'");
										}
										b = p;
									}
								}
								if (b == -1) {
									throw Exception("Missing port assignment for vertex '"+v->getName()+"'");
								}
								this->portAssignments[e] = b;
								foundPorts.insert(b);
							}
							if (foundPorts.size() != numPorts) {
								throw Exception("Determined corrupt port assignments for vertex '"+v->getName()+"'");
							}
						}
					}
					// update binding container
					this->binding = getBindingContainerFromBinding(this->binding.resourceBindings, this->binding.portAssignments, this->g, this->rm, this->sched, this->II);
					if (!this->quiet) {
						std::cout << "OptimalIntegerIISATBinding: computed binding has #Registers = " << this->binding.registerCosts
						  << " and #Connections = " << this->binding.multiplexerCosts << std::endl;
					}
					auto valid = verifyIntIIBinding(this->g, this->rm, this->sched, this->II, this->binding, this->commutativeOps, this->quiet);
					if (!valid) {
						throw Exception("Binding algorithm determined invalid binding - this should never happen!");
					}
					// adjust upper bound
					switch (opt) {
						case SATFormulationType::regMin_t: {
							this->registerBounds.second = this->binding.registerCosts;
							break;
						}
						case SATFormulationType::muxMin_t: {
							this->interconnectBounds.second = this->binding.multiplexerCosts;
							break;
						}
						case SATFormulationType::muxMinCommutative_t: {
							this->interconnectCommutativeBounds.second = this->binding.multiplexerCosts;
							break;
						}
						case SATFormulationType::regAndMuxMin_t: {
							this->registerBounds.second = this->binding.registerCosts;
							this->interconnectBounds.second = this->binding.multiplexerCosts;
							break;
						}
						case SATFormulationType::regAndMuxMinCommutative_t: {
							this->registerBounds.second = this->binding.registerCosts;
							this->interconnectCommutativeBounds.second = this->binding.multiplexerCosts;
							break;
						}
					}
				}
				else {
					if (status == CADICAL_UNSAT and !this->quiet) {
						std::cout << "OptimalIntegerIISATBinding: SAT solver proved infeasibility" << std::endl;
					}
					// adjust lower bound
					switch (opt) {
						case SATFormulationType::regMin_t: {
							this->registerBounds.first = regMax+1;
							break;
						}
						case SATFormulationType::muxMin_t: {
							this->interconnectBounds.first = connectionsMax+1;
							break;
						}
						case SATFormulationType::muxMinCommutative_t: {
							this->interconnectCommutativeBounds.first = connectionsMax+1;
							break;
						}
						case SATFormulationType::regAndMuxMin_t: {
							if (this->registerBounds.first != this->registerBounds.second) this->registerBounds.first = regMax+1;
							if (this->interconnectBounds.first != this->interconnectBounds.second) this->interconnectBounds.first = connectionsMax+1;
							break;
						}
						case SATFormulationType::regAndMuxMinCommutative_t: {
							if (this->registerBounds.first != this->registerBounds.second) this->registerBounds.first = regMax+1;
							if (this->interconnectCommutativeBounds.first != this->interconnectCommutativeBounds.second) this->interconnectCommutativeBounds.first = connectionsMax+1;
							break;
						}
					}
				}
				//throw Exception("ABORT");

				//////////////////
				// reset solver //
				//////////////////
				this->elapsedTime += this->terminator.getElapsedTime();
				this->solver = std::unique_ptr<CaDiCaL::Solver>(new CaDiCaL::Solver);
				this->solver->connect_terminator(&this->terminator);
				this->variableCounter = 0;
				this->operationBindingVarCounter = 0;
				this->connectionBindingVarCounter = 0;
				this->lifetimeRegBindingVarCounter = 0;
				this->edgePortBindingVarCounter = 0;
				this->clauseCounter = 0;
				this->operationBindingClauseCounter = 0;
				this->noBindingConflictsClauseCounter = 0;
				this->registerBindingClauseCounter = 0;
				this->noRegisterConflictsClauseCounter = 0;
				this->interconnectBindingClauseCounter = 0;
				this->noInterconnectConflictsClauseCounter = 0;
				this->portBindingClauseCounter = 0;
				this->edgeBindingClauseCounter = 0;
				this->noPortBindingConflictsClauseCounter = 0;
			}
		}

		//////////////
		// finished //
		//////////////
		// check whether the solution is optimal
		if (!this->terminator.terminate()) {
			this->binding.solutionStatus = "OPTIMAL";
		}
		this->solutionStatus = this->binding.solutionStatus;
		if (!this->quiet) {
			std::cout << "OptimalIntegerIISATBinding: finished binding" << std::endl;
		}
	}

	void OptimalIntegerIISATBinding::getBinding(Binding::RegChainBindingContainer *b) {
		*b = this->binding;
	}

	void OptimalIntegerIISATBinding::computeInitialBinding() {
		auto simpleBinding = Binding::getSimpleBinding(this->sched, this->rm, this->II);
		this->binding = getBindingContainerFromBinding(simpleBinding, this->portAssignments, this->g, this->rm, this->sched, this->II);
		if (!this->quiet) {
			std::cout << "Computed initial binding with #Regs = " << this->binding.registerCosts << " and #Connections = " << this->binding.multiplexerCosts << std::endl;
			for (auto &v : this->g->Vertices()) {
				std::cout << "  " << v->getName() << " - " << this->rm->getResource(v)->getName() << " (" << simpleBinding.at(v) << ")" << std::endl;
			}
		}
		this->binding.solutionStatus = "TIMEOUT_FEASIBLE";
	}

	int OptimalIntegerIISATBinding::getEdgeLifetime(Edge *e, std::map<Edge*, int>* edgeLifetimes, const std::map<Vertex*, int> &sched, const int &II, ResourceModel* rm) {
		try {
			return edgeLifetimes->at(e);
		}
		catch (std::out_of_range &) {
			auto vSrc = &e->getVertexSrc();
			auto vDst = &e->getVertexDst();
			auto lSrc = rm->getVertexLatency(vSrc);
			auto offset = II * e->getDistance();
			auto tSrc = sched.at(vSrc);
			auto tDst = sched.at(vDst);
			auto lifetime = tDst + offset - tSrc - lSrc;
			(*edgeLifetimes)[e] = lifetime;
			return lifetime;
		}
	}

	Binding::RegChainBindingContainer
	OptimalIntegerIISATBinding::getBindingContainerFromBinding(const map<const Vertex *, int> &fuBindings,
																														 const map<Edge *, int> &portAssignments, Graph *g,
																														 ResourceModel *rm, const std::map<Vertex*, int> &sched,
																														 const int &II) {
		std::map<std::string, int> fuBindingsStr;
		for (auto &it : fuBindings) {
			fuBindingsStr[it.first->getName()] = it.second;
		}
		return getBindingContainerFromBinding(fuBindingsStr, portAssignments, g, rm, sched, II);
	}

	Binding::RegChainBindingContainer
	OptimalIntegerIISATBinding::getBindingContainerFromBinding(const map<std::string, int> &fuBindings,
																														 const map<Edge *, int> &portAssignments, Graph *g,
																														 ResourceModel *rm, const std::map<Vertex*, int> &sched,
																														 const int &II) {
		auto binding = Binding::RegChainBindingContainer();
		binding.fuConnections.clear();
		binding.resourceBindings.clear();
		binding.portAssignments = portAssignments;
		binding.multiplexerCosts = 0;
		binding.registerCosts = 0;
		binding.solutionStatus = "UNKNOWN";
		// fix resource bindings
		binding.resourceBindings = fuBindings;
		// fix fu connections & costs
		std::map<Edge*, int> edgeLifetimes;
		std::map<std::pair<std::string, int>, int> fuRegs;
		for (auto &e : g->Edges()) {
			auto vSrc = &e->getVertexSrc();
			auto vDst = &e->getVertexDst();
			auto vSrcName = vSrc->getName();
			auto vDstName = vDst->getName();
			auto lifetime = getEdgeLifetime(e, &edgeLifetimes, sched, II, rm);
			auto fuSrc = binding.resourceBindings.at(vSrcName);
			auto fuDst = binding.resourceBindings.at(vDstName);
			auto rSrc = rm->getResource(vSrc);
			auto rDst = rm->getResource(vDst);
			auto &rSrcName = rSrc->getName();
			auto &rDstName = rDst->getName();
			auto port = portAssignments.at(e);
			auto findMe = std::make_pair(std::make_pair(std::make_pair(rSrcName, fuSrc), std::make_pair(rDstName, fuDst)), std::make_pair(lifetime, port));
			auto it = std::find(binding.fuConnections.begin(), binding.fuConnections.end(), findMe);
			// fu connection
			if (it == binding.fuConnections.end()) {
				binding.fuConnections.emplace_back(findMe);
			}
			// track lifetime regs after each FU
			if (fuRegs[{rSrcName, fuSrc}] < lifetime) {
				fuRegs[{rSrcName, fuSrc}] = lifetime;
			}
		}
		// calc reg costs
		for (auto &it : fuRegs) {
			binding.registerCosts += it.second;
		}
		// calc mux costs
		binding.multiplexerCosts = binding.fuConnections.size();
		// return it
		return binding;
	}

	void OptimalIntegerIISATBinding::setFirstObjective(OptimalIntegerIISATBinding::firstObjectiveType t) {
		this->firstObjective = t;
	}

	void OptimalIntegerIISATBinding::defineLowerBounds() {
		// registers: use lifetimes as lower bound
		std::map<const Resource*, std::map<int, int>> resourceLifetimes;
		for (auto &v : this->g->Vertices()) {
			auto r = this->rm->getResource(v);
			auto lifetime = this->variableLifetimes[v];
			resourceLifetimes[r][lifetime]++;
		}
		std::map<const Resource*, std::list<int>> reverseSortedResourceLifetimes;
		for (auto &it : resourceLifetimes) {
			for (auto &lifePair : it.second) {
				for (int i=0; i<lifePair.second; i++) {
					reverseSortedResourceLifetimes[it.first].emplace_front(lifePair.first);
				}
			}
		}
		int lifetimeRegsMin = 0;
		for (auto &r : this->rm->Resources()) {
			this->registerBreakPoints[r].first = lifetimeRegsMin;
			int lifeMin = 0;
			auto &l = reverseSortedResourceLifetimes[r];
			int mod = this->II;
			if (r->isUnlimited()) {
				mod = 1;
			}
			auto it = l.begin();
			int cnt = 0;
			while (it != l.end()) {
				if (cnt++ % mod == 0) {
					lifeMin += *it;
				}
				std::advance(it,1);
			}
			lifetimeRegsMin += lifeMin;
			this->registerBreakPoints[r].second = lifetimeRegsMin-1;
		}
		this->numReservedRegisters = this->registerBounds.first = lifetimeRegsMin;

		// mux: use the number of connection types or the total number of FU inputs as lower bound (whatever is larger)
		int totalNumFUInputs = 0;
		for (auto &r : this->rm->Resources()) {
			auto numFUInputs = this->numResourcePorts.at(r);
			int numImplementedFUs;
			if (r->isUnlimited()) {
				numImplementedFUs = this->rm->getNumVerticesRegisteredToResource(r);
			}
			else {
				numImplementedFUs = r->getLimit();
			}
			totalNumFUInputs += (numFUInputs * numImplementedFUs);
		}
		/* todo: also consider schedule times */
		std::set<std::tuple<const Resource*, const Resource*, int, int, int>> uniqueEdgeTypesNoComm;
		std::set<std::tuple<const Resource*, const Resource*, int, int, int>> uniqueEdgeTypesComm;
		for (auto &e : this->g->Edges()) {
			uniqueEdgeTypesNoComm.insert(this->getEdgeType(e, false));
			uniqueEdgeTypesComm.insert(this->getEdgeType(e, true));
		}
		auto totalNumUniqueEdgeTypesNoComm = (int)uniqueEdgeTypesNoComm.size();
		auto totalNumUniqueEdgeTypesComm = (int)uniqueEdgeTypesComm.size();
#if 0
		std::cout << "#q# #FU inputs = " << totalNumFUInputs << std::endl;
		std::cout << "#q# #unique edge types = " << totalNumUniqueEdgeTypesNoComm << std::endl;
		std::cout << "#q# #unique edge types (comm) = " << totalNumUniqueEdgeTypesComm << std::endl;
#endif
		this->interconnectBounds.first = std::max(totalNumFUInputs, totalNumUniqueEdgeTypesNoComm);
		this->interconnectCommutativeBounds.first = std::max(totalNumFUInputs, totalNumUniqueEdgeTypesComm);
	}

	std::tuple<const Resource *, const Resource *, int, int, int> &OptimalIntegerIISATBinding::getEdgeType(Edge *e, const bool &considerCommutativity) {
		try {
			if (considerCommutativity) {
				return this->edgeTypesCommutative.at(e);
			}
			else {
				return this->edgeTypes.at(e);
			}
		}
		catch (std::out_of_range&) {
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			auto rSrc = this->rm->getResource(vSrc);
			auto rDst = this->rm->getResource(vDst);
			auto dstPort = this->portAssignments.at(e);
			auto dstPortComm = dstPort;
			if (this->commutativeOps.find(rDst) != this->commutativeOps.end()) {
				dstPortComm = -1;
			}
			auto life = getEdgeLifetime(e, &this->edgeLifetimes, this->sched, this->II, this->rm);
			auto &retNoComm = this->edgeTypes[e] = {rSrc, rDst, 0, dstPort, life};
			auto &retComm = this->edgeTypesCommutative[e] = {rSrc, rDst, 0, dstPortComm, life};
			if (considerCommutativity) {
				return retComm;
			}
			else {
				return retNoComm;
			}
		}
	}

	void OptimalIntegerIISATBinding::createVariables(const int &maxNumRegs, const int &maxNumConnections, const bool &considerCommutativity) {
		/*
		 * init the following containers:
		 *     - operationBindingVars
		 *     - connectionBindingVars
		 *     - lifetimeRegBindingVars
		 *     - edgePortBindingVars
		*/
		this->variableCounter = 0;
		// operationBindingVars -> always needed
		for (auto &v : this->g->Vertices()) {
			auto &possibleBindings = this->possibleResourceBindings.at(v);
			for (auto &i : possibleBindings) {
				this->operationBindingVars[{v, i}] = ++this->variableCounter;
				this->operationBindingVarCounter++;
			}
		}
		// connectionBindingVars -> only needed if #connections is limited
		if (maxNumConnections != UNLIMITED) {
			for (auto &e : this->g->Edges()) {
				for (int i=0; i<maxNumConnections; i++) {
					this->connectionBindingVars[{e, i}] = ++this->variableCounter;
					this->connectionBindingVarCounter++;
				}
			}
		}
		// edgePortBindingVars -> only needed if #connections is limited and commutativity should be considered
		if (maxNumConnections != UNLIMITED and considerCommutativity) {
			for (auto &e : this->g->Edges()) {
				auto *vDst = &e->getVertexDst();
				auto *rDst = this->rm->getResource(vDst);
				if (this->commutativeOps.find(rDst) == this->commutativeOps.end()) continue;
				auto numPorts = this->numResourcePorts[rDst];
				for (int p=0; p<numPorts; p++) {
					this->edgePortBindingVars[{e, p}] = ++this->variableCounter;
					this->edgePortBindingVarCounter++;
				}
			}
		}
		// lifetimeRegBindingVars -> only needed if #registers is limited
		if (maxNumRegs != UNLIMITED) {
			for (auto &r : this->rm->Resources()) {
				auto lim = this->resourceLimits[r];
				auto &regBP = this->registerBreakPoints[r];
				for (int fu=0; fu<lim; fu++) {
					auto rIdxPair = std::make_pair(r, fu);
					auto maxLife = this->maxResourceLifetimes[rIdxPair];
					for (int l=1; l<=maxLife; l++) {
						//for (int reg=0; reg<maxNumRegs; reg++) {
						for (int reg=regBP.first; reg<=regBP.second; reg++) {
							this->lifetimeRegBindingVars[{r, fu, l, reg}] = ++this->variableCounter;
							this->lifetimeRegBindingVarCounter++;
						}
						for (int reg=this->numReservedRegisters+1; reg<maxNumRegs; reg++) {
							this->lifetimeRegBindingVars[{r, fu, l, reg}] = ++this->variableCounter;
							this->lifetimeRegBindingVarCounter++;
						}
					}
				}
			}
		}
	}

	void OptimalIntegerIISATBinding::calcResourceLifetimes() {
		for (auto &v : this->g->Vertices()) {
			auto &life = this->variableLifetimes[v];
			for (auto &idx : this->possibleResourceBindings.at(v)) {
				auto &rIdxPair = this->indexFuMap.at(idx);
				auto &r = rIdxPair.first;
				auto &fu = rIdxPair.second;
				if (this->maxResourceLifetimes[rIdxPair] < life) this->maxResourceLifetimes[rIdxPair] = life;
			}
		}
	}

	void OptimalIntegerIISATBinding::createClauses(const int &maxNumRegs, const int &maxNumConnections,
																								 const bool &considerCommutativity) {
		// at least 1 operation binding per operation -> always needed
#if 1
		for (auto &v : this->g->Vertices()) {
			for (auto &i : this->possibleResourceBindings[v]) {
				this->solver->add(this->operationBindingVars[{v, i}]);
			}
			this->solver->add(0);
			this->clauseCounter++;
			this->operationBindingClauseCounter++;
		}
#endif
		// prohibit operation binding conflicts -> always needed
#if 1
		for (auto &r : this->rm->Resources()) {
			if (r->isUnlimited()) continue;
			auto lim = this->resourceLimits[r];
			for (int i=0; i<lim; i++) {
				auto idx = this->fuIndexMap[{r, i}];
				for (int mod=0; mod<this->II; mod++) {
					auto &vertices = this->operationsInModSlots[{r, i, mod}];
					for (auto vi : vertices) {
						for (auto vj : vertices) {
							if (vi->getId() <= vj->getId()) continue; // avoid adding duplicate clauses
							this->solver->add(-this->operationBindingVars[{vi, idx}]);
							this->solver->add(-this->operationBindingVars[{vj, idx}]);
							this->solver->add(0);
							this->clauseCounter++;
							this->noBindingConflictsClauseCounter++;
						}
					}
				}
			}
		}
#endif
		// register clauses -> only needed when registers are limited
		if (maxNumRegs != UNLIMITED) {
			// each FU gets as many registers as needed
#if 1
			for (auto &v : this->g->Vertices()) {
				auto &life = this->variableLifetimes[v];
				for (auto &i : this->possibleResourceBindings[v]) {
					auto &p = indexFuMap[i];
					auto &opVar = this->operationBindingVars[{v, i}];
					for (auto l=1; l<=life; l++) {
						//std::string clause = "not("+std::to_string(opVar)+")";
						this->solver->add(-opVar);
						//for (auto reg=0; reg<maxNumRegs; reg++) {
						auto &regBP = this->registerBreakPoints[p.first];
						for (int reg=regBP.first; reg <= regBP.second; reg++) { // reserved registers for that resource
							//clause += " + " + std::to_string(this->lifetimeRegBindingVars[{p.first, p.second, l, reg}]);
							this->solver->add(this->lifetimeRegBindingVars[{p.first, p.second, l, reg}]);
						}
						for (int reg=this->numReservedRegisters+1; reg < maxNumRegs; reg++) { // shared registers between all resources
							//clause += " + " + std::to_string(this->lifetimeRegBindingVars[{p.first, p.second, l, reg}]);
							this->solver->add(this->lifetimeRegBindingVars[{p.first, p.second, l, reg}]);
						}
						//std::cout << "#q# clause: " << clause << std::endl;
						this->solver->add(0);
						this->clauseCounter++;
						this->registerBindingClauseCounter++;
					}
				}
			}
#endif
			// prohibit register binding conflicts
#if 1
			for (auto ri : this->rm->Resources()) {
				auto limi = this->resourceLimits[ri];
				auto &regBP = this->registerBreakPoints[ri];
				for (auto fui=0; fui<limi; fui++) {
					auto pi = this->fuIndexMap[{ri, fui}];
					auto maxLifei = this->maxResourceLifetimes[{ri, fui}];
					for (auto rj : this->rm->Resources()) {
						auto limj = this->resourceLimits[rj];
						for (auto fuj=0; fuj<limj; fuj++) {
							auto pj = this->fuIndexMap[{rj, fuj}];
							auto maxLifej = this->maxResourceLifetimes[{rj, fuj}];
							for (int li=1; li<=maxLifei; li++) {
								for (int lj=1; lj<=maxLifej; lj++) {
									if (pi == pj and li == lj) continue;
									if (ri == rj) {
										for (int reg=regBP.first; reg<=regBP.second; reg++) {
											auto ai = this->lifetimeRegBindingVars[{ri, fui, li, reg}];
											auto aj = this->lifetimeRegBindingVars[{rj, fuj, lj, reg}];
											this->solver->add(-ai);
											this->solver->add(-aj);
											this->solver->add(0);
											this->clauseCounter++;
											this->noRegisterConflictsClauseCounter++;
										}
									}
									for (int reg=this->numReservedRegisters+1; reg<maxNumRegs; reg++) { // shared registers
										auto ai = this->lifetimeRegBindingVars[{ri, fui, li, reg}];
										auto aj = this->lifetimeRegBindingVars[{rj, fuj, lj, reg}];
										this->solver->add(-ai);
										this->solver->add(-aj);
										this->solver->add(0);
										this->clauseCounter++;
										this->noRegisterConflictsClauseCounter++;
									}
								}
							}
						}
					}
				}
			}
#endif
		}
		// interconnect clauses -> only needed when the number of connections are limited
		if (maxNumConnections != UNLIMITED) {
			// at least one connection binding per edge
#if 1
			for (auto &e : this->g->Edges()) {
				for (int c=0; c<maxNumConnections; c++) {
					this->solver->add(this->connectionBindingVars[{e, c}]);
				}
				this->solver->add(0);
				this->clauseCounter++;
				this->interconnectBindingClauseCounter++;
			}
#endif
			// prohibit edge binding conflicts
			for (auto &ei : this->g->Edges()) {
				auto *vSrci = &ei->getVertexSrc();
				auto *vDsti = &ei->getVertexDst();
				auto *rDsti = this->rm->getResource(vDsti);
				auto numPorts = this->numResourcePorts[rDsti];
				auto fuIndicesSrci = this->possibleResourceBindings[vSrci];
				auto fuIndicesDsti = this->possibleResourceBindings[vDsti];
				auto edgeTypei = this->getEdgeType(ei, considerCommutativity);
				auto dstCommutative = this->commutativeOps.find(rDsti) != this->commutativeOps.end();
				auto commutativityRelevant = dstCommutative and considerCommutativity;
				for (auto &ej : this->g->Edges()) {
					if (ei->getId() <= ej->getId()) continue;
					auto edgeTypej = this->getEdgeType(ej, commutativityRelevant);
					if (edgeTypei == edgeTypej) {
						// edges CAN POTENTIALLY share a mux port
						auto *vSrcj = &ej->getVertexSrc();
						auto *vDstj = &ej->getVertexDst();
						auto fuIndicesSrcj = this->possibleResourceBindings[vSrcj];
						auto fuIndicesDstj = this->possibleResourceBindings[vDstj];
						// prohibit src fu conflicts
#if 1
						for (auto idxi : fuIndicesSrci) {
							auto opBindingVari = this->operationBindingVars[{vSrci, idxi}];
							for (auto idxj : fuIndicesSrcj) {
								if (idxi == idxj) continue; // idxi = idxj would be ok -> only prohibit different FUs
								auto opBindingVarj = this->operationBindingVars[{vSrcj, idxj}];
								for (int c=0; c<maxNumConnections; c++) {
									this->solver->add(-this->connectionBindingVars[{ei, c}]);
									this->solver->add(-this->connectionBindingVars[{ej, c}]);
									this->solver->add(-opBindingVari);
									this->solver->add(-opBindingVarj);
									this->solver->add(0);
									this->clauseCounter++;
									this->noInterconnectConflictsClauseCounter++;
								}
							}
						}
#endif
						// prohibit dst fu conflicts
#if 1
						for (auto idxi : fuIndicesDsti) {
							auto opBindingVari = this->operationBindingVars[{vDsti, idxi}];
							for (auto idxj : fuIndicesDstj) {
								if (idxi == idxj) continue; // idxi = idxj would be ok -> only prohibit different FUs
								auto opBindingVarj = this->operationBindingVars[{vDstj, idxj}];
								for (int c=0; c<maxNumConnections; c++) {
									this->solver->add(-this->connectionBindingVars[{ei, c}]);
									this->solver->add(-this->connectionBindingVars[{ej, c}]);
									this->solver->add(-opBindingVari);
									this->solver->add(-opBindingVarj);
									this->solver->add(0);
									this->clauseCounter++;
									this->noInterconnectConflictsClauseCounter++;
								}
							}
						}
#endif
						// prohibit port conflicts
#if 1
						if (commutativityRelevant) {
							for (auto pi=0; pi<numPorts; pi++) {
								auto portBindingVari = this->edgePortBindingVars[{ei, pi}];
								for (auto pj=0; pj<numPorts; pj++) {
									if (pi == pj) continue; // pi = pj would be ok -> only prohibit different ports
									auto portBindingVarj = this->edgePortBindingVars[{ej, pj}];
									for (int c=0; c<maxNumConnections; c++) {
										this->solver->add(-this->connectionBindingVars[{ei, c}]);
										this->solver->add(-this->connectionBindingVars[{ej, c}]);
										this->solver->add(-portBindingVari);
										this->solver->add(-portBindingVarj);
										this->solver->add(0);
										this->clauseCounter++;
										this->noInterconnectConflictsClauseCounter++;
									}
								}
							}
						}
#endif
					}
					else {
						// edges CAN'T EVER share a mux port
#if 1
						for (int c=0; c<maxNumConnections; c++) {
							this->solver->add(-this->connectionBindingVars[{ei, c}]);
							this->solver->add(-this->connectionBindingVars[{ej, c}]);
							this->solver->add(0);
							this->clauseCounter++;
							this->noInterconnectConflictsClauseCounter++;
						}
#endif
					}
				}
			}
			// port clauses -> only needed if commutativity is considered
			if (considerCommutativity) {
				// at least one port binding per edge
#if 1
				for (auto &e : this->g->Edges()) {
					auto *rDst = this->rm->getResource(&e->getVertexDst());
					if (this->commutativeOps.find(rDst) == this->commutativeOps.end()) continue;
					auto numPorts = this->numResourcePorts[rDst];
					for (int p=0; p<numPorts; p++) {
						this->solver->add(this->edgePortBindingVars[{e,p}]);
					}
					this->solver->add(0);
					this->clauseCounter++;
					this->portBindingClauseCounter++;
				}
#endif
				// at least one edge binding per port AND no port binding conflicts
				for (auto &v : this->g->Vertices()) {
					auto *r = this->rm->getResource(v);
					if (this->commutativeOps.find(r) == this->commutativeOps.end()) continue;
					auto numPorts = this->numResourcePorts[r];
					auto incomingEdges = this->g->getIncomingEdges(v);
					// at least one edge binding per port
#if 1
					for (int p=0; p<numPorts; p++) {
						for (auto &e : incomingEdges) {
							this->solver->add(this->edgePortBindingVars[{e,p}]);
						}
						this->solver->add(0);
						this->clauseCounter++;
						this->portBindingClauseCounter++;
					}
#endif
					// no port binding conflicts
#if 1
					for (auto &ei : incomingEdges) {
						for (auto &ej : incomingEdges) {
							if (ei->getId() <= ej->getId()) continue; // avoid duplicate clauses
							for (int p=0; p<numPorts; p++) {
								this->solver->add(-this->edgePortBindingVars[{ei,p}]);
								this->solver->add(-this->edgePortBindingVars[{ej,p}]);
								this->solver->add(0);
								this->clauseCounter++;
								this->noPortBindingConflictsClauseCounter++;
							}
						}
					}
#endif
				}
			}
		}
	}
}

#endif //USE_CADICAL