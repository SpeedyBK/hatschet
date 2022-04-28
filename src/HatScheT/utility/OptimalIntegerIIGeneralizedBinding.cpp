//
// Created by nfiege on 3/4/22.
//
#include <ScaLP/Solver.h>
#include <sstream>
#include "OptimalIntegerIIGeneralizedBinding.h"
#include "HatScheT/utility/Utility.h"

namespace HatScheT {
	OptimalIntegerIIGeneralizedBinding::OptimalIntegerIIGeneralizedBinding(
		HatScheT::Graph *g, HatScheT::ResourceModel *rm, map<Vertex *, int> sched, int II,
		map<Edge *, std::pair<int, int>> portAssignments, set<const Resource *> commutativeOps, std::list<std::string> sw) :
	  BindingBase(g, rm, sched, II, commutativeOps), portAssignments(portAssignments), sw(sw), numRegs(-1),
	  allocMinRegs(true), allowMultipleBindings(false)
	{}

	void OptimalIntegerIIGeneralizedBinding::bind() {
		if (!this->quiet) std::cout << "OptimalIntegerIIGeneralizedBinding::bind: let's go!" << std::endl;

		//////////////////////
		// check for errors //
		//////////////////////
		for (auto &e : this->g->Edges()) {
			if (!e->isDataEdge()) {
				continue;
			}
			if (this->portAssignments.find(e) != this->portAssignments.end()) {
				continue;
			}
			auto vSrc = e->getVertexSrcName();
			auto vDst = e->getVertexDstName();
			throw Exception("OptimalIntegerIIGeneralizedBinding::bind: port assignment container corrupt - failed to find edge '"+vSrc+"' -> '"+vDst+"'");
		}
		//////////////////////////////////////////////////////////////////
		// set objective weights to meaningful values if they are zero //
		//////////////////////////////////////////////////////////////////
		double wMuxInternally = this->wMux;
		double wRegInternally = this->wReg;
		if (this->wReg < 0.0 or this->wMux < 0.0) {
			auto costsPair = Utility::getMaxRegsAndMuxs(this->g, this->rm, this->sched, this->II);
			if (wRegInternally < 0.0) {
				wRegInternally = 1.0 / (1.0 + ((double) costsPair.first));
			}
			if (wMuxInternally < 0.0) {
				// function computes actual multiplexer costs instead of interconnect costs
				// => adjust
				costsPair.second = Utility::getNumberOfFUConnections(costsPair.second, this->g, this->rm);
				wMuxInternally = 1.0 / (1.0 + ((double) costsPair.second));
			}
		}

		if (!quiet) {
			std::cout << "Mux cost weighting factor: " << wMuxInternally << std::endl;
			std::cout << "Reg cost weighting factor: " << wRegInternally << std::endl;
		}

		/////////////////////////
		// container to return //
		/////////////////////////
		this->bin = Binding::BindingContainer();
		this->bin.portAssignments = this->portAssignments;

		///////////////////
		// create solver //
		///////////////////
		if(this->sw.empty()) this->sw = {"Gurobi","CPLEX","SCIP","LPSolve"};
		auto solver = ScaLP::Solver(sw);
		solver.quiet = true;
		solver.threads = this->numThreads;
		solver.timeout = (long)this->timeBudget;

		/////////////////////////////////////////////////
		// count if less FUs are needed than allocated //
		/////////////////////////////////////////////////
		std::map<const Resource*,int> resourceLimits;
		for(auto &r : this->rm->Resources()) {
			resourceLimits[r] = 0;
			if(r->isUnlimited()) {
				resourceLimits[r] = this->rm->getNumVerticesRegisteredToResource(r);
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
		std::map<const Resource*, int> numResourcePorts;
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

		////////////////////////////////////////
		// check if input ports are left open //
		////////////////////////////////////////
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
				auto port = it.second.second;
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

		///////////////////////////////////////////////////////
		// determine birth and death times for all variables //
		///////////////////////////////////////////////////////////////////////
		// while we're at it, also determine ports of vertices and resources //
		///////////////////////////////////////////////////////////////////////
		// map <vertex and output port that produces this variable> -> birth/death time of that variable
		std::map<std::pair<const Vertex*, int>, int> birthTimes;
		std::map<std::pair<const Vertex*, int>, int> deathTimes;
		std::map<const Vertex*, std::set<int>> outputPortsOfVertex;
		std::map<const Vertex*, std::set<int>> inputPortsOfVertex;
		std::map<const Resource*, std::set<int>> outputPortsOfResource;
		std::map<const Resource*, std::set<int>> inputPortsOfResource;
		std::map<std::pair<const Vertex*, int>, Edge*> inputEdges;
		std::map<std::pair<const Vertex*, int>, std::set<Edge*>> outputEdges;
		std::map<Edge*, int> lifetimes;
		for (auto &r : this->rm->Resources()) {
			outputPortsOfResource[r] = {};
			inputPortsOfResource[r] = {};
			for (auto &v : this->rm->getVerticesOfResource(r)) {
				outputPortsOfVertex[v] = {};
				inputPortsOfVertex[v] = {};
			}
		}
		for (auto &e : this->g->Edges()) {
			if (!e->isDataEdge()) {
				// skip chaining edges
				continue;
			}
			auto* vSrc = &e->getVertexSrc();
			auto* vDst = &e->getVertexDst();
			auto pSrc = this->portAssignments.at(e).first;
			auto pDst = this->portAssignments.at(e).second;
			auto* rSrc = this->rm->getResource(vSrc);
			auto* rDst = this->rm->getResource(vDst);
			auto lSrc = rSrc->getLatency();
			auto birthTime = this->sched.at(vSrc) + lSrc;
			auto deathTime = this->sched.at(vDst) + (e->getDistance() * this->II);
			birthTimes[{vSrc, pSrc}] = birthTime;
			deathTimes[{vSrc, pSrc}] = max(deathTime, deathTimes[{vSrc, pSrc}]);
			outputPortsOfVertex[vSrc].insert(pSrc);
			inputPortsOfVertex[vDst].insert(pDst);
			outputPortsOfResource[rSrc].insert(pSrc);
			inputPortsOfResource[rDst].insert(pDst);
			inputEdges[{vDst, pDst}] = e;
			outputEdges[{vSrc, pSrc}].insert(e);
			lifetimes[e] = deathTime - birthTime;
		}

		///////////////////////////////////////////
		// determine minimum amount of registers //
		///////////////////////////////////////////
		std::map<int,int> numAliveVariables;
		for (auto &v : this->g->Vertices()) {
			for (auto &p : outputPortsOfVertex[v]) {
				auto bt = birthTimes[{v, p}];
				auto dt = deathTimes[{v, p}];
				if (bt == dt) {
					// no register needed to save this variable
					continue;
				}
				for (int t=bt+1; t<=dt; t++) {
					auto congruenceClass = t % this->II;
					numAliveVariables[congruenceClass]++;
				}
			}
		}
		int minNumRegs = 0;
		for (auto &it : numAliveVariables) {
			minNumRegs = max(minNumRegs, it.second);
		}
		if (minNumRegs > this->maxReg) {
			this->solutionStatus = "INFEASIBLE";
			if (!this->quiet) {
				std::cout << "OptimalIntegerIIGeneralizedBinding::bind: requested infeasible number of registers ("
				  << this->maxReg << ") - at least " << minNumRegs << " registers are required for implementation" << std::endl;
			}
			return;
		}
		if (this->allocMinRegs or this->maxReg == std::numeric_limits<double>::infinity()) {
			// can only model a finite number of registers
			this->maxReg = minNumRegs;
		}

		///////////////////////////////////////////////////
		// assign each FU of each resource type a number //
		///////////////////////////////////////////////////
		std::map<std::pair<const Resource*,int>,int> fuIndexMap;
		std::map<int,std::pair<const Resource*,int>> indexFuMap;
		std::map<const Vertex*, std::vector<int>> compatibleFUs;
		std::map<int, bool> isReg;
		this->numFUs = 0;
		for(auto &r : this->rm->Resources()) {
			auto verticesOfResource = this->rm->getVerticesOfResource(r);
			for(auto i=0; i<resourceLimits[r]; i++) {
				isReg[this->numFUs] = false;
				auto p = make_pair(r,i);
				fuIndexMap[p] = this->numFUs;
				indexFuMap[this->numFUs] = p;
				for (auto &v : verticesOfResource) {
					compatibleFUs[v].emplace_back(this->numFUs);
				}
				if (!this->quiet) std::cout << "  resource '" << r->getName() << "' (" << i << ") has idx=" << this->numFUs << std::endl;
				this->numFUs++;
			}
		}
		if(!this->quiet) std::cout << "assigned all " << this->numFUs << " fus a number" << std::endl;

		///////////////////////////////////
		// assign each register a number //
		///////////////////////////////////
		std::map<int,int> regIndexMap;
		//std::map<int,int> indexRegMap;
		for(this->numRegs=0; this->numRegs<(int)this->maxReg; this->numRegs++) {
			isReg[this->numFUs+this->numRegs] = true;
			regIndexMap[this->numRegs] = this->numRegs + this->numFUs;
			//indexRegMap[this->numRegs + this->numFUs] = this->numRegs;
			if (!this->quiet) std::cout << "  register (" << this->numRegs << ") has idx=" << regIndexMap.at(this->numRegs) << std::endl;
		}
		if(!this->quiet) std::cout << "assigned all " << this->numRegs << " registers a number" << std::endl;

		//////////////////////////
		// FU binding variables //
		//////////////////////////
		std::map<std::pair<const Vertex*, int>, ScaLP::Variable> F;
		for (auto &v : this->g->Vertices()) {
			for (auto &fuIdx : compatibleFUs.at(v)) {
				std::stringstream name;
				name << "F_" << v->getId() << "_" << fuIdx;
				F[{v, fuIdx}] = ScaLP::newBinaryVariable(name.str());
			}
		}

		/////////////////////////////////
		// variable location variables //
		/////////////////////////////////
		// map from <vertex that produces this variable and the output port where it comes out of>
		// to another map from time step to scalp variable
		std::map<std::tuple<const Vertex*, int, int, int>, ScaLP::Variable> T;
		bool registerSelfLoops = false;
		for (auto &v : this->g->Vertices()) {
			for (auto &p : outputPortsOfVertex.at(v)) {
				std::pair<Vertex*, int> vp = {v, p};
				auto bt = birthTimes.at(vp);
				auto dt = deathTimes.at(vp);
				if (dt - bt >= 2) registerSelfLoops = true;
				// at its birth time, the variable comes out of an FU
				// afterwards, the variable remains in some register
				if (bt != dt) {
					for (int i=0; i<this->numRegs; i++) {
						auto regIdx = regIndexMap.at(i);
						for (int t=bt+1; t<=dt; t++) {
							std::stringstream name;
							name << "T_" << v->getId() << "_" << p << "_" << t << "_" << regIdx;
							T[{v, p, t, regIdx}] = ScaLP::newBinaryVariable(name.str());
						}
					}
				}
			}
		}
		if(!this->quiet) std::cout << "created variable location variables" << std::endl;

		//////////////////////////
		// connection variables //
		//////////////////////////
		std::map<std::tuple<int,int,int,int>, ScaLP::Variable> C;
		for (int src=0; src<this->numFUs+this->numRegs; src++) {
			std::set<int> portsSrc;
			if (!isReg.at(src)) {
				// src hardware element is an FU
				portsSrc = outputPortsOfResource.at(indexFuMap.at(src).first);
			}
			else {
				// src hardware element is a register
				// register only have 1 output
				portsSrc = {0};
			}
			if (portsSrc.empty()) {
				// skip hardware elements without outputs
				continue;
			}
			for (int dst=0; dst<this->numFUs+this->numRegs; dst++) {
				std::set<int> portsDst;
				if (!isReg.at(dst)) {
					// dst hardware element is an FU
					portsDst = inputPortsOfResource.at(indexFuMap.at(dst).first);
				}
				else {
					// dst hardware element is a register
					// register only have 1 input
					portsDst = {0};
				}
				if (portsDst.empty()) {
					// skip hardware elements without inputs
					continue;
				}
				if (isReg.at(src) and isReg.at(dst) and (!registerSelfLoops)) {
					// omit register self loops if they can not occur
					continue;
				}
				for (auto& pSrc : portsSrc) {
					for (auto& pDst : portsDst) {
						std::tuple<int,int,int,int> key = {src, dst, pSrc, pDst};
						std::stringstream name;
						name << "C_" << src << "_" << dst << "_" << pSrc << "_" << pDst;
						C[key] = ScaLP::newBinaryVariable(name.str());
					}
				}
			}
		}
		if(!this->quiet) std::cout << "created connection variables" << std::endl;

		//////////////////////////////
		// register usage variables //
		//////////////////////////////
		std::map<int, ScaLP::Variable> R;
		if (this->numRegs > minNumRegs) {
			for (int i=0; i<this->numRegs; i++) {
				auto regIdx = regIndexMap.at(i);
				std::stringstream name;
				name << "R_" << regIdx;
				R[regIdx] = ScaLP::newBinaryVariable(name.str());
			}
		}
		if(!this->quiet) std::cout << "created register variables" << std::endl;

		/////////////////////////////////////////
		// operator source selection variables //
		/////////////////////////////////////////
		std::map<std::tuple<Vertex*, int, int, int>, ScaLP::Variable> SO;
		if (this->allowMultipleBindings) {
			for (auto &vDst : this->g->Vertices()) {
				for (auto &pDst : inputPortsOfVertex.at(vDst)) {
					auto &inputEdge = inputEdges.at({vDst, pDst});
					auto &lifetime = lifetimes.at(inputEdge);
					for (auto &fuDstIdx : compatibleFUs.at(vDst)) {
						if (lifetime == 0) {
							// source is an FU
							auto *vSrc = &inputEdge->getVertexSrc();
							for (auto &fuSrcIdx : compatibleFUs.at(vSrc)) {
								std::stringstream name;
								name << "I_" << vDst->getId() << "_" << pDst << "_" << fuDstIdx << "_" << fuSrcIdx;
								SO[{vDst, pDst, fuDstIdx, fuSrcIdx}] = ScaLP::newBinaryVariable(name.str());
							}
						}
						else {
							// source is a register
							for (int i=0; i<this->numRegs; i++) {
								auto regSrcIdx = regIndexMap.at(i);
								std::stringstream name;
								name << "I_" << vDst->getId() << "_" << pDst << "_" << fuDstIdx << "_" << regSrcIdx;
								SO[{vDst, pDst, fuDstIdx, regSrcIdx}] = ScaLP::newBinaryVariable(name.str());
							}
						}
					}
				}
			}
			if(!this->quiet) std::cout << "created operator source selection variables" << std::endl;
		}

		/////////////////////////////////////////
		// register source selection variables //
		/////////////////////////////////////////
		std::map<std::tuple<Vertex*, int, int, int, int>, ScaLP::Variable> SR;
		if (this->allowMultipleBindings) {
			for (auto &v : this->g->Vertices()) {
				for (auto &p : outputPortsOfVertex.at(v)) {
					auto bt = birthTimes.at({v, p});
					auto dt = deathTimes.at({v, p});
					if (bt == dt) {
						// no register needed for this variable
						continue;
					}
					for (int t=bt+1; t<=dt; t++) {
						for (int i=0; i<this->numRegs; i++) {
							auto regDstIdx = regIndexMap.at(i);
							if (t == bt+1) {
								// source is the FU
								for (auto &fuSrcIdx : compatibleFUs.at(v)) {
									std::stringstream name;
									name << "I_" << v->getId() << "_" << p << "_" << t << "_" << regDstIdx << "_" << fuSrcIdx;
									SR[{v, p, t, regDstIdx, fuSrcIdx}] = ScaLP::newBinaryVariable(name.str());
								}
							}
							else {
								// source is another register
								for (int j=0; j<this->numRegs; j++) {
									auto regSrcIdx = regIndexMap.at(j);
									std::stringstream name;
									name << "I_" << v->getId() << "_" << p << "_" << t << "_" << regDstIdx << "_" << regSrcIdx;
									SR[{v, p, t, regDstIdx, regSrcIdx}] = ScaLP::newBinaryVariable(name.str());
								}
							}
						}
					}
				}
			}
			if(!this->quiet) std::cout << "created register source selection variables" << std::endl;
		}

		///////////////////////////////////////////////////////////////////////////////
		// constraints that each operation is bound to exactly/at least one operator //
		///////////////////////////////////////////////////////////////////////////////
		for (auto &v : this->g->Vertices()) {
			ScaLP::Term lhs;
			for (auto fuIdx : compatibleFUs[v]) {
				lhs += F.at({v, fuIdx});
			}
			if (this->allowMultipleBindings) {
				solver.addConstraint(lhs >= 1);
			}
			else {
				solver.addConstraint(lhs == 1);
			}
		}
		if(!this->quiet) std::cout << "created operator constraints" << std::endl;

		////////////////////////////////////////////////////////////////////////////////////////////////
		// constraints that each variable is bound to exactly/at least one register in each time step //
		////////////////////////////////////////////////////////////////////////////////////////////////
		for (auto &v : this->g->Vertices()) {
			for (auto &p : outputPortsOfVertex.at(v)) {
				std::pair<Vertex *, int> vp = {v, p};
				auto bt = birthTimes.at(vp);
				auto dt = deathTimes.at(vp);
				if (bt == dt) continue;
				for (int t=bt+1; t<=dt; t++) {
					ScaLP::Term lhs;
					for (int i=0; i<this->numRegs; i++) {
						auto regIdx = regIndexMap.at(i);
						lhs += T.at({v, p, t, regIdx});
					}
					if (this->allowMultipleBindings) {
						solver.addConstraint(lhs >= 1);
					}
					else {
						solver.addConstraint(lhs == 1);
					}
				}
			}
		}
		if(!this->quiet) std::cout << "created register constraints" << std::endl;

		///////////////////////////////////
		// operator conflict constraints //
		///////////////////////////////////
		for (auto &v1 : this->g->Vertices()) {
			auto resIndices = compatibleFUs.at(v1);
			auto r1 = this->rm->getResource(v1);
			for (auto &v2 : this->g->Vertices()) {
				auto r2 = this->rm->getResource(v2);
				if (v1 == v2) {
					// no conflict with itself
					continue;
				}
				if (r1 != r2) {
					// no conflict with different resources
					continue;
				}
				if (v1->getId() > v2->getId()) {
					// do not create unnecessary/duplicate constraints
					// e.g. we already have all constraints for v_5 vs v_6
					// then we do not need to create all constraints again for v_6 vs v_5
				}
				if ((this->sched.at(v1) % this->II) != (this->sched.at(v2) % this->II)) {
					// no conflict for different congruence classes
					continue;
				}
				for (auto &idx : resIndices) {
					auto fu1Var = F.at({v1, idx});
					auto fu2Var = F.at({v2, idx});
					solver.addConstraint(fu1Var + fu2Var <= 1);
				}
			}
		}
		if(!this->quiet) std::cout << "created operator conflict constraints" << std::endl;

		///////////////////////////////////
		// register conflict constraints //
		///////////////////////////////////
		for (auto &v1 : this->g->Vertices()) {
			for (auto &v2 : this->g->Vertices()) {
				for (auto &p1 : outputPortsOfVertex.at(v1)) {
					auto tBirth1 = birthTimes.at({v1, p1});
					auto tDeath1 = deathTimes.at({v1, p1});
					if (tBirth1 == tDeath1) {
						// no register conflict possible when this variable is never stored in a register
						continue;
					}
					for (auto &p2 : outputPortsOfVertex.at(v2)) {
						auto tBirth2 = birthTimes.at({v2, p2});
						auto tDeath2 = deathTimes.at({v2, p2});
						if (tBirth2 == tDeath2) {
							// no register conflict possible when this variable is never stored in a register
							continue;
						}
						for (int t1 = tBirth1+1; t1<=tDeath1; t1++) {
							for (int t2 = tBirth2+1; t2<=tDeath2; t2++) {
								if (v1->getId() > v2->getId()) {
									// do not create unnecessary/duplicate constraints
									// e.g. we already have all constraints for v_5 vs v_6
									// then we do not need to create all constraints again for v_6 vs v_5
								}
								if ((v1 == v2) and (t1 == t2)) {
									// no conflict with itself
									continue;
								}
								if ((t1 % this->II) != (t2 % this->II)) {
									// no conflict for different congruence classes
									continue;
								}
								for (int i=0; i<this->numRegs; i++) {
									auto regIdx = regIndexMap.at(i);
									auto reg1Var = T.at({v1, p1, t1, regIdx});
									auto reg2Var = T.at({v2, p2, t2, regIdx});
									solver.addConstraint(reg1Var + reg2Var <= 1);
								}
							}
						}
					}
				}
			}
		}
		if(!this->quiet) std::cout << "created register conflict constraints" << std::endl;

		////////////////////////////////
		// register usage constraints //
		////////////////////////////////
		if (this->numRegs > minNumRegs) {
			for (int i=0; i<this->numRegs; i++) {
				auto regIdx = regIndexMap.at(i);
				auto regVar = R.at(regIdx);
				for (auto &it : T) {
					if (std::get<3>(it.first) != regIdx) {
						// variable does not match register
						continue;
					}
					auto tVar = it.second;
					solver.addConstraint(tVar - regVar <= 0);
				}
			}
		}
		if(!this->quiet) std::cout << "created register usage constraints" << std::endl;

		//////////////////////////////////////////////////
		// unique operator source selection constraints //
		//////////////////////////////////////////////////
		if (this->allowMultipleBindings) {
			for (auto &vDst : this->g->Vertices()) {
				for (auto &pDst : inputPortsOfVertex.at(vDst)) {
					auto &inputEdge = inputEdges.at({vDst, pDst});
					auto &lifetime = lifetimes.at(inputEdge);
					for (auto &fuDstIdx : compatibleFUs.at(vDst)) {
						if (lifetime == 0) {
							// source is an FU
							auto *vSrc = &inputEdge->getVertexSrc();
							ScaLP::Term lhs;
							for (auto &fuSrcIdx : compatibleFUs.at(vSrc)) {
								lhs += SO.at({vDst, pDst, fuDstIdx, fuSrcIdx});
							}
							solver.addConstraint(lhs == 1);
						}
						else {
							// source is a register
							ScaLP::Term lhs;
							for (int i=0; i<this->numRegs; i++) {
								auto regSrcIdx = regIndexMap.at(i);
								lhs += SO.at({vDst, pDst, fuDstIdx, regSrcIdx});
							}
							solver.addConstraint(lhs == 1);
						}
					}
				}
			}
			if(!this->quiet) std::cout << "created unique operator source selection constraints" << std::endl;
		}

		//////////////////////////////////////////////////
		// unique register source selection constraints //
		//////////////////////////////////////////////////
		if (this->allowMultipleBindings) {
			for (auto &v : this->g->Vertices()) {
				for (auto &p : outputPortsOfVertex.at(v)) {
					auto bt = birthTimes.at({v, p});
					auto dt = deathTimes.at({v, p});
					if (bt == dt) {
						// no register needed for this variable
						continue;
					}
					for (int t=bt+1; t<=dt; t++) {
						for (int i=0; i<this->numRegs; i++) {
							auto regDstIdx = regIndexMap.at(i);
							if (t == bt+1) {
								// source is the FU
								ScaLP::Term lhs;
								for (auto &fuSrcIdx : compatibleFUs.at(v)) {
									lhs += SR.at({v, p, t, regDstIdx, fuSrcIdx});
								}
								solver.addConstraint(lhs == 1);
							}
							else {
								// source is another register
								ScaLP::Term lhs;
								for (int j=0; j<this->numRegs; j++) {
									auto regSrcIdx = regIndexMap.at(j);
									lhs += SR.at({v, p, t, regDstIdx, regSrcIdx});
								}
								solver.addConstraint(lhs == 1);
							}
						}
					}
				}
			}
			if(!this->quiet) std::cout << "created unique register source selection constraints" << std::endl;
		}

		///////////////////////////////////////////////////////
		// operator source selection implication constraints //
		///////////////////////////////////////////////////////
		if (this->allowMultipleBindings) {
			for (auto &e : this->g->Edges()) {
				if (!e->isDataEdge()) continue;
				auto *vSrc = &e->getVertexSrc();
				auto *vDst = &e->getVertexDst();
				auto lSrc = this->rm->getVertexLatency(vSrc);
				auto lDst = this->rm->getVertexLatency(vDst);
				auto pSrc = this->portAssignments.at(e).first;
				auto pDst = this->portAssignments.at(e).second;
				auto lifetime = lifetimes.at(e);
				auto birthTime = birthTimes.at({vSrc, pSrc});
				auto deathTime = birthTime + lifetime;
				for (auto &fuDstIdx : compatibleFUs.at(vDst)) {
					auto fDstVar = F.at({vDst, fuDstIdx});
					if (lifetime == 0) {
						// src: FU
						for (int fuSrcIdx : compatibleFUs.at(vSrc)) {
							auto soVar = SO.at({vDst, pDst, fuDstIdx, fuSrcIdx});
							auto fSrcVar = F.at({vSrc, fuSrcIdx});
							//solver.addConstraint(soVar - fVar <= 0);
							solver.addConstraint(fDstVar + soVar - fSrcVar <= 1);
						}
					}
					else {
						// src: register
						for (int i=0; i<this->numRegs; i++) {
							auto regSrcIdx = regIndexMap.at(i);
							auto soVar = SO.at({vDst, pDst, fuDstIdx, regSrcIdx});
							auto tVar = T.at({vSrc, pSrc, deathTime, regSrcIdx});
							//solver.addConstraint(soVar - tVar <= 0);
							solver.addConstraint(fDstVar + soVar - tVar <= 1);
						}
					}
				}
			}
			if(!this->quiet) std::cout << "created operator source selection implication constraints" << std::endl;
		}

		///////////////////////////////////////////////////////
		// register source selection implication constraints //
		///////////////////////////////////////////////////////
		if (this->allowMultipleBindings) {
			for (auto &v : this->g->Vertices()) {
				for (auto &p : outputPortsOfVertex.at(v)) {
					auto bt = birthTimes.at({v, p});
					auto dt = deathTimes.at({v, p});
					if (bt == dt) {
						// no register needed for this variable
						continue;
					}
					for (int t=bt+1; t<=dt; t++) {
						for (int i=0; i<this->numRegs; i++) {
							auto regDstIdx = regIndexMap.at(i);
							auto tDstVar = T.at({v, p, t, regDstIdx});
							if (t == bt+1) {
								// source is the FU
								for (auto &fuSrcIdx : compatibleFUs.at(v)) {
									auto srVar = SR.at({v, p, t, regDstIdx, fuSrcIdx});
									auto fVar = F.at({v, fuSrcIdx});
									//solver.addConstraint(srVar - fVar <= 0);
									solver.addConstraint(tDstVar + srVar - fVar <= 1);
								}
							}
							else {
								// source is another register
								for (int j=0; j<this->numRegs; j++) {
									auto regSrcIdx = regIndexMap.at(j);
									auto srVar = SR.at({v, p, t, regDstIdx, regSrcIdx});
									//auto tVar = T.at({v, p, t, regSrcIdx});
									auto tSrcVar = T.at({v, p, t-1, regSrcIdx});
									//solver.addConstraint(srVar - tVar <= 0);
									solver.addConstraint(tDstVar + srVar - tSrcVar <= 1);
								}
							}
						}
					}
				}
			}
			if(!this->quiet) std::cout << "created register source selection implication constraints" << std::endl;
		}

		/////////////////////////////////////////////////////
		// FU -> Reg and Reg -> Reg connection constraints //
		/////////////////////////////////////////////////////
		for (auto &v : this->g->Vertices()) {
			for (auto &p : outputPortsOfVertex.at(v)) {
				std::pair<Vertex *, int> vp = {v, p};
				auto bt = birthTimes.at(vp);
				auto dt = deathTimes.at(vp);
				if (bt == dt) {
					// this variable is not stored in any register
					continue;
				}
				// FU -> Reg
				for (auto fuIdx : compatibleFUs.at(v)) {
					//auto fuVar = T.at({v, p, bt, fuIdx});
					auto fuVar = F.at({v, fuIdx});
					for (int i=0; i<this->numRegs; i++) {
						auto regIdx = regIndexMap.at(i);
						auto regVar = T.at({v, p, bt+1, regIdx});
						auto connectionVar = C.at({fuIdx, regIdx, p, 0});
						if (this->allowMultipleBindings) {
							auto srVar = SR.at({v, p, bt+1, regIdx, fuIdx});
							solver.addConstraint(fuVar + regVar + srVar - connectionVar <= 2);
						}
						else {
							solver.addConstraint(fuVar + regVar - connectionVar <= 1);
						}
					}
				}
				// Reg -> Reg
				if (bt+1 == dt) {
					// this variable is only alive for 1 clock cycle
					continue;
				}
				for (auto t=bt+1; t<dt; t++) {
					for (int i=0; i<this->numRegs; i++) {
						auto regSrcIdx = regIndexMap.at(i);
						auto regSrcVar = T.at({v, p, t, regSrcIdx});
						for (int j=0; j<this->numRegs; j++) {
							auto regDstIdx = regIndexMap.at(j);
							auto regDstVar = T.at({v, p, t+1, regDstIdx});
							auto connectionVar = C.at({regSrcIdx, regDstIdx, 0, 0});
							if (this->allowMultipleBindings) {
								auto srVar = SR.at({v, p, t+1, regDstIdx, regSrcIdx});
								solver.addConstraint(regSrcVar + regDstVar + srVar - connectionVar <= 2);
							}
							else {
								solver.addConstraint(regSrcVar + regDstVar - connectionVar <= 1);
							}
						}
					}
				}
			}
		}
		if(!this->quiet) std::cout << "created FU -> Reg and Reg -> Reg constraints" << std::endl;

		///////////////////////////////////////////////////
		// FU -> FU and Reg -> FU connection constraints //
		///////////////////////////////////////////////////
		for (auto &e : this->g->Edges()) {
			if (!e->isDataEdge()) continue;
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			auto tSrc = this->sched.at(vSrc);
			auto tDst = this->sched.at(vDst);
			auto lSrc = this->rm->getVertexLatency(vSrc);
			//auto lDst = this->rm->getVertexLatency(vDst);
			auto tBirth = tSrc + lSrc;
			auto tDeath = tDst + (e->getDistance() * this->II);
			//auto tProdDst = tDst + lDst;
			auto pSrc = this->portAssignments.at(e).first;
			auto pDst = this->portAssignments.at(e).second;
			for (auto &fuDstIdx : compatibleFUs.at(vDst)) {
				//auto fuDstVar = T.at({vDst, pDst, tProdDst, fuDstIdx});
				auto fuDstVar = F.at({vDst, fuDstIdx});
				if (tBirth == tDeath) {
					// FU -> FU connection
					for (auto &fuSrcIdx : compatibleFUs.at(vSrc)) {
						//auto fuSrcVar = T.at({vSrc, pSrc, tBirth, fuSrcIdx});
						auto fuSrcVar = F.at({vSrc, fuSrcIdx});
						auto connectionVar = C.at({fuSrcIdx, fuDstIdx, pSrc, pDst});
						if (this->allowMultipleBindings) {
							auto soVar = SO.at({vDst, pDst, fuDstIdx, fuSrcIdx});
							solver.addConstraint(fuSrcVar + fuDstVar + soVar - connectionVar <= 2);
						}
						else {
							solver.addConstraint(fuSrcVar + fuDstVar - connectionVar <= 1);
						}
					}
				}
				else {
					// Reg -> FU connection
					for (int i=0; i<this->numRegs; i++) {
						auto regSrcIdx = regIndexMap.at(i);
						auto regSrcVar = T.at({vSrc, pSrc, tDeath, regSrcIdx});
						auto connectionVar = C.at({regSrcIdx, fuDstIdx, 0, pDst});
						if (this->allowMultipleBindings) {
							auto soVar = SO.at({vDst, pDst, fuDstIdx, regSrcIdx});
							solver.addConstraint(regSrcVar + fuDstVar + soVar - connectionVar <= 2);
						}
						else {
							solver.addConstraint(regSrcVar + fuDstVar - connectionVar <= 1);
						}
					}
				}
			}
		}
		if(!this->quiet) std::cout << "created FU -> FU and Reg -> FU constraints" << std::endl;

		//////////////////////
		// create objective //
		//////////////////////
		ScaLP::Term obj;
		if (this->numRegs > minNumRegs) {
			for (int i=0; i<this->numRegs; i++) {
				auto regIdx = regIndexMap.at(i);
				auto regVar = R.at(regIdx);
				obj += (wRegInternally * regVar);
			}
		}
		for (auto &c : C) {
			if (std::get<0>(c.first) == std::get<1>(c.first)) {
				// found a self loop
				if (isReg.at(std::get<0>(c.first))) {
					// self loops in registers do not cost a connection and should therefore not appear in the objective function
					continue;
				}
			}
			obj += (wMuxInternally * c.second);
		}
		solver.setObjective(ScaLP::minimize(obj));
		if(!this->quiet) std::cout << "set objective function" << std::endl;

		////////////////////////////
		// actually start solving //
		////////////////////////////
		//if(!this->quiet) std::cout << solver.showLP() << std::endl;
		if(!this->quiet) std::cout << "start solving now" << std::endl;
		auto stat = solver.solve();
		this->solutionStatus = ScaLP::showStatus(stat);
		this->bin.solutionStatus = this->solutionStatus;
		if(!this->quiet) std::cout << "finished solving with status " << stat << std::endl;
		if(stat != ScaLP::status::FEASIBLE and stat != ScaLP::status::TIMEOUT_FEASIBLE and stat != ScaLP::status::OPTIMAL) {
			if (!this->quiet) {
				std::cout << "OptimalIntegerIIGeneralizedBinding::bind: could not solve binding problem, ScaLP status " << stat
									<< std::endl;
			}
			return; // empty binding
		}
		if(!this->quiet) std::cout << "start filling solution structure" << std::endl;
		auto results = solver.getResult();
		auto resultValues = results.values;
		//std::cout << results << std::endl;

		///////////////////////
		// resource bindings //
		///////////////////////
		for (auto &v : this->g->Vertices()) {
			for (auto &fuIdx : compatibleFUs.at(v)) {
				auto val = (int)round(resultValues.at(F.at({v, fuIdx})));
				if (val == 1) {
					this->bin.resourceBindings[v->getName()].insert(indexFuMap.at(fuIdx).second);
				}
			}
		}
		if(!this->quiet) {
			std::cout << "determined resource bindings" << std::endl;
			for (auto &it : this->bin.resourceBindings) {
				std::cout << "  " << it.first;
				for (auto &it2 : it.second) {
					std::cout << " " << it2;
				}
				std::cout << std::endl;
			}
		}

		////////////////////
		// register costs //
		////////////////////
		std::map<int, int> indexRegMap; // reverse mapping from register indices to actual registers
		if (this->numRegs == minNumRegs) {
			this->bin.registerCosts = this->numRegs;
			for (int i=0; i<this->numRegs; i++) {
				auto regIdx = regIndexMap.at(i);
				indexRegMap[regIdx] = i;
			}
		}
		else {
			int actualNumRegs = 0;
			for (int i=0; i<this->numRegs; i++) {
				auto regIdx = regIndexMap.at(i);
				auto val = (int)round(resultValues.at(R.at(regIdx)));
				if (val == 1) {
					indexRegMap[regIdx] = actualNumRegs;
					actualNumRegs++;
				}
			}
			this->bin.registerCosts = actualNumRegs;
		}
		if(!this->quiet) {
			std::cout << "determined register costs = " << this->bin.registerCosts << std::endl;
		}

		///////////////////////
		// multiplexer costs //
		///////////////////////
		this->bin.multiplexerCosts = 0;
		for (auto &it : C) {
			auto &src = std::get<0>(it.first);
			auto &dst = std::get<1>(it.first);
			auto val = (int)round(resultValues.at(it.second));
			if (val == 1) {
				if (src == dst) {
					// found a self loop
					if (isReg.at(src)) {
						// self loops in registers do not cost a connection
						continue;
					}
				}
				this->bin.multiplexerCosts++;
			}
		}
		if(!this->quiet) {
			std::cout << "determined multiplexer costs = " << this->bin.multiplexerCosts << std::endl;
		}

		////////////////////////
		// variable locations //
		////////////////////////
		std::map<std::tuple<const Vertex*, int, int>, std::set<int>> variableLocations;
		for (auto &it : T) {
			auto &v = std::get<0>(it.first);
			auto &p = std::get<1>(it.first);
			auto &t = std::get<2>(it.first);
			auto &regIdx = std::get<3>(it.first);
			if (!isReg.at(regIdx)) {
				// only registers are of interest now
				continue;
			}
			auto &var = it.second;
			auto val = (int)round(resultValues.at(var));

			if (val == 1) {
				variableLocations[{v, p, t}].insert(regIdx);
			}
		}
		if(!this->quiet) {
			std::cout << "determined variable locations:" << std::endl;
			for (auto &it : variableLocations) {
				std::cout << "  " << std::get<0>(it.first)->getName() << " port " << std::get<1>(it.first) << " in t="
				  << std::get<2>(it.first) << " is located in " << (it.second.size()==1?"register ":"registers ");
				for (auto &it2 : it.second) {
					std::cout << it2 << " ";
				}
				std::cout << std::endl;
			}
		}

		//////////////////////////////////////////
		// FU -> Reg and Reg -> Reg connections //
		//////////////////////////////////////////
		for (auto &v : this->g->Vertices()) {
			auto fuIdxs = this->bin.resourceBindings.at(v->getName());
			for (auto &p : outputPortsOfVertex[v]) {
				std::pair<Vertex *, int> vp = {v, p};
				auto bt = birthTimes.at(vp);
				auto dt = deathTimes.at(vp);
				if (bt == dt) {
					// this variable is not stored in any register
					continue;
				}
				auto srcRes = this->rm->getResource(v);
				std::string srcResName = srcRes->getName();
				int srcOutputPort = p;
				std::string dstResName = "register";
				int dstInputPort = 0;
				auto srcIndices = std::set<int>();
				for (auto fuIdx : fuIdxs) {
					srcIndices.insert(fuIndexMap.at({this->rm->getResource(srcResName), fuIdx}));
				}
				//int srcIndex = fuIndexMap.at({this->rm->getResource(srcResName), fuIdx});
				for (auto tDst=bt+1; tDst<=dt; tDst++) {
					// get current variable location
					auto dstIndices = variableLocations.at({v, p, tDst});
					// for each dst index, make a connection to its source
					for (auto &dstIndex : dstIndices) {
						// check which source is selected
						int srcIndex = -1;
						int srcFuRegIdx = -1;
						bool foundSrc = false;
						for (auto &possibleSrcIndex : srcIndices) {
							// check if it is actually the source
							if (this->allowMultipleBindings) {
								auto srVar = SR.at({v, p, tDst, dstIndex, possibleSrcIndex}); // v, p, t, regDstIdx, fuSrcIdx
								if ((int)round(resultValues.at(srVar)) == 0) {
									continue;
								}
							}
							// check for multiple sources (this should never happen)
							if (foundSrc) {
								throw Exception("OptimalIntegerIIGeneralizedBinding::bind: found multiple sources for register '"+std::to_string(dstIndex)+"' in time step '"+std::to_string(tDst)+"' for the variable produced by vertex '"+v->getName()+"', port '"+std::to_string(p)+"'");
							}
							foundSrc = true;
							// seems like we found the source
							srcIndex = possibleSrcIndex;
							if (srcResName == "register") {
								srcFuRegIdx = indexRegMap.at(srcIndex);
							}
							else {
								srcFuRegIdx = indexFuMap.at(srcIndex).second;
							}
						}
						if (not foundSrc) {
							throw Exception("OptimalIntegerIIGeneralizedBinding::bind: failed to find source for register '"+std::to_string(dstIndex)+"' in time step '"+std::to_string(tDst)+"' for the variable produced by vertex '"+v->getName()+"', port '"+std::to_string(p)+"'");
						}
						// sanity check if corresponding C-variable is actually set
						auto cVar = C.at({srcIndex, dstIndex, srcOutputPort, dstInputPort});
						auto cVal = (int)round(resultValues.at(cVar));
						if (cVal != 1) {
							std::stringstream errMsg;
							errMsg << "error in ILP formulation" << std::endl;
							errMsg << "C_" << srcIndex << "_" << dstIndex << "_" << srcOutputPort << "_" << dstInputPort << " should have been set but isn't" << std::endl;
							throw HatScheT::Exception("OptimalIntegerIIGeneralizedBinding::bind: "+errMsg.str());
						}

						int dstRegIndex = indexRegMap.at(dstIndex);
						if (not (srcResName == "register" and dstResName == "register" and srcFuRegIdx == dstRegIndex)) {
							// insert connection if it is not a register self loop
							// additionally, set enable=1 for that register on that clock cycle
							auto enableTimeInserted = this->bin.registerEnableTimes[dstRegIndex].insert((tDst-1) % this->II);
							if (!enableTimeInserted.second) {
								// register conflict detected
								throw Exception("OptimalIntegerIIGeneralizedBinding::bind: register conflict detected for register '"+
																std::to_string(dstRegIndex)+"' in congruence class '"+std::to_string((tDst-1) % this->II)+
																"' - this should never happen");
							}
							bool foundConnection = false;
							for (auto &c : this->bin.connections) {
								if (std::get<0>(c) != srcResName) continue;
								if (std::get<1>(c) != srcFuRegIdx) continue;
								if (std::get<2>(c) != srcOutputPort) continue;
								if (std::get<3>(c) != dstResName) continue;
								if (std::get<4>(c) != dstRegIndex) continue;
								if (std::get<5>(c) != dstInputPort) continue;
								auto inserted = std::get<6>(c).insert((tDst-1) % this->II);
								if (!inserted.second) {
									// FU connection conflict detected
									throw Exception("OptimalIntegerIIGeneralizedBinding::bind: register connection conflict detected for '"
																	+std::to_string(dstIndex)+"' in congruence class '"+std::to_string((tDst-1) % this->II)+
																	"' - this should never happen");
								}
								foundConnection = true;
								break;
							}
							if (!foundConnection) {
								this->bin.connections.push_back({srcResName, srcFuRegIdx, srcOutputPort, dstResName, dstRegIndex, dstInputPort, {(tDst-1) % this->II}});
							}
						}
					}

					// update sources
					srcResName = "register";
					srcOutputPort = 0;
					srcIndices = dstIndices;
				}
			}
		}
		if(!this->quiet) std::cout << "determined FU -> Reg and Reg -> Reg connections" << std::endl;

		////////////////////////////////////////
		// FU -> FU and Reg -> FU connections //
		////////////////////////////////////////
		for (auto &e : this->g->Edges()) {
			if (!e->isDataEdge()) continue;
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			auto *rSrc = this->rm->getResource(vSrc);
			auto *rDst = this->rm->getResource(vDst);
			auto tSrc = this->sched.at(vSrc);
			auto tDst = this->sched.at(vDst);
			auto lSrc = this->rm->getVertexLatency(vSrc);
			auto lDst = this->rm->getVertexLatency(vDst);
			auto tBirth = tSrc + lSrc;
			auto tDeath = tDst + (e->getDistance() * this->II);
			auto tProdDst = tDst + lDst;
			auto pSrc = this->portAssignments.at(e).first;
			auto pDst = this->portAssignments.at(e).second;
			auto fuSrcs = this->bin.resourceBindings.at(vSrc->getName());
			auto fuDsts = this->bin.resourceBindings.at(vDst->getName());
			// create/reuse a connection for each dst
			for (auto &fuDst : fuDsts) {
				// convert resource and fu into unique fu idx
				auto fuDstIdx = fuIndexMap.at({rDst, fuDst});
				// what we need for a connection
				std::string srcResName;
				int srcIndex;
				int srcOutputPort;
				auto &dstResName = rDst->getName();
				int dstIndex = fuDst;
				int dstInputPort = pDst;

				// check which source is selected
				bool foundSrc = false;
				int srcFuRegIdx;
				int dstFuRegIdx;

				// std::list<std::tuple<src resource name, src index, src output port, dst resource name, dst index, dst input port, set of active times modulo II>> connections;
				if (tBirth == tDeath) {
					// FU -> FU connection
					for (auto &possibleFuSrc : fuSrcs) {
						auto possibleFuSrcIdx = fuIndexMap.at({rSrc, possibleFuSrc});
						// check if it is actually the source
						if (this->allowMultipleBindings) {
							auto soVar = SO.at({vDst, pDst, fuDstIdx, possibleFuSrcIdx}); // vDst, pDst, fuDstIdx, fuSrcIdx/regSrcIdx
							if ((int)round(resultValues.at(soVar)) == 0) {
								continue;
							}
						}
						// check for multiple sources (this should never happen)
						if (foundSrc) {
							throw Exception("OptimalIntegerIIGeneralizedBinding::bind: found multiple sources for FU '"+std::to_string(dstIndex)+"' for vertex '"+vDst->getName()+"', port '"+std::to_string(pDst)+"'");
						}
						foundSrc = true;
						// seems like we found the source
						srcResName = rSrc->getName();
						srcIndex = possibleFuSrc;
						srcOutputPort = pSrc;
						srcFuRegIdx = fuIndexMap.at({this->rm->getResource(srcResName), srcIndex});
					}
				}
				else {
					// Reg -> FU connection
					for (auto &possibleFuSrc : variableLocations.at({vSrc, pSrc, tDeath})) {
						// check if it is actually the source
						if (this->allowMultipleBindings) {
							//auto soVar = SO.at({vDst, pDst, fuDst, possibleFuSrc}); // vDst, pDst, fuDstIdx, fuSrcIdx/regSrcIdx
							auto soVar = SO.at({vDst, pDst, fuDstIdx, possibleFuSrc}); // vDst, pDst, fuDstIdx, fuSrcIdx/regSrcIdx
							if ((int)round(resultValues.at(soVar)) == 0) {
								continue;
							}
						}
						// check for multiple sources (this should never happen)
						if (foundSrc) {
							throw Exception("OptimalIntegerIIGeneralizedBinding::bind: found multiple sources for FU '"+std::to_string(dstIndex)+"' for vertex '"+vDst->getName()+"', port '"+std::to_string(pDst)+"'");
						}
						foundSrc = true;
						// seems like we found the source
						srcResName = "register";
						srcIndex = possibleFuSrc;
						srcOutputPort = 0;
						srcFuRegIdx = srcIndex;
					}
				}
				dstFuRegIdx = fuIndexMap.at({this->rm->getResource(dstResName), dstIndex});

				// sanity check if corresponding C variable was set
				auto cVar = C.at({srcFuRegIdx, dstFuRegIdx, srcOutputPort, dstInputPort});
				auto cVal = (int)round(resultValues.at(cVar));
				if (cVal != 1) {
					std::stringstream errMsg;
					std::cout << "error in ILP formulation" << std::endl;
					std::cout << "C_" << srcFuRegIdx << "_" << dstFuRegIdx << "_" << srcOutputPort << "_" << dstInputPort << " should have been set but isn't" << std::endl;
					errMsg << "error in ILP formulation" << std::endl;
					errMsg << "C_" << srcFuRegIdx << "_" << dstFuRegIdx << "_" << srcOutputPort << "_" << dstInputPort << " should have been set but isn't" << std::endl;
					throw HatScheT::Exception("OptimalIntegerIIGeneralizedBinding::bind: "+errMsg.str());
				}
				if (srcResName == "register") {
					srcIndex = indexRegMap.at(srcFuRegIdx);
				}
				//std::tuple<std::string, int, int, std::string, int, int> connectionKey = {srcResName, srcIndex, srcOutputPort, dstResName, dstIndex, dstInputPort};
				bool foundConnection = false;
				for (auto &c : this->bin.connections) {
					if (std::get<0>(c) != srcResName) continue;
					if (std::get<1>(c) != srcIndex) continue;
					if (std::get<2>(c) != srcOutputPort) continue;
					if (std::get<3>(c) != dstResName) continue;
					if (std::get<4>(c) != dstIndex) continue;
					if (std::get<5>(c) != dstInputPort) continue;
					auto inserted = std::get<6>(c).insert(tDst % this->II);
					if (!inserted.second) {
						// FU connection conflict detected
						throw Exception("OptimalIntegerIIGeneralizedBinding::bind: FU connection conflict detected for FU '"+
														std::to_string(dstIndex)+"' of resource type '"+dstResName+"' in congruence class '"+
														std::to_string(tDst % this->II)+"' - this should never happen");
					}
					foundConnection = true;
					break;
				}
				if (!foundConnection) {
					this->bin.connections.push_back({srcResName, srcIndex, srcOutputPort, dstResName, dstIndex, dstInputPort, {tDst % this->II}});
				}
			} // fuDst
		} // e
		if(!this->quiet) std::cout << "determined FU -> FU and Reg -> FU connections" << std::endl;
		if (this->bin.connections.size() != this->bin.multiplexerCosts) {
			//throw HatScheT::Exception("OptimalIntegerIIGeneralizedBinding::bind: something went wrong when calculating multiplexer costs - this should never happen");
			std::cout << "OptimalIntegerIIGeneralizedBinding::bind: something went wrong when calculating multiplexer costs - this should never happen" << std::endl;
			std::cout << "  calculated costs=" << this->bin.multiplexerCosts << " but actual number of connections=" << this->bin.connections.size() << std::endl;
		}

		///////////////////
		// print results //
		///////////////////
		if (!this->quiet) {
			std::cout << "----solution status----" << std::endl;
			std::cout << this->bin.solutionStatus << std::endl;
			std::cout << "----costs----" << std::endl;
			std::cout << "register costs: " << this->bin.registerCosts << std::endl;
			std::cout << "connection costs: " << this->bin.multiplexerCosts << std::endl;
			std::cout << "----resource bindings----" << std::endl;
			for (auto &rb : this->bin.resourceBindings) {
				std::cout << rb.first << " -> ";
				for (auto &fu : rb.second) {
					std::cout << fu << " " << std::endl;
				}
			}
			std::cout << "----register enable times----" << std::endl;
			for (auto &et : this->bin.registerEnableTimes) {
				std::cout << "reg #" << et.first << std::endl;
				for (auto &t : et.second) {
					std::cout << "  " << t << std::endl;
				}
			}
			std::cout << "----connections----" << std::endl;
			for (auto &c : this->bin.connections) {
				std::cout << "'" << std::get<0>(c) << "' (" << std::get<1>(c) << ") port '" << std::get<2>(c) << "' -> '" << std::get<3>(c) << "' (" << std::get<4>(c) << ") port '" << std::get<5>(c) << "'" << std::endl;
				for (auto &t : std::get<6>(c)) {
					std::cout << "  in t=" << t << std::endl;
				}
			}
		}
	}

	void HatScheT::OptimalIntegerIIGeneralizedBinding::getBinding(Binding::BindingContainer *bi) {
		*bi = this->bin;
	}

	void OptimalIntegerIIGeneralizedBinding::setAllocMinRegs(bool a) {
		this->allocMinRegs = a;
	}

	void OptimalIntegerIIGeneralizedBinding::setAllowMultipleBindings(bool m) {
		this->allowMultipleBindings = m;
	}
}

