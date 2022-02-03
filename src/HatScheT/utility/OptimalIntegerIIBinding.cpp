//
// Created by nfiege on 11/11/21.
//

#include "OptimalIntegerIIBinding.h"
#include <HatScheT/utility/Utility.h>
#include <limits>
#include <sstream>
#include <cmath>

namespace HatScheT {

	OptimalIntegerIIBinding::OptimalIntegerIIBinding(Graph *g, ResourceModel *rm, std::map<Vertex *, int> sched, int II,
																									 std::map<Edge *, int> portAssignments, std::set<const Resource*> commutativeOps, std::list<std::string> sw) :
	g(g), rm(rm), sched(sched), II(II), portAssignments(portAssignments), timeBudget(300), quiet(true),
	wMux(1.0), wReg(1.0), maxMux(std::numeric_limits<double>::infinity()), sw(sw), commutativeOps(commutativeOps),
	maxReg(std::numeric_limits<double>::infinity()), obj(Binding::objective::minimize)
	{
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

	void OptimalIntegerIIBinding::setTimeout(unsigned int timeout) {
		this->timeBudget = timeout;
	}

	void OptimalIntegerIIBinding::setQuiet(bool q) {
		this->quiet = q;
	}

	void OptimalIntegerIIBinding::bind() {
		//////////////////////
		// check for errors //
		//////////////////////
		// edges are missing in port assignment container
		for(auto e : this->g->Edges()) {
			try {
				this->portAssignments.at(e);
			}
			catch(...) {
				throw HatScheT::Exception("Binding::getILPBasedIntIIBinding: Edge '"+e->getVertexSrcName()+"' -> '"+e->getVertexDstName()+"' missing in port assignment container");
			}
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
		this->binding = Binding::RegChainBindingContainer();

		///////////////////
		// create solver //
		///////////////////
		if(this->sw.empty()) this->sw = {"Gurobi","CPLEX","SCIP","LPSolve"};
		auto solver = ScaLP::Solver(sw);
		solver.quiet = true;
		solver.threads = 1;
		solver.timeout = (long)this->timeBudget;

		/////////////////////////////////////////////////
		// count if less FUs are needed than allocated //
		/////////////////////////////////////////////////
		std::map<const Resource*,int> resourceLimits;
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
		std::map<std::pair<const Resource*,int>,int> fuIndexMap;
		std::map<int,std::pair<const Resource*,int>> indexFuMap;
		int fuIndexCounter = 0;
		for(auto &r : this->rm->Resources()) {
			for(auto i=0; i<resourceLimits[r]; i++) {
				auto p = make_pair(r,i);
				fuIndexMap[p] = fuIndexCounter;
				indexFuMap[fuIndexCounter] = p;
				fuIndexCounter++;
			}
		}
		if(!this->quiet) std::cout << "assigned all fus a number" << std::endl;

		///////////////////////////////////////////////////////////
		// check possible lifetimes for each fu -> fu connection //
		///////////////////////////////////////////////////////////
		std::map<std::pair<int,int>,std::set<int>> possibleLifetimes;
		std::map<Edge*,int> lifetimes;
		for(auto &e : this->g->Edges()) {
			auto &vi = e->getVertexSrc();
			auto &vj = e->getVertexDst();
			auto lifetime = this->sched[&vj] - this->sched[&vi] - this->rm->getResource(&vi)->getLatency() + (this->II * e->getDistance());
			lifetimes[e] = lifetime;
			auto *ri = this->rm->getResource(&vi);
			auto *rj= this->rm->getResource(&vj);
			for(int a=0; a<resourceLimits[ri]; a++) {
				auto m = fuIndexMap[{ri,a}];
				for(int b=0; b<resourceLimits[rj]; b++) {
					auto n = fuIndexMap[{rj,b}];
					possibleLifetimes[{m,n}].insert(lifetime);
				}
			}
		}
		if(!this->quiet) std::cout << "checked possible lifetimes" << std::endl;

		//////////////////////////////////////////////////////////////////
		// check possible port connections for each fu -> fu connection //
		//////////////////////////////////////////////////////////////////
		std::map<std::pair<int,int>,std::set<int>> possiblePortConnections;
		for(auto &e : this->g->Edges()) {
			auto &vi = e->getVertexSrc();
			auto &vj = e->getVertexDst();
			auto *ri = this->rm->getResource(&vi);
			auto *rj= this->rm->getResource(&vj);
			for(int a=0; a<resourceLimits[ri]; a++) {
				auto m = fuIndexMap[{ri,a}];
				for(int b=0; b<resourceLimits[rj]; b++) {
					auto n = fuIndexMap[{rj,b}];
					if(this->commutativeOps.find(rj) == this->commutativeOps.end()) {
						// non-commutative operation
						// only the user-given port assignment is allowed
						possiblePortConnections[{m,n}].insert(this->portAssignments[e]);
					}
					else {
						// commutative operations
						// all ports are allowed
						for(int p=0; p<numResourcePorts[rj]; p++) {
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
		std::map<Vertex*,int> unlimitedOpFUs;
		for(auto &r : this->rm->Resources()) {
			if(!r->isUnlimited()) continue;
			auto vertices = this->rm->getVerticesOfResource(r);
			int fuCounter = 0;
			for(auto v : vertices) {
				unlimitedOpFUs[const_cast<Vertex*>(v)] = fuCounter;
				fuCounter++;
			}
		}

		//////////////////////
		// create variables //
		//////////////////////
		// i : vertex index
		// m/n : fu index
		// k : lifetime register stage after fu
		// p ; input port number
		// e : edge
		std::map<std::pair<int,int>,ScaLP::Variable> v_i_m; // binding vertex i -> fu m
		std::map<std::pair<std::pair<int,int>,std::pair<int,int>>,ScaLP::Variable> r_m_n_k_p; // connection from fu m to fu n port p over k registers
		std::map<std::pair<Edge*,int>,ScaLP::Variable> a_e_p; // if edge e is assigned to port p
		std::map<int,ScaLP::Variable> z_m; // number of lifetime registers after FU m

		// create vertex binding variables
		for(auto v : this->g->Vertices()) {
			auto i = v->getId();
			auto r = this->rm->getResource(v);
			if(!this->quiet) std::cout << "  resource limit for resource " << r->getName() << ": " << resourceLimits[r] << std::endl;
			for(int a=0; a<resourceLimits[r]; a++) {
				if(r->isUnlimited() and a != unlimitedOpFUs[v]) {
					// only create variables that are actually needed
					// unlimited operations need unique fus assignments that were already pre-computed above
					continue;
				}
				auto m = fuIndexMap[{r,a}];
				std::stringstream name;
				name << "v_" << i << "_" << m;
				v_i_m[{i,m}] = ScaLP::newBinaryVariable(name.str());
				if(!this->quiet) std::cout << "  Successfully created variable " << v_i_m[{i,m}] << std::endl;
			}
		}
		if(!this->quiet) std::cout << "created vertex-binding variables" << std::endl;

		// create connection variables
		for(auto ri : this->rm->Resources()) {
			for(auto rj : this->rm->Resources()) {
				for(int a=0; a<resourceLimits[ri]; a++) {
					auto m = fuIndexMap[{ri,a}];
					for(int b=0; b<resourceLimits[rj]; b++) {
						auto n = fuIndexMap[{rj,b}];
						for(auto k : possibleLifetimes[{m,n}]) {
							for(auto p : possiblePortConnections[{m,n}]) {
								std::stringstream name;
								name << "r_" << m << "_" << n << "_" << k << "_" << p;
								r_m_n_k_p[{{m,n},{k,p}}] = ScaLP::newBinaryVariable(name.str());
								if(!this->quiet) std::cout << "  Successfully created variable " << r_m_n_k_p[{{m,n},{k,p}}] << std::endl;
							}
						}
					}
				}
			}
		}
		if(!this->quiet) std::cout << "created connection variables" << std::endl;

		// create edge-port variables for commutative sink vertices
		for(auto &e : this->g->Edges()) {
			auto vi = &e->getVertexSrc();
			auto vj = &e->getVertexDst();
			auto rj = this->rm->getResource(vj);
			if(this->commutativeOps.find(rj) == this->commutativeOps.end()) continue;
			for(int p=0; p<numResourcePorts[rj]; p++) {
				std::stringstream name;
				name << "a_" << e << "_" << p;
				a_e_p[{e,p}] = ScaLP::newBinaryVariable(name.str());
				if(!this->quiet) std::cout << "  Successfully created variable " << a_e_p[{e,p}] << std::endl;
			}
		}
		if(!this->quiet) std::cout << "created edge-port variables" << std::endl;

		// create lifetime register variables
		for (int m=0; m<fuIndexCounter; m++) {
			// we can use real variables here because they only appear in >= constraints with integers on the RHS
			// and those variables get minimized in the objective function so their optimal values are already integers
			// this means that we can save computation time in the ILP solving algorithm by relaxing the integer constraint
			z_m[m] = ScaLP::newRealVariable("z_"+std::to_string(m), 0.0, ScaLP::INF());
		}
		if(!this->quiet) std::cout << "created lifetime registers variables" << std::endl;

		////////////////////////
		// create constraints //
		////////////////////////
		// commutative operations part 1
		// only one edge-port variable is 1 per edge
		for(auto &e : this->g->Edges()) {
			auto vj = &e->getVertexDst();
			auto rj = this->rm->getResource(vj);
			if (this->commutativeOps.find(rj) == this->commutativeOps.end()) continue;
			auto vi = &e->getVertexSrc();
			if(!this->quiet) {
				std::cout << "Creating constraints that edge-port variables are equal 1 per edge for " << vi->getName() << "->"
									<< vj->getName() << std::endl;
			}
			ScaLP::Term lhs;
			for(int p=0; p<numResourcePorts[rj]; p++) {
				lhs += a_e_p[{e,p}];
			}
			ScaLP::Constraint constr = lhs == 1;
			solver.addConstraint(constr);
			if(!this->quiet) std::cout << "  Successfully added constraint " << constr << std::endl;
		}
		if(!this->quiet) std::cout << "created commutativity constraints part 1" << std::endl;

		// commutative operations part 2
		// only one edge-port variable is 1 per port and sink vertex
		for(auto &vj : this->g->Vertices()) {
			auto rj = this->rm->getResource(vj);
			if (this->commutativeOps.find(rj) == this->commutativeOps.end()) continue;
			if(!this->quiet) {
				std::cout << "Creating constraints that edge-port variables are equal 1 per per port for " << vj->getName()
									<< std::endl;
			}
			for(int p=0; p<numResourcePorts[rj]; p++) {
				ScaLP::Term lhs;
				for(auto &e : this->g->Edges()) {
					if(vj != &e->getVertexDst()) continue;
					lhs += a_e_p[{e,p}];
				}
				ScaLP::Constraint constr = lhs == 1;
				solver.addConstraint(constr);
				if(!this->quiet) std::cout << "  Successfully added constraint " << constr << std::endl;
			}
		}
		if(!this->quiet) std::cout << "created commutativity constraints part 2" << std::endl;

		// commutative operations part 3
		// the sum of edge-port variables is p per sink vertex
		/*
		for(auto &vj : this->g->Vertices()) {
			ScaLP::Term lhs;
			auto rj = this->rm->getResource(vj);
			if (this->commutativeOps.find(rj) == this->commutativeOps.end()) continue;
			if(!this->quiet) {
				std::cout << "Creating constraints that edge-port variables are equal to " << numResourcePorts[rj] << " for "
									<< vj->getName() << std::endl;
			}

			for(int p=0; p<numResourcePorts[rj]; p++) {
				for(auto &e : this->g->Edges()) {
					if(vj != &e->getVertexDst()) continue;
					lhs += a_e_p[{e,p}];
				}
			}
			ScaLP::Constraint constr = lhs == numResourcePorts[rj];
			solver.addConstraint(constr);
			if(!this->quiet) std::cout << "  Successfully added constraint " << constr << std::endl;
		}
		if(!this->quiet) std::cout << "created commutativity constraints part 3" << std::endl;
		 */

		// commutative operations part 4
		// map edge-port variables to actual binding variables
		for(auto e : this->g->Edges()) {
			auto vj = &e->getVertexDst();
			auto rj = this->rm->getResource(vj);
			if(this->commutativeOps.find(rj) == this->commutativeOps.end()) continue;
			auto vi = &e->getVertexSrc();
			auto ri = this->rm->getResource(vi);
			auto i = vi->getId();
			auto j = vj->getId();
			auto k = lifetimes[e];
			if(!this->quiet) {
				std::cout << "Creating edge commutativity constraint for " << vi->getName() << "->" << vj->getName()
									<< " with distance=" << e->getDistance() << std::endl;
			}
			for(int a = 0; a < resourceLimits[ri]; a++) {
				if(ri->isUnlimited() and a != unlimitedOpFUs[vi]) continue;
				auto m = fuIndexMap[{ri, a}];
				for(int b = 0; b < resourceLimits[rj]; b++) {
					if(rj->isUnlimited() and b != unlimitedOpFUs[vj]) continue;
					auto n = fuIndexMap[{rj, b}];
					for(int p = 0; p < numResourcePorts[rj]; p++) {
						ScaLP::Constraint constr = v_i_m[{i,m}] + v_i_m[{j,n}] + a_e_p[{e,p}] - r_m_n_k_p[{{m, n},{k, p}}] <= 2;
						solver.addConstraint(constr);
						if(!this->quiet) std::cout << "  Successfully added constraint " << constr << std::endl;
					}
				}
			}
		}
		if(!this->quiet) std::cout << "created commutativity constraints part 4" << std::endl;

		// each vertex is bound to exactly one resource
		for(auto v : this->g->Vertices()) {
			auto i = v->getId();
			auto r = this->rm->getResource(v);
			if(!this->quiet) std::cout << "Creating constraint for vertex " << v->getName() << " and resource " << r->getName() << std::endl;
			ScaLP::Term t;
			for(int a=0; a<resourceLimits[r]; a++) {
				if(r->isUnlimited() and a != unlimitedOpFUs[v]) continue;
				auto m = fuIndexMap[{r,a}];
				t += v_i_m[{i,m}];
			}
			ScaLP::Constraint constr = t == 1;
			solver.addConstraint(constr);
			if(!this->quiet) std::cout << "  Successfully added constraint " << constr << std::endl;
		}
		if(!this->quiet) std::cout << "created vertex-binding constraints" << std::endl;

		// one connection for each edge
		for(auto e : this->g->Edges()) {
			auto *vi = &e->getVertexSrc();
			auto *vj = &e->getVertexDst();
			auto ri = this->rm->getResource(vi);
			auto rj = this->rm->getResource(vj);
			auto i = vi->getId();
			auto j = vj->getId();
			auto k = lifetimes[e];
			if(!this->quiet) {
				std::cout << "Creating edge constraint for " << vi->getName() << "->" << vj->getName() << " with distance="
									<< e->getDistance() << std::endl;
			}
			for(int a = 0; a < resourceLimits[ri]; a++) {
				if(ri->isUnlimited() and a != unlimitedOpFUs[vi]) continue;
				auto m = fuIndexMap[{ri, a}];
				for(int b = 0; b < resourceLimits[rj]; b++) {
					if(rj->isUnlimited() and b != unlimitedOpFUs[vj]) continue;
					auto n = fuIndexMap[{rj, b}];
					ScaLP::Term t;
					if (this->commutativeOps.find(rj) != this->commutativeOps.end()) {
						/*
						// commutative operation
						for(auto p : possiblePortConnections[{m,n}]) {
							t += r_m_n_k_p[{{m, n},{k, p}}];
						}
						 */
						// skip them because they were already handled earlier
						continue;
					}
					else {
						// noncommutative operation
						auto p = this->portAssignments.at(e);
						t += r_m_n_k_p[{{m, n},{k, p}}];
					}
					ScaLP::Constraint constr = v_i_m[{i,m}] + v_i_m[{j,n}] - t <= 1;
					solver.addConstraint(constr);
					if(!this->quiet) std::cout << "  Successfully added constraint " << constr << std::endl;
				}
			}
		}
		if(!this->quiet) std::cout << "created edge connection constraints" << std::endl;

		// no resource conflicts
		for(auto r : this->rm->Resources()) {
			for(int a=0; a<resourceLimits[r]; a++) {
				auto m = fuIndexMap[{r, a}];
				auto vertices = this->rm->getVerticesOfResource(r);
				for(int t=0; t<this->II; t++) {
					ScaLP::Term lhs;
					bool constraintEmpty = true;
					for(auto v : vertices) {
						if(this->sched[const_cast<Vertex*>(v)] % this->II != t) continue;
						if(r->isUnlimited() and a != unlimitedOpFUs[const_cast<Vertex*>(v)]) continue;
						constraintEmpty = false;
						int i = v->getId();
						lhs += v_i_m[{i,m}];
					}
					if(constraintEmpty) continue;
					ScaLP::Constraint constr = lhs <= 1;
					solver.addConstraint(lhs <= 1);
					if(!this->quiet) std::cout << "  Successfully added constraint " << constr << std::endl;
				}
			}
		}
		if(!this->quiet) std::cout << "created resource conflict constraints" << std::endl;

		for (auto &it : r_m_n_k_p) {
			auto rVar = it.second;
			auto k = it.first.second.first;
			auto m = it.first.first.first;
			auto zVar = z_m[m];
			ScaLP::Constraint constr = (zVar - k*rVar >= 0);
			solver.addConstraint(constr);
			if(!this->quiet) std::cout << "  Successfully added constraint " << constr << std::endl;
		}
		if(!this->quiet) std::cout << "created lifetime register constraints" << std::endl;

		if (this->maxMux >= 0.0) {
			ScaLP::Term t;
			for(auto &it : r_m_n_k_p) {
				t += it.second;
			}
			ScaLP::Constraint constr = t <= this->maxMux;
			solver.addConstraint(constr);
			if(!this->quiet) std::cout << "  Successfully added constraint " << constr << std::endl;
		}

		if (this->maxReg >= 0.0) {
			ScaLP::Term t;
			for(auto &it : z_m) {
				t += it.second;
			}
			ScaLP::Constraint constr = t <= this->maxReg;
			solver.addConstraint(constr);
			if(!this->quiet) std::cout << "  Successfully added constraint " << constr << std::endl;
		}
		if(!this->quiet) std::cout << "created maximum mux/reg cost constraints if necessary" << std::endl;

		///////////////////
		// set objective //
		///////////////////
		ScaLP::Term object;
		if (wMuxInternally > 0) {
			for(auto &it : r_m_n_k_p) {
				object += (wMuxInternally * it.second);
			}
		}
		if (wRegInternally > 0) {
			for(auto &it : z_m) {
				object += (wRegInternally * it.second);
			}
		}
		switch (this->obj) {
			case Binding::objective::minimize: {
				solver.setObjective(ScaLP::minimize(object));
				break;
			}
			case Binding::objective::maximize: {
				solver.setObjective(ScaLP::maximize(object));
				break;
			}
			default: {
				throw HatScheT::Exception("OptimalIntegerIIBinding::bind: unknown objective - only minimize and maximize are supported");
			}
		}


		//////////////////
		// start solver //
		//////////////////
		if(!this->quiet) std::cout << solver.showLP() << std::endl;
		if(!this->quiet) std::cout << "start solving now" << std::endl;
		auto stat = solver.solve();
		this->binding.solutionStatus = ScaLP::showStatus(stat);
		if(!this->quiet) std::cout << "finished solving with status " << stat << std::endl;
		if(stat != ScaLP::status::FEASIBLE and stat != ScaLP::status::TIMEOUT_FEASIBLE and stat != ScaLP::status::OPTIMAL) {
			std::cout << "Binding::getILPBasedIntIIBinding: could not solve binding problem, ScaLP status " << stat
								<< std::endl;
			return; // empty binding
		}
		if(!this->quiet) std::cout << "start filling solution structure" << std::endl;

		/////////////////////////////
		// fill solution structure //
		/////////////////////////////
		this->binding.multiplexerCosts = 0.0;
		this->binding.registerCosts = 0.0;
		auto res = solver.getResult().values;
		// binding variables
		for(auto &it : v_i_m) {
			auto var = it.second;
			auto i = it.first.first;
			auto m = it.first.second;
			auto value = round(res[var]);
			if(value != 1.0) continue;
			auto v = &this->g->getVertexById(i);
			auto r = indexFuMap[m].first;
			auto fu = indexFuMap[m].second;
			this->binding.resourceBindings[v->getName()] = fu;
			if(!quiet) {
				std::cout << "Vertex " << v->getName() << " is bound to FU " << fu << " of resource type "
									<< r->getName() << std::endl;
			}
		}
		// connection variables
		for(auto &it : r_m_n_k_p) {
			auto var = it.second;
			auto m = it.first.first.first;
			auto n = it.first.first.second;
			auto k = it.first.second.first;
			auto p = it.first.second.second;
			auto value = round(res[var]);
			if(value != 1.0) continue;
			// ToDo: check if this connection can be omitted in case of a nonoptimal solution
			this->binding.multiplexerCosts += 1.0;
			auto ri = indexFuMap[m].first;
			auto fui = indexFuMap[m].second;
			auto rj = indexFuMap[n].first;
			auto fuj = indexFuMap[n].second;
			if(!quiet) {
				std::cout << "fu " << fui << " of resource type " << ri->getName() << " is connected to port " << p << " of fu "
									<< fuj << " of resource type " << rj->getName() << " over " << k << " lifetime registers" << std::endl;
			}
			this->binding.fuConnections.emplace_back(std::make_pair(std::make_pair(std::make_pair(ri->getName(),fui),std::make_pair(rj->getName(),fuj)),std::make_pair(k,p)));
		}

		// register costs
		for (auto &it : z_m) {
			this->binding.registerCosts += round(res[it.second]);
		}

		// port assignments
		// first: copy port assignments
		this->binding.portAssignments = this->portAssignments;
		// overwrite assignments for commutative operations
		for (auto &it : a_e_p) {
			if (not (bool)round(res[it.second])) {
				continue;
			}
			auto edge = it.first.first;
			auto port = it.first.second;
			this->binding.portAssignments[edge] = port;
		}

		if(!this->quiet) {
			std::cout << "total multiplexer costs: " << this->binding.multiplexerCosts << std::endl;
			std::cout << "total register costs: " << this->binding.registerCosts << std::endl;
		}
	}

	Binding::RegChainBindingContainer OptimalIntegerIIBinding::getBinding() {
		return this->binding;
	}

	void OptimalIntegerIIBinding::setMuxCostFactor(double wMux) {
		this->wMux = wMux;
	}

	void OptimalIntegerIIBinding::setRegCostFactor(double wReg) {
		this->wReg = wReg;
	}

	std::string OptimalIntegerIIBinding::getSolutionStatus() {
		return ScaLP::showStatus(this->status);
	}

	void OptimalIntegerIIBinding::setMuxLimit(double l) {
		this->maxMux = l;
	}

	void OptimalIntegerIIBinding::setRegLimit(double l) {
		this->maxReg = l;
	}

	void OptimalIntegerIIBinding::setObjective(Binding::objective o) {
		this->obj = o;
	}
}