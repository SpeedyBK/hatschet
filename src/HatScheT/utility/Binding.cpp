//
// Created by nfiege on 03/12/19.
//

#include "Binding.h"
#include <cmath>
#include <HatScheT/utility/Utility.h>
#include <sstream>
#ifdef USE_SCALP
#include <ScaLP/Solver.h>
#endif

namespace HatScheT {

	std::map<const Vertex *, int> Binding::getSimpleBinding(std::map<Vertex *, int> sched, ResourceModel *rm, int II) {
		std::map<const Vertex *, int> binding;
		std::map<const Resource*, std::map<int, int>> resourceCounters;

		for(auto it : sched) {
			auto v = it.first;
			const Resource* res = rm->getResource(v);
			if(res->getLimit()<0) {
				binding[v] = resourceCounters[res][0];
				resourceCounters[res][0]++;
			} else {
				int time = it.second % II;
				binding[v] = resourceCounters[res][time];
				if(resourceCounters[res][time] >= res->getLimit())
					throw HatScheT::Exception("Binding::getSimpleBinding: found resource conflict while creating binding for resource "
																		+ res->getName() + "(limit " + to_string(res->getLimit()) + " )");
				resourceCounters[res][time]++;
			}
		}
		return binding;
	}

	std::vector<std::map<const Vertex *, int>>
	Binding::getSimpleRationalIIBinding(std::vector<std::map<Vertex *, int>> sched, ResourceModel *rm, int M, int S) {
		if(sched.size() != S) {
			throw HatScheT::Exception("Binding::getSimpleRationalIIBinding: number of samples in the schedule ("+to_string(sched.size())+") is not equal to the number of samples ("+to_string(S)+")");
		}

		std::vector<std::map<const Vertex *, int>> binding;
		std::map<const Resource*, std::map<int, int>> resourceCounters;

		for(int s=0; s<S; ++s) {
			std::map<const Vertex *, int> tempBinding;

			// create binding for sample number 's'
			for(auto it : sched[s]) {
				auto v = it.first;
				const Resource* res = rm->getResource(v);
				if(res->getLimit()<0) {
					tempBinding[v] = resourceCounters[res][0];
					resourceCounters[res][0]++;
				} else {
					int time = it.second % M;
					tempBinding[v] = resourceCounters[res][time];
					if(resourceCounters[res][time] >= res->getLimit())
						throw HatScheT::Exception("Binding::getSimpleRationalIIBinding: found resource conflict while creating binding for resource "
																			+ res->getName() + "(limit " + to_string(res->getLimit()) + " )");
					resourceCounters[res][time]++;
				}
			}

			// insert into binding container
			binding.emplace_back(tempBinding);
		}

		return binding;
	}

#ifdef USE_SCALP
	vector<std::map<const Vertex *, int> >
	Binding::getILPBasedRatIIBinding(map<Vertex *, int> sched, Graph *g, ResourceModel *rm, int modulo,
																	 vector<int> initIntervalls, list<string> sw, int timeout) {
			// create and setup solver
			if(sw.empty()) sw = {"Gurobi","CPLEX","SCIP","LPSolve"};
			auto solver = ScaLP::Solver(sw);
			solver.quiet = true;
			solver.threads = 1;
			solver.timeout=timeout;

			int samples = initIntervalls.size();

			vector<std::map<const Vertex*,int> > ratIIBindings;
			ratIIBindings.resize(samples);

			// samples, vertex, resource unit (true/false)
			vector<map<const Vertex*, vector<ScaLP::Variable > > > binding_vars;
			binding_vars.resize(samples);

			//fill binding_vars
			for(int i = 0; i < samples; i++) {
				for (auto it = g->verticesBegin(); it != g->verticesEnd(); ++it) {
					Vertex *v = *it;
					const Resource *r = rm->getResource(v);
					//skip unlimited
					//TODO handle unlimited resources
					//TODO fix their binding variables using '==constraint' ?
					if (r->getLimit() == -1) continue;

					//generate ilp variables for binding
					vector<ScaLP::Variable> vars;
					for (int j = 0; j < r->getLimit(); j++) {
						vars.push_back(ScaLP::newBinaryVariable("t'" + v->getName() + "_s" + to_string(i) + "_r" + to_string(j)));
					}

					binding_vars[i].insert(make_pair(v, vars));
				}
			}

			//add explicitness constraint(1) from the paper
			for(auto it : binding_vars){
				for(auto it2 : it) {
					//TODO get here all vertices that use samples in this time step and combine them
					ScaLP::Term expl;
					for(auto it3 : it2.second) {
						expl += it3;
					}
					solver.addConstraint(expl == 1);
				}
			}

			//sort the vertices by the time slot they are scheduled
			//map<int, vector<const Vertex*> > v_timesorted;

			//add binding constraints
			//respect the MRT in respect to modulo slots and resource constraints
			for(auto it = rm->resourcesBegin(); it != rm->resourcesEnd(); ++it){
				const Resource* r = *it;
				set<const Vertex*> verts = rm->getVerticesOfResource(r);

				//sort all ILP variables that fall into the same modulo time slot
				map<int, vector<ScaLP::Variable > > variables_timesorted;

				//use scalp term for every hardware unit and MRT time slot
				vector<vector<ScaLP::Term> > slot_terms;
				for(int i = 0; i < r->getLimit(); i++) {
					vector<ScaLP::Term> terms;
					for (int j = 0; j < modulo; j++) {
						ScaLP::Term t;
						terms.push_back(t);
					}
					slot_terms.push_back(terms);
				}

				//add the ScaLP::Variables to the corresponding ScaLP::Term
				//this describes the MRT
				for(auto it2 : verts) {
					const Vertex* v = it2;
					int modulo_slot = 0;
					for(auto it3: sched) {
						Vertex* v2 = it3.first;
						if(v==v2) {
							int distance = 0;
							for(int i = 0; i < samples; i++){
								if(i>0) distance+=initIntervalls[i-1];

								modulo_slot = (it3.second + distance) % modulo;
								for(int j = 0; j < r->getLimit(); j++){
									slot_terms[j][modulo_slot] += binding_vars[i][v][j];
								}
							}
						}
					}

				}
				//every hardware unit can only perform one operation each time step
				//according to (3) / (4) in the paper
				//question : what about blocking time > 1 ? this should have harder constraints than a fixed '1' (patrick)
				for(auto it2: slot_terms){
					for(auto it3 : it2){
						solver.addConstraint(it3 <= 1);
					}
				}
			}

			//TODO add MUX constraints
			//this is an attempt for two layer if->else using big-M
			//for all inports -> boolean variable if there is a mux input needed for this input edge
			vector<vector<ScaLP::Variable > > mux_inputs;
			//for all inports: boolean variable if mux is needed at all
			vector<ScaLP::Variable> mux_number;

			//TODO model mux_inputs
			for(auto it = g->verticesBegin(); it != g->verticesEnd(); ++it){
				Vertex* v = *it;
				set<Vertex*> pred = g->getPredecessors(v);

				//TODO experimental, debugging reasons
				if(pred.size() > 1) continue;
				list<const Edge*> edges = g->getEdges(v, *pred.begin());
				const Edge* e = *edges.begin();

				//TODO if
			}

			//TODO model total mux number

			//TODO add objective

			//write lp file
			solver.writeLP("binding.lp");

			//solve the problem
			ScaLP::status stat = solver.solve();
			cout << "Utility.getILPBasedRatIIBinding: The binding problem was solved: " << stat << endl;
			cout << solver.getResult() << endl;

			//add the solution to return container and print the binding
			//TODO include check for consistency / feasible binding
			int sample=0;
			for(auto it : binding_vars){
				for(auto it2 : it) {
					const Vertex* v = it2.first;
					int unit = 0;
					for(auto it3 : it2.second) {
						if(solver.getResult().values[it3] == true) {
							//add to solution structure
							ratIIBindings[sample][v] = unit;

							cout << "Bound " << v->getName() << "_s" << to_string(sample) << " to unit " << to_string(unit) << endl;
						}
						else unit++;
					}
				}
				sample++;
			}

			Utility::printRationalIIMRT(sched, ratIIBindings, rm, modulo, initIntervalls);

			//throw error, this binding function is not finished yet
			cout << "Utility.getILPBasedRatIIBinding: this binding function is not finished yet" << endl;
			throw Exception("Utility.getILPBasedRatIIBinding: this binding function is not finished yet");

			return ratIIBindings;
	}

	std::map<const Vertex *, int>
	Binding::getILPMinRegBinding(map<Vertex *, int> sched, Graph *g, ResourceModel *rm, int II, list<string> sw,
															 int timeout) {
		// container to return
		map<const Vertex*,int> binding;
		// create solver
		if(sw.empty()) sw = {"Gurobi","CPLEX","SCIP","LPSolve"};
		auto solver = ScaLP::Solver(sw);
		solver.quiet = true;
		solver.threads = 1;
		solver.timeout = timeout;

		// count if less FUs are needed that allocated
		std::map<const Resource*,int> resourceLimits;
		for(auto &r : rm->Resources()) {
			resourceLimits[r] = 0;
			if(r->isUnlimited()) {
				resourceLimits[r] = rm->getVerticesOfResource(r).size();
				std::cout << "Resource limits for resource '" << r->getName() << "': " << resourceLimits[r] << std::endl;
				continue;
			}
			std::map<int,int> modSlotHeight;
			for(int i=0; i<II; i++) {
				modSlotHeight[i] = 0;
			}
			for(auto &it : sched) {
				auto v = it.first;
				if(rm->getResource(v) != r) continue;
				auto modSlot = it.second % II;
				modSlotHeight[modSlot]++;
				std::cout << "Mod slot for vertex '" << v->getName() << "' = " << modSlot << " = " << it.second << " mod "
					<< II << std::endl;
				std::cout << "  mod slot height = " << modSlotHeight[modSlot] << std::endl;
			}
			for(auto it : modSlotHeight) {
				if(it.second > resourceLimits[r]) resourceLimits[r] = it.second;
			}
			std::cout << "Resource limits for resource '" << r->getName() << "': " << resourceLimits[r] << std::endl;
		}

		// create vertex-binding variables
		std::map<Vertex*,std::vector<ScaLP::Variable>> vertexVariables;
		std::map<const Resource*,std::vector<ScaLP::Variable>> registerVariables;
		std::map<const Resource*,std::list<Vertex*>> sameResources;
		std::map<const Resource*,int> unlimitedResourceCounter;
		for(auto &it : sched){
			auto res = rm->getResource(it.first);
			try{
				sameResources.at(res).emplace_back(it.first);
			}
			catch(std::out_of_range&){
				sameResources[res] = {it.first};
				registerVariables[res] = std::vector<ScaLP::Variable>();
			}
			//int limit = res->getLimit();
			if(res->isUnlimited()) {
				binding[it.first] = unlimitedResourceCounter[res];
				unlimitedResourceCounter[res]++;
				continue;
			}
			int limit = resourceLimits[res];

			vertexVariables[it.first] = std::vector<ScaLP::Variable>();
			for(auto i = 0; i<limit; i++){
				vertexVariables[it.first].emplace_back(ScaLP::newIntegerVariable(it.first->getName()+"_"+to_string(i),0,1));
			}
		}

		// create register variables
		for(auto &it : registerVariables){
			auto res = it.first;
			//auto limit = res->getLimit();
			int limit = resourceLimits[res];
			if(res->isUnlimited()) limit = UNLIMITED;
			for(int i=0; i<limit; i++){
				auto var = ScaLP::newIntegerVariable(res->getName()+to_string(i),0,ScaLP::INF());
				it.second.emplace_back(var);
			}
		}

		// calculate lifetimes
		std::map<Vertex*,int> lifetimes;
		for(auto &it : g->Edges()){
			auto* src = &it->getVertexSrc();
			auto* dst = &it->getVertexDst();
			int tempLifetime = sched[dst] - sched[src] - rm->getResource(src)->getLatency() + (II * it->getDistance());
			if(tempLifetime>lifetimes[src]) lifetimes[src] = tempLifetime;
		}

		// check, which vertices can potetially be bind to the same resource at the same control step
		std::map<const Resource*,std::map<int,std::list<Vertex*>>> potentiallySame;
		for(auto &it : sched){
			auto vert = it.first;
			auto timepoint = it.second % II;
			auto res = rm->getResource(vert);
			if(potentiallySame.find(res)==potentiallySame.end())
				potentiallySame[res] = std::map<int,std::list<Vertex*>>();
			try{
				potentiallySame[res].at(timepoint).emplace_back(vert);
			}
			catch(std::out_of_range&){
				potentiallySame[res][timepoint] = {vert};
			}
		}

		// create constraints: bind each vertex to EXACTLY one resource
		for(auto &it1 : vertexVariables){
			ScaLP::Term t;
			for(auto &it2 : it1.second){
				t += it2;
			}
			auto c = ScaLP::Constraint(t == 1);
			solver << c;
		}

		// create constraints: bind AT MOST one vertex to each resource in each control step
		for(auto &it1 : potentiallySame){
			auto res = it1.first;
			for(auto &it2 : it1.second){
				int limit = resourceLimits[res];
				if(res->isUnlimited()) limit = UNLIMITED;
				for(int resourceCounter = 0; resourceCounter<limit; resourceCounter++){
					ScaLP::Term t;
					for(auto &it3 : it2.second){
						t += vertexVariables[it3][resourceCounter];
					}
					auto c = ScaLP::Constraint(t <= 1);
					solver << c;
				}
			}
		}

		// create edge register constraints
		for(auto &it1 : registerVariables){
			auto res = it1.first;
			for(int resoureCounter = 0; resoureCounter<it1.second.size(); resoureCounter++){
				auto var = it1.second[resoureCounter];
				for(auto &it2 : sameResources[res]){
					auto c = ScaLP::Constraint(var - (lifetimes[it2] * vertexVariables[it2][resoureCounter]) >= 0);
					solver << c;
				}
			}
		}

		// minimize the sum of all registers
		ScaLP::Term obj;
		for(auto &it1 : registerVariables){
			for(auto &it2 : it1.second){
				obj += it2;
			}
		}
		solver.setObjective(ScaLP::minimize(obj));

		// solve and put results into binding map
		try{
			auto status = solver.solve();
			if(status != ScaLP::status::OPTIMAL && status != ScaLP::status::FEASIBLE && status != ScaLP::status::TIMEOUT_FEASIBLE){
				cout << "Utility::getILPMinRegBinding: didn't find solution, returning simple binding" << endl;
				return Binding::getSimpleBinding(sched,rm,II);
			}
		}
		catch(ScaLP::Exception& e){
			cout << "Utility::getILPMinRegBinding: caught ScaLP exception: '" << std::string(e.what()) << "' returning simple binding";
			return Binding::getSimpleBinding(sched,rm,II);
		}

		auto results = solver.getResult().values;
		for(auto &it1 : vertexVariables){
			auto vertex = it1.first;
			for(int i = 0; i < (int)it1.second.size(); i++){
				auto &it2 = it1.second[i];
				auto val = results[it2];
				if(val==1.0){
					binding[vertex] = i;
				}
			}
		}

		return binding;
	}

	int Binding::countTotalLifetimeRegisters(Graph *g, ResourceModel *rm, std::vector<std::map<Vertex *, int>> schedule,
																					 std::vector<std::map<Vertex *, int>> binding, int M, int S, bool quiet) {
		// total number of lifetime registers
		int totalRegs = 0;

		// each implemented resource has a chain of lifetime registers attached to it
		std::map<const Resource*,std::map<int,int>> regs;

		// init lifetime register map
		for (auto& r : rm->Resources()) {
			regs[r] = std::map<int,int>();
			for (auto& v : rm->getVerticesOfResource(r)) {
				for (auto s=0; s<S; ++s) {
					auto vnc = const_cast<Vertex*>(v);
					auto fu = binding[s][vnc];
					regs[r][fu] = 0;
				}
			}
		}

		// iterate over edges and count lifetime registers
		for(auto s=0; s<S; ++s) {
			for (auto& e : g->Edges()) {
				// get sample index and new distance for unrolled graph
				auto sampleIndexOffset = Utility::getSampleIndexAndOffset(e->getDistance(),s,S,M);
				auto sSrc = sampleIndexOffset.first;
				auto offset = sampleIndexOffset.second; // = distance in unrolled graph times M
				// vertices and start times
				auto &vsrc = e->getVertexSrc();
				auto &vdst = e->getVertexDst();
				auto tsrc = schedule[sSrc][&vsrc];
				auto tdst = schedule[s][&vdst];
				// get implemented resource
				auto *r = rm->getResource(&vsrc);
				auto lat = r->getLatency();
				auto fu = binding[s][&vsrc];
				// check if current number of lifetime regs are enough to support that edge
				auto neededRegs = tdst - lat - tsrc + offset;
				// debug messages
				if(!quiet) {
					std::cout << "Binding::countTotalLifetimeRegisters:" << std::endl;
					std::cout << "  edge '" << vsrc.getName() << "' -> '" << vdst.getName() << "'" << std::endl;
					std::cout << "  edge distance: " << e->getDistance() << std::endl;
					std::cout << "    dst sample: " << s << std::endl;
					std::cout << "    S: " << S << std::endl;
					std::cout << "    M: " << M << std::endl;
					std::cout << "    src sample: " << sSrc << std::endl;
					std::cout << "    offset (distance in unrolled graph times M): " << offset << std::endl;
					std::cout << "  tsrc: " << tsrc << std::endl;
					std::cout << "  tdst: " << tdst << std::endl;
					std::cout << "  FU: " << fu << std::endl;
					std::cout << "  latency: " << lat << std::endl;
					std::cout << "  => needed regs = " << tdst << " - " << lat << " - " << tsrc << " + " << offset << " = " << neededRegs << std::endl;
				}
				if (neededRegs > regs[r][fu]) {
					regs[r][fu] = neededRegs;
				}
			}
		}

		// sum them up
		for (auto &it1 : regs) {
			for (auto &it2 : it1.second) {
				totalRegs += it2.second;
			}
		}

		// return that badboy
		return totalRegs;
	}

	int Binding::countTotalLifetimeRegisters(Graph *g, ResourceModel *rm, std::map<Vertex *, int> schedule,
																					 std::map<Vertex *, int> binding, int II, bool quiet) {
		return Binding::countTotalLifetimeRegisters(g,rm,{schedule},{binding},II,1,quiet);
	}

	Binding::BindingContainer
	Binding::getILPBasedIntIIBinding(map<Vertex *, int> sched, Graph *g, ResourceModel *rm, int II, list<string> sw,
																	 int timeout) {
		///////////////////////////////////////////////
		// find conflicting operations and variables //
		///////////////////////////////////////////////

		// resource compatibility graph
		Graph resourceCompatibilityGraph;
		// create vertices
		for(auto &v : g->Vertices()) {
			resourceCompatibilityGraph.createVertex(v->getId());
		}
		// check compatibility with other vertices
		for(auto &v : g->Vertices()) {
			for (auto &it : sched) {
				// no edge with itself
				if(v == it.first) continue;
				// source vertex: the one who comes first in the schedule
				if (sched[v] > it.second) continue;
				// incompatible if resource types are different
				// ACTUALLY I think they should always be compatible in that case...
				//if (rm->getResource(v) != rm->getResource(it.first)) continue;
				// incompatible if modulo slots are equal and resource types are also equal
				if ((sched[v] % II == it.second % II) and (rm->getResource(v) == rm->getResource(it.first))) {
					std::cout << "Vertices '" << v->getName() << "' and '" << it.first->getName() << "' are INCOMPATIBLE because modulo slots overlap" << std::endl;
					std::cout << "  " << v->getName() << " t = " << sched[v] << " mod " << II << " = " << sched[v] % II << std::endl;
					std::cout << "  " << it.first->getName() << " t = " << it.second << " mod " << II << " = " << it.second % II << std::endl;
					continue;
				}
				// create edge
				std::cout << "Vertices '" << v->getName() << "' and '" << it.first->getName() << "' are COMPATIBLE because modulo slots DO NOT overlap" << std::endl;
				std::cout << "  " << v->getName() << " t = " << sched[v] << " mod " << II << " = " << sched[v] % II << std::endl;
				std::cout << "  " << it.first->getName() << " t = " << it.second << " mod " << II << " = " << it.second % II << std::endl;
				resourceCompatibilityGraph.createEdge(resourceCompatibilityGraph.getVertexById(v->getId()),resourceCompatibilityGraph.getVertexById(it.first->getId()));
			}
		}

		// count minimum number of needed registers to store info
		// (NOT IN PAPER - IN PAPER IT WAS AN INPUT TO THE ILP FORMULATION)
		int minRegs = 0;
		// variable compatibility graph
		Graph variableCompatibilityGraph;
		// create vertices
		for(auto &v : g->Vertices()) {
			variableCompatibilityGraph.createVertex(v->getId());
		}
		// check compatibility with other vertices
		for(auto &v : g->Vertices()) {
			// calc lifetime of v
			int v1Lat = rm->getResource(v)->getLatency();
			int v1LifeStart = sched[v] + v1Lat;
			int v1LifeEnd = 0;
			int numIncompatibilities = 0;
			for(auto &e : g->Edges()) {
				if(e->getVertexSrc().getId() != v->getId()) continue;
				auto dstStart = sched[&e->getVertexDst()] + e->getDistance() * II;
				if(dstStart > v1LifeEnd) v1LifeEnd = dstStart;
			}
			bool v1Omnicompatible = (v1LifeStart == v1LifeEnd);
			for(auto &it : sched) {
				// calc lifetime of v
				int v2Lat = rm->getResource(it.first)->getLatency();
				int v2LifeStart = it.second + v2Lat;
				int v2LifeEnd = 0;
				for(auto &e : g->Edges()) {
					if(e->getVertexSrc().getId() != it.first->getId()) continue;
					auto dstStart = sched[&e->getVertexDst()] + e->getDistance() * II;
					if(dstStart > v2LifeEnd) v2LifeEnd = dstStart;
				}
				bool v2Omnicompatible = (v2LifeStart == v2LifeEnd);
				// incompatible if lifetimes overlap
				if(!v1Omnicompatible and !v2Omnicompatible and ((v2LifeStart >= v1LifeStart and v2LifeStart <= v1LifeEnd) or (v2LifeEnd >= v1LifeStart and v2LifeEnd <= v1LifeEnd))) {
					std::cout << "Vertices '" << v->getName() << "' and '" << it.first->getName() << "' are INCOMPATIBLE because lifetimes overlap" << std::endl;
					std::cout << "  " << v->getName() << ": " << v1LifeStart << " -> " << v1LifeEnd << " with latency " << v1Lat << std::endl;
					std::cout << "  " << it.first->getName() << ": " << v2LifeStart << " -> " << v2LifeEnd << " with latency " << v2Lat << std::endl;
					numIncompatibilities++;
					continue;
				}
				// no edge with itself
				if(v == it.first) continue;
				// source vertex: the one who comes first in the schedule
				if(sched[v] > it.second) continue;
				// create edge
				std::cout << "Vertices '" << v->getName() << "' and '" << it.first->getName() << "' are COMPATIBLE because lifetimes DO NOT overlap" << std::endl;
				std::cout << "  " << v->getName() << ": " << v1LifeStart << " -> " << v1LifeEnd << " with latency " << v1Lat << std::endl;
				std::cout << "  " << it.first->getName() << ": " << v2LifeStart << " -> " << v2LifeEnd << " with latency " << v2Lat << std::endl;
				variableCompatibilityGraph.createEdge(variableCompatibilityGraph.getVertexById(v->getId()),variableCompatibilityGraph.getVertexById(it.first->getId()));
			}
			// update minimum number of registers if necessary
			std::cout << "Incompatibilities for vertex '" << v->getName() << "': " << numIncompatibilities << std::endl;
			if(numIncompatibilities > minRegs) minRegs = numIncompatibilities;
		}

		std::cout << "Minimum number of needed registers = " << minRegs << std::endl;

		///////////////////
		// set up solver //
		///////////////////
		ScaLP::Solver s(sw);
		if(timeout >= 0) {
			s.timeout = timeout;
		}
		s.quiet = true;

		//////////////////////
		// create variables //
		//////////////////////

		// boolean variables whether vertex v_i is bound to functional unit k of resource type r (x_i_k)
		std::map<std::pair<int,int>,ScaLP::Variable> x_i_k;
		for(auto &v : g->Vertices()) {
			auto r = rm->getResource(v);
			auto limit = r->getLimit();
			if(r->isUnlimited()) limit = rm->getVerticesOfResource(r).size();
			for(int k=0; k<limit; ++k) {
				auto i = v->getId();
				x_i_k[{i,k}] = ScaLP::newBinaryVariable("x_" + to_string(i) + "_" + to_string(k));
				std::cout << "  Created variable '" << x_i_k[{i,k}] << "'" << std::endl;
			}
		}
		std::cout << "Created x_i_k variables" << std::endl;
		s.showLP(); // this line does nothing - only for debugging

		// boolean variables whether variable of v_i is bound to register k (y_i_l)
		std::map<std::pair<int,int>,ScaLP::Variable> y_i_l;
		for(auto &v : g->Vertices()) {
			for(int l=0; l<minRegs; ++l) {
				auto i = v->getId();
				y_i_l[{i,l}] = ScaLP::newBinaryVariable("y_" + to_string(i) + "_" + to_string(l));
				std::cout << "  Created variable '" << y_i_l[{i,l}] << "'" << std::endl;
			}
		}
		std::cout << "Created y_i_l variables" << std::endl;
		s.showLP(); // this line does nothing - only for debugging

		// boolean variables for each FU -> register connection (c_r_k_l) for each resource type r
		std::map<std::pair<std::string,std::pair<int,int>>,ScaLP::Variable> c_r_k_l;
		for(auto &r : rm->Resources()) {
			int resLimit = r->getLimit();
			if(r->isUnlimited()) resLimit = rm->getVerticesOfResource(r).size();
			for(int k=0; k<resLimit; ++k) {
				for(int l=0; l<minRegs; ++l) {
					c_r_k_l[{r->getName(),{k,l}}] = ScaLP::newBinaryVariable("c_" + r->getName() + "_" + to_string(k) + "_" + to_string(l));
					std::cout << "  Created variable '" << c_r_k_l[{r->getName(),{k,l}}] << "'" << std::endl;
				}
			}
		}
		std::cout << "Created c_r_k_l variables" << std::endl;
		s.showLP(); // this line does nothing - only for debugging

		// number of inputs of each resource type
		std::map<const Resource*, int> numResourcePorts;

		// boolean variables for each register -> FU connection for each port (n) of that FU (a_r_n_k_l)
		std::map<std::pair<std::pair<std::string,int>,std::pair<int,int>>,ScaLP::Variable> a_r_n_k_l;
		for(auto &r : rm->Resources()) {
			int resLimit = r->getLimit();
			auto vertices = rm->getVerticesOfResource(r);
			if(r->isUnlimited()) resLimit = vertices.size();
			// compute number of inputs for that resource
			int numPorts = 0;
			for(auto &v : vertices) {
				int numInputs = 0;
				for(auto &e : g->Edges()) {
					if(e->getDependencyType() == Edge::DependencyType::Precedence) continue; // skip chaining edges
					if(&e->getVertexDst() != v) continue;
					numInputs++;
				}
				if(numInputs > numPorts) numPorts = numInputs;
			}
			numResourcePorts[r] = numPorts;
			// actually create variables
			for(int k=0; k<resLimit; ++k) {
				for(int l=0; l<minRegs; ++l) {
					for(int n=0; n<numPorts; ++n) {
						a_r_n_k_l[{{r->getName(),n},{k,l}}] = ScaLP::newBinaryVariable("a_" + r->getName() + "_" + to_string(n)
							+ "_" + to_string(k) + "_" + to_string(l));
						std::cout << "  Created variable '" << a_r_n_k_l[{{r->getName(),n},{k,l}}] << "'" << std::endl;
					}
				}
			}
		}
		std::cout << "Created a_r_n_k_l variables" << std::endl;
		s.showLP(); // this line does nothing - only for debugging

		// boolean variables for each FU -> FU connection for each port of that FU (needed for edges with lifetime=0) (b_r1_r2_k1_k2_n)
		std::map<std::pair<std::pair<std::pair<std::string,std::string>,std::pair<int,int>>,int>,ScaLP::Variable> b_r1_r2_k1_k2_n;
		for(auto &r1 : rm->Resources()) {
			int resLimit = r1->getLimit();
			auto vertices = rm->getVerticesOfResource(r1);
			if(r1->isUnlimited()) resLimit = vertices.size();
			// compute number of inputs for that resource
			int numPorts = 0;
			for(auto &v : vertices) {
				int numInputs = 0;
				for(auto &e : g->Edges()) {
					if(e->getDependencyType() == Edge::DependencyType::Precedence) continue; // skip chaining edges
					if(&e->getVertexDst() != v) continue;
					numInputs++;
				}
				if(numInputs > numPorts) numPorts = numInputs;
			}
			// actually create variables
			for(int k1=0; k1<resLimit; ++k1) {
				for(int n=0; n<numPorts; ++n) {
					for(auto &r2 : rm->Resources()) {
						int res2Limit = r2->getLimit();
						if(r2->isUnlimited()) res2Limit = rm->getVerticesOfResource(r2).size();
						for(int k2=0; k2<res2Limit; ++k2) {
							b_r1_r2_k1_k2_n[{{{r1->getName(),r2->getName()},{k1,k2}},n}] = ScaLP::newBinaryVariable("b_" + r1->getName() + "_" + r2->getName() + "_" + to_string(k1) + "_" + to_string(k2) + "_" + to_string(n));
							std::cout << "  Created variable '" << b_r1_r2_k1_k2_n[{{{r1->getName(),r2->getName()},{k1,k2}},n}] << "'" << std::endl;
						}
					}
				}
			}
		}
		std::cout << "Created b_r1_r2_k1_k2_n variables" << std::endl;
		s.showLP(); // this line does nothing - only for debugging

		// number of MUX inputs for each FU and each port (m_r_k_n)
		std::map<std::pair<std::string,std::pair<int,int>>,ScaLP::Variable> m_r_k_n;
		for(auto &r : rm->Resources()) {
			int resLimit = r->getLimit();
			auto vertices = rm->getVerticesOfResource(r);
			if(r->isUnlimited()) resLimit = vertices.size();
			// compute number of inputs for that resource
			int numPorts = 0;
			for(auto &v : vertices) {
				int numInputs = 0;
				for(auto &e : g->Edges()) {
					if(e->getDependencyType() == Edge::DependencyType::Precedence) continue; // skip chaining edges
					if(&e->getVertexDst() != v) continue;
					numInputs++;
				}
				if(numInputs > numPorts) numPorts = numInputs;
			}
			// actually create variables
			for(int k=0; k<resLimit; ++k) {
				for(int n=0; n<numPorts; ++n) {
					m_r_k_n[{r->getName(),{k,n}}] = ScaLP::newIntegerVariable("m_" + r->getName() + "_" + to_string(k) + "_" + to_string(n),0,ScaLP::INF());
					std::cout << "  Created variable '" << m_r_k_n[{r->getName(),{k,n}}] << "'" << std::endl;
				}
			}
		}
		std::cout << "Created m_r_k_n variables" << std::endl;
		s.showLP(); // this line does nothing - only for debugging

		// number of MUX inputs for each register (w_l)
		std::map<int,ScaLP::Variable> w_l;
		for(int l=0; l<minRegs; l++) {
			w_l[l] = ScaLP::newIntegerVariable("w_" + to_string(l));
			std::cout << "  Created variable '" << w_l[l] << "'" << std::endl;
		}
		std::cout << "Created w_l variables" << std::endl;
		s.showLP(); // this line does nothing - only for debugging

		////////////////////////
		// create constraints //
		////////////////////////

		// each operation is bound to one resource
		for(auto &v : g->Vertices()) {
			ScaLP::Term t;
			auto i = v->getId();
			auto r = rm->getResource(v);
			int resLimit = r->getLimit();
			auto vertices = rm->getVerticesOfResource(r);
			if(r->isUnlimited()) resLimit = vertices.size();
			for(int k=0; k<resLimit; ++k) {
				t += x_i_k[{i,k}];
			}
			s.addConstraint(t == 1);
		}
		std::cout << "Created x_i_k=1 constraints" << std::endl;
		s.showLP(); // this line does nothing - only for debugging

		// each variable is bound to one resource (if registers are needed)
		if(minRegs >= 1) {
			for(auto &v : g->Vertices()) {
				ScaLP::Term t;
				auto i = v->getId();
				for(int l=0; l<minRegs; ++l) {
					t += y_i_l[{i,l}];
				}
				s.addConstraint(t == 1);
			}
		}

		std::cout << "Created y_i_l=1 constraints" << std::endl;
		s.showLP(); // this line does nothing - only for debugging

		// no overlaps in operations (resource conflict graph)
		for(auto &v : g->Vertices()) {
			std::cout << "vertex: " << v->getName() << std::endl;
			// find conflicting operations
			auto predecessors = resourceCompatibilityGraph.getPredecessors(&resourceCompatibilityGraph.getVertexById(v->getId()));
			auto successors = resourceCompatibilityGraph.getSuccessors(&resourceCompatibilityGraph.getVertexById(v->getId()));
			std::set<int> conflictVertexIDs;
			for(auto &v2 : resourceCompatibilityGraph.Vertices()) {
				// no conflict with itself
				if(v->getId() == v2->getId()) continue;
				// no conflict with predecessors in compatibility graph
				if(predecessors.find(v2) != predecessors.end()) continue;
				// no conflict with successors in compatibility graph
				if(successors.find(v2) != successors.end()) continue;
				// store ID of conflict vertex
				conflictVertexIDs.insert(v2->getId());
				std::cout << "  conflict vertex: " << v2->getName() << std::endl;
			}
			// create constraints for all vertices with conflict
			auto i = v->getId();
			auto r = rm->getResource(v);
			int resLimit = r->getLimit();
			auto vertices = rm->getVerticesOfResource(r);
			if(r->isUnlimited()) resLimit = vertices.size();
			for(int k = 0; k<resLimit; k++) {
				for(auto j : conflictVertexIDs) {
					auto constraint = (x_i_k[{i,k}] + x_i_k[{j,k}]) <= 1;
					s.addConstraint(constraint);
					std::cout << "  Added constraint '" << constraint << "'" << std::endl;
				}
			}
		}
		std::cout << "Created no resource overlap constraints" << std::endl;
		s.showLP(); // this line does nothing - only for debugging

		// no overlaps in registers (variable conflict graph)
		// no overlaps in operations (resource conflict graph)
		for(auto &v : g->Vertices()) {
			// find conflicting operations
			auto predecessors = variableCompatibilityGraph.getPredecessors(&variableCompatibilityGraph.getVertexById(v->getId()));
			auto successors = variableCompatibilityGraph.getSuccessors(&variableCompatibilityGraph.getVertexById(v->getId()));
			std::set<int> conflictVertexIDs;
			for(auto &v2 : variableCompatibilityGraph.Vertices()) {
				// no conflict with itself
				if(v->getId() == v2->getId()) continue;
				// no conflict with predecessors in compatibility graph
				if(predecessors.find(v2) != predecessors.end()) continue;
				// no conflict with successors in compatibility graph
				if(successors.find(v2) != successors.end()) continue;
				// store ID of conflict vertex
				conflictVertexIDs.insert(v2->getId());
			}
			// create constraints for all vertices with conflict
			auto i = v->getId();
			for(int l = 0; l<minRegs; l++) {
				for(auto j : conflictVertexIDs) {
					s.addConstraint((y_i_l[{i,l}] + y_i_l[{j,l}]) <= 1);
				}
			}
		}
		std::cout << "Created no variable overlap constraints" << std::endl;
		s.showLP(); // this line does nothing - only for debugging

		// A) for edges with lifetime > 0
		for(auto &e : g->Edges()) {
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			auto lifetime = sched[vDst] - sched[vSrc] - rm->getResource(vSrc)->getLatency() + (II * e->getDistance());
			if(lifetime < 0) {
				throw HatScheT::Exception("Detected lifetime < 0 from '" + vSrc->getName() + "' (t='" + to_string(sched[vSrc])
				  + "', lat='" + to_string(rm->getResource(vSrc)->getLatency()) + "', distance='" + to_string(e->getDistance())
				  + "', II='" + to_string(II) + "') to '" + vDst->getName() + "' (t='" + to_string(sched[vDst]) + "')");
			}
			// edges with lifetime = 0 are handled in B)
			if(lifetime == 0) continue;

			// A) check if there is a connection from resource instance k of resource r (source) to register l
			auto rSrc = rm->getResource(vSrc);
			auto i = vSrc->getId();
			int resLimitSrc = rSrc->getLimit();
			auto verticesSrc = rm->getVerticesOfResource(rSrc);
			if(rSrc->isUnlimited()) resLimitSrc = verticesSrc.size();
			for(int k=0; k<resLimitSrc; k++) {
				for(int l=0; l<minRegs; ++l) {
					s.addConstraint(x_i_k[{i,k}] + y_i_l[{i,l}] - c_r_k_l[{rSrc->getName(),{k,l}}] <= 1);
				}
			}

			// A) check if there is a connection from register l to port n of resource instance k of resource r (destination)
			auto rDst = rm->getResource(vDst);
			int resLimitDst = rDst->getLimit();
			auto verticesDst = rm->getVerticesOfResource(rDst);
			if(rDst->isUnlimited()) resLimitDst = verticesDst.size();
			auto j = vDst->getId();
			for(int k=0; k<resLimitDst; k++) {
				for(int n=0; n<numResourcePorts[rDst]; n++) {
					for(int l=0; l<minRegs; l++) {
						s.addConstraint(y_i_l[{i,l}] + x_i_k[{j,k}] - a_r_n_k_l[{{rDst->getName(),n},{k,l}}] <= 1);
					}
				}
			}
		}
		std::cout << "Created edge constraints for lifetime > 0" << std::endl;
		s.showLP(); // this line does nothing - only for debugging

		// B) for edges with lifetime = 0
		for(auto &e : g->Edges()) {
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			auto lifetime = sched[vDst] - sched[vSrc] - rm->getResource(vSrc)->getLatency() + (II * e->getDistance());
			if (lifetime < 0) {
				throw HatScheT::Exception("Detected lifetime < 0 from '" + vSrc->getName() + "' (t='" + to_string(sched[vSrc])
																	+ "', lat='" + to_string(rm->getResource(vSrc)->getLatency()) + "', distance='" +
																	to_string(e->getDistance())
																	+ "', II='" + to_string(II) + "') to '" + vDst->getName() + "' (t='" +
																	to_string(sched[vDst]) + "')");
			}
			// edges with lifetime > 0 are handled in A)
			if (lifetime > 0) continue;

			// B) check if there is a connection from resource instance k2 of resource r2 to port n of resource instance k1 of resource r1
			//b_r1_r2_k1_k2_n
			auto *r2 = rm->getResource(vSrc);
			auto *r1 = rm->getResource(vDst);
			auto i = vSrc->getId();
			auto j = vDst->getId();

			int resLimitSrc = r2->getLimit();
			auto verticesSrc = rm->getVerticesOfResource(r2);
			if(r2->isUnlimited()) resLimitSrc = verticesSrc.size();
			int resLimitDst = r1->getLimit();
			auto verticesDst = rm->getVerticesOfResource(r1);
			if(r1->isUnlimited()) resLimitDst = verticesDst.size();

			for(int k1=0; k1<resLimitDst; k1++) {
				for(int k2=0; k2<resLimitSrc; k2++) {
					for(int n=0; n<numResourcePorts[r1]; n++) {
						s.addConstraint(x_i_k[{i,k2}] + x_i_k[{j,k1}] - b_r1_r2_k1_k2_n[{{{r1->getName(),r2->getName()},{k1,k2}},n}] <= 1);
					}
				}
			}
		}
		std::cout << "Created edge constraints for lifetime = 0" << std::endl;
		s.showLP(); // this line does nothing - only for debugging

		// calculate number of MUX inputs per port for each FU (m_r_k_n)
		for(auto r : rm->Resources()) {
			int resLimit = r->getLimit();
			auto vertices = rm->getVerticesOfResource(r);
			if(r->isUnlimited()) resLimit = vertices.size();
			for(int k=0; k<resLimit; k++) {
				// count number of inputs for each port of this FU separately
				for(int n=0; n<numResourcePorts[r]; n++) {
					ScaLP::Term portSum;
					// sum up all connections from registers to this FU
					for(int l=0; l<minRegs; l++) {
						portSum += a_r_n_k_l[{{r->getName(),n},{k,l}}];
					}
					// sum up all connections from other FUs to this FU
					auto r1 = r;
					auto k1 = k;
					for(auto r2 : rm->Resources()) {
						int resLimit2 = r2->getLimit();
						auto vertices2 = rm->getVerticesOfResource(r2);
						if(r2->isUnlimited()) resLimit2 = vertices2.size();
						for(int k2=0; k2<resLimit2; k2++) {
							portSum += b_r1_r2_k1_k2_n[{{{r1->getName(),r2->getName()},{k1,k2}},n}];
						}
					}
					s.addConstraint(portSum - m_r_k_n[{r->getName(),{k,n}}] == 0);
				}
			}
		}
		std::cout << "Created number of resource mux input constraints" << std::endl;
		s.showLP(); // this line does nothing - only for debugging

		// calculate number of MUX inputs per port for each register
		for(int l=0; l<minRegs; l++) {
			ScaLP::Term portSum;
			// c_r_k_l
			for(auto r : rm->Resources()) {
				int resLimit = r->getLimit();
				auto vertices = rm->getVerticesOfResource(r);
				if(r->isUnlimited()) resLimit = vertices.size();
				for(int k=0; k<resLimit; k++) {
					portSum += c_r_k_l[{r->getName(),{k,l}}];
				}
			}
			s.addConstraint(portSum - w_l[l] == 0);
		}
		std::cout << "Created number of register mux input constraints" << std::endl;
		s.showLP(); // this line does nothing - only for debugging

		///////////////////////////////
		// create objective function //
		///////////////////////////////

		// minimize total number of MUX inputs
		ScaLP::Term obj;
		for(int l=0; l<minRegs; l++) {
			obj += w_l[l];
		}
		for(auto r : rm->Resources()) {
			int resLimit = r->getLimit();
			auto vertices = rm->getVerticesOfResource(r);
			if(r->isUnlimited()) resLimit = vertices.size();
			std::cout << "Resource '";
			flush(cout);
			std::cout << r->getName();
			flush(cout);
			std::cout << "' - limit ";
			flush(cout);
			std::cout << resLimit;
			flush(cout);
			std::cout << " - number of ports ";
			flush(cout);
			std::cout << numResourcePorts[r];
			flush(cout);
			std::cout << std::endl;
			for(int k=0; k<resLimit; k++) {
				for(int n=0; n<numResourcePorts[r]; n++) {
					obj += m_r_k_n[{r->getName(),{k,n}}];
				}
			}
		}
		std::cout << "Created objective" << std::endl;

		///////////
		// solve //
		///////////
		std::cout << "start solving" << std::endl;
		auto stat = s.solve();
		std::cout << "finished solving" << std::endl;

		/////////////////
		// get results //
		/////////////////
		BindingContainer b;

		if(stat != ScaLP::status::TIMEOUT_FEASIBLE and stat != ScaLP::status::OPTIMAL and stat != ScaLP::status::FEASIBLE) {
			std::cout << "Could not solve optimal binidng ILP formulation for II = " << II << std::endl;
			std::cout << "ScaLP solver status: " << stat << std::endl;
			return b;
		}

		std::cout << "SOLVER RESULTS: " << std::endl;
		std::cout << s.getResult();

		auto results = s.getResult().values;

		for(auto v : g->Vertices()) {
			bool hasBinding = false;
			auto i = v->getId();
			auto r = rm->getResource(v);
			int resLimit = r->getLimit();
			auto vertices = rm->getVerticesOfResource(r);
			if(r->isUnlimited()) resLimit = vertices.size();
			for(int k=0; k<resLimit; ++k) {
				auto var = x_i_k[{i,k}];
				auto bTemp = (bool)((int) round(results[var]));
				if(!bTemp) continue;
				if(hasBinding) {
					// this vertex is bound to multiple FUs
					// that should never happen!
					throw HatScheT::Exception("Vertex '" + v->getName() + "' is bound to multiple FUs - that should never happen");
				}
				hasBinding = true;
				b.resourceBindings[v] = k;
			}
		}

		for(auto v : g->Vertices()) {
			bool hasBinding = false;
			auto i = v->getId();
			for(int l=0; l<minRegs; l++) {
				auto var = y_i_l[{i,l}];
				auto bTemp = (bool)((int) round(results[var]));
				if(!bTemp) continue;
				if(hasBinding) {
					// this variable is bound to multiple Register
					// that should never happen!
					throw HatScheT::Exception("Variable '" + v->getName() + "' is bound to multiple Registers - that should never happen");
				}
				hasBinding = true;
				b.registerBindings[v] = l;
			}
		}

		std::cout << "ILP formulation SOLVED for II = " << II << std::endl;
		std::cout << "ScaLP solver status: " << stat << std::endl;

		// print results


		return b;
	}

	std::map<const Vertex *, int>
	Binding::getILPMinMuxBinding(map<Vertex *, int> sched, Graph *g, ResourceModel *rm, int II,
		std::map<Edge*,int> portAssignments, std::set<const Resource*> commutativeOps, std::list <string> sw, int timeout) {

		//////////////////////
		// check for errors //
		//////////////////////
		// commutative operations are not supported yet
		if(!commutativeOps.empty()) {
			throw HatScheT::Exception(
				"Binding::getILPMinMuxBinding: Commutative operations are not supported yet, leave commutativeOps empty please"
			);
		}

		// edges are missing in port assignment container
		for(auto e : g->Edges()) {
			try {
				portAssignments.at(e);
			}
			catch(...) {
				throw HatScheT::Exception("Binding::getILPMinMuxBinding: Edge missing in port assignment container");
			}
		}

		/////////////////////////
		// container to return //
		/////////////////////////
		map<const Vertex*,int> binding;

		///////////////////
		// create solver //
		///////////////////
		if(sw.empty()) sw = {"Gurobi","CPLEX","SCIP","LPSolve"};
		auto solver = ScaLP::Solver(sw);
		solver.quiet = true;
		solver.threads = 1;
		solver.timeout = timeout;

		/////////////////////////////////////////////////
		// count if less FUs are needed than allocated //
		/////////////////////////////////////////////////
		std::map<const Resource*,int> resourceLimits;
		for(auto &r : rm->Resources()) {
			resourceLimits[r] = 0;
			if(r->isUnlimited()) {
				resourceLimits[r] = rm->getVerticesOfResource(r).size();
				std::cout << "Resource limits for resource '" << r->getName() << "': " << resourceLimits[r] << std::endl;
				continue;
			}
			std::map<int,int> modSlotHeight;
			for(int i=0; i<II; i++) {
				modSlotHeight[i] = 0;
			}
			for(auto &it : sched) {
				auto v = it.first;
				if(rm->getResource(v) != r) continue;
				auto modSlot = it.second % II;
				modSlotHeight[modSlot]++;
				std::cout << "Mod slot for vertex '" << v->getName() << "' = " << modSlot << " = " << it.second << " mod "
									<< II << std::endl;
				std::cout << "  mod slot height = " << modSlotHeight[modSlot] << std::endl;
			}
			for(auto it : modSlotHeight) {
				if(it.second > resourceLimits[r]) resourceLimits[r] = it.second;
			}
			std::cout << "Resource limits for resource '" << r->getName() << "': " << resourceLimits[r] << std::endl;
		}

		/////////////////////////////////////////////
		// count number of ports for each resource //
		/////////////////////////////////////////////
		std::map<const Resource*, int> numResourcePorts;
		for(auto &r : rm->Resources()) {
			auto vertices = rm->getVerticesOfResource(r);
			int numPorts = 0;
			for(auto &v : vertices) {
				int numInputs = 0;
				for(auto &e : g->Edges()) {
					if(e->getDependencyType() == Edge::DependencyType::Precedence) continue; // skip chaining edges
					if(&e->getVertexDst() != v) continue;
					numInputs++;
				}
				if(numInputs > numPorts) numPorts = numInputs;
			}
			numResourcePorts[r] = numPorts;
		}
		std::cout << "counted number of ports for each resource" << std::endl;

		//////////////////////////////////
		// check if ports are left open //
		//////////////////////////////////
		// this might happen if the portAssignment container is not filled properly
		// this is not supported here
		for(auto vj : g->Vertices()) {
			auto rj = rm->getResource(vj);
			std::set<int> foundPorts;
			for(auto it : portAssignments) {
				auto e = it.first;
				auto port = it.second;
				if(&e->getVertexDst() != vj) continue;
				if(port >= numResourcePorts[rj]) {
					std::stringstream exc;
					exc << "Binding::getILPMinMuxBinding: Illegal port number (" << port << ") provided for edge from "
					  << e->getVertexSrc().getName() << " to " << e->getVertexDst().getName() << " - resource " << rj->getName()
					  << " only has " << numResourcePorts[rj] << " ports";
					throw HatScheT::Exception(exc.str());
				}
				foundPorts.insert(port);
			}
			if(foundPorts.size() != numResourcePorts[rj]) {
				std::stringstream exc;
				exc << "Binding::getILPMinMuxBinding: not all ports are occupied for vertex " << vj->getName()
				  << ", found ports: " << foundPorts.size() << ", required ports: " << numResourcePorts[rj]
				  << " - open ports are not supported yet";
				throw HatScheT::Exception(exc.str());
			}
		}
		std::cout << "checked if ports are left open" << std::endl;

		///////////////////////////////////////////////////
		// assign each FU of each resource type a number //
		///////////////////////////////////////////////////
		std::map<std::pair<const Resource*,int>,int> fuIndexMap;
		std::map<int,std::pair<const Resource*,int>> indexFuMap;
		int fuIndexCounter = 0;
		for(auto &r : rm->Resources()) {
			for(auto i=0; i<resourceLimits[r]; i++) {
				auto p = make_pair(r,i);
				fuIndexMap[p] = fuIndexCounter;
				indexFuMap[fuIndexCounter] = p;
				fuIndexCounter++;
			}
		}
		std::cout << "assigned all fus a number" << std::endl;

		///////////////////////////////////////////////////////////
		// check possible lifetimes for each fu -> fu connection //
		///////////////////////////////////////////////////////////
		std::map<std::pair<int,int>,std::set<int>> possibleLifetimes;
		std::map<Edge*,int> lifetimes;
		for(auto &e : g->Edges()) {
			auto &vi = e->getVertexSrc();
			auto &vj = e->getVertexDst();
			auto lifetime = sched[&vj] - sched[&vi] - rm->getResource(&vi)->getLatency() + (II * e->getDistance());
			lifetimes[e] = lifetime;
			auto *ri = rm->getResource(&vi);
			auto *rj= rm->getResource(&vj);
			for(int a=0; a<resourceLimits[ri]; a++) {
				auto m = fuIndexMap[{ri,a}];
				for(int b=0; b<resourceLimits[rj]; b++) {
					auto n = fuIndexMap[{rj,b}];
					possibleLifetimes[{m,n}].insert(lifetime);
				}
			}
		}
		std::cout << "checked possible lifetimes" << std::endl;

		//////////////////////
		// create variables //
		//////////////////////
		// i : vertex index
		// m/n : fu index
		// k : lifetime register stage after fu
		// p ; input port number
		std::map<std::pair<int,int>,ScaLP::Variable> v_i_m; // binding vertex -> fu
		//std::map<std::pair<int,std::pair<int,int>>,ScaLP::Variable> r_m_n_k; // connection from fu m to fu n over k registers
		std::map<std::pair<std::pair<int,int>,std::pair<int,int>>,ScaLP::Variable> r_m_n_k_p; // connection from fu m to fu n port p over k registers

		// create vertex binding variables
		for(auto v : g->Vertices()) {
			auto i = v->getId();
			auto r = rm->getResource(v);
			std::cout << "  resource limit for resource " << r->getName() << ": " << resourceLimits[r] << std::endl;
			for(int a=0; a<resourceLimits[r]; a++) {
				auto m = fuIndexMap[{r,a}];
				std::stringstream name;
				name << "v_" << i << "_" << m;
				v_i_m[{i,m}] = ScaLP::newBinaryVariable(name.str());
				std::cout << "  Successfully created variable " << v_i_m[{i,m}] << std::endl;
			}
		}
		std::cout << "created vertex-binding variables" << std::endl;

		// create connection variables
		for(auto ri : rm->Resources()) {
			for(auto rj : rm->Resources()) {
				for(int a=0; a<resourceLimits[ri]; a++) {
					auto m = fuIndexMap[{ri,a}];
					for(int b=0; b<resourceLimits[rj]; b++) {
						auto n = fuIndexMap[{rj,b}];
						for(auto k : possibleLifetimes[{m,n}]) {
							for(int p=0; p<numResourcePorts[rj]; p++) {
								std::stringstream name;
								name << "r_" << m << "_" << n << "_" << k << "_" << p;
								r_m_n_k_p[{{m,n},{k,p}}] = ScaLP::newBinaryVariable(name.str());
								std::cout << "  Successfully created variable " << r_m_n_k_p[{{m,n},{k,p}}] << std::endl;
							}
						}
					}
				}
			}
		}
		std::cout << "created connection variables" << std::endl;

		////////////////////////
		// create constraints //
		////////////////////////
		// each vertex is bound to exactly one resource
		for(auto v : g->Vertices()) {
			auto i = v->getId();
			auto r = rm->getResource(v);
			std::cout << "Creating constraint for vertex " << v->getName() << " and resource " << r->getName() << std::endl;
			ScaLP::Term t;
			for(int a=0; a<resourceLimits[r]; a++) {
				auto m = fuIndexMap[{r,a}];
				t += v_i_m[{i,m}];
			}
			ScaLP::Constraint constr = t == 1;
			solver.addConstraint(constr);
			std::cout << "  Successfully added constraint " << constr << std::endl;
		}
		std::cout << "created vertex-binding constraints" << std::endl;

		// one connection for each edge
		for(auto e : g->Edges()) {
			auto *vi = &e->getVertexSrc();
			auto *vj = &e->getVertexDst();
			auto ri = rm->getResource(vi);
			auto rj = rm->getResource(vj);
			auto i = vi->getId();
			auto j = vj->getId();
			auto k = lifetimes[e];
			std::cout << "Creating edge constraint for " << vi->getName() << "->" << vj->getName() << " with distance="
				<< e->getDistance() << std::endl;
			for(int a = 0; a < resourceLimits[ri]; a++) {
				auto m = fuIndexMap[{ri, a}];
				for(int b = 0; b < resourceLimits[rj]; b++) {
					auto n = fuIndexMap[{rj, b}];
					ScaLP::Term t;
					for(int p = 0; p < numResourcePorts[rj]; p++) {
						t += r_m_n_k_p[{{m, n},{k, p}}];
					}
					ScaLP::Constraint constr = v_i_m[{i,m}] + v_i_m[{j,n}] - t <= 1;
					solver.addConstraint(constr);
					std::cout << "  Successfully added constraint " << constr << std::endl;
				}
			}
		}
		std::cout << "created edge connection constraints" << std::endl;

		// no resource conflicts
		for(auto r : rm->Resources()) {
			for(int a=0; a<resourceLimits[r]; a++) {
				auto m = fuIndexMap[{r, a}];
				auto vertices = rm->getVerticesOfResource(r);
				for(int t=0; t<II; t++) {
					ScaLP::Term lhs;
					for(auto v : vertices) {
						if(sched[const_cast<Vertex*>(v)] % II != t) continue;
						int i = v->getId();
						lhs += v_i_m[{i,m}];
					}
					ScaLP::Constraint constr = lhs <= 1;
					solver.addConstraint(lhs <= 1);
					std::cout << "  Successfully added constraint " << constr << std::endl;
				}
			}
		}
		std::cout << "created resource conflict constraints" << std::endl;

		// respect port assignments for non-commutative resources
		for(auto it : portAssignments) {
			auto e = it.first;
			auto port = it.second;
			auto vi = &e->getVertexSrc();
			auto vj = &e->getVertexDst();
			auto ri = rm->getResource(vi);
			auto rj = rm->getResource(vj);
			if(commutativeOps.find(rj) != commutativeOps.end()) {
				// skip commutative operations
				continue;
			}
			for(auto &rContainer : r_m_n_k_p) {
				auto m = rContainer.first.first.first;
				auto n = rContainer.first.first.second;
				if(indexFuMap[m].first != ri or indexFuMap[n].first != rj) {
					// variable is not relevant for this edge
					continue;
				}
				auto var = rContainer.second;
				auto p = rContainer.first.second.second;
				if(p == port) {
					// binding is allowed to happen according to portAssignment container
					continue;
				}
				ScaLP::Constraint constr = var == 0;
				solver.addConstraint(constr);
				std::cout << "  Successfully added constraint " << constr << std::endl;
			}
		}
		std::cout << "created port assignment constraints" << std::endl;

		///////////////////
		// set objective //
		///////////////////
		ScaLP::Term obj;
		for(auto &it : r_m_n_k_p) {
			obj += it.second;
		}
		solver.setObjective(ScaLP::minimize(obj));

		//////////////////
		// start solver //
		//////////////////
		std::cout << solver.showLP() << std::endl;
		std::cout << "start solving now" << std::endl;
		auto stat = solver.solve();
		std::cout << "finished solving with status " << stat << std::endl;
		if(stat != ScaLP::status::FEASIBLE and stat != ScaLP::status::TIMEOUT_FEASIBLE and stat != ScaLP::status::OPTIMAL) {
			std::cout << "Binding::getILPMinMuxBinding: could not solve binding problem, ScaLP status " << stat << std::endl;
			return binding; // empty binding
		}
		std::cout << "start filling solution structure" << std::endl;

		/////////////////////////////
		// fill solution structure //
		/////////////////////////////
		auto res = solver.getResult().values;
		// binding variables
		for(auto &it : v_i_m) {
			auto var = it.second;
			auto i = it.first.first;
			auto m = it.first.second;
			auto value = round(res[var]);
			if(value != 1.0) continue;
			auto v = &g->getVertexById(i);
			auto r = indexFuMap[m].first;
			auto fu = indexFuMap[m].second;
			binding[v] = fu;
			std::cout << "Vertex " << v->getName() << " is bound to FU " << fu << " of resource type "
			  << r->getName() << std::endl;
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
			auto ri = indexFuMap[m].first;
			auto fui = indexFuMap[m].second;
			auto rj = indexFuMap[n].first;
			auto fuj = indexFuMap[n].second;
			std::cout << "fu " << fui << " of resource type " << ri->getName() << " is connected to port " << p << " of fu "
			  << fuj << " of resource type " << rj->getName() << " over " << k << " lifetime registers" << std::endl;
		}

		return binding;
	}

#endif
}