//
// Created by nfiege on 03/12/19.
//

#include "Binding.h"
#include <HatScheT/utility/Utility.h>
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

		// create vertex-binding variables
		std::map<Vertex*,std::vector<ScaLP::Variable>> vertexVariables;
		std::map<const Resource*,std::vector<ScaLP::Variable>> registerVariables;
		std::map<const Resource*,std::list<Vertex*>> sameResources;
		std::map<const Resource*,int> resourceLimits;
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
			int limit = res->getLimit();
			if(limit<0) {
				binding[it.first] = unlimitedResourceCounter[res];
				unlimitedResourceCounter[res]++;
				continue;
			}

			vertexVariables[it.first] = std::vector<ScaLP::Variable>();
			for(auto i = 0; i<limit; i++){
				vertexVariables[it.first].emplace_back(ScaLP::newIntegerVariable(it.first->getName()+"_"+to_string(i),0,1));
			}
		}

		// create register variables
		for(auto &it : registerVariables){
			auto res = it.first;
			auto limit = res->getLimit();
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
				for(int resourceCounter = 0; resourceCounter<res->getLimit(); resourceCounter++){
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
#endif
}