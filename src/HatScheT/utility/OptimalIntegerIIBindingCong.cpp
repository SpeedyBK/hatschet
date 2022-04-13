//
// Created by nfiege on 3/28/22.
//

#include "OptimalIntegerIIBindingCong.h"
#include <ScaLP/Solver.h>
#include <cmath>

namespace HatScheT {

	OptimalIntegerIIBindingCong::OptimalIntegerIIBindingCong(Graph *g, ResourceModel *rm, std::map<Vertex *, int> sched,
																													 int II, std::map<Edge *, int> portAssignments,
																													 std::list<std::string> sw) :
  BindingBase(g, rm, std::move(sched), II, {}), portAssignments(std::move(portAssignments)), sw(std::move(sw)) {

	}

	void OptimalIntegerIIBindingCong::bind() {
		if (!this->quiet) {
			std::cout << "Binding::getILPBasedIntIIBindingCong: start for the following graph with associated resource model and schedule:" << std::endl;
			std::cout << *this->g << std::endl;
			std::cout << *this->rm << std::endl;
			std::cout << "schedule with II=" << this->II << std::endl;
			for (auto &it : this->sched) {
				std::cout << "  " << it.first->getName() << " (" << it.first->getId() << ") - " << it.second << std::endl;
			}
		}

		///////////////////////////////////////////////
		// find conflicting operations and variables //
		///////////////////////////////////////////////

		// resource compatibility graph
		Graph resourceCompatibilityGraph;
		// create vertices
		for(auto &v : this->g->Vertices()) {
			resourceCompatibilityGraph.createVertex(v->getId());
		}
		// check compatibility with other vertices
		for(auto &v : this->g->Vertices()) {
			for (auto &it : this->sched) {
				// no edge with itself
				if(v == it.first) continue;
				// source vertex: the one who comes first in the schedule
				if (this->sched[v] > it.second) continue;
				// incompatible if resource types are different
				// ACTUALLY I think they should always be compatible in that case...
				//if (rm->getResource(v) != rm->getResource(it.first)) continue;
				// incompatible if modulo slots are equal and resource types are also equal
				if ((this->sched[v] % this->II == it.second % this->II) and (this->rm->getResource(v) == this->rm->getResource(it.first))) {
					if (!this->quiet) {
						std::cout << "Vertices '" << v->getName() << "' and '" << it.first->getName() << "' are INCOMPATIBLE because modulo slots overlap" << std::endl;
						std::cout << "  " << v->getName() << " t = " << this->sched[v] << " mod " << this->II << " = " << this->sched[v] % this->II << std::endl;
						std::cout << "  " << it.first->getName() << " t = " << it.second << " mod " << this->II << " = " << it.second % this->II << std::endl;
					}
					continue;
				}
				// create edge
				if (!this->quiet) {
					std::cout << "Vertices '" << v->getName() << "' and '" << it.first->getName()
										<< "' are COMPATIBLE because modulo slots DO NOT overlap" << std::endl;
					std::cout << "  " << v->getName() << " t = " << this->sched[v] << " mod " << this->II << " = " << this->sched[v] % this->II
										<< std::endl;
					std::cout << "  " << it.first->getName() << " t = " << it.second << " mod " << this->II << " = " << it.second % this->II
										<< std::endl;
				}
				resourceCompatibilityGraph.createEdge(resourceCompatibilityGraph.getVertexById(v->getId()),resourceCompatibilityGraph.getVertexById(it.first->getId()));
			}
		}

		// keep track of vertices that produce registered variables
		std::map<Vertex*, bool> producesRegisteredVariable;
		for (auto &v : this->g->Vertices()) {
			producesRegisteredVariable[v] = false;
			for (auto &e : this->g->Edges()) {
				if (!e->isDataEdge()) continue;
				auto *vSrc = &e->getVertexSrc();
				auto *vDst = &e->getVertexDst();
				if (vSrc != v) continue;
				auto tSrc = this->sched.at(vSrc);
				auto tDst = this->sched.at(vDst);
				auto lSrc = this->rm->getVertexLatency(v);
				auto lifetime = tDst - tSrc - lSrc + (e->getDistance() * this->II);
				if (lifetime > 0) {
					producesRegisteredVariable[v] = true;
					break;
				}
			}
		}

		// count minimum number of needed registers to store info
		// (NOT IN PAPER - IN PAPER IT WAS AN INPUT TO THE ILP FORMULATION)
		int minRegs = 0;
		// variable compatibility graph
		Graph variableCompatibilityGraph;
		// create vertices
		for(auto &v : this->g->Vertices()) {
			variableCompatibilityGraph.createVertex(v->getId());
		}
		// check compatibility with other vertices and determine min number of registers
		std::map<int, int> numAliveVariables;
		std::map<Vertex*, std::set<int>> registeredTimeSteps;

		for(auto &v : this->g->Vertices()) {
			// calc lifetime of v
			int v1Lat = this->rm->getResource(v)->getLatency();
			int v1LifeStart = this->sched[v] + v1Lat;
			int v1LifeEnd = v1LifeStart; // default value if vertex has no outgoing edges
			int numIncompatibilities = 0;
			for (auto &e : this->g->Edges()) {
				if (e->getVertexSrc().getId() != v->getId()) continue;
				auto dstStart = this->sched[&e->getVertexDst()] + (e->getDistance() * this->II);
				if (dstStart > v1LifeEnd) v1LifeEnd = dstStart;
			}
			if (v1LifeEnd - v1LifeStart > this->II) {
				// variable overlaps with itself from future or past iterations
				throw HatScheT::Exception(
					"Variable '" + v->getName() + "' has lifetime > II (" + std::to_string(v1LifeEnd - v1LifeStart) + " > " +
					std::to_string(this->II) +
					"); this is not supported, yet; please choose another binding algorithm or unroll the graph to get a longer cycle length");
			}
			if (v1LifeStart == v1LifeEnd) {
				// variable does not have to be registered
				continue;
			}
			for (int t=v1LifeStart+1; t<=v1LifeEnd; t++) {
				auto cc = t % this->II;
				registeredTimeSteps[v].insert(cc);
				numAliveVariables[cc]++;
			}
		}
		for(auto &v1 : this->g->Vertices()) {
			for(auto &v2 : this->g->Vertices()) {
				if (v1 == v2) {
					// no self loops in variable compatibility graph
					continue;
				}
				// source vertex: the one who comes first in the schedule
				// tiebreaker: vertex IDs
				if(this->sched[v1] > this->sched[v2] or (this->sched[v1] == this->sched[v2] and v1->getId() > v2->getId())) {
					continue;
				}
				bool compatible = true;
				if (registeredTimeSteps.find(v1) != registeredTimeSteps.end() and registeredTimeSteps.find(v2) != registeredTimeSteps.end()) {
					// both variables must be stored in registers
					// check if they are compatible
					for (auto &t1 : registeredTimeSteps.at(v1)) {
						for (auto &t2 : registeredTimeSteps.at(v2)) {
							if (t1 == t2) {
								compatible = false;
								break;
							}
						}
						if (not compatible) break;
					}
				}
				if (compatible) {
					// create edge
					if (!this->quiet) {
						std::cout << "Vertices '" << v1->getName() << "' and '" << v2->getName()
											<< "' are COMPATIBLE because lifetimes DO NOT overlap" << std::endl;
					}
					variableCompatibilityGraph.createEdge(variableCompatibilityGraph.getVertexById(v1->getId()),
																								variableCompatibilityGraph.getVertexById(v2->getId()));
				}
				else {
					if (!this->quiet) {
						std::cout << "Vertices '" << v1->getName() << "' and '" << v2->getName()
											<< "' are INCOMPATIBLE because lifetimes overlap" << std::endl;
					}
				}
			}
		}

		for (auto &it : numAliveVariables) {
			if (minRegs < it.second) minRegs = it.second;
		}
		if (!this->quiet) {
			std::cout << "Minimum number of needed registers = " << minRegs << std::endl;
		}

		///////////////////
		// set up solver //
		///////////////////
		ScaLP::Solver s(this->sw);
		if(this->timeBudget >= 0) {
			s.timeout = (long)this->timeBudget;
		}
		s.quiet = true;

		//////////////////////
		// create variables //
		//////////////////////

		// boolean variables whether vertex v_i is bound to functional unit k of resource type r (x_i_k)
		std::map<std::pair<int,int>,ScaLP::Variable> x_i_k;
		for(auto &v : this->g->Vertices()) {
			auto r = this->rm->getResource(v);
			auto limit = r->getLimit();
			if(r->isUnlimited()) limit = this->rm->getVerticesOfResource(r).size();
			for(int k=0; k<limit; ++k) {
				auto i = v->getId();
				x_i_k[{i,k}] = ScaLP::newBinaryVariable("x_" + to_string(i) + "_" + to_string(k));
				if (!this->quiet) {
					std::cout << "  Created variable '" << x_i_k[{i, k}] << "'" << std::endl;
				}
			}
		}
		if (!this->quiet) {
			std::cout << "Created x_i_k variables" << std::endl;
		}
		s.showLP(); // this line does nothing - only for debugging

		// boolean variables whether variable of v_i is bound to register l (y_i_l)
		std::map<std::pair<int,int>,ScaLP::Variable> y_i_l;
		for(auto &v : this->g->Vertices()) {
			if (!producesRegisteredVariable.at(v)) {
				// skip vertices that do not produce registered variables
				continue;
			}
			auto i = v->getId();
			for(int l=0; l<minRegs; ++l) {
				y_i_l[{i,l}] = ScaLP::newBinaryVariable("y_" + to_string(i) + "_" + to_string(l));
				if (!this->quiet) {
					std::cout << "  Created variable '" << y_i_l[{i, l}] << "'" << std::endl;
				}
			}
		}
		if (!this->quiet) {
			std::cout << "Created y_i_l variables" << std::endl;
		}
		s.showLP(); // this line does nothing - only for debugging

		// boolean variables for each FU -> register connection (c_r_k_l) for each resource type r
		std::map<std::pair<std::string,std::pair<int,int>>,ScaLP::Variable> c_r_k_l;
		for(auto &r : rm->Resources()) {
			int resLimit = r->getLimit();
			if(r->isUnlimited()) resLimit = rm->getVerticesOfResource(r).size();
			for(int k=0; k<resLimit; ++k) {
				for(int l=0; l<minRegs; ++l) {
					c_r_k_l[{r->getName(),{k,l}}] = ScaLP::newBinaryVariable("c_" + r->getName() + "_" + to_string(k) + "_" + to_string(l));
					if (!this->quiet) {
						std::cout << "  Created variable '" << c_r_k_l[{r->getName(), {k, l}}] << "'" << std::endl;
					}
				}
			}
		}
		if (!this->quiet) {
			std::cout << "Created c_r_k_l variables" << std::endl;
			s.showLP(); // this line does nothing - only for debugging
		}

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
						if (!this->quiet) {
							std::cout << "  Created variable '" << a_r_n_k_l[{{r->getName(), n},
																																{k,            l}}] << "'" << std::endl;
						}
					}
				}
			}
		}
		if (!this->quiet) {
			std::cout << "Created a_r_n_k_l variables" << std::endl;
			s.showLP(); // this line does nothing - only for debugging
		}

		// boolean variables for each FU -> FU connection for each port of that FU (needed for edges with lifetime=0) (b_r1_r2_k1_k2_n)
		std::map<std::pair<std::pair<std::pair<std::string,std::string>,std::pair<int,int>>,int>,ScaLP::Variable> b_r1_r2_k1_k2_n;
		for(auto &r1 : this->rm->Resources()) {
			int resLimit = r1->getLimit();
			auto vertices = this->rm->getVerticesOfResource(r1);
			if(r1->isUnlimited()) resLimit = vertices.size();
			// compute number of inputs for that resource
			int numPorts = 0;
			for(auto &v : vertices) {
				int numInputs = 0;
				for(auto &e : this->g->Edges()) {
					if(e->getDependencyType() == Edge::DependencyType::Precedence) continue; // skip chaining edges
					if(&e->getVertexDst() != v) continue;
					numInputs++;
				}
				if(numInputs > numPorts) numPorts = numInputs;
			}
			// actually create variables
			for(int k1=0; k1<resLimit; ++k1) {
				for(int n=0; n<numPorts; ++n) {
					for(auto &r2 : this->rm->Resources()) {
						int res2Limit = r2->getLimit();
						if(r2->isUnlimited()) res2Limit = this->rm->getVerticesOfResource(r2).size();
						for(int k2=0; k2<res2Limit; ++k2) {
							b_r1_r2_k1_k2_n[{{{r1->getName(),r2->getName()},{k1,k2}},n}] = ScaLP::newBinaryVariable("b_" + r1->getName() + "_" + r2->getName() + "_" + to_string(k1) + "_" + to_string(k2) + "_" + to_string(n));
							if (!this->quiet) {
								std::cout << "  Created variable '" << b_r1_r2_k1_k2_n[{{{r1->getName(), r2->getName()}, {k1, k2}}, n}]
													<< "'" << std::endl;
							}
						}
					}
				}
			}
		}
		if (!this->quiet) {
			std::cout << "Created b_r1_r2_k1_k2_n variables" << std::endl;
		}
		s.showLP(); // this line does nothing - only for debugging

		// number of MUX inputs for each FU and each port (m_r_k_n)
		std::map<std::pair<std::string,std::pair<int,int>>,ScaLP::Variable> m_r_k_n;
		for(auto &r : this->rm->Resources()) {
			int resLimit = r->getLimit();
			auto vertices = this->rm->getVerticesOfResource(r);
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
					if (!this->quiet) {
						std::cout << "  Created variable '" << m_r_k_n[{r->getName(), {k, n}}] << "'" << std::endl;
					}
				}
			}
		}
		if (!this->quiet) {
			std::cout << "Created m_r_k_n variables" << std::endl;
			s.showLP(); // this line does nothing - only for debugging
		}

		// number of MUX inputs for each register (w_l)
		std::map<int,ScaLP::Variable> w_l;
		for(int l=0; l<minRegs; l++) {
			w_l[l] = ScaLP::newIntegerVariable("w_" + to_string(l));
			if (!this->quiet) {
				std::cout << "  Created variable '" << w_l[l] << "'" << std::endl;
			}
		}
		if (!this->quiet) {
			std::cout << "Created w_l variables" << std::endl;
			s.showLP(); // this line does nothing - only for debugging
		}

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
		if (!this->quiet) {
			std::cout << "Created x_i_k=1 constraints" << std::endl;
			s.showLP(); // this line does nothing - only for debugging
		}

		// each variable is bound to one resource (if registers are needed)
		if(minRegs >= 1) {
			for(auto &v : this->g->Vertices()) {
				if (!producesRegisteredVariable.at(v)) {
					// skip vertices that do not produce registered variables
					continue;
				}
				ScaLP::Term t;
				auto i = v->getId();
				for(int l=0; l<minRegs; ++l) {
					t += y_i_l[{i,l}];
				}
				s.addConstraint(t == 1);
			}
		}

		if (!this->quiet) {
			std::cout << "Created y_i_l=1 constraints" << std::endl;
			s.showLP(); // this line does nothing - only for debugging
		}

		// no overlaps in operations (resource conflict graph)
		for(auto &v : this->g->Vertices()) {
			if (!this->quiet) {
				std::cout << "vertex: " << v->getName() << std::endl;
			}
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
				if (!this->quiet) {
					std::cout << "  conflict vertex: " << v2->getName() << std::endl;
				}
			}
			// create constraints for all vertices with conflict
			auto i = v->getId();
			auto r = this->rm->getResource(v);
			int resLimit = r->getLimit();
			auto vertices = this->rm->getVerticesOfResource(r);
			if(r->isUnlimited()) resLimit = vertices.size();
			for(int k = 0; k<resLimit; k++) {
				for(auto j : conflictVertexIDs) {
					auto constraint = (x_i_k[{i,k}] + x_i_k[{j,k}]) <= 1;
					s.addConstraint(constraint);
					if (!this->quiet) {
						std::cout << "  Added constraint '" << constraint << "'" << std::endl;
					}
				}
			}
		}
		if (!this->quiet) {
			std::cout << "Created resource overlap constraints" << std::endl;
			s.showLP(); // this line does nothing - only for debugging
		}

		// no overlaps in registers (variable conflict graph)
		for(auto &v : this->g->Vertices()) {
			if (!producesRegisteredVariable.at(v)) {
				// skip vertices that do not produce registered variables
				continue;
			}
			// find conflicting operations
			auto predecessors = variableCompatibilityGraph.getPredecessors(&variableCompatibilityGraph.getVertexById(v->getId()));
			auto successors = variableCompatibilityGraph.getSuccessors(&variableCompatibilityGraph.getVertexById(v->getId()));
			std::set<int> conflictVertexIDs;
			for(auto &v2 : variableCompatibilityGraph.Vertices()) {
				// skip vertices that do not produce registered variables
				//if (!producesRegisteredVariable.at(v2)) continue;
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
					std::cout << "creating constraint y_" << i << "_" << l << " + y_" << j << "_" << l << std::endl;
					auto constr = (y_i_l[{i,l}] + y_i_l[{j,l}]) <= 1;
					s.addConstraint(constr);
					std::cout << "created constraint " << constr << std::endl;
				}
			}
		}
		if (!this->quiet) {
			std::cout << "Created register overlap constraints" << std::endl;
			s.showLP(); // this line does nothing - only for debugging
			std::cout << "#q# 0" << std::endl;
		}

		// A) for edges with lifetime > 0
		for(auto &e : this->g->Edges()) {
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			auto lifetime = this->sched[vDst] - this->sched[vSrc] - this->rm->getResource(vSrc)->getLatency() + (this->II * e->getDistance());
			if(lifetime < 0) {
				throw HatScheT::Exception("Detected lifetime < 0 from '" + vSrc->getName() + "' (t='" + to_string(this->sched[vSrc])
																	+ "', lat='" + to_string(this->rm->getResource(vSrc)->getLatency()) + "', distance='" + to_string(e->getDistance())
																	+ "', II='" + to_string(this->II) + "') to '" + vDst->getName() + "' (t='" + to_string(this->sched[vDst]) + "')");
			}
			// edges with lifetime = 0 are handled in B)
			if(lifetime == 0) continue;

			// A) check if there is a connection from resource instance k of resource r (source) to register l
			auto rSrc = this->rm->getResource(vSrc);
			auto i = vSrc->getId();
			int resLimitSrc = rSrc->getLimit();
			auto verticesSrc = this->rm->getVerticesOfResource(rSrc);
			if(rSrc->isUnlimited()) resLimitSrc = verticesSrc.size();
			for(int k=0; k<resLimitSrc; k++) {
				for(int l=0; l<minRegs; ++l) {
					if (!this->quiet) {
						std::cout << "adding constraint x_" << i << "_" << k << " + y_" << i << "_" << l << " - c_"
											<< rSrc->getName() << "_" << k << "_" << l << " <= 1" << std::endl;
					}
					s.addConstraint(x_i_k[{i,k}] + y_i_l[{i,l}] - c_r_k_l[{rSrc->getName(),{k,l}}] <= 1);
				}
			}

			// A) check if there is a connection from register l to port n of resource instance k of resource r (destination)
			auto rDst = this->rm->getResource(vDst);
			int resLimitDst = rDst->getLimit();
			auto verticesDst = this->rm->getVerticesOfResource(rDst);
			if(rDst->isUnlimited()) resLimitDst = verticesDst.size();
			auto j = vDst->getId();
			for(int k=0; k<resLimitDst; k++) {
				try {
					auto n = this->portAssignments.at(e);
					for(int l=0; l<minRegs; l++) {
						if (!this->quiet) {
							std::cout << "adding constraint y_" << i << "_" << l << " + x_" << i << "_" << k << " - a_"
												<< rSrc->getName() << "_" << n << "_" << k << "_" << l << " <= 1" << std::endl;
						}
						s.addConstraint(y_i_l[{i,l}] + x_i_k[{j,k}] - a_r_n_k_l[{{rDst->getName(),n},{k,l}}] <= 1);
					}
				}
				catch (std::out_of_range&) {
					throw HatScheT::Exception("Could not find port assignment for edge '"+vSrc->getName()+"' -> '"+vDst->getName()+"'");
				}
			}
		}
		if (!this->quiet) {
			std::cout << "Created edge constraints for lifetime > 0" << std::endl;
			s.showLP(); // this line does nothing - only for debugging
		}

		// B) for edges with lifetime = 0
		for(auto &e : this->g->Edges()) {
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			auto lifetime = this->sched[vDst] - this->sched[vSrc] - this->rm->getResource(vSrc)->getLatency() + (this->II * e->getDistance());
			if (lifetime < 0) {
				throw HatScheT::Exception("Detected lifetime < 0 from '" + vSrc->getName() + "' (t='" + to_string(this->sched[vSrc])
																	+ "', lat='" + to_string(this->rm->getResource(vSrc)->getLatency()) + "', distance='" +
																	to_string(e->getDistance())
																	+ "', II='" + to_string(this->II) + "') to '" + vDst->getName() + "' (t='" +
																	to_string(this->sched[vDst]) + "')");
			}
			// edges with lifetime > 0 are handled in A)
			if (lifetime > 0) continue;

			// B) check if there is a connection from resource instance k2 of resource r2 to port n of resource instance k1 of resource r1
			//b_r1_r2_k1_k2_n
			auto *r2 = this->rm->getResource(vSrc);
			auto *r1 = this->rm->getResource(vDst);
			auto i = vSrc->getId();
			auto j = vDst->getId();

			int resLimitSrc = r2->getLimit();
			auto verticesSrc = this->rm->getVerticesOfResource(r2);
			if(r2->isUnlimited()) resLimitSrc = verticesSrc.size();
			int resLimitDst = r1->getLimit();
			auto verticesDst = this->rm->getVerticesOfResource(r1);
			if(r1->isUnlimited()) resLimitDst = verticesDst.size();

			for(int k1=0; k1<resLimitDst; k1++) {
				for(int k2=0; k2<resLimitSrc; k2++) {
					try {
						auto n = this->portAssignments.at(e);
						s.addConstraint(x_i_k[{i,k2}] + x_i_k[{j,k1}] - b_r1_r2_k1_k2_n[{{{r1->getName(),r2->getName()},{k1,k2}},n}] <= 1);
					}
					catch (std::out_of_range&) {
						throw HatScheT::Exception("Could not find port assignment for edge '"+vSrc->getName()+"' -> '"+vDst->getName()+"'");
					}
				}
			}
		}
		if (!this->quiet) {
			std::cout << "Created edge constraints for lifetime = 0" << std::endl;
			s.showLP(); // this line does nothing - only for debugging
		}

		// calculate number of MUX inputs per port for each FU (m_r_k_n)
		for(auto r : this->rm->Resources()) {
			int resLimit = r->getLimit();
			auto vertices = this->rm->getVerticesOfResource(r);
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
					for(auto r2 : this->rm->Resources()) {
						int resLimit2 = r2->getLimit();
						auto vertices2 = this->rm->getVerticesOfResource(r2);
						if(r2->isUnlimited()) resLimit2 = vertices2.size();
						for(int k2=0; k2<resLimit2; k2++) {
							portSum += b_r1_r2_k1_k2_n[{{{r1->getName(),r2->getName()},{k1,k2}},n}];
						}
					}
					s.addConstraint(portSum - m_r_k_n[{r->getName(),{k,n}}] == 0);
				}
			}
		}
		if (!this->quiet) {
			std::cout << "Created number of resource mux input constraints" << std::endl;
			s.showLP(); // this line does nothing - only for debugging
		}

		// calculate number of MUX inputs per port for each register
		for(int l=0; l<minRegs; l++) {
			ScaLP::Term portSum;
			// c_r_k_l
			for(auto r : this->rm->Resources()) {
				int resLimit = r->getLimit();
				auto vertices = this->rm->getVerticesOfResource(r);
				if(r->isUnlimited()) resLimit = vertices.size();
				for(int k=0; k<resLimit; k++) {
					portSum += c_r_k_l[{r->getName(),{k,l}}];
				}
			}
			s.addConstraint(portSum - w_l[l] == 0);
		}
		if (!this->quiet) {
			std::cout << "Created number of register mux input constraints" << std::endl;
			s.showLP(); // this line does nothing - only for debugging
		}

		///////////////////////////////
		// create objective function //
		///////////////////////////////

		// minimize total number of MUX inputs
		ScaLP::Term obj;
		for(int l=0; l<minRegs; l++) {
			obj += w_l[l];
		}
		for(auto r : this->rm->Resources()) {
			int resLimit = r->getLimit();
			auto vertices = this->rm->getVerticesOfResource(r);
			if(r->isUnlimited()) resLimit = vertices.size();
			if (!this->quiet) {
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
			}
			for(int k=0; k<resLimit; k++) {
				for(int n=0; n<numResourcePorts[r]; n++) {
					obj += m_r_k_n[{r->getName(),{k,n}}];
				}
			}
		}
		if (!this->quiet) {
			std::cout << "Created objective" << std::endl;
		}
		s.setObjective(ScaLP::minimize(obj));

		///////////
		// solve //
		///////////
		if (!this->quiet) {
			std::cout << "start solving" << std::endl;
		}
		auto stat = s.solve();
		if (!this->quiet) {
			std::cout << "finished solving with status " << ScaLP::showStatus(stat) << std::endl;
		}

		/////////////////
		// get results //
		/////////////////
		this->bin = Binding::BindingContainer();
		this->bin.solutionStatus = ScaLP::showStatus(stat);

		///////////////////////////
		// copy port assignments //
		///////////////////////////
		// assume that all operators have exactly 1 output port
		for (auto &it : this->portAssignments) {
			this->bin.portAssignments[it.first] = {0, it.second};
		}

		if(stat != ScaLP::status::TIMEOUT_FEASIBLE and stat != ScaLP::status::OPTIMAL and stat != ScaLP::status::FEASIBLE) {
			if (!this->quiet) {
				std::cout << "Could not solve optimal binding ILP formulation for II = " << this->II << std::endl;
				std::cout << "ScaLP solver status: " << stat << std::endl;
			}
			return;
		}

		/////////////////
		// get results //
		/////////////////
		this->bin.registerCosts = minRegs;

		if (!this->quiet) {
			std::cout << "SOLVER RESULTS: " << std::endl;
			std::cout << s.getResult();
		}

		auto results = s.getResult().values;

		for(auto v : this->g->Vertices()) {
			bool hasBinding = false;
			auto i = v->getId();
			auto r = this->rm->getResource(v);
			int resLimit = r->getLimit();
			auto vertices = this->rm->getVerticesOfResource(r);
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
				this->bin.resourceBindings[v->getName()] = {k};
				if (!this->quiet) {
					std::cout << "Vertex '" << v->getName() << "' is bound to FU number '" << k << "' of resource type '"
										<< this->rm->getResource(v)->getName() << "'" << std::endl;
				}
			}
		}

		std::map<Vertex*, int> registerBindings;
		for(auto v : this->g->Vertices()) {
			if (!producesRegisteredVariable.at(v)) {
				// skip vertices that do not produce registered variables
				continue;
			}
			bool hasBinding = false;
			auto i = v->getId();
			for(int l=0; l<minRegs; l++) {
				auto var = y_i_l[{i,l}];
				auto bTemp = (bool)((int) round(results[var]));
				if(!bTemp) continue;
				if(hasBinding) {
					// this variable is bound to multiple Registers
					// that should never happen!
					throw HatScheT::Exception("Variable '" + v->getName() + "' is bound to multiple Registers - that should never happen");
				}
				hasBinding = true;
				auto inserted = this->bin.registerEnableTimes[l].insert((this->sched[v] + this->rm->getResource(v)->getLatency()) % this->II);
				if (!inserted.second) {
					// register conflict detected
					throw HatScheT::Exception("Binding::getILPBasedIntIIBindingCong: register conflict detected for register "+
																		std::to_string(l)+" in congruence class "+std::to_string(*inserted.first));
				}
				std::cout << "Variable '" << v->getName() << "' is bound to register number '" << l << "'" << std::endl;
				registerBindings[v] = l;
			}
		}

		// FU -> register connections
		for(auto &r : this->rm->Resources()) {
			int resLimit = r->getLimit();
			if (r->isUnlimited()) resLimit = this->rm->getVerticesOfResource(r).size();
			for (int k = 0; k < resLimit; ++k) {
				for (int l = 0; l < minRegs; ++l) {
					auto var = c_r_k_l[{r->getName(),{k,l}}];
					auto bTemp = (bool)((int) round(results[var]));
					if(!bTemp) continue;
					//b.fuRegConnections.push_back({{r->getName(),k},l});
					// assume that FUs only have 1 output port
					this->bin.connections.push_back({r->getName(), k, 0, "register", l, 0, {}});
					if (!quiet)
						std::cout << "FU '" << k << "' of type '" << r->getName() << "' is connected to register '" << l << "'" << std::endl;
				}
			}
		}

		// register -> FU connections
		for (auto &it : a_r_n_k_l) {
			auto var = it.second;
			auto bTemp = (bool)((int) round(results[var]));
			if(!bTemp) continue;
			auto r = it.first.first.first;
			auto n = it.first.first.second;
			auto k = it.first.second.first;
			auto l = it.first.second.second;
			this->bin.connections.push_back({"register", l, 0, r, k, n, {}}); // registers only have 1 output port
			if (!quiet)
				std::cout << "Register '" << l << "' is connected to port '" << n << "' of FU '" << k << "' of type '" << r << "'" << std::endl;
		}

		// FU -> FU connections
		for (auto &it : b_r1_r2_k1_k2_n) {
			auto var = it.second;
			auto bTemp = (bool)((int) round(results[var]));
			if(!bTemp) continue;
			auto r1 = it.first.first.first.first;
			auto r2 = it.first.first.first.second;
			auto k1 = it.first.first.second.first;
			auto k2 = it.first.first.second.second;
			auto n = it.first.second;
			this->bin.connections.push_back({r2, k2, 0, r1, k1, n, {}}); // assume that FUs only have 1 output port
			if (!this->quiet)
				std::cout << "FU '" << k2 << "' of type '" << r2 << "' is connected to port '" << n << "' of FU '" << k1 << "' of type '" << r1 << "'" << std::endl;
		}

		// fill sets in which the connections are active
		for (auto &e : this->g->Edges()) {
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			auto lifetime = this->sched[vDst] - this->sched[vSrc] - this->rm->getResource(vSrc)->getLatency() + (this->II * e->getDistance());
			auto srcFU = *this->bin.resourceBindings.at(vSrc->getName()).begin();
			auto dstFU = *this->bin.resourceBindings.at(vDst->getName()).begin();
			auto srcResName = this->rm->getResource(vSrc)->getName();
			auto dstResName = this->rm->getResource(vDst)->getName();
			if (lifetime > 0) {
				// FU -> reg -> FU
				int regIndex = registerBindings[vSrc];
				for (auto &it : this->bin.connections) {
					if (
						std::get<0>(it) == srcResName and
						std::get<1>(it) == srcFU and
						std::get<2>(it) == 0 and
						std::get<3>(it) == "register" and
						std::get<4>(it) == regIndex and
						std::get<5>(it) == 0
						) {
						// insert time step for register
						std::get<6>(it).insert((this->sched[vSrc] + this->rm->getResource(vSrc)->getLatency()) % this->II);
					}
					else if (
						std::get<0>(it) == "register" and
						std::get<1>(it) == regIndex and
						std::get<2>(it) == 0 and
						std::get<3>(it) == dstResName and
						std::get<4>(it) == dstFU and
						std::get<5>(it) == this->portAssignments[e]
						) {
						// insert time step for dst FU
						std::get<6>(it).insert(this->sched[vDst] % this->II);
					}
				}
			}
			else {
				// FU -> FU
				for (auto &it : this->bin.connections) {
					if (
						std::get<0>(it) == srcResName and
						std::get<1>(it) == srcFU and
						std::get<2>(it) == 0 and
						std::get<3>(it) == dstResName and
						std::get<4>(it) == dstFU and
						std::get<5>(it) == this->portAssignments[e]
						) {
						std::get<6>(it).insert(this->sched[vDst] % this->II);
					}
				}
			}
		}

		if (!this->quiet) {
			// info about register enable times
			for (auto &it : this->bin.registerEnableTimes) {
				std::cout << "register '" << it.first << "' is enabled in time steps ";
				for (auto &t : it.second) {
					std::cout << t << " ";
				}
				std::cout << std::endl;
			}

			// info about solving status
			std::cout << "ILP formulation SOLVED for II = " << II << std::endl;
			std::cout << "ScaLP solver status: " << stat << std::endl;
		}

		this->bin.multiplexerCosts = (int)this->bin.connections.size();
	}

	void OptimalIntegerIIBindingCong::getBinding(Binding::BindingContainer *b) {
		*b = this->bin;
	}
}