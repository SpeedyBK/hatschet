//
// Created by nfiege on 03/12/19.
//

#include "Binding.h"
#include <cmath>
#include <algorithm>
#include <HatScheT/utility/Utility.h>
#include <sstream>
#ifdef USE_SCALP
#include <ScaLP/Solver.h>
#include <HatScheT/utility/OptimalIntegerIIBinding.h>
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

#ifdef USE_SCALP

	Binding::BindingContainer
	Binding::getILPBasedIntIIBindingCong(map<Vertex *, int> sched, Graph *g, ResourceModel *rm, int II,
																			 std::map<Edge*,int> portAssignments, list<string> sw, int timeout, bool quiet) {
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
					if (!quiet) {
						std::cout << "Vertices '" << v->getName() << "' and '" << it.first->getName() << "' are INCOMPATIBLE because modulo slots overlap" << std::endl;
						std::cout << "  " << v->getName() << " t = " << sched[v] << " mod " << II << " = " << sched[v] % II << std::endl;
						std::cout << "  " << it.first->getName() << " t = " << it.second << " mod " << II << " = " << it.second % II << std::endl;
					}
					continue;
				}
				// create edge
				if (!quiet) {
					std::cout << "Vertices '" << v->getName() << "' and '" << it.first->getName()
										<< "' are COMPATIBLE because modulo slots DO NOT overlap" << std::endl;
					std::cout << "  " << v->getName() << " t = " << sched[v] << " mod " << II << " = " << sched[v] % II
										<< std::endl;
					std::cout << "  " << it.first->getName() << " t = " << it.second << " mod " << II << " = " << it.second % II
										<< std::endl;
				}
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
			int v1LifeEnd = -1; // default value if vertex has no outgoing edges
			int numIncompatibilities = 0;
			for(auto &e : g->Edges()) {
				if(e->getVertexSrc().getId() != v->getId()) continue;
				auto dstStart = sched[&e->getVertexDst()] + e->getDistance() * II;
				if(dstStart > v1LifeEnd) v1LifeEnd = dstStart;
			}
			if (v1LifeEnd - v1LifeStart > II) {
				// variable overlaps with itself from future or past iterations
				throw HatScheT::Exception("Variable '"+v->getName()+"' has lifetime > II ("+std::to_string(v1LifeEnd - v1LifeStart)+" > "+std::to_string(II)+"); this is not supported, yet; please choose another binding algorithm");
			}
			bool v1Omnicompatible = (v1LifeStart == v1LifeEnd) or (v1LifeEnd == -1);
			for(auto &it : sched) {
				// calc lifetime of v2
				int v2Lat = rm->getResource(it.first)->getLatency();
				int v2LifeStart = it.second + v2Lat;
				int v2LifeEnd = -1; // default value if vertex has no outgoing edges
				for(auto &e : g->Edges()) {
					if(e->getVertexSrc().getId() != it.first->getId()) continue;
					auto dstStart = sched[&e->getVertexDst()] + e->getDistance() * II;
					if(dstStart > v2LifeEnd) v2LifeEnd = dstStart;
				}
				bool v2Omnicompatible = (v2LifeStart == v2LifeEnd) or (v2LifeEnd == -1);
				// incompatible if lifetimes overlap
				if(!v1Omnicompatible and !v2Omnicompatible and ((v2LifeStart >= v1LifeStart and v2LifeStart <= v1LifeEnd) or (v2LifeEnd >= v1LifeStart and v2LifeEnd <= v1LifeEnd))) {
					if (!quiet) {
						std::cout << "Vertices '" << v->getName() << "' and '" << it.first->getName()
											<< "' are INCOMPATIBLE because lifetimes overlap" << std::endl;
						std::cout << "  " << v->getName() << ": " << v1LifeStart << " -> " << v1LifeEnd << " with latency " << v1Lat
											<< std::endl;
						std::cout << "  " << it.first->getName() << ": " << v2LifeStart << " -> " << v2LifeEnd << " with latency "
											<< v2Lat << std::endl;
					}
					numIncompatibilities++;
					continue;
				}
				// no edge with itself
				if(v == it.first) continue;
				// source vertex: the one who comes first in the schedule
				if(sched[v] > it.second) continue;
				// create edge
				if (!quiet) {
					std::cout << "Vertices '" << v->getName() << "' and '" << it.first->getName()
										<< "' are COMPATIBLE because lifetimes DO NOT overlap" << std::endl;
					std::cout << "  " << v->getName() << ": " << v1LifeStart << " -> " << v1LifeEnd << " with latency " << v1Lat
										<< std::endl;
					std::cout << "  " << it.first->getName() << ": " << v2LifeStart << " -> " << v2LifeEnd << " with latency "
										<< v2Lat << std::endl;
				}
				variableCompatibilityGraph.createEdge(variableCompatibilityGraph.getVertexById(v->getId()),variableCompatibilityGraph.getVertexById(it.first->getId()));
			}
			// update minimum number of registers if necessary
			if (!quiet) {
				std::cout << "Incompatibilities for vertex '" << v->getName() << "': " << numIncompatibilities << std::endl;
			}
			if(numIncompatibilities > minRegs) minRegs = numIncompatibilities;
		}
		if (!quiet) {
			std::cout << "Minimum number of needed registers = " << minRegs << std::endl;
		}

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
				if (!quiet) {
					std::cout << "  Created variable '" << x_i_k[{i, k}] << "'" << std::endl;
				}
			}
		}
		if (!quiet) {
			std::cout << "Created x_i_k variables" << std::endl;
		}
		s.showLP(); // this line does nothing - only for debugging

		// boolean variables whether variable of v_i is bound to register l (y_i_l)
		std::map<std::pair<int,int>,ScaLP::Variable> y_i_l;
		for(auto &v : g->Vertices()) {
			for(int l=0; l<minRegs; ++l) {
				auto i = v->getId();
				y_i_l[{i,l}] = ScaLP::newBinaryVariable("y_" + to_string(i) + "_" + to_string(l));
				if (!quiet) {
					std::cout << "  Created variable '" << y_i_l[{i, l}] << "'" << std::endl;
				}
			}
		}
		if (!quiet) {
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
					if (!quiet) {
						std::cout << "  Created variable '" << c_r_k_l[{r->getName(), {k, l}}] << "'" << std::endl;
					}
				}
			}
		}
		if (!quiet) {
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
						if (!quiet) {
							std::cout << "  Created variable '" << a_r_n_k_l[{{r->getName(), n},
																																{k,            l}}] << "'" << std::endl;
						}
					}
				}
			}
		}
		if (!quiet) {
			std::cout << "Created a_r_n_k_l variables" << std::endl;
			s.showLP(); // this line does nothing - only for debugging
		}

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
							if (!quiet) {
								std::cout << "  Created variable '" << b_r1_r2_k1_k2_n[{{{r1->getName(), r2->getName()}, {k1, k2}}, n}]
													<< "'" << std::endl;
							}
						}
					}
				}
			}
		}
		if (!quiet) {
			std::cout << "Created b_r1_r2_k1_k2_n variables" << std::endl;
		}
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
					if (!quiet) {
						std::cout << "  Created variable '" << m_r_k_n[{r->getName(), {k, n}}] << "'" << std::endl;
					}
				}
			}
		}
		if (!quiet) {
			std::cout << "Created m_r_k_n variables" << std::endl;
			s.showLP(); // this line does nothing - only for debugging
		}

		// number of MUX inputs for each register (w_l)
		std::map<int,ScaLP::Variable> w_l;
		for(int l=0; l<minRegs; l++) {
			w_l[l] = ScaLP::newIntegerVariable("w_" + to_string(l));
			if (!quiet) {
				std::cout << "  Created variable '" << w_l[l] << "'" << std::endl;
			}
		}
		if (!quiet) {
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
		if (!quiet) {
			std::cout << "Created x_i_k=1 constraints" << std::endl;
			s.showLP(); // this line does nothing - only for debugging
		}

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

		if (!quiet) {
			std::cout << "Created y_i_l=1 constraints" << std::endl;
			s.showLP(); // this line does nothing - only for debugging
		}

		// no overlaps in operations (resource conflict graph)
		for(auto &v : g->Vertices()) {
			if (!quiet) {
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
				if (!quiet) {
					std::cout << "  conflict vertex: " << v2->getName() << std::endl;
				}
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
					if (!quiet) {
						std::cout << "  Added constraint '" << constraint << "'" << std::endl;
					}
				}
			}
		}
		if (!quiet) {
			std::cout << "Created no resource overlap constraints" << std::endl;
			s.showLP(); // this line does nothing - only for debugging
		}

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
		if (!quiet) {
			std::cout << "Created no variable overlap constraints" << std::endl;
			s.showLP(); // this line does nothing - only for debugging
		}

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
				try {
					auto n = portAssignments.at(e);
					for(int l=0; l<minRegs; l++) {
						s.addConstraint(y_i_l[{i,l}] + x_i_k[{j,k}] - a_r_n_k_l[{{rDst->getName(),n},{k,l}}] <= 1);
					}
				}
				catch (std::out_of_range&) {
					throw HatScheT::Exception("Could not find port assignment for edge '"+vSrc->getName()+"' -> '"+vDst->getName()+"'");
				}
			}
		}
		if (!quiet) {
			std::cout << "Created edge constraints for lifetime > 0" << std::endl;
			s.showLP(); // this line does nothing - only for debugging
		}

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
					try {
						auto n = portAssignments.at(e);
						s.addConstraint(x_i_k[{i,k2}] + x_i_k[{j,k1}] - b_r1_r2_k1_k2_n[{{{r1->getName(),r2->getName()},{k1,k2}},n}] <= 1);
					}
					catch (std::out_of_range&) {
						throw HatScheT::Exception("Could not find port assignment for edge '"+vSrc->getName()+"' -> '"+vDst->getName()+"'");
					}
				}
			}
		}
		if (!quiet) {
			std::cout << "Created edge constraints for lifetime = 0" << std::endl;
			s.showLP(); // this line does nothing - only for debugging
		}

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
		if (!quiet) {
			std::cout << "Created number of resource mux input constraints" << std::endl;
			s.showLP(); // this line does nothing - only for debugging
		}

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
		if (!quiet) {
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
		for(auto r : rm->Resources()) {
			int resLimit = r->getLimit();
			auto vertices = rm->getVerticesOfResource(r);
			if(r->isUnlimited()) resLimit = vertices.size();
			if (!quiet) {
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
		if (!quiet) {
			std::cout << "Created objective" << std::endl;
		}

		///////////
		// solve //
		///////////
		if (!quiet) {
			std::cout << "start solving" << std::endl;
		}
		auto stat = s.solve();
		if (!quiet) {
			std::cout << "finished solving" << std::endl;
		}

		/////////////////
		// get results //
		/////////////////
		BindingContainer b;

		if(stat != ScaLP::status::TIMEOUT_FEASIBLE and stat != ScaLP::status::OPTIMAL and stat != ScaLP::status::FEASIBLE) {
			if (!quiet) {
				std::cout << "Could not solve optimal binidng ILP formulation for II = " << II << std::endl;
				std::cout << "ScaLP solver status: " << stat << std::endl;
			}
			return b;
		}

		if (!quiet) {
			std::cout << "SOLVER RESULTS: " << std::endl;
			std::cout << s.getResult();
		}

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
				b.resourceBindings[v->getName()] = k;
				if (!quiet)
					std::cout << "Vertex '" << v->getName() << "' is bound to FU number '" << k << "'" << std::endl;
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
				b.registerEnableTimes[l] = {(sched[v] + rm->getResource(v)->getLatency()) % II};
				std::cout << "Variable '" << v->getName() << "' is bound to register number '" << l << "'" << std::endl;
			}
		}

		for(auto &r : rm->Resources()) {
			int resLimit = r->getLimit();
			if (r->isUnlimited()) resLimit = rm->getVerticesOfResource(r).size();
			for (int k = 0; k < resLimit; ++k) {
				for (int l = 0; l < minRegs; ++l) {
					auto var = c_r_k_l[{r->getName(),{k,l}}];
					auto bTemp = (bool)((int) round(results[var]));
					if(!bTemp) continue;
					//b.fuRegConnections.push_back({{r->getName(),k},l});
					b.connections.push_back({r->getName(), k, "register", l, 0});
					if (!quiet)
						std::cout << "FU '" << k << "' of type '" << r->getName() << "' is connected to register '" << l << "'" << std::endl;
				}
			}
		}

		for (auto &it : a_r_n_k_l) {
			auto var = it.second;
			auto bTemp = (bool)((int) round(results[var]));
			if(!bTemp) continue;
			auto r = it.first.first.first;
			auto n = it.first.first.second;
			auto k = it.first.second.first;
			auto l = it.first.second.second;
			//b.regFuConnections.push_back({{l,n},{r,k}});
			b.connections.push_back({"register", l, r, k, n});
			if (!quiet)
				std::cout << "Register '" << l << "' is connected to port '" << n << "' of FU '" << k << "' of type '" << r << "'" << std::endl;
		}

		for (auto &it : b_r1_r2_k1_k2_n) {
			auto var = it.second;
			auto bTemp = (bool)((int) round(results[var]));
			if(!bTemp) continue;
			auto r1 = it.first.first.first.first;
			auto r2 = it.first.first.first.second;
			auto k1 = it.first.first.second.first;
			auto k2 = it.first.first.second.second;
			auto n = it.first.second;
			//b.fuConnections.push_back({{{r2,k2},{r1,k1}},{0,n}});
			b.connections.push_back({r2, k2, r1, k1, n});
			if (!quiet)
				std::cout << "FU '" << k2 << "' of type '" << r2 << "' is connected to port '" << n << "' of FU '" << k1 << "' of type '" << r1 << "'" << std::endl;
		}

		if (!quiet) {
			std::cout << "ILP formulation SOLVED for II = " << II << std::endl;
			std::cout << "ScaLP solver status: " << stat << std::endl;
		}

		return b;
	}

	Binding::RegChainBindingContainer
	Binding::getILPBasedIntIIBinding(map<Vertex *, int> sched, Graph *g, ResourceModel *rm, int II, int wMux,
																	 int wReg, std::map<Edge *, int> portAssignments, double maxMux, double maxReg,
																	 std::set<const Resource *> commutativeOps, std::list<std::string> sw, int timeout,
																	 bool quiet) {
		OptimalIntegerIIBinding oib(g,rm,sched,II,portAssignments,commutativeOps,sw);
		oib.setMuxCostFactor(wMux);
		oib.setRegCostFactor(wReg);
		oib.setMuxLimit(maxMux);
		oib.setRegLimit(maxReg);
		oib.setTimeout(timeout);
		oib.setQuiet(quiet);
		oib.bind();
		return oib.getBinding();
	}

	Binding::RatIIRegChainBindingContainer
	Binding::getILPBasedRatIIBinding(std::vector<map<Vertex*, int>> sched, Graph *g, ResourceModel *rm, int samples, int modulo,
																	 int wMux, int wReg, std::map<Edge *, int> portAssignments, double maxMux,
																	 double maxReg, std::set<const Resource *> commutativeOps, std::list<std::string> sw,
																	 int timeout, bool quiet) {
		// mappings between vertices of original and unrolled graphs
		std::map<std::pair<std::string,int>,std::string> unrolledVertexMappings;
		std::map<std::string,std::pair<std::string,int>> unrolledVertexMappingsReverse;
		std::map<std::pair<Edge*,int>,Edge*> unrolledEdgeMappings;
		std::map<Edge*,std::pair<Edge*,int>> unrolledEdgeMappingsReverse;
		std::map<Edge*,int> unrolledPortAssignments;
		std::set<const Resource*> unrolledCommutativeOps;
		std::map<Vertex*,int> unrolledSchedule;

		// unroll graph
		// unroll graph and create corresponding resource model
		Graph g_unroll;
		ResourceModel rm_unroll;

		for(auto res : rm->Resources()) {
			auto res_unroll = &rm_unroll.makeResource(res->getName(),res->getLimit(),res->getLatency(),res->getBlockingTime());
			if(commutativeOps.find(res) != commutativeOps.end()) {
				unrolledCommutativeOps.insert(res_unroll);
			}
		}

		for(auto v : g->Vertices()) {
			for(int s=0; s<samples; ++s) {
				auto newVertex = &g_unroll.createVertex();
				newVertex->setName(v->getName()+"_"+to_string(s));
				auto originalResource = rm->getResource(v);
				rm_unroll.registerVertex(newVertex,rm_unroll.getResource(originalResource->getName()));
				// mapping
				unrolledVertexMappings[{v->getName(),s}] = newVertex->getName();
				unrolledVertexMappingsReverse[newVertex->getName()] = {v->getName(),s};
				unrolledSchedule[newVertex] = sched[s][v];
			}
		}

		for(auto e : g->Edges()) {
			auto srcName = e->getVertexSrc().getName();
			auto dstName = e->getVertexDst().getName();

			int distance = e->getDistance();
			int offset = 0;

			// adjust distance/offset so distance < samples
			while(distance>samples) {
				distance -= samples;
				++offset;
			}

			for(int s=0; s<samples; ++s) {
				// adjust distance again (only once)
				int sourceSampleNumber = s - distance;
				int edgeOffset = offset;
				if(sourceSampleNumber < 0) {
					sourceSampleNumber += samples;
					++edgeOffset;
				}

				// create edge
				Vertex* srcVertex = nullptr;
				Vertex* dstVertex = nullptr;

				for(auto v : g_unroll.Vertices()) {
					if(v->getName() == srcName + "_" + to_string(sourceSampleNumber))
						srcVertex = v;
					if(v->getName() == dstName + "_" + to_string(s))
						dstVertex = v;
				}

				auto newEdge = &g_unroll.createEdge(*srcVertex,*dstVertex,edgeOffset,e->getDependencyType());

				// mapping
				unrolledEdgeMappings[{e,s}] = newEdge;

				// copy port assignment
				unrolledPortAssignments[newEdge] = portAssignments[e];
			}
		}

		// call binding function on unrolled graph
		auto unrolledBindingContainer = getILPBasedIntIIBinding(unrolledSchedule,&g_unroll,&rm_unroll,modulo,wMux,wReg,unrolledPortAssignments,maxMux,maxReg,unrolledCommutativeOps,sw,timeout,quiet);
		RatIIRegChainBindingContainer b;
		b.solutionStatus = unrolledBindingContainer.solutionStatus;

		// fill solution structure if binding was found
		// vertex->fu bindings
		for(auto &it : unrolledBindingContainer.resourceBindings) {
			auto unrolledVertex = it.first;
			auto fu = it.second;
			auto originalVertex = unrolledVertexMappingsReverse[unrolledVertex].first;
			auto sample = unrolledVertexMappingsReverse[unrolledVertex].second;
			if(b.resourceBindings.size() <= sample) b.resourceBindings.emplace_back(std::map<std::string,int>());
			b.resourceBindings[sample][originalVertex] = fu;
		}
		// fu->fu connection bindings
		for(auto &it : unrolledBindingContainer.fuConnections) {
			auto rSrc = it.first.first.first;
			auto fuSrc = it.first.first.second;
			auto rDst = it.first.second.first;
			auto fuDst = it.first.second.second;
			auto lifetime = it.second.first;
			auto port = it.second.second;
			b.fuConnections.emplace_back(std::make_pair(std::make_pair(std::make_pair(rSrc,fuSrc),std::make_pair(rDst,fuDst)),std::make_pair(lifetime,port)));
		}
		b.multiplexerCosts = unrolledBindingContainer.multiplexerCosts;
		b.registerCosts = unrolledBindingContainer.registerCosts;

		return b;
	}

#endif

	void
	Binding::calcFUConnectionsAndCosts(Binding::RegChainBindingContainer* b, Graph* g, ResourceModel* rm, std::map<Vertex*, int>* sched, int II, std::map<Edge*,int>* portAssignments) {
		b->multiplexerCosts = 0;
		b->registerCosts = 0;
		b->fuConnections.clear();
		std::map<std::pair<std::string, int>, int> lifetimeRegsAfterResources;
		for (auto &e : g->Edges()) {
			// skip chaining edges (although I am not sure if they can even exist at this point...)
			if (!e->isDataEdge()) continue;
			// compute connection
			auto *vSrc = &e->getVertexSrc();
			auto *vDst = &e->getVertexDst();
			// get binding of src vertex
			auto fuSrcIt = b->resourceBindings.find(vSrc->getName());
			// check if src vertex has binding
			if (fuSrcIt == b->resourceBindings.end()) continue;
			// get binding of dst vertex
			auto fuDstIt = b->resourceBindings.find(vDst->getName());
			// check if dst vertex has binding
			if (fuDstIt == b->resourceBindings.end()) continue;
			// process connection
			auto &fuSrc = fuSrcIt->second;
			auto &fuDst = fuDstIt->second;
			auto *rSrc = rm->getResource(vSrc);
			auto *rDst = rm->getResource(vDst);
			auto &tSrc = sched->at(vSrc);
			auto &tDst = sched->at(vDst);
			auto latSrc = rm->getVertexLatency(vSrc);
			auto distance = e->getDistance();
			auto lifetime = tDst - tSrc - latSrc + (II * distance);
			auto port = portAssignments->at(e);
			auto connection = std::make_pair(std::make_pair(std::make_pair(rSrc->getName(), fuSrc), std::make_pair(rDst->getName(), fuDst)), std::make_pair(lifetime, port));
			if (std::find(b->fuConnections.begin(), b->fuConnections.end(), connection) == b->fuConnections.end()) {
				// connection does not already exist
				// create connection
				b->fuConnections.emplace_back(connection);
				// update mux costs
				b->multiplexerCosts++;
				// update register costs if needed
				auto currentRegs = lifetimeRegsAfterResources[std::make_pair(rSrc->getName(), fuSrc)];
				auto diff = lifetime - currentRegs;
				if (diff > 0) {
					b->registerCosts += diff;
					lifetimeRegsAfterResources[std::make_pair(rSrc->getName(), fuSrc)] += diff;
				}
			}
		}
	}
}
