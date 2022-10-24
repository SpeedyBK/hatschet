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
#include <HatScheT/utility/OptimalIntegerIIBindingCong.h>
#endif

namespace HatScheT {

	std::map<const Vertex *, int> Binding::getSimpleBinding(const std::map<Vertex *, int> &sched, ResourceModel *rm, int II) {
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
		// set it up
		OptimalIntegerIIBindingCong oib (g, rm, std::move(sched), II, std::move(portAssignments), std::move(sw));
		oib.setQuiet(quiet);
		oib.setTimeout(timeout);
		// start it
		oib.bind();
		// get results
		BindingContainer b;
		oib.getBinding(&b);
		// return results
		return b;
	}

	Binding::RegChainBindingContainer
	Binding::getILPBasedIntIIBinding(map<Vertex *, int> sched, Graph *g, ResourceModel *rm, int II, int wMux,
																	 int wReg, std::map<Edge *, int> portAssignments, double maxMux, double maxReg,
																	 std::set<const Resource *> commutativeOps, std::list<std::string> sw, int timeout,
																	 bool quiet) {
		OptimalIntegerIIBinding oib(g,rm,std::move(sched),II,std::move(portAssignments),std::move(commutativeOps),std::move(sw));
		oib.setMuxCostFactor(wMux);
		oib.setRegCostFactor(wReg);
		oib.setMuxLimit(maxMux);
		oib.setRegLimit(maxReg);
		oib.setTimeout(timeout);
		oib.setQuiet(quiet);
		oib.bind();
		RegChainBindingContainer b;
		oib.getBinding(&b);
		return b;
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
		// solution status
		b.solutionStatus = unrolledBindingContainer.solutionStatus;
		// port assignments
		for (auto &it : unrolledBindingContainer.portAssignments) {
			auto unrolledEdge = it.first;
			auto portAssignment = it.second;
			auto originalEdge = unrolledEdgeMappingsReverse[unrolledEdge].first;
			auto sample = unrolledEdgeMappingsReverse[unrolledEdge].second;
			if (sample >= b.portAssignments[originalEdge].size()) {
				b.portAssignments[originalEdge].resize(sample+1);
			}
			b.portAssignments[originalEdge][sample] = portAssignment;
		}
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
		// costs
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

	ostream& operator<<(ostream& os, const Binding::BindingContainer& bin) {
		os << "Binding container" << std::endl;
		os << "Mux costs: " << bin.multiplexerCosts << std::endl;
		os << "Reg costs: " << bin.registerCosts << std::endl;
		os << "Solution status: " << bin.solutionStatus << std::endl;
		os << "Port assignments: " << std::endl;
		if (bin.portAssignments.empty()) {
			os << "  *empty*" << std::endl;
		}
		else {
			for (auto &it : bin.portAssignments) {
				os << "  edge '" << it.first->getVertexSrcName() << "' (" << it.second.first << ") -("
				   << it.first->getDistance() << ")-> '" << it.first->getVertexDstName() << "' (" << it.second.second << ")"
				   << std::endl;
			}
		}
		os << "Resource bindings: " << std::endl;
		if (bin.resourceBindings.empty()) {
			os << "  *empty*" << std::endl;
		}
		else {
			for (auto &it : bin.resourceBindings) {
				os << "  " << it.first << " -> ";
				for (auto &it2 : it.second) {
					os << it2 << " ";
				}
				os << std::endl;
			}
		}
		os << "Connections: " << std::endl;
		if (bin.connections.empty()) {
			os << "  *empty*" << std::endl;
		}
		else {
			for (auto &it : bin.connections) {
				os << "  "   << std::get<0>(it) << " (" << std::get<1>(it) << ") port " << std::get<2>(it)
				   << " -> " << std::get<3>(it) << " (" << std::get<4>(it) << ") port " << std::get<5>(it) << std::endl;
			}
		}
		os << "Register enable times: " << std::endl;
		if (bin.registerEnableTimes.empty()) {
			os << "  *empty*" << std::endl;
		}
		else {
			for (auto &it : bin.registerEnableTimes) {
				os << "  reg #" << it.first << ": ";
				for (auto &it2 : it.second) {
					os << it2 << " ";
				}
				os << std::endl;
			}
		}
		return os;
	}
}
