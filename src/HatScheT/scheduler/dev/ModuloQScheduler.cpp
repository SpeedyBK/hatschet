//
// Created by nfiege on 17/10/19.
//

#include "ModuloQScheduler.h"
#include <HatScheT/utility/Utility.h>
#include <HatScheT/utility/subgraphs/KosarajuSCC.h>
#include <HatScheT/scheduler/ilpbased/RationalIIScheduler.h>
#include <cmath>

namespace HatScheT {
	ModuloQScheduler::ModuloQScheduler(HatScheT::Graph &g, HatScheT::ResourceModel &resourceModel,
																							 std::list<std::string> solverWishlist) :
		SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist), latencySequence(),
		solverWishlist(solverWishlist)
	{

		this->computeMinII(&this->g, &this->resourceModel);
		double minII = this->getMinII();
		//ceiling
		this->integerMinII = (int)ceil(minII);
		pair<int, int> frac = Utility::splitRational(minII);

		cout << "rational min II is " << minII << endl;
		cout << "integer min II is " << this->integerMinII << endl;
		cout << "auto setting samples to " << frac.second << endl;
		cout << "auto setting modulo to " << frac.first << endl;

		this->S = frac.second;
		this->M = frac.first;

	}

	void ModuloQScheduler::schedule() {
		std::cout << "graph: " << std::endl;
		std::cout << this->g << std::endl;
		std::cout << "resource model: " << std::endl;
		std::cout << this->resourceModel << std::endl;
		std::cout << "M: " << this->M << std::endl;
		std::cout << "S: " << this->S << std::endl;

		// find SCCs
		KosarajuSCC k(this->g);
		k.setQuiet();
		auto sccs = k.getSCCs();

		// schedule SCCs
		auto sccSchedule = this->getSCCSchedule(sccs);

		// print schedule for debugging reasons
		std::cout << "scc schedule:" << std::endl;
		for(auto it : sccSchedule) {
			auto v = it.first;
			auto t = it.second.first;
			auto id = it.second.second;
			std::cout << "    " << v->getName() << ": " << t << ", " << id << std::endl;
		}

		throw HatScheT::Exception("ModuloQScheduler::schedule: I don't work yet :(");
	}

	std::vector<std::vector<unsigned int>> ModuloQScheduler::getAllLatencySequences(int M, int S) {
		if(M<1 or S<1)
			throw HatScheT::Exception("Invalid values for M and S given: "+to_string(M)+" and "+to_string(S));
		vector<vector<unsigned int>> latencySequences;

		vector<unsigned int> nextSequence = {0};
		for(unsigned int i=0; i<S-1; ++i) {
			nextSequence.emplace_back(nextSequence[i]+1);
		}

		bool finished = false;
		while(!finished) {
			// push latency sequence into list
			latencySequences.emplace_back(nextSequence);

			// calculate next sequence
			for(unsigned int i=0; i<=S-1; ++i) {
				unsigned int index = S - 1 - i;
				++nextSequence[index];
				auto diff = nextSequence.size()-index;
				if(nextSequence[index]<M-diff+1) break;
			}
			for(unsigned int i=0; i<=S-1; ++i) {
				unsigned int index = S - 1 - i;
				auto diff = nextSequence.size()-index;
				if(nextSequence[index]==M-diff+1) nextSequence[index] = index;
			}
			for(unsigned int i=0; i<S-1; ++i) {
				unsigned int index = i;
				while(nextSequence[index+1]<=nextSequence[index]) ++nextSequence[index+1];
			}

			// check if finished
			if(nextSequence[0] != 0) finished = true;
		}

		return latencySequences;
	}

	std::map<Vertex *, pair<int, int>> ModuloQScheduler::getSCCSchedule(std::vector<SCC *> &sccs) {
		map<Vertex *, pair<int, int>> sccSchedule;
		////////////////////////////////////////////////////////
		// generate new graph containing all non-trivial SCCs //
		////////////////////////////////////////////////////////
		Graph tempG;
		std::map<Vertex*,Vertex*> originalVertexMap; // original vertex -> tempG vertex
		std::map<Vertex*,Vertex*> tempGVertexMap; // tempG vertex -> original vertex
		std::map<Vertex*,int> sccMap; // tempG vertex -> id of the scc it belongs to
		for(auto scc : sccs) {
			if(scc->getSccType(&this->resourceModel) == scctype::trivial) {
				std::cout << "trivial" << std::endl;
				for(auto v : scc->getVerticesOfSCC()) {
					std::cout << "    " << v->getName() << std::endl;
				}
				continue;
			}
			else if(scc->getSccType(&this->resourceModel) == scctype::unknown) {
				std::cout << "unknown" << std::endl;
				for(auto v : scc->getVerticesOfSCC()) {
					std::cout << "    " << v->getName() << std::endl;
				}
				continue;
			}
			else {
				std::cout << "penis" << std::endl;
				for(auto v : scc->getVerticesOfSCC()) {
					std::cout << "    " << v->getName() << std::endl;
				}
			}
			// insert scc into tempG
			// insert vertices into tempG
			for(auto v : scc->getVerticesOfSCC()) {
				auto &newV = tempG.createVertex(v->getId());
				originalVertexMap[v] = &newV;
				tempGVertexMap[&newV] = v;
				sccMap[&newV] = scc->getId();
			}
			// insert edges into tempG
			for(auto e : scc->getSCCEdges()) {
				auto src = originalVertexMap[&e->getVertexSrc()];
				auto dst = originalVertexMap[&e->getVertexDst()];
				auto &newE = tempG.createEdge(*src,*dst,e->getDistance(),e->getDependencyType());
				newE.setDelay(e->getDelay());
			}
		}

		/////////////////////////////
		// generate resource model //
		/////////////////////////////
		ResourceModel rm;
		for(auto v : this->g.Vertices()) {
			// skip vertices that are only in trivial SCCs
			if(originalVertexMap.find(v) == originalVertexMap.end()) continue;
			auto res = this->resourceModel.getResource(v);
			Resource* newRes;
			// only create new resource if it does not already exist
			try {
				newRes = rm.getResource(res->getName());
			}
			catch(HatScheT::Exception&) {
				newRes = &rm.makeResource(res->getName(),res->getLimit(),res->getLatency(),res->getBlockingTime());
			}
			// register vertex of tempG to new resource
			rm.registerVertex(originalVertexMap[v],newRes);
		}

		///////////////
		// debugging //
		///////////////
		std::cout << "graph to be scheduled by ratII:" << std::endl;
		std::cout << tempG;
		std::cout << "corresponding resource model:" << std::endl;
		std::cout << rm;

		/////////////////////////////////////////
		// schedule graph with ratII scheduler //
		/////////////////////////////////////////
		auto* ratII = new RationalIIScheduler(tempG,rm,this->solverWishlist);
		ratII->setModulo(this->M);
		ratII->setSamples(this->S);
		ratII->schedule();

		if(!ratII->getScheduleFound()) {
			sccSchedule.clear(); // clear to be safe that it's empty
			return sccSchedule; // return empty schedule
		}

		this->latencySequence = ratII->getInitIntervalls();

		/////////////////////
		// return solution //
		/////////////////////
		for(auto p : ratII->getSchedule()) {
			auto v = p.first;
			auto t = p.second;
			sccSchedule[tempGVertexMap[v]] = std::make_pair(t,sccMap[v]);
		}
		return sccSchedule;
	}
}


