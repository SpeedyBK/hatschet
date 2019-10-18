//
// Created by nfiege on 17/10/19.
//

#include "ModuloQScheduler.h"
#include <HatScheT/utility/Utility.h>
#include <HatScheT/utility/subgraphs/KosarajuSCC.h>
#include <cmath>

namespace HatScheT {
	ModuloQScheduler::ModuloQScheduler(HatScheT::Graph &g, HatScheT::ResourceModel &resourceModel,
																							 std::list<std::string> solverWishlist) :
		SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist), latencySequences()
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

		// enumerate latency sequence
		this->latencySequences = ModuloQScheduler::getAllLatencySequences(this->M,this->S);

		std::cout << "latency sequences:" << std::endl;
		for(auto it : this->latencySequences) {
			for(auto it2 : it) {
				std::cout << it2 << " ";
			}
			std::cout << endl;
		}

		bool foundSolution = false;
		for(unsigned int i=0; (i<this->latencySequences.size()) and (!foundSolution); ++i) {
			// find SCCs
			KosarajuSCC k(this->g);
			k.setQuiet();
			auto sccs = k.getSCCs();

			for(auto scc : sccs) {
				cout << "vertices of scc:" << endl;
				for(auto v : scc->getVerticesOfSCC()) {
					cout << "  " << v->getName() << endl;
				}
			}

			foundSolution = true;
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
}


