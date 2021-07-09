//
// Created by nfiege on 7/9/21.
//

#include "PBScheduler.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/Exception.h"
#include "HatScheT/utility/subgraphs/KosarajuSCC.h"
#include "HatScheT/scheduler/ASAPScheduler.h"

#include <iostream>
#include <map>
#include <cmath>

namespace HatScheT {
	PBScheduler::PBScheduler(HatScheT::Graph &g, HatScheT::ResourceModel &resourceModel,
																		 std::list<std::string> solverWishlist) : SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist) {
		// reset previous solutions
		this->II = -1;
		this->timeouts = 0;
		this->startTimes.clear();
		this->scheduleFound = false;
		this->optimalResult = true;
		computeMinII(&g, &resourceModel);
		this->minII = ceil(this->minII);
		computeMaxII(&g, &resourceModel);
	}

	PBScheduler::~PBScheduler() {
		for(auto subG : this->subgraphs) {
			delete subG;
		}
	}

	void PBScheduler::schedule() {
		if(!this->quiet){
			std::cout << "PBS: min/maxII = " << this->minII << " " << this->maxII << ", (minResII/minRecII " << this->resMinII << " / " << this->recMinII << ")" << std::endl;
			std::cout << "PBS: solver timeout = " << this->solverTimeout << " (sec)" << endl;
		}

		//set maxRuns, e.g., maxII - minII, iff value if not -1
		if(this->maxRuns > 0){
			int runs = this->maxII - this->minII;
			if(runs > this->maxRuns) this->maxII = this->minII + this->maxRuns - 1;
			if(this->quiet==false) std::cout << "PBS: maxII changed due to maxRuns value set by user!" << endl;
			if(this->quiet==false) std::cout << "PBS: min/maxII = " << this->minII << " " << this->maxII << std::endl;
		}

		if (this->minII > this->maxII)
			throw HatScheT::Exception("Inconsistent II bounds");

		// find subgraphs based on SCCs
		this->createSCCs();
		this->orderSCCs();
		this->partitionSCCs();

		// iterative modulo scheduling loop
		for (int candII = this->minII; candII <= this->maxII; ++candII) {
			scheduleAttempt(candII);
			if (this->scheduleFound) {
				this->II = candII;
				auto solution = this->solver->getResult().values;

				if(!this->quiet) {
					std::cout << "PBS: found " << (this->optimalResult ? "optimal" : "feasible") << " solution with II=" << this->II << std::endl;
					for(auto it : this->startTimes) {
						std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
					}
				}
				break;
			}
			if(!this->scheduleFound) if(this->quiet==false) cout << "  II=" << candII << " : " << this->stat << endl;
		}
		if(scheduleFound == false) this->II = -1;
		if(this->quiet==false) std::cout << "PBS: solving time was " << this->solvingTime << " seconds" << std::endl;
	}

	void PBScheduler::scheduleAttempt(int candidateII) {
		// VARIABLES TO SET:
		//   this->begin
		//   this->end
		//   this->scheduleFound
		//   this->optimalResult
		//   this->startTimes
		//   this->solvingTime

		// WHAT TO DO:
		//
	}

	void PBScheduler::createSCCs() {
		// use kosaraju scc finder to find sccs
		KosarajuSCC k(this->g);
		k.setQuiet(this->quiet);
		this->sccs = k.getSCCs();
	}

	void PBScheduler::orderSCCs() {
		// topologically sort sccs based on an asap schedule
		Graph sccG;

		for (auto scc : this->sccs) {
			// create a vertex for each scc
			sccG.createVertex(scc->getId());
		}
	}

	void PBScheduler::partitionSCCs() {

	}

	void PBScheduler::scheduleSubgraphs() {

	}

	void PBScheduler::sortSubgraphs() {

	}

	void PBScheduler::postProcessSchedule() {

	}
}

