//
// Created by nfiege on 29/04/21.
//

#include "CombinedRationalIIScheduler.h"
#include "SCCQScheduler.h"
#include "UniformRationalIISchedulerNew.h"
#include <cmath>

HatScheT::CombinedRationalIIScheduler::CombinedRationalIIScheduler(Graph &g, ResourceModel &resourceModel,
	std::list <string> solverWishlist) :
	RationalIISchedulerLayer(g, resourceModel), ILPSchedulerBase(solverWishlist), solverWishlist(solverWishlist)
{
	this->s_start = -1;
	this->m_start = -1;
	this->solverWishlist = solverWishlist;

	this->scheduleFound = false;

	this->computeMinII(&g,&resourceModel);
	this->minII = ceil(this->minII);
	this->computeMaxII(&g,&resourceModel);
	if (this->minII >= this->maxII) this->maxII = this->minII+1;
	this->initialSolutionRatII.clear();
}

void HatScheT::CombinedRationalIIScheduler::scheduleIteration() {
	// reset II
	this->II = -1;

	// clear container
	this->initialSolutionRatII.clear();

	// heuristic scheduling first
	SCCQScheduler sccq(this->g,this->resourceModel,this->solverWishlist);
	sccq.setMaxRuns(1);
	sccq.disableVerifier();
	sccq.setSolverTimeout(this->solverTimeout);
	sccq.setSamples(this->samples);
	sccq.setModulo(this->modulo);
	sccq.schedule();

	// track time
	double heuristicTime = sccq.getSolvingTime();
	long ilpTimeoutSec = this->solverTimeout - (long)heuristicTime;
	if(ilpTimeoutSec <= 0) ilpTimeoutSec = 1;

	// set up optimal algorithm
	UniformRationalIISchedulerNew opt(this->g,this->resourceModel,this->solverWishlist);
	opt.setMaxRuns(1);
	opt.disableVerifier();
	opt.setSolverTimeout(ilpTimeoutSec);
	opt.setSamples(this->samples);
	opt.setModulo(this->modulo);

	// track if heuristic scheduler found solution
	bool heuristicFoundSolution = sccq.getScheduleFound();
	this->stat = opt.getScaLPStatus();

	if(sccq.getScheduleFound()) {
		// heuristic scheduler found solution, yay! :)
		this->scheduleFound = true;
		this->startTimesVector = sccq.getStartTimeVector();
		opt.initialSolutionRatII = sccq.getStartTimeVector();
		if(!this->quiet) {
			std::cout << "Heuristic scheduler found solution for II=" << sccq.getM_Found() << "/" << sccq.getS_Found()
								<< " with latency=" << sccq.getScheduleLength() << " in " << heuristicTime << " sec" << std::endl;
		}
	}
	else {
		// handle situation in which heuristic scheduler could not find solution :(
		this->scheduleFound = false;
	}

	// start optimal algorithm
	opt.schedule();

	// track if optimal scheduler found solution
	bool optimalFoundSolution = opt.getScheduleFound();
	this->stat = opt.getScaLPStatus();

	// track time
	double optimalTime = opt.getSolvingTime();
	this->solvingTime = heuristicTime + optimalTime;

	if(opt.getScheduleFound()) {
		// fill solution structure if schedule was found
		this->startTimesVector = opt.getStartTimeVector();
		if(!this->quiet) {
			std::cout << "Optimal scheduler found solution for II=" << opt.getM_Found() << "/" << opt.getS_Found()
								<< " with latency=" << opt.getScheduleLength() << " and ScaLP status " << this->stat
								<< " in additional " << optimalTime << " sec (total: " << this->solvingTime << " sec)" << std::endl;
		}
	}
	else {
		// handle situation in which optimal scheduler does not find solution but heuristic scheduler does
		if(heuristicFoundSolution and (!optimalFoundSolution)) {
			this->stat = ScaLP::status::UNKNOWN;
		}
	}

	this->solvingTime = heuristicTime + optimalTime;
	this->scheduleFound = opt.getScheduleFound();
}
