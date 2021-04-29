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

	double heuristicTime = sccq.getSolvingTime();

	// set up optimal algorithm
	UniformRationalIISchedulerNew opt(this->g,this->resourceModel,this->solverWishlist);
	opt.setMaxRuns(1);
	opt.disableVerifier();
	opt.setSolverTimeout(this->solverTimeout - heuristicTime);
	opt.setSamples(this->samples);
	opt.setModulo(this->modulo);

	// use heuristic solution as start solution for optimal algorithm
	if(sccq.getScheduleFound()) {
		opt.initialSolutionRatII = sccq.getStartTimeVector();
		if(!this->quiet) {
			std::cout << "Heuristic scheduler found solution for II=" << sccq.getM_Found() << "/" << sccq.getS_Found()
			  << " with latency=" << sccq.getScheduleLength() << " in " << heuristicTime << " sec" << std::endl;
		}
	}

	// start optimal algorithm
	opt.schedule();

	double optimalTime = opt.getSolvingTime();

	// fill solution structure if schedule was found
	if(opt.getScheduleFound()) {
		this->startTimesVector = opt.getStartTimeVector();
		this->stat = opt.getScaLPStatus();
		if(!this->quiet) {
			std::cout << "Optimal scheduler found solution for II=" << opt.getM_Found() << "/" << opt.getS_Found()
								<< " with latency=" << opt.getScheduleLength() << " and ScaLP status " << this->stat
								<< " in " << optimalTime << " sec" << std::endl;
		}
	}

	this->solvingTime = heuristicTime + optimalTime;
	this->scheduleFound = opt.getScheduleFound();
}
