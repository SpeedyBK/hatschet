//
// Created by bkessler on 10/12/22.
//

#include "IterativeModuloSchedulerLayer.h"
#include <chrono>
#include <cmath>

HatScheT::IterativeModuloSchedulerLayer::IterativeModuloSchedulerLayer(HatScheT::Graph &g, HatScheT::ResourceModel &resourceModel, double II) : SchedulerBase(g, resourceModel) {

    // ----------------------------------------------- //
    // Setting up basic variables with default values: //
    // ----------------------------------------------- //

    // IterativeSchedulerBase:
    this->maxRuns = -1;
    this->timeouts = 0;

    // SchedulerBase:
    this->startTimes.clear();
    this->scheduleFound = false;
    this->maxLatencyConstraint = -1;

    // ModuloSchedulerBase:
    this->firstObjectiveOptimal = true;
    this->secondObjectiveOptimal = false;

    // IterativeModuloSchedulerLayer:
    this->disableSecObj = false;
    this->layerQuiet = true;

    // ---------------------------------------- //
    // Calculate min. and max. II if not given. //
    // ---------------------------------------- //
    if (II <= 0)
    {
        computeMinII(&g, &resourceModel);
        this->minII = ceil(this->minII);
        computeMaxII(&g, &resourceModel);
    }
    else
    {
        this->minII = II;
        this->maxII = (int)II;
        this->resMinII = II;
        this->recMinII = II;
    }
}

void HatScheT::IterativeModuloSchedulerLayer::schedule() {

    // Define maxII if maxRuns Constraint is given:
    if (this->maxRuns > 0)
    {
        maxII = (int)this->minII + maxRuns - 1;
        if(!this->quiet or !this->layerQuiet)
        {
            std::cout << "IterativeModuloSchedulerLayer: maxII changed due to maxRuns value set by user to " << this->maxRuns << endl;
            std::cout << "IterativeModuloSchedulerLayer: min/maxII = " << minII << " / " << maxII << std::endl;
        }
    }

    // ------------------------------------------------- //
    // Initiate what is needed before II - Search - Loop //
    // ------------------------------------------------- //
    if (!this->quiet or !this->layerQuiet)
    {
        cout << endl;
        cout << "********************************************************************" << endl;
        cout << "* IterativeModuloSchedulerLayer: Initialize Schedule Parameters... *" << endl;
        cout << "********************************************************************" << endl;
        cout << endl;
    }
    scheduleInit();

    // -------------------------------------- //
    // Iterative Search for best possible II: //
    // -------------------------------------- //
    if (!this->quiet or !this->layerQuiet)
    {
        cout << endl;
        cout << "*********************************************************" << endl;
        cout << "* IterativeModuloSchedulerLayer: Begin of Scheduling... *" << endl;
        cout << "*********************************************************" << endl;
        cout << endl;
    }
    // II - search - Loop
    for (int ii = (int)minII; ii <= maxII; ii++){
        // For Debugging
        if (!this->quiet or !this->layerQuiet)
        {
            cout << "IterativeModuloSchedulerLayer: " << ii - minII + 1 << ". Iteration, time budget: ";
            cout << solverTimeout << " seconds." << endl;
        }
        // Passing II from II-search-loop to class, so scheduler can use it.
        this->II = ii;
				// schedule length estimation
				this->calculateScheduleLengthEstimation();
				if (!this->boundSL) {
					this->minSL = -1;
					this->maxSL = this->maxLatencyConstraint;
				}
        // Schedule Iteration.
        scheduleIteration();
        // Print time for the current iteration.
        if (!this->quiet or !this->layerQuiet)
        {
            cout << "IterativeModuloSchedulerLayer: Iteration done in " << this->solvingTimePerIteration << " seconds, time remaining: ";
            cout << this->timeRemaining << endl << endl;
        }
			updateSolvingTimeTotal();
        // If a schedule is found, we break the loop.
        // Scheduler has to fill the solution structure by itself!!!
        if (scheduleFound)
        {
            scheduleCleanup();
            if (!this->quiet or !this->layerQuiet){
                cout << "1st. Objective Optimal: " << firstObjectiveOptimal << endl;
                cout << "2nd. Objective Optimal: " << secondObjectiveOptimal << endl;
            }
            return;
        }
    }
    // ----------------------------------------------------- //
    // Define what has to be done, when there is no schedule //
    // ----------------------------------------------------- //
    this->II = -1;
    startTimes.clear();
    if(!this->quiet or !this->layerQuiet)
    {
        cout << "IterativeModuloSchedulerLayer: Schedule Loop done, no schedule found!" << endl;
    }
}

void HatScheT::IterativeModuloSchedulerLayer::getDebugPrintouts() {

    // ----------------- //
    // Debugging Prints. //
    // ----------------- //
    cout << "IterativeModuloSchedulerLayer: " << endl;
    cout << "Minimal II is " << this->minII << endl;
    cout << "Maximal II is " << this->maxII << endl;
    cout << "RecMin II is " << this->recMinII << endl;
    cout << "ResMin II is " << this->resMinII << endl;

}

void HatScheT::IterativeModuloSchedulerLayer::calculateScheduleLengthEstimation() {
#ifdef USE_SCALP
	// create object
	this->scheduleLengthEstimation = std::unique_ptr<ILPScheduleLengthEstimation>(new ILPScheduleLengthEstimation(&this->g, &this->resourceModel, {"Gurobi", "CPLEX", "LPSolve", "SCIP"}, 1));
	//ILPScheduleLengthEstimation scheduleLengthEstimation(&this->g, &this->resourceModel, {"Gurobi", "CPLEX", "LPSolve", "SCIP"}, 1);
	this->scheduleLengthEstimation->setQuiet(this->quiet);
	if (!this->boundSL) return; // only init object but don't actually do the work

	// min SL estimation
	this->scheduleLengthEstimation->estimateMinSL((int)this->II, (int)this->solverTimeout);
	if (this->scheduleLengthEstimation->minSLEstimationFound()) {
		this->minSL = this->scheduleLengthEstimation->getMinSLEstimation();
		this->earliestStartTimes = this->scheduleLengthEstimation->getASAPTimesSDC();
	}
	/*
	// max SL estimation
	this->scheduleLengthEstimation->estimateMaxSL((int)this->II, (int)this->solverTimeout);
	if (this->scheduleLengthEstimation->maxSLEstimationFound()) {
		this->maxSL = this->scheduleLengthEstimation->getMaxSLEstimation();
	}
	if (!this->quiet) {
		std::cerr << "IterativeModuloSchedulerLayer::calculateScheduleLengthEstimation: II=" << (int)this->II << " SL >= " << this->minSL << std::endl;
		std::cerr << "IterativeModuloSchedulerLayer::calculateScheduleLengthEstimation: II=" << (int)this->II << " SL <= " << this->maxSL << std::endl;
	}
	// consider max latency constraint if it was given by the user
	if (this->maxLatencyConstraint >= 0) {
		this->maxSL = std::min(this->maxSL, this->maxLatencyConstraint);
		if (!this->quiet) {
			std::cerr << "IterativeModuloSchedulerLayer::calculateScheduleLengthEstimation: corrected max SL to " << this->maxSL << std::endl;
		}
	}
	 */
#endif
}
