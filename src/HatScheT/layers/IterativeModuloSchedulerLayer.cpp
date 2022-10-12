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
    this->scheduleFound = false;
    this->maxLatencyConstraint = -1;

    // ModuloSchedulerBase:
    this->firstObjectiveOptimal = true;
    this->secondObjectiveOptimal = true;

    // IterativeModuloSchedulerLayer:
    this->disableObj = false;
    this->timeBudget = INT32_MAX/2;
    this->solvingTimePerIteration = 0;

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

    // ----------------- //
    // Debugging Prints. //
    // ----------------- //
    if(this->quiet)
    {
        cout << "Minimal II is " << this->minII << endl;
        cout << "Maximal II is " << this->maxII << endl;
        cout << "RecMin II is " << this->recMinII << endl;
        cout << "ResMin II is " << this->resMinII << endl;
    }
}

void HatScheT::IterativeModuloSchedulerLayer::schedule() {

    // Define maxII if maxRuns Constraint is given:
    if (this->maxRuns > 0)
    {
        maxII = (int)this->minII + maxRuns;
    }

    // -------------------------------------- //
    // Iterative Search for best possible II: //
    // -------------------------------------- //
    for (int ii = (int)minII; ii <= maxII; ii++){
        auto start_t = std::chrono::high_resolution_clock::now();
        scheduleIteration();
        auto end_t = std::chrono::high_resolution_clock::now();
        solvingTimePerIteration = floor(std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count());
    }
}
