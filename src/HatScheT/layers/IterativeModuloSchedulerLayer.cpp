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
    this->secondObjectiveOptimal = false;

    // IterativeModuloSchedulerLayer:
    this->disableSecObj = false;
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
        cout << "IterativeModuloSchedulerLayer: " << endl;
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
        if(!this->quiet)
        {
            std::cout << "IterativeModuloSchedulerLayer: maxII changed due to maxRuns value set by user to " << this->maxRuns << endl;
            std::cout << "IterativeModuloSchedulerLayer: min/maxII = " << minII << " / " << maxII << std::endl;
        }
    }

    // ------------------------------------------------- //
    // Initiate what is needed before II - Search - Loop //
    // ------------------------------------------------- //
    if (!this->quiet)
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
    if (!this->quiet)
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
        if (!this->quiet)
        {
            cout << "IterativeModuloSchedulerLayer: " << ii - minII + 1 << ". Iteration, time budget: ";
            cout << timeBudget * 1000 << " milliseconds." << endl;
        }
        // Time tracking begin.
        auto start_t = std::chrono::high_resolution_clock::now();
        // Passing II from II - search - loop to class, so scheduler can use it.
        this->II = ii;
        // Schedule Iteration.
        scheduleIteration();
        // Time tracking end.
        auto end_t = std::chrono::high_resolution_clock::now();
        // Saving solving time.
        solvingTimePerIteration = floor(std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count());
        // Print time for the current iteration.
        if (!this->quiet)
        {
            cout << "IterativeModuloSchedulerLayer: Iteration done in " << solvingTimePerIteration << " milliseconds.";
            cout << endl << endl;
        }
        // If a schedule is found, we break the loop.
        // Scheduler has to fill the solution structure by itself!!!
        if (scheduleFound)
        {
            return;
        }
    }
    // ----------------------------------------------------- //
    // Define what has to be done, when there is no schedule //
    // ----------------------------------------------------- //
    this->II = -1;
    startTimes.clear();
    if(!this->quiet)
    {
        cout << "IterativeModuloSchedulerLayer: Schedule Loop done, no schedule found!" << endl;
    }
}
