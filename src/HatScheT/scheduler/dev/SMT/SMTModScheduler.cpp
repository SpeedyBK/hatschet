//
// Created by bkessler on 5/7/22.
//

#include "SMTModScheduler.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/Exception.h"

#include "HatScheT/scheduler/ilpbased/EichenbergerDavidson97Scheduler.h"

#include <iostream>
#include <map>
#include <limits>
#include <cmath>

namespace HatScheT {

  SMTModScheduler::SMTModScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string> solverWishlist)
      : SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist) {
      // reset previous solutions
      II = -1;
      this->timeouts = 0;
      startTimes.clear();
      scheduleFound = false;
      optimalResult = true;
      computeMinII(&g, &resourceModel);
      minII = ceil(minII);
      computeMaxII(&g, &resourceModel);
  }

  void SMTModScheduler::schedule() {

      auto es = EichenbergerDavidson97Scheduler(g, resourceModel, {"CPLEX", "Gurobi", "SCIP", "LPSolve"});
      es.schedule();
      this->startTimes = es.getSchedule();
      this->II = es.getII();

      resourceModel.

  }
}