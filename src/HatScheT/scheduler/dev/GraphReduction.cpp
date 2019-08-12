//
// Created by bkessler on 23/07/19.
//

#include "GraphReduction.h"

#include <cmath>
#include "HatScheT/utility/subgraphs/KosarajuSCC.h"

namespace HatScheT {

  GraphReduction::GraphReduction(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist) : SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist) {
    II = -1;
    this->timeouts = 0;
    startTimes.clear();
    scheduleFound = false;
    optimalResult = true;
    computeMinII(&g, &resourceModel);
    minII = ceil(minII);
    computeMaxII(&g, &resourceModel);

  }


  void GraphReduction::schedule() {
    cout << "GraphReduction::schedule: start" << endl;

    //ToDo implement all the stuff!
    KosarajuSCC kscc(this->g);
    kscc.getSCCs();

    cout << "GraphReduction::schedule: finished" << endl;

  }
}