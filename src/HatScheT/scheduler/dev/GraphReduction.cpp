//
// Created by bkessler on 23/07/19.
//

#include "GraphReduction.h"

#include <cmath>
#include "HatScheT/utility/subgraphs/KosarajuSCC.h"

namespace HatScheT {

  GraphReduction::GraphReduction(Graph &g, ResourceModel &resourceModel, std::list<std::string> solverWishlist)
      : SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist) {
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
    cout << endl << "GraphReduction::schedule: start" << endl;

    cout << endl << "Starting Kosajaru's SCC Algorithm..." << endl;
    KosarajuSCC kscc(this->g);
    this->sccs = kscc.getSCCs();
    cout << endl << "Finished Kosajaru's SCC Algorithm." << endl;

    cout << endl << "Setting SCC-Types" << endl;

    for (auto it : sccs){
      it->setSCCType(determineType(it));
      //Debug
      cout << endl << "SCC " << it->getId() << " is Type (0 = unknown, 1 = Basic, 2 = Complex, 3 = Trivial): " << it->getSccType() << endl;
    }

    cout << endl << "Done setting Types." << endl;



    cout << endl << "GraphReduction::schedule: done!" << endl;

  }

  scctype GraphReduction::determineType(SCC* scc) {

    if (scc->getNumberOfVertices() == 1){
      return trivial;
    }

    auto vertexMap = scc->getVertexMap();

    for (auto it:vertexMap){
      if (resourceModel.getResource(it.second)->getLimit() != -1){
        return complex;
      }
    }

    return basic;
  }
}
