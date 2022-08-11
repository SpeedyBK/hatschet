//
// Created by bkessler on 8/11/22.
//

#include <cmath>

#include "SMTSCCScheduler.h"
#include "HatScheT/utility/subgraphs/KosarajuSCC.h"

namespace HatScheT {

  SMTSCCScheduler::SMTSCCScheduler(Graph &g, ResourceModel &resourceModel) : SchedulerBase(g, resourceModel) {

      this->timeouts = 0;
      startTimes.clear();
      scheduleFound = false;
      computeMinII(&g, &resourceModel);
      minII = ceil(minII);
      computeMaxII(&g, &resourceModel);

  }

  void SMTSCCScheduler::schedule() {

      computeSCCs();

      cout << "Trivial: " << endl;
      for (auto &it : trivialSCCs){
          cout << it->getId() << ": " << it->getNumberOfVertices() << endl;
      }

      cout << "Basic: " << endl;
      for (auto &it : basicSCCs){
          cout << it->getId() << ": " << it->getNumberOfVertices() << endl;
      }

      cout << "Complex: " << endl;
      for (auto &it : complexSCCs){
          cout << it->getId() << ": " << it->getNumberOfVertices() << endl;
      }
  }

  void SMTSCCScheduler::computeSCCs() {

      KosarajuSCC kscc(g);
      auto sccs = kscc.getSCCs();

      for (auto &it : sccs) {
          if (it->getSccType(&resourceModel) == trivial) {
              trivialSCCs.insert(it);
          }
          if (it->getSccType(&resourceModel) == basic) {
              basicSCCs.insert(it);
          }
          if (it->getSccType(&resourceModel) == complex) {
              complexSCCs.insert(it);
          }
      }
  }

}