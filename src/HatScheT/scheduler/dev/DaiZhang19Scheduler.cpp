//
// Created by bkessler on 23/07/19.
//

#include "DaiZhang19Scheduler.h"

#include <cmath>
#include "HatScheT/utility/subgraphs/SCC.h"
#include "HatScheT/utility/subgraphs/KosarajuSCC.h"
#include "HatScheT/scheduler/ilpbased/ModuloSDCScheduler.h"

namespace HatScheT {

  DaiZhang19Scheduler::DaiZhang19Scheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string> solverWishlist)
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


  void DaiZhang19Scheduler::schedule() {
    cout << endl << "DaiZhang19Scheduler::schedule: start" << endl;



    cout << endl << "DaiZhang19Scheduler::schedule: done!" << endl;

  }

  scctype DaiZhang19Scheduler::determineType(SCC *scc) {



    return basic;
  }


  int DaiZhang19Scheduler::getSccIdbyVertex(Vertex *v) {



  }

  void DaiZhang19Scheduler::sortSCCs(SCC *scc) {

    if (scc->getSccType() == basic) {
      basicSCCs.push_back(scc);
    } else if (scc->getSccType() == complex) {
      complexSCCs.push_back(scc);
    } else if (scc->getSccType() == trivial) {
      trivialSCCs.push_back(scc);
    } else {
      throw HatScheT::Exception("DaiZhang19Scheduler.sortSCCs: SCC with of type unknown, set SccType first.");
    }
  }

  void DaiZhang19Scheduler::findMaximalIndependentSet(vector<SCC *> SCCvec, scctype sT) {

    if (!SCCvec.empty()) {
      vector<SCC *> superGraph;

      //Marking all SCCs as not checked.
      map<SCC *, bool> checked;
      for (auto it : SCCvec) {
        checked.insert(make_pair(it, false));
        //cout << it->getId() << " - " << checked[it] << endl;
      }

      //Finding connected stuff.
      for (auto it: SCCvec) {
        if (!checked[it]) {
          for (auto &connections : it->getConnections()) {
            for (auto &itr : SCCvec) {
              if (connections == itr->getId()) {
                checked[itr] = true;
              }
            }
          }
        }
      }

      SCCvec.clear();

      //Putting unconnected stuff in a Vector
      for (auto it : checked) {
        if (!it.second) {
          //cout << it.first->getId() << " - " << it.second << endl;
          superGraph.push_back(it.first);
        } else {
          SCCvec.push_back(it.first);
        }
      }

      if (sT == basic) {
        basicSupergraphSCCs.push_back(superGraph);
      } else if (sT == complex) {
        complexSupergraphSCCs.push_back(superGraph);
      }

      findMaximalIndependentSet(SCCvec, sT);

    }
  }

  std::pair<Graph*, map<Vertex*, Vertex*>> DaiZhang19Scheduler::buildSupergraph(vector <SCC*> superGraph) {

  }

}


