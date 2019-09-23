//
// Created by bkessler on 23/07/19.
//

#include "DaiZhang19Scheduler.h"

#include <cmath>
#include "HatScheT/utility/subgraphs/SCC.h"
#include "HatScheT/utility/subgraphs/KosarajuSCC.h"
#include "HatScheT/scheduler/ilpbased/ModuloSDCScheduler.h"

namespace HatScheT {

  DaiZhang19Scheduler::DaiZhang19Scheduler(Graph &g, ResourceModel &resourceModel,
                                           std::list<std::string> solverWishlist)
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

    cout << "-----------------------------------------------------------------------------------------------" << endl;
    cout << "Finding the SCCs in Graph..." << endl;
    cout << "-----------------------------------------------------------------------------------------------" << endl;
    KosarajuSCC kosaraju(this -> g);
    sccs = kosaraju.getSCCs();

    cout << endl;
    cout << "-----------------------------------------------------------------------------------------------" << endl;
    cout << "Determine type of SCCs..." << endl;
    cout << "-----------------------------------------------------------------------------------------------" << endl;
    for (auto &it : sccs){
      it->setSCCType(determineType(it));
      cout << it->getName() << "Type is: " << it->getSccType() << " (0 = unknown, 1 = Basic, 2 = Complex, 3 = Trivial)" << endl;
    }

    cout << endl;
    cout << "-----------------------------------------------------------------------------------------------" << endl;
    cout << "Sorting the SCCs by type..." << endl;
    cout << "-----------------------------------------------------------------------------------------------" << endl;
    for (auto &it : sccs){
      sortSCCs(it);
    }

    cout << endl << "Basic SCCs:" << endl;

    for (auto &it : basicSCCs){
      cout << it->getName() + to_string(it->getId()) << endl;
      auto v = it->getVerticesOfSCC();
      for (auto &itr : v){
        cout << itr->getName() << endl;
      }
    }

    cout << endl << "Complex SCCs:" << endl;
    for (auto &it : complexSCCs){
      cout << it->getName() + to_string(it->getId()) << endl;
      auto v = it->getVerticesOfSCC();
      for (auto &itr : v){
        cout << itr->getName() << endl;
      }
    }

    cout << endl << "Trivial SCCs:" << endl;
    for (auto &it : trivialSCCs){
      cout << it->getName() + to_string(it->getId()) << endl;
      auto v = it->getVerticesOfSCC();
      for (auto &itr : v){
        cout << itr->getName() << endl;
      }
    }

    cout << endl;
    cout << "-----------------------------------------------------------------------------------------------" << endl;
    cout << "Searching connected SCCs ..." << endl;
    cout << "-----------------------------------------------------------------------------------------------" << endl;
    for (auto &it : sccs){
      findConnectedSCCs(it);
    }

    for (auto &it : sccs){
      cout << endl << it->getName() + to_string(it->getId()) << endl;
      for (auto &sIt : it->getConnectedSCCs()){
        cout << sIt->getId() << " ";
      }
    }

    cout << endl;
    cout << "-----------------------------------------------------------------------------------------------" << endl;
    cout << "Searching max. independent sets ..." << endl;
    cout << "-----------------------------------------------------------------------------------------------" << endl;

    cout << endl << "Basic Component:" << endl;
    //findMaximalIndependentSet(basicSCCs, basic);

    cout << endl << "Complex Component:" << endl;
    //findMaximalIndependentSet(complexSCCs, complex);

    cout << endl;
    cout << "-----------------------------------------------------------------------------------------------" << endl;
    cout << "Building SuperGraphs ..." << endl;
    cout << "-----------------------------------------------------------------------------------------------" << endl;


    cout << endl << "DaiZhang19Scheduler::schedule: done!" << endl;
  }


  scctype DaiZhang19Scheduler::determineType(SCC *scc) {

    if (scc->getNumberOfVertices() == 1) {
      return trivial;
    }

    list<Vertex*> VerticesOfSCC = scc->getVerticesOfSCC();

    for (auto &it:VerticesOfSCC) {
      if (resourceModel.getResource(it)->getLimit() != -1) {
        return complex;
      }
    }

    return basic;
  }


  int DaiZhang19Scheduler::getSccIdbyVertex(Vertex *v) {

    for (auto &it : sccs) {
      map <Vertex*, bool> vMap = it->getVertexInSccMap();
      for (auto &mapElements : vMap){
        if (mapElements.first->getId() == v->getId()){
          if (mapElements.second){
            return it->getId();
          }
        }
      }
    }

    throw HatScheT::Exception("DaiZhang19Scheduler.getSccIdbyVertex: Vertex is not in any SCC.");
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

  void DaiZhang19Scheduler::findConnectedSCCs(SCC *scc) {

    list <Vertex*> conVertices;
    set <Vertex*> PreSuccessors;
    list <SCC*> conSCCs;

    for (auto &it : scc->getVerticesOfSCC()){
      PreSuccessors = g.getPredecessors(it);
      for (auto &psIt : PreSuccessors){
        conVertices.push_back(psIt);
      }
      PreSuccessors = g.getSuccessors(it);
      for (auto &psIt : PreSuccessors){
        conVertices.push_back(psIt);
      }
    }

    conVertices.sort();
    conVertices.unique();

    for(auto &sVIt : scc->getVerticesOfSCC()){
      conVertices.remove(sVIt);
    }

    for (auto &it : sccs){
      for (auto &vIt : it->getVerticesOfSCC()){
        for (auto &cVIt : conVertices){
          if (vIt->getId() == cVIt->getId()){
            conSCCs.push_back(it);
          }
        }
      }
    }

    scc->setConnectedSCCs(conSCCs);

  }

}