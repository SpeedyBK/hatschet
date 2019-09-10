//
// Created by bkessler on 23/07/19.
//

#include "DaiZhang19Scheduler.h"

#include <cmath>
#include "HatScheT/utility/subgraphs/SCC.h"
#include "HatScheT/utility/subgraphs/KosarajuSCC.h"

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

    cout << endl << "Starting Kosajaru's SCC Algorithm..." << endl;
    KosarajuSCC kscc(this->g);
    this->sccs = kscc.getSCCs();
    cout << endl << "Finished Kosajaru's SCC Algorithm." << endl;

    cout << endl << "Setting SCC-Types" << endl;

    for (auto it : sccs) {
      it->setSCCType(determineType(it));
      //Debug
      cout << endl << "SCC " << it->getId() << " is Type (0 = unknown, 1 = Basic, 2 = Complex, 3 = Trivial): "
           << it->getSccType() << endl;
    }

    cout << endl << "Done setting Types." << endl;

    cout << endl << "Creating edges in SCCs" << endl;

    for (auto it:g.Edges()) {
      if (getSccIdbyVertex(&it->getVertexSrc()) == getSccIdbyVertex(&it->getVertexDst())) {
        auto vertexMapReverse = sccs[getSccIdbyVertex(&it->getVertexSrc())]->getVertexMapReverse();
        sccs[getSccIdbyVertex(&it->getVertexSrc())]->createEdge(*vertexMapReverse[&it->getVertexSrc()],
                                                                *vertexMapReverse[&it->getVertexDst()],
                                                                it->getDistance(), it->getDependencyType());
      } else {
        sccs[getSccIdbyVertex(&it->getVertexSrc())]->setConnections(getSccIdbyVertex(&it->getVertexDst()));
        sccs[getSccIdbyVertex(&it->getVertexDst())]->setConnections(getSccIdbyVertex(&it->getVertexSrc()));
      }
    }

    cout << endl << "Done creating edges." << endl;

    //Has to be removed:
    for (auto it:sccs) {
      cout << "SCC_" << it->getId() << endl;
      cout << "Has " << it->getNumberOfEdges() << " Edges" << endl;
      for (auto e:it->Edges()) {
        cout << e->getVertexSrcName() << " - " << e->getVertexDstName() << endl;
      }
      cout << "SCC_" << it->getId() << " is connected to the following SCCs:" << endl;
      auto connections = it->getConnections();
      for (auto itr:connections) {
        cout << itr << ", ";
      }
      cout << endl << endl;
    }

    cout << "Sorting SCCs by Type..." << endl;

    for (auto it:sccs) {
      sortSCCs(it);
    }

    cout << "Sorting Done..." << endl;

    cout << endl << "Basic: " << endl;
    for (auto it:basicSCCs) {
      cout << it->getId() << " ";
    }

    cout << endl << "Complex: " << endl;
    for (auto it:complexSCCs) {
      cout << it->getId() << " ";
    }

    cout << endl << "Trivial: " << endl;
    for (auto it:trivialSCCs) {
      cout << it->getId() << " ";
    }

    //ToDo: Has to be checked.
    cout << endl << "Building Basic Supergraphs.." << endl;
    if (!basicSCCs.empty()){
      findMaximalIndependentSet(basicSCCs, basic);
    }else{
      cout << "No basic SCCs, skipping.." << endl;
    }

    cout << endl << "Building Complex Supergraphs.." << endl;
    if (!complexSCCs.empty()){
      findMaximalIndependentSet(complexSCCs, complex);
    }else{
      cout << "No complex SCCs, skipping.." << endl;
    }
    //Todo: End!

    cout << "-------------------------------------------------------------------------" << endl;
    cout << "Iteration through Basic SuperGraphs" << endl;
    cout << "-------------------------------------------------------------------------" << endl;
    for (auto &it:basicSupergraphs){
      for (auto itr:it){
        cout << itr->getId() << " ";
      }
      cout << endl;
    }

    cout << "-------------------------------------------------------------------------" << endl;
    cout << "Iteration through Complex SuperGraphs" << endl;
    cout << "-------------------------------------------------------------------------" << endl;
    for (auto &it:complexSupergraphs){
      for (auto itr:it){
        cout << itr->getId() << " ";
      }
      cout << endl;
    }

    cout << endl << "DaiZhang19Scheduler::schedule: done!" << endl;

  }


  scctype DaiZhang19Scheduler::determineType(SCC *scc) {

    if (scc->getNumberOfVertices() == 1) {
      return trivial;
    }

    auto vertexMap = scc->getVertexMap();

    for (auto it:vertexMap) {
      if (resourceModel.getResource(it.second)->getLimit() != -1) {
        return complex;
      }
    }

    return basic;
  }


  int DaiZhang19Scheduler::getSccIdbyVertex(Vertex *v) {

    int sccID = 0;

    for (auto it:sccs) {
      auto vertexMap = it->getVertexMap();
      for (auto V:it->Vertices()) {
        if (vertexMap[V] == v) {
          sccID = it->getId();
        }
      }
    }

    return sccID;

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
        basicSupergraphs.push_back(superGraph);
      } else if (sT == complex) {
        complexSupergraphs.push_back(superGraph);
      }

      findMaximalIndependentSet(SCCvec, sT);

    }
  }

  std::pair<Graph*, map<Vertex*, Vertex*>> DaiZhang19Scheduler::buildSupergraph(vector <SCC*> superGraph) {

    for (auto &it : superGraph){

    }

    return pair<Graph *, map<Vertex *, Vertex *>>();
  }

}


