//
// Created by bkessler on 23/07/19.
//

#include "DaiZhang19Scheduler.h"

#include <cmath>
#include "HatScheT/utility/subgraphs/SCC.h"
#include "HatScheT/utility/subgraphs/KosarajuSCC.h"
#include "HatScheT/scheduler/ilpbased/MoovacScheduler.h"
#include "HatScheT/scheduler/ilpbased/ModuloSDCScheduler.h"
#include "HatScheT/scheduler/dev/ModSDC.h"
#include "HatScheT/scheduler/ilpbased/EichenbergerDavidson97Scheduler.h"
#include "HatScheT/utility/writer/DotWriter.h"
#include "HatScheT/utility/Utility.h"

namespace HatScheT {

  DaiZhang19Scheduler::DaiZhang19Scheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string> solverWishlist) : SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist) {
    II = -1;
    this->timeouts = 0;
    startTimes.clear();
    scheduleFound = false;
    optimalResult = true;
    computeMinII(&g, &resourceModel);
    minII = ceil(minII);
    computeMaxII(&g, &resourceModel);
    this->solverWishlist = solverWishlist;
  }


  void DaiZhang19Scheduler::schedule() {
    cout << endl << "DaiZhang19Scheduler::schedule: start" << endl;

    if (Utility::iscyclic(&g)) {
      cout << "Graph is cyclic..." << endl << endl;

      cout << "-----------------------------------------------------------------------------------------------" << endl;
      cout << "Finding the SCCs in Graph..." << endl;
      cout << "-----------------------------------------------------------------------------------------------" << endl;
      KosarajuSCC kosaraju(this->g);
      sccs = kosaraju.getSCCs();

      cout << endl;
      cout << "-----------------------------------------------------------------------------------------------" << endl;
      cout << "Determine type of SCCs..." << endl;
      cout << "-----------------------------------------------------------------------------------------------" << endl;
      for (auto &it : sccs) {
        it->setSCCType(determineType(it));
        cout << it->getName() << "Type is: " << it->getSccType()
             << " (0 = unknown, 1 = Basic, 2 = Complex, 3 = Trivial)" << endl;
      }

      cout << endl;
      cout << "-----------------------------------------------------------------------------------------------" << endl;
      cout << "Sorting the SCCs by type..." << endl;
      cout << "-----------------------------------------------------------------------------------------------" << endl;
      for (auto &it : sccs) {
        sortSCCs(it);
      }

      cout << endl << "Basic SCCs:" << endl;

      for (auto &it : basicSCCs) {
        cout << it->getName() + to_string(it->getId()) << endl;
        auto v = it->getVerticesOfSCC();
        for (auto &itr : v) {
          cout << itr->getName() << endl;
        }
      }

      cout << endl << "Complex SCCs:" << endl;
      for (auto &it : complexSCCs) {
        cout << it->getName() + to_string(it->getId()) << endl;
        auto v = it->getVerticesOfSCC();
        for (auto &itr : v) {
          cout << itr->getName() << endl;
        }
      }

      cout << endl << "Trivial SCCs:" << endl;
      for (auto &it : trivialSCCs) {
        cout << it->getName() + to_string(it->getId()) << endl;
        auto v = it->getVerticesOfSCC();
        for (auto &itr : v) {
          cout << itr->getName() << endl;
        }
      }

      cout << endl;
      cout << "-----------------------------------------------------------------------------------------------" << endl;
      cout << "Searching connected SCCs ..." << endl;
      cout << "-----------------------------------------------------------------------------------------------" << endl;
      for (auto &it : sccs) {
        findConnectedSCCs(it);
      }

      for (auto &it : sccs) {
        cout << endl << it->getName() + to_string(it->getId()) << endl;
        for (auto &sIt : it->getConnectedSCCs()) {
          cout << sIt->getId() << " ";
        }
      }

      cout << endl;
      cout << "-----------------------------------------------------------------------------------------------" << endl;
      cout << "Searching max. independent sets ..." << endl;
      cout << "-----------------------------------------------------------------------------------------------" << endl;

      cout << endl << "Basic max independent Sets:" << endl;
      findMaximalIndependentSet(basicSCCs, basic);
      cout << "Size of basicSupergraphSCCs is " << basicSupergraphSCCs.size() << endl;

      for (auto &it : basicSupergraphSCCs) {
        for (auto &iTem : it) {
          cout << iTem->getName() + to_string(iTem->getId()) << " ";
        }
        cout << endl;
      }

      cout << endl << "Complex max independent Sets:" << endl;
      findMaximalIndependentSet(complexSCCs, complex);
      cout << "Size of complexSupergraphSCCs is " << complexSupergraphSCCs.size() << endl;

      for (auto &it : complexSupergraphSCCs) {
        for (auto &iTem : it) {
          cout << iTem->getName() + to_string(iTem->getId()) << " ";
        }
        cout << endl;
      }

      cout << endl;
      cout << "-----------------------------------------------------------------------------------------------" << endl;
      cout << "Edges inside the SCCs ..." << endl;
      cout << "-----------------------------------------------------------------------------------------------" << endl;

      for (auto &sIt : sccs) {
        cout << sIt->getName() + to_string(sIt->getId()) << ": " << endl;
        for (auto &eIt : sIt->getSCCEdges()) {
          cout << eIt->getId() << ": " << eIt->getVertexSrcName() << " -- " << eIt->getVertexDstName() << endl;
        }
        cout << endl;
      }


      cout << endl;
      cout << "-----------------------------------------------------------------------------------------------" << endl;
      cout << "Building SuperGraphs ..." << endl;
      cout << "-----------------------------------------------------------------------------------------------" << endl;

      for (auto &it:basicSupergraphSCCs) {
        buildSuperGraph(it, basic);
      }

      for (auto &it:complexSupergraphSCCs) {
        buildSuperGraph(it, complex);
      }

    }
    else{
      cout << "Graph is acyclic, passing it directly to scheduler!" << endl << endl;
      //ToDo call scheduler
      EichenbergerDavidson97Scheduler ed(g, resourceModel, solverWishlist);
      ed.schedule();
      auto sched = ed.getSchedule();
      for (auto &it : sched){
        cout << it.first->getName() << " - " << it.second << endl;
      }
    }

    cout << endl << "DaiZhang19Scheduler::schedule: done!" << endl;
  }


  scctype DaiZhang19Scheduler::determineType(HatScheT::SCC *scc){

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

  void DaiZhang19Scheduler::findMaximalIndependentSet(vector<SCC*> SCCvec, scctype sT) {

    if (!SCCvec.empty()) {
      vector<SCC *> superGraph;

      //Marking all SCCs as not checked.
      map<SCC *, bool> checked;
      for (auto &it : SCCvec) {
        checked.insert(make_pair(it, false));
        //cout << it->getId() << " - " << checked[it] << endl;
      }

      //Finding connected stuff.
      for (auto &it: SCCvec) {
        if (!checked[it]) {
          for (auto &connections : it->getConnectedSCCs()) {
            for (auto &itr : SCCvec) {
              if (connections->getId() == itr->getId()) {
                checked[itr] = true;
              }
            }
          }
        }
      }

      SCCvec.clear();

      //Putting unconnected stuff in a Vector
      for (auto &it : checked) {
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

  void DaiZhang19Scheduler::buildSuperGraph(vector<SCC*> SCCvec, scctype sT) {

    Graph h;
    std::map<Vertex*,Vertex*> m;
    ResourceModel rmTemp;

    // create resources
    for(auto it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); ++it) {
      auto rIt = *it;
      rmTemp.makeResource(rIt->getName(),rIt->getLimit(),rIt->getLatency(),rIt->getBlockingTime());
    }

    //Creting Verticies
    //(gVertex : hVertex).
    for (auto &sccIt : SCCvec) {
      for (auto &VSCC:sccIt->getVerticesOfSCC()) {
        m[VSCC] = &h.createVertex(VSCC->getId());
        if (sT == basic) {
          rmTemp.registerVertex(m[VSCC], rmTemp.getResource(resourceModel.getResource(VSCC)->getName()));
        }else{
          rmTemp.registerVertex(m[VSCC], rmTemp.getResource(resourceModel.getResource(VSCC)->getName()));
        }
      }
    }

    //Creating the edges
    for (auto &sccIt : SCCvec) {
      for (auto &E:sccIt->getSCCEdges()) {
        h.createEdge(*m[&E->getVertexDst()], *m[&E->getVertexSrc()], E->getDistance(), E->getDependencyType());
      }
    }

    //Displaying Stuff
    cout << "Writing Graph..."<< endl;
    cout << "Vertices..."<< endl;
    for (auto &vIt : h.Vertices()) {
      cout << vIt->getName() << " Resource: " << rmTemp.getResource(vIt)->getName() << endl;
    }
    cout << "Edges..."<< endl;
    for (auto &eIt : h.Edges()){
      cout << eIt->getId() << ": " << eIt->getVertexSrcName() << " -- " << eIt->getVertexDstName() << " -- " << eIt->getDistance() << endl;
    }

    //Writing a Graphml ToDo: Remove and remove folder as well.
    string path = "SuperGraphml/Supergraph"+to_string(SCCvec[0]->getId())+".graphml";
    DotWriter dotW (path , &h, &rmTemp);
    dotW.write();

    cout << endl;
    cout << "-----------------------------------------------------------------------------------------------" << endl;
    cout << "Computing relative schedule ..." << endl;
    cout << "-----------------------------------------------------------------------------------------------" << endl;

    std::map <Vertex*, int> relativSchedule;
    relativSchedule = computeRelativeSchedule(h, rmTemp, sT);

    for (auto &it : relativSchedule){
      cout << it.first->getName() << " - " << it.second << endl;
    }

    //Mapping the relative schedules
    if (sT == basic){
      auto funnyThing = make_pair(SCCvec, relativSchedule);
      scheduledBasicSupergraphSCCs.push_back(funnyThing);
    }else if (sT == complex){
      auto funnyThing = make_pair(SCCvec, relativSchedule);
      scheduledComplexSupergraphSCCs.push_back(funnyThing);
    }

    cout << endl;
    cout << "-----------------------------------------------------------------------------------------------" << endl;
    cout << "Computing relative schedule DONE ..." << endl;
    cout << "-----------------------------------------------------------------------------------------------" << endl;

  }


  std::map<Vertex*, int> DaiZhang19Scheduler::computeRelativeSchedule(Graph &g, ResourceModel &rm, scctype sT) {

    std::map<Vertex*, int> relativSchedule;

    if (sT == basic){
      EichenbergerDavidson97Scheduler mSDC(g,rm,solverWishlist);
      mSDC.setSolverTimeout(300);
      mSDC.schedule();
      relativSchedule = mSDC.getSchedule();
    }
    else if (sT == complex){
      //ToDo: Probably replace with a SAT-Based approach.
      EichenbergerDavidson97Scheduler mSDCTwo(g,rm,solverWishlist);
      mSDCTwo.setSolverTimeout(300);
      mSDCTwo.schedule();
      relativSchedule = mSDCTwo.getSchedule();
    }
    return relativSchedule;
  }
}