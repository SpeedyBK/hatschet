//
// Created by bkessler on 8/11/22.
//

#include <cmath>
#include <algorithm>
#include <z3++.h>

#include "SMTSCCScheduler.h"
#include "HatScheT/scheduler/ASAPScheduler.h"
#include "HatScheT/utility/subgraphs/KosarajuSCC.h"
#include "HatScheT/utility/Utility.h"
#include "SMTBinaryScheduler.h"

namespace HatScheT {

  SMTSCCScheduler::SMTSCCScheduler(Graph &g, ResourceModel &resourceModel) : SchedulerBase(g, resourceModel) {
      this->timeouts = 0;
      startTimes.clear();
      scheduleFound = false;
      computeMinII(&g, &resourceModel);
      minII = ceil(minII);
      computeMaxII(&g, &resourceModel);
      II = minII;
  }

  void SMTSCCScheduler::schedule() {

      modifyResourceModel();

      // Get SCCs and sort them:
      computeSCCs();

      // Compute relative Schedules:
      deque<SCC*> complexSCCs;
      for (auto &sc : topoSortedSccs){
          if(sc.first->getSccType(&resourceModel) == unknown){
              cout << sc.second << ": SCC_" << sc.first->getId() << " - " << "Unknown" << endl;
              throw(HatScheT::Exception("SMTSCCScheduler::schedule() : SCC-Type not determined!"));
          }
          if (sc.first->getSccType(&resourceModel) == trivial){
              cout << sc.second << ": SCC_" << sc.first->getId() << " - " << "Trivial" << endl;
          }
          if (sc.first->getSccType(&resourceModel) == basic){
              cout << sc.second << ": SCC_" << sc.first->getId() << " - " << "Basic" << endl;
              auto relSched = computeBasicSchedules(sc.first);
              basicRelSchedules.emplace_back(sc.first, relSched);
          }
          if (sc.first->getSccType(&resourceModel) == complex){
              cout << sc.second << ": SCC_" << sc.first->getId() << " - " << "Complex" << endl;
              complexSCCs.push_back(sc.first);
          }
      }
      if (!complexSCCs.empty()) {
          complexRelSchedule = computeComplexSchedule(complexSCCs);
      }
      // Combining relative Schedules:
      combineRelScheds();

      cout << "Schedule:" << endl;
      for (auto &it : startTimes){
          cout << it.first->getName() << ": " << it.second << endl;
      }

      resetResourceModel();
  }

  void SMTSCCScheduler::computeSCCs() {
      KosarajuSCC kscc(g);
      auto sccs = kscc.getSCCs();
      for (auto &sc : sccs){
          sc->getSccType(&resourceModel);
          for (auto &v_in_SCC : sc->getVerticesOfSCC()){
              vertexToSCC[v_in_SCC] = sc;
          }
      }
      inversePriority = computeTopologicalSCCOrder(sccs);

      for (auto &ip : inversePriority){
          topoSortedSccs.insert(ip);
      }
  }

  void SMTSCCScheduler::modifyResourceModel() {
      for (auto &r : resourceModel.Resources()){
          resourceLimits[r]=r->getLimit();
          if (resourceModel.getNumVerticesRegisteredToResource(r) <= r->getLimit()){
              r->setLimit(UNLIMITED);
          }
      }
  }

  void SMTSCCScheduler::resetResourceModel() {
      for (auto &r : resourceLimits){
          r.first->setLimit(resourceLimits.at(r.first));
      }
  }

  map<Vertex*, int> SMTSCCScheduler::computeBasicSchedules(SCC *sc) {
      // Creating maps
      map<Vertex*, Vertex*> sccVertexToVertex;
      map<Vertex*, Vertex*> vertexToSccVertex;
      // Generate Graph and ResourceModel
      auto gr = std::make_shared<Graph>();
      auto rm = std::make_shared<ResourceModel>();
      // Inset Vertices
      auto vertices = sc->getVerticesOfSCC();
      for (auto &v : vertices){
          auto &newV = gr->createVertex(v->getId());
          sccVertexToVertex[&newV] = v;
          vertexToSccVertex[v] = &newV;
      }
      // Create Resources
      for (auto &v : gr->Vertices()){
          auto res = resourceModel.getResource(sccVertexToVertex.at(v));
          Resource* newRes;
          try {
              newRes = rm->getResource(res->getName());
          }catch (HatScheT::Exception&){
              newRes = &rm->makeResource(res->getName(), res->getLimit(), res->getLatency(), res->getBlockingTime());
          }
          rm->registerVertex(v, newRes);
      }
      // Create Edges
      auto edges = sc->getSCCEdges();
      for (auto &e : edges){
          auto &newSrc = vertexToSccVertex.at(&e->getVertexSrc());
          auto &newDst = vertexToSccVertex.at(&e->getVertexDst());
          auto &newE = gr->createEdge(*newSrc, *newDst, e->getDistance(), e->getDependencyType());
          newE.setDelay(e->getDelay());
      }

      return sdcSchedule(gr, rm, sccVertexToVertex);

  }

  map<Vertex *, int>
  SMTSCCScheduler::sdcSchedule(std::shared_ptr<Graph>&gr, std::shared_ptr<ResourceModel>&rm, map<Vertex*, Vertex*>&sccVertexToVertex) {
      map<Vertex*, int> relSched;
      map<Vertex*, int> relSchedTemp;
      relSchedTemp = Utility::getSDCAsapAndAlapTimes(&(*gr), &(*rm), this->II).first;
      for (auto &vtPair : relSchedTemp){
          relSched[sccVertexToVertex.at(vtPair.first)] = vtPair.second;
      }
      return relSched;
  }

  map<Vertex *, int> SMTSCCScheduler::computeComplexSchedule(deque<SCC*> &complexSCCs) {
      // Creating maps
      map<Vertex*, Vertex*> sccVertexToVertex;
      map<Vertex*, Vertex*> vertexToSccVertex;
      // Generate Graph and ResourceModel
      auto gr = std::make_shared<Graph>();
      auto rm = std::make_shared<ResourceModel>();
      // Inset Vertices
      for (auto &sc : complexSCCs){
          for (auto &v : sc->getVerticesOfSCC()) {
              auto &newV = gr->createVertex(v->getId());
              sccVertexToVertex[&newV] = v;
              vertexToSccVertex[v] = &newV;
          }
      }
      // Create Resources
      for (auto &v : gr->Vertices()){
          auto res = resourceModel.getResource(sccVertexToVertex.at(v));
          Resource* newRes;
          try {
              newRes = rm->getResource(res->getName());
          }catch (HatScheT::Exception&){
              newRes = &rm->makeResource(res->getName(), res->getLimit(), res->getLatency(), res->getBlockingTime());
          }
          rm->registerVertex(v, newRes);
      }
      // Create Edges
      for (auto &sc : complexSCCs) {
          auto edges = sc->getSCCEdges();
          for (auto &e : edges) {
              auto &newSrc = vertexToSccVertex.at(&e->getVertexSrc());
              auto &newDst = vertexToSccVertex.at(&e->getVertexDst());
              auto &newE = gr->createEdge(*newSrc, *newDst, e->getDistance(), e->getDependencyType());
              newE.setDelay(e->getDelay());
          }
      }
      return smtSchedule(gr, rm, sccVertexToVertex);
  }

  map<Vertex *, int> SMTSCCScheduler::smtSchedule(shared_ptr<Graph> &gr, shared_ptr<ResourceModel> &rm,
                                                  map<Vertex *, Vertex *> &sccVertexToVertex) {
      map<Vertex*, int> relSched;
      map<Vertex*, int> relSchedTemp;
      SMTBinaryScheduler smt(*gr, *rm, this->II);
      smt.schedule();
      relSchedTemp = smt.getSchedule();
      for (auto &vtPair : relSchedTemp){
          relSched[sccVertexToVertex.at(vtPair.first)] = vtPair.second;
      }
      return relSched;
  }

  SMTSCCScheduler::~SMTSCCScheduler() {
      for (auto &it : topoSortedSccs){
          delete it.first;
      }
  }

  map<SCC *, int> SMTSCCScheduler::computeTopologicalSCCOrder(vector<SCC*>&tempsccs) {
      // Creating maps
      map<SCC*, Vertex*> sccToVertex;
      map<Vertex*, SCC*> locVertexToScc;
      // Generate Graph and ResourceModel
      auto gr = std::make_shared<Graph>();
      auto rm = std::make_shared<ResourceModel>();
      // Create a Dummyresource
      auto &dummyRes = rm->makeResource("SCC_DUMMY", UNLIMITED, 1);
      // Insert SCCs
      for (auto &sc : tempsccs){
          auto &sccV = gr->createVertex(sc->getId());
          sccToVertex[sc] = &sccV;
          locVertexToScc[&sccV] = sc;
          rm->registerVertex(&sccV, &dummyRes);
      }
      // create edges
      for (auto &e : this->g.Edges()) {
          // check if e is inside one of the sccs
          bool isInsideSCC = false;
          for (auto &scc : tempsccs) {
              auto sccEdges = scc->getSCCEdges();
              isInsideSCC = std::find(sccEdges.begin(), sccEdges.end(), e) != sccEdges.end();
              if (isInsideSCC) {
                  break;
              }
          }
          if (isInsideSCC){
              continue;
          }
          //Saving Edge for later.
          connectingEdges.insert(e);
          // If not inside a SCC, create edge.
          // Find Source SCC:
          auto vSrc = sccToVertex.at(vertexToSCC.at(&e->getVertexSrc()));
          // Find Destination SCC:
          auto vDst = sccToVertex.at(vertexToSCC.at(&e->getVertexDst()));
          // Add Edge:
          gr->createEdge(*vSrc, *vDst, 0);
      }

      // Topological Sort with Asap Schedule:
      ASAPScheduler asap (*gr, *rm);
      map<SCC*, int> inverseSccPriority;
      asap.schedule();
      for (auto &vtPair: asap.getSchedule()){
          inverseSccPriority[locVertexToScc.at(vtPair.first)] = vtPair.second;
      }

      return inverseSccPriority;
  }

  void SMTSCCScheduler::combineRelScheds() {
      map<SCC*, bool> IsSccChecked;
      map<pair<Resource*, int>, int> usedFUsInModSlot;
      for (auto &r : resourceModel.Resources()){
          for (int i = 0; i < II; i++){
              usedFUsInModSlot[{r, i}] = 0;
          }
      }
      //Going through topo sorted SCCs, and deciding based on sccType, what to do:
      for (auto &sc : topoSortedSccs){
          auto sccType = sc.first->getSccType(nullptr);
          if (sccType == unknown) {
              // Should never happen, throw Error.
              cout << "SCC_" << sc.first->getId() << " is Unknown!" << endl;
              throw (HatScheT::Exception("SMTSCCScheduler::combineRelScheds() SCC of Type 'Uknown' found!"));
          }
          if (sccType == trivial) {
              // Trivial SCCs only have one Vertex.
              if (sc.first->getVerticesOfSCC().size() != 1) {
                  cout << "SCC_" << sc.first->getId() << endl;
                  for (auto &v : sc.first->getVerticesOfSCC()) {
                      cout << v->getName() << endl;
                  }
                  throw (HatScheT::Exception(
                      "SMTSCCScheduler::combineRelScheds() Trivial SCC with now exactly 1 Vertex!"));
              }
              bool foundSlot = false;
              auto cvp = (const Vertex *) sc.first->getVerticesOfSCC().back();
              auto rp = (Resource *) resourceModel.getResource(cvp);
              int i = 0;
              if (sc.second == 0) {
                  while (!foundSlot) {
                      if (usedFUsInModSlot.at({rp, i % (int)II}) < rp->getLimit()) {
                          cout << "Hallo" << endl;
                          startTimes[(Vertex *) cvp] = i;
                          usedFUsInModSlot.at({rp, i % (int)II})++;
                          cout << i << endl;
                          foundSlot = true;
                      }
                  }
                  i++;
              }
          }
          if (sccType == basic) {

          }
          if (sccType == complex) {

          }
      }
  }
}