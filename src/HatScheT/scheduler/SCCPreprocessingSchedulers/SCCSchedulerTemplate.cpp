//
// Created by bkessler on 10/5/22.
//

#include "SCCSchedulerTemplate.h"

#include <sstream>

#ifdef USE_Z3

#include "HatScheT/scheduler/smtbased/SMTUnaryScheduler.h"
#include "HatScheT/scheduler/smtbased/SMTCDLScheduler.h"

#endif

#ifdef USE_SCALP

#include "HatScheT/scheduler/ilpbased/EichenbergerDavidson97Scheduler.h"
#include "HatScheT/scheduler/ilpbased/MoovacScheduler.h"
#include "HatScheT/scheduler/ilpbased/SuchaHanzalek11Scheduler.h"
#include "HatScheT/scheduler/ilpbased/ModuloSDCScheduler.h"

#endif

#ifdef USE_CADICAL

#include "HatScheT/scheduler/satbased/SATSchedulerBinEnc.h"
#include "HatScheT/scheduler/satbased/SATSchedulerRes.h"
#include "HatScheT/scheduler/dev/SDSScheduler.h"

#endif

#include <cmath>
#include <chrono>
#include <algorithm>
#include <HatScheT/scheduler/ALAPScheduler.h>

#include "HatScheT/utility/subgraphs/KosarajuSCC.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/Verifier.h"

namespace HatScheT {

  SCCSchedulerTemplate::SCCSchedulerTemplate(Graph &g, ResourceModel &resourceModel,
                                             SCCSchedulerTemplate::scheduler sccScheduler,
                                             SCCSchedulerTemplate::scheduler finalScheduler, double II)
      : IterativeModuloSchedulerLayer(g, resourceModel, II) {

      this->sccScheduler = sccScheduler;
      this->finalScheduler = finalScheduler;
      this->sccMode = sccExpandMode::automatic;

      this->threads = 1;

      for (auto &r : resourceModel.Resources()) {
          for (int i = 0; i < II; i++) {
              usedFUsInModSlot[{r, i}] = 0;
          }
      }

      for (auto &v : g.Vertices()) {
          this->startTimes[v] = 0;
      }

      complexGraph = std::make_shared<Graph>();
      complexRm = std::make_shared<ResourceModel>();
      numOfCmplxSCCs = 0;

  }

  void SCCSchedulerTemplate::scheduleInit() {

      string names[8] = {"MOOVAC", "ED97", "SMT", "SAT", "SH11", "MODSDC", "SMTCDL", "NONE"};
      if (!this->quiet) { cout << endl << "Scheduling with " << this->getName() << "!" << endl; }
      if (!this->quiet) { cout << "1st-Stage-Scheduler: " << names[(int) sccScheduler] << endl; }
      if (!this->quiet) { cout << "2nd-Stage-Scheduler: " << names[(int) finalScheduler] << endl << endl; }

      modifyResourceModel();

      computeSCCs();

      sortSCCsByType();

      buildComplexGraph();

      this->backboneSchedulerBoundSL = this->boundSL;
      this->boundSL = false;
  }

  void SCCSchedulerTemplate::scheduleCleanup() {
      this->resetResourceModel();

      this->boundSL = this->backboneSchedulerBoundSL;
  }

  void SCCSchedulerTemplate::scheduleIteration() {

#if defined(USE_Z3)
      if (!this->quiet) { cout << "Using Z3 ..." << endl;}
      if (this->sccMode != sccExpandMode::automatic) calcStarttimesPerComplexSCC();
#elif defined(USE_SCALP)
      if (!this->quiet) { cout << "Using ScaLP ..." << endl;}
      if (this->sccMode != sccExpandMode::automatic) calcStarttimesPerComplexSCCScaLP();
#else
      throw (HatScheT::Exception("Either Z3 or ScaLP is needed for this Scheduler"));
#endif

      if (!complexSCCs.empty()) {
          bigComplexSchedule = scheduleComplexGraph();
      }
      for (auto &r : resourceModel.Resources()) {
          for (int i = 0; i < this->II; i++) {
              usedFUsInModSlot[{r, i}] = 0;
          }
      }
      if (!quiet) { cout << "Complex Done!" << endl; }
      if (scheduleFound or complexSCCs.empty()) {
          finalizeComplexSchedule();
          if (!basicSCCs.empty()) {
              for (auto &sc : basicSCCs) {
                  auto relativSchedule = computeBasicSchedules(sc);
                  basicRelSchedules[sc] = relativSchedule;
              }
              if (!quiet) { cout << "Basic Done!" << endl; }
          }
          combineRelativeSchedules();
          if (!quiet) { cout << "Combining Done!" << endl; }
      }
      scheduleFound = verifyModuloSchedule(g, resourceModel, startTimes, (int) II);

      if (this->scheduleFound and finalScheduler != scheduler::NONE) {
          if (this->timeRemaining == -1) {
              this->timeRemaining = this->solverTimeout;
          }
          if (!this->quiet) { cout << "Computing final schedule..." << endl; }
          auto finalSchedulerSelected = selectSccScheduler(finalScheduler, g, resourceModel);
          if (finalSchedulerSelected->getName() == "SDSScheduler") {
              throw Exception(
                  "SCCSchedulerTemplate::scheduleIteration: SDS Scheduler does not support latency optimization, yet -> choose another scheduler");
          }
          finalSchedulerSelected->setQuiet(this->quiet);
          cout << "Time Remaining:" << this->timeRemaining << endl;
          finalSchedulerSelected->setSolverTimeout(this->timeRemaining);
          finalSchedulerSelected->disableSecObjective(false);
          finalSchedulerSelected->setMaxLatencyConstraint(this->getScheduleLength());
          finalSchedulerSelected->setBoundSL(this->backboneSchedulerBoundSL);
          sharedPointerCastAndSetup(finalSchedulerSelected);
          finalSchedulerSelected->schedule();
          if (finalSchedulerSelected->getScheduleFound()) {
              startTimes = finalSchedulerSelected->getSchedule();
              scheduleFound = verifyModuloSchedule(g, resourceModel, startTimes, (int) II);
              this->secondObjectiveOptimal = finalSchedulerSelected->getObjectivesOptimal().second;
          }
          this->timeRemaining = finalSchedulerSelected->getTimeRemaining();
          this->solvingTimePerIteration += finalSchedulerSelected->getSolvingTimePerIteration();
      }

  }

  void SCCSchedulerTemplate::modifyResourceModel() {
      for (auto &r : resourceModel.Resources()) {
          resourceLimits[r] = r->getLimit();
          if (resourceModel.getNumVerticesRegisteredToResource(r) <= r->getLimit()) {
              r->setLimit(UNLIMITED, false);
          }
      }
  }

  void SCCSchedulerTemplate::resetResourceModel() {
      for (auto &r : resourceLimits) {
          r.first->setLimit(resourceLimits.at(r.first), false);
      }
  }

  void SCCSchedulerTemplate::computeSCCs() {
      KosarajuSCC kscc(g);
      auto sccs = kscc.getSCCs();
      for (auto &sc : sccs) {
          sc->getSccType(&resourceModel);
          for (auto &v_in_SCC : sc->getVerticesOfSCC()) {
              vertexToSCC[v_in_SCC] = sc;
          }
      }
      inversePriority = computeTopologicalSCCOrder(sccs);

      for (auto &ip : inversePriority) {
          topoSortedSccs.insert(ip);
      }
  }

  map<SCC *, int> SCCSchedulerTemplate::computeTopologicalSCCOrder(vector<SCC *> &tempsccs) {
      // Creating maps
      map<SCC *, Vertex *> sccToVertex;
      map<Vertex *, SCC *> locVertexToScc;
      // Generate Graph and ResourceModel
      auto gr = std::make_shared<Graph>();
      auto rm = std::make_shared<ResourceModel>();
      // Create a Dummyresource
      auto &dummyRes = rm->makeResource("SCC_DUMMY", UNLIMITED, 1);
      // Insert SCCs
      for (auto &sc : tempsccs) {
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
          if (isInsideSCC) {
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

      // Topological Order with Asap Schedule:
      ALAPScheduler alap(*gr, *rm);
      map<SCC *, int> inverseSccPriority;
      alap.schedule();
      auto asapSched = alap.getSchedule();
      for (auto &vtPair: asapSched) {
          inverseSccPriority[locVertexToScc.at(vtPair.first)] = vtPair.second;
      }

      return inverseSccPriority;
  }

  void SCCSchedulerTemplate::sortSCCsByType() {

      struct sccComp {
        bool operator()(const SCC *a, const SCC *b) {
            return !(*a < *b);
        }
      };

      for (auto &sc : topoSortedSccs) {
          if (sc.first->getSccType() == unknown) {
              cout << sc.second << ": SCC_" << sc.first->getId() << " - " << "Unknown" << endl;
              throw (HatScheT::Exception("SMTSCCScheduler::schedule() : SCC-Type not determined!"));
          }
          if (sc.first->getSccType() == trivial) {
              cout << sc.second << ": SCC_" << sc.first->getId() << " - " << "Trivial - Vertices: "
                   << sc.first->getNumberOfVertices() << endl;
              continue;
          }
          if (sc.first->getSccType() == basic) {
              cout << sc.second << ": SCC_" << sc.first->getId() << " - " << "Basic - Vertices: "
                   << sc.first->getNumberOfVertices() << endl;
              basicSCCs.push_back(sc.first);
          }
          if (sc.first->getSccType() == complex) {
              cout << sc.second << ": SCC_" << sc.first->getId() << " - " << "Complex - Vertices: "
                   << sc.first->getNumberOfVertices() << endl;
              complexSCCs.push_back(sc.first);
          }
      }

      std::sort(complexSCCs.begin(), complexSCCs.end(), sccComp());
      std::sort(basicSCCs.begin(), basicSCCs.end(), sccComp());

      numOfCmplxSCCs = (int)complexSCCs.size();
  }

  void SCCSchedulerTemplate::buildComplexGraph() {

      // Creating maps and Generate Graph and ResourceModel and Inset Vertices
      for (auto &sc : complexSCCs) {
          for (auto &v : sc->getVerticesOfSCC()) {
              auto &newV = complexGraph->createVertex(v->getId());
              sccVertexToVertex[&newV] = v;
              vertexToSccVertex[v] = &newV;
          }
      }
      resetResourceModel(); // Needed because of strange exception in constructor of resource class.
      // Create Resources
      for (auto &v : complexGraph->Vertices()) {
          auto res = resourceModel.getResource(sccVertexToVertex.at(v));
          Resource *newRes;
          try {
              newRes = complexRm->getResource(res->getName());
          } catch (HatScheT::Exception &) {
              newRes = &complexRm->makeResource(res->getName(), res->getLimit(), res->getLatency(),
                                                res->getBlockingTime());
          }
          complexRm->registerVertex(v, newRes);
      }
      modifyResourceModel(); // Needed because of strange exception in constructor of resource class.
      // Create Edges
      for (auto &sc : complexSCCs) {
          auto edges = sc->getSCCEdges();
          for (auto &e : edges) {
              auto &newSrc = vertexToSccVertex.at(&e->getVertexSrc());
              auto &newDst = vertexToSccVertex.at(&e->getVertexDst());
              auto &newE = complexGraph->createEdge(*newSrc, *newDst, e->getDistance(), e->getDependencyType());
              newE.setDelay(e->getDelay());
          }
      }
  }

  void SCCSchedulerTemplate::setExpandMode(SCCSchedulerTemplate::sccExpandMode expandMode) {

      if (expandMode == sccExpandMode::fast) {
          cout << "'Fast-Mode'" << endl;
      } else if (expandMode == sccExpandMode::optimal) {
          cout << "'Optimal-Mode'" << endl;
      } else if (expandMode == sccExpandMode::automatic) {
          cout << "'Automatic-Mode'" << endl;
      }
      this->sccMode = expandMode;
  }

  map<Vertex *, int> SCCSchedulerTemplate::scheduleComplexGraph() {

      map<Vertex *, int> relSched;
      map<Vertex *, int> relSchedTemp;
      auto sccSchedulerSelected = selectSccScheduler(sccScheduler, *complexGraph, *complexRm);
      sccSchedulerSelected->setQuiet(this->quiet);
      sccSchedulerSelected->setSolverTimeout(this->getSolverTimeout());
      sccSchedulerSelected->disableSecObjective(true);
      if (this->sccMode == sccExpandMode::automatic) {
          sccSchedulerSelected->setBoundSL(this->backboneSchedulerBoundSL);
      }
      int maxSCCslat;
      if (sccMode == sccExpandMode::fast) {
          //maxSCCslat = std::max(expandSCC() + numOfCmplxSCCs-1, (int) II+1);
          maxSCCslat = *std::max_element(maxTimesSCC.begin(), maxTimesSCC.end());
          sccSchedulerSelected->setMaxLatencyConstraint(maxSCCslat);
      } else if (sccMode == sccExpandMode::optimal) {
          //maxSCCslat = expandSCC() + (int) II;
          maxSCCslat = *std::max_element(maxTimesSCC.begin(), maxTimesSCC.end()) + (int) II;
          sccSchedulerSelected->setMaxLatencyConstraint(maxSCCslat);
      } else {
          //Let Scheduler Search...
      }
      sccSchedulerSelected->schedule();
      if (!sccSchedulerSelected->getObjectivesOptimal().first) {
          this->firstObjectiveOptimal = false;
      }
      this->timeRemaining = sccSchedulerSelected->getTimeRemaining();
      this->solvingTimePerIteration = sccSchedulerSelected->getSolvingTimePerIteration();
      this->timeouts += sccSchedulerSelected->getTimeouts();
      if (!quiet) { cout << "Timeouts of ComplexSCC-Scheduler: " << timeouts << endl; }

      if (sccSchedulerSelected->getScheduleFound()) {

          /*if (sccScheduler == scheduler::SMT) {
              auto ptr = dynamic_pointer_cast<SMTUnaryScheduler>(sccSchedulerSelected);
              auto st = ptr->printVertexStarttimes();

              cout << "Earliest and Latest Starttimes:" << endl;
              for (auto &v : st){
                  cout << sccVertexToVertex.at(v.first)->getName() << ": " << v.second.first << " / " << v.second.second << endl;
              }
          }*/

          relSchedTemp = sccSchedulerSelected->getSchedule();
          scheduleFound = sccSchedulerSelected->getScheduleFound();
          this->II = sccSchedulerSelected->getII();
      }

      if (!quiet) {
          if (verifyModuloSchedule(*complexGraph, *complexRm, relSchedTemp, (int) this->II)) {
              if (!quiet) { cout << "Big Complex Relative Schedule is Valid!" << endl; }
          } else {
              if (!quiet) { cout << "Big Complex Relative Schedule is NOT!!!!!!!111111 Valid!" << endl; }
          }
      }

      for (auto &vtPair : relSchedTemp) {
          relSched[sccVertexToVertex.at(vtPair.first)] = vtPair.second;
      }

      return relSched;

  }

  shared_ptr<IterativeModuloSchedulerLayer>
  SCCSchedulerTemplate::selectSccScheduler(scheduler schedEnum, Graph &gr, ResourceModel &rm) {
      switch (schedEnum) {
          case scheduler::SMT: {
#if defined(USE_Z3)
              auto schedulePtr = std::make_shared<SMTUnaryScheduler>(gr, rm, this->II);
              if (!quiet) { cout << "Using " << schedulePtr->getName() << endl; }
              if (!quiet) schedulePtr->getDebugPrintouts();
              schedulePtr->setLatencySearchMethod(SMTUnaryScheduler::latSearchMethod::BINARY);
              schedulePtr->setSchedulePreference(SMTUnaryScheduler::schedulePreference::MOD_ASAP);
              if (this->threads > 1) { schedulePtr->setThreads(this->threads); }
              return schedulePtr;
#else
              throw (HatScheT::Exception("SCC-Template: SMT-Unary-Scheduler needs Z3... Z3 not linked"));
#endif
          }
          case scheduler::ED97: {
#if defined(USE_SCALP)
              auto ED = new EichenbergerDavidson97Scheduler(gr, rm, {"Gurobi", "CPLEX", "SCIP", "LPSolve"}, (int) this->II);
              shared_ptr<EichenbergerDavidson97Scheduler> schedulePtr(ED);
              if (!quiet) { cout << "Using " << schedulePtr->getName() << endl; }
              if (!quiet) schedulePtr->getDebugPrintouts();
              if (this->threads > 1) { schedulePtr->setThreads(this->threads); }
              return schedulePtr;
#else
              throw (HatScheT::Exception("SCC-Template: ED97-Scheduler needs ScaLP... ScaLP not linked"));
#endif
          }
          case scheduler::MOOVAC: {
#if defined(USE_SCALP)
              auto MOOVAC = new MoovacScheduler(gr, rm, {"Gurobi", "CPLEX", "SCIP", "LPSolve"}, (int) this->II);
              shared_ptr<MoovacScheduler> schedulePtr(MOOVAC);
              if (!quiet) { cout << "Using " << schedulePtr->getName() << endl; }
              if (!quiet) schedulePtr->getDebugPrintouts();
              if (this->threads > 1) { schedulePtr->setThreads(this->threads); }
              return schedulePtr;
#else
              throw (HatScheT::Exception("SCC-Template: MOOVAC-Scheduler needs ScaLP... ScaLP not linked"));
#endif
          }
          case scheduler::SH11: {
#if defined(USE_SCALP)
              auto SH = new SuchaHanzalek11Scheduler(gr, rm, {"Gurobi", "CPLEX", "SCIP", "LPSolve"}, (int) this->II);
              shared_ptr<SuchaHanzalek11Scheduler> schedulePtr(SH);
              if (!quiet) { cout << "Using " << schedulePtr->getName() << endl; }
              if (!quiet) schedulePtr->getDebugPrintouts();
              if (this->threads > 1) { schedulePtr->setThreads(this->threads); }
              return schedulePtr;
#else
              throw (HatScheT::Exception("SCC-Template: SH11-Scheduler needs ScaLP... ScaLP not linked"));
#endif
          }
          case scheduler::MODSDC: {
#if defined(USE_SCALP)
              auto MODSDC = new ModuloSDCScheduler(gr, rm, {"Gurobi", "CPLEX", "SCIP", "LPSolve"}, (int) this->II);
              shared_ptr<ModuloSDCScheduler> schedulePtr(MODSDC);
              if (!quiet) { cout << "Using " << schedulePtr->getName() << endl; }
              if (!quiet) schedulePtr->getDebugPrintouts();
              if (this->threads > 1) { schedulePtr->setThreads(this->threads); }
              return schedulePtr;
#else
              throw (HatScheT::Exception("SCC-Template: MODSDC-Scheduler needs ScaLP... ScaLP not linked"));
#endif
          }
          case scheduler::SMTCDL: {
#if defined(USE_Z3)
              auto SMTCDL = std::make_shared<SMTCDLScheduler>(gr, rm, (int) this->II);
              shared_ptr<SMTCDLScheduler> schedulePtr(SMTCDL);
              if (!quiet) { cout << "Using " << schedulePtr->getName() << endl; }
              if (this->threads > 1) { schedulePtr->setThreads(this->threads); }
              return schedulePtr;
#else
              throw (HatScheT::Exception("SCC-Template: SMT-CDL-Scheduler needs Z3... Z3 not linked"));
#endif
          }
          /*!
           * Stuff from Nico, will check that later... BLK
           */
#ifdef USE_CADICAL
          case scheduler::SAT: {
#if 0
              auto schedulePtr = std::make_shared<SATSchedulerBinEnc>(gr, rm, this->II);
              if (this->recMinII > 1.0) {
                  schedulePtr->setLatencyOptimizationStrategy(SATSchedulerBase::LatencyOptimizationStrategy::REVERSE_LINEAR);
              }
              else {
                  schedulePtr->setLatencyOptimizationStrategy(SATSchedulerBase::LatencyOptimizationStrategy::LINEAR_JUMP);
              }
#else
              auto schedulePtr = std::make_shared<SATSchedulerRes>(gr, rm, this->II);
#endif
              if (!quiet) { cout << "Using " << schedulePtr->getName() << endl; }
              //if (!quiet) schedulePtr->getDebugPrintouts();
              return schedulePtr;
          }
          case scheduler::SDS: {
              auto schedulePtr = std::make_shared<SDSScheduler>(gr, rm, this->II);
              if (!quiet) { cout << "Using " << schedulePtr->getName() << endl; }
              return schedulePtr;
          }
#endif
          case scheduler::NONE: {
              return nullptr;
          }
          default:
              throw (HatScheT::Exception("SCCSchedulerTemplate: No SCC-Scheduler specified, terminating..."));
      }
  }

  map<Vertex *, int> SCCSchedulerTemplate::computeBasicSchedules(SCC *sc) {
      // Creating maps
      map<Vertex *, Vertex *> _sccVertexToVertex;
      map<Vertex *, Vertex *> _vertexToSccVertex;
      // Generate Graph and ResourceModel
      auto gr = std::make_shared<Graph>();
      auto rm = std::make_shared<ResourceModel>();
      // Inset Vertices
      auto vertices = sc->getVerticesOfSCC();
      for (auto &v : vertices) {
          auto &newV = gr->createVertex(v->getId());
          _sccVertexToVertex[&newV] = v;
          _vertexToSccVertex[v] = &newV;
      }
      // Create Resources
      for (auto &v : gr->Vertices()) {
          auto res = resourceModel.getResource(_sccVertexToVertex.at(v));
          Resource *newRes;
          try {
              newRes = rm->getResource(res->getName());
          } catch (HatScheT::Exception &) {
              newRes = &rm->makeResource(res->getName(), resourceLimits.at((Resource *) res), res->getLatency(),
                                         res->getBlockingTime());
          }
          rm->registerVertex(v, newRes);
      }
      // Create Edges
      auto edges = sc->getSCCEdges();
      for (auto &e : edges) {
          auto &newSrc = _vertexToSccVertex.at(&e->getVertexSrc());
          auto &newDst = _vertexToSccVertex.at(&e->getVertexDst());
          auto &newE = gr->createEdge(*newSrc, *newDst, e->getDistance(), e->getDependencyType());
          newE.setDelay(e->getDelay());
      }

      return sdcSchedule(gr, rm, _sccVertexToVertex);
  }

  map<Vertex *, int> SCCSchedulerTemplate::sdcSchedule(shared_ptr<Graph> &gr, shared_ptr<ResourceModel> &rm,
                                                       map<Vertex *, Vertex *> &_sccVertexToVertex) {
      map<Vertex *, int> relSched;
      map<Vertex *, int> relSchedTemp;
      relSchedTemp = Utility::getSDCAsapAndAlapTimes(&(*gr), &(*rm), this->II).first;
      for (auto &vtPair : relSchedTemp) {
          relSched[_sccVertexToVertex.at(vtPair.first)] = vtPair.second;
      }
      return relSched;
  }

  void SCCSchedulerTemplate::combineRelativeSchedules() {
      //Going through topo sorted SCCs, and deciding based on sccType, what to do:
      for (auto &sc : topoSortedSccs) {
          auto sccType = sc.first->getSccType();
          if (sccType == unknown) {
              // Should never happen, throw Error.
              cout << "SCC_" << sc.first->getId() << " is Unknown!" << endl;
              throw (HatScheT::Exception(
                  "SCCSchedulerTemplate::combineRelativeSchedules(): SCC of Type 'Unknown' found!"));
          }
          if (sccType == trivial) {
              //if (!quiet) { cout << "SCC_" << sc.first->getId() << " is trivial!" << endl; }
              // Trivial SCCs only have one Vertex.
              if (sc.first->getVerticesOfSCC().size() != 1) {
                  cout << "SCC_" << sc.first->getId() << endl;
                  for (auto &v : sc.first->getVerticesOfSCC()) {
                      cout << v->getName() << endl;
                  }
                  throw (HatScheT::Exception(
                      "SCCSchedulerTemplate::combineRelativeSchedules(): Trivial SCC with now exactly 1 Vertex!"));
              }
              auto cvp = (const Vertex *) sc.first->getVerticesOfSCC().back();
              auto rp = (Resource *) resourceModel.getResource(cvp);
              int timeSlot = 0;
              for (auto &e : connectingEdges) {
                  if (&e->getVertexDst() != cvp) {
                      continue;
                  }
                  auto vSrc = &e->getVertexSrc();
                  int tSrc = startTimes.at(vSrc); // Wirft Error
                  int t = tSrc + e->getDelay() - ((int) this->II * e->getDistance()) +
                          this->resourceModel.getVertexLatency(vSrc);
                  if (t > timeSlot) {
                      timeSlot = t;
                  }
              }
              int offsetTime = 0;
              if (!rp->isUnlimited()) {
                  auto rLim = rp->getLimit();
                  for (int i = 0; i < this->II; i++) {
                      int slot = (timeSlot + i) % (int) this->II;
                      try {
                          if (usedFUsInModSlot.at({rp, slot}) < rLim) {
                              offsetTime = i;
                              usedFUsInModSlot.at({rp, slot})++;
                              break;
                          }
                      } catch (std::out_of_range &) {
                          cout << rp->getName() << " Slot " << slot << " II: " << II << endl;
                          throw (HatScheT::Exception("Doofer Error"));
                      }
                  }
              }
              this->startTimes.at((Vertex *) cvp) = timeSlot + offsetTime;
          }
          if (sccType == basic) {
              //if (!quiet) { cout << "SCC_" << sc.first->getId() << " is basic!" << endl; }
              int maxMinOffset = 0;
              for (auto &e : connectingEdges) {
                  if (vertexToSCC.at(&e->getVertexDst()) != sc.first) {
                      // Skip edges that do not end in the current SCC.
                      continue;
                  }
                  auto vSrc = &e->getVertexSrc();
                  int tSrc = this->startTimes.at(vSrc);
                  int distance = e->getDistance();
                  int delay = e->getDelay();
                  int minOffset = (int) (tSrc + this->resourceModel.getVertexLatency(vSrc) + delay -
                                         distance * this->II);
                  minOffset = std::max(minOffset,
                                       0); // do not let the minimum offset be negative to prevent negative starting times
                  maxMinOffset = std::max(maxMinOffset, minOffset);
              }
              // offset every vertex by the minimum offset
              for (auto &vtpair : basicRelSchedules.at(sc.first)) {
                  int tSrc = vtpair.second + maxMinOffset;
                  startTimes.at(vtpair.first) = tSrc;
              }
          }
          if (sccType == complex) {
              //if (!quiet) { cout << "SCC_" << sc.first->getId() << " is complex!" << endl; }
              int maxMinOffset = 0;
              for (auto &e : connectingEdges) {
                  if (vertexToSCC.at(&e->getVertexDst()) != sc.first) {
                      // Skip edges that do not end in the current SCC.
                      continue;
                  }
                  auto vSrc = &e->getVertexSrc();
                  auto vDst = &e->getVertexDst();
                  auto lSrc = this->resourceModel.getVertexLatency(vSrc);
                  auto tSrc = this->startTimes.at(vSrc);
                  auto tDstRel = this->complexRelSchedules.at(sc.first).at(vDst);
                  auto distance = e->getDistance();
                  auto delay = e->getDelay();
                  auto minOffset = (int) std::ceil(
                      ((double) tSrc - tDstRel + lSrc + delay - (distance * this->II)) / (this->II));
                  minOffset = std::max(minOffset, 0);
                  maxMinOffset = std::max(maxMinOffset, minOffset);
              }
              // offset every vertex by the minimum offset
              for (auto &vtpair : complexRelSchedules.at(sc.first)) {
                  int tSrc = vtpair.second + (int) (maxMinOffset * II);
                  startTimes.at(vtpair.first) = tSrc;
              }
          }
      }
  }

  void SCCSchedulerTemplate::finalizeComplexSchedule() {
      for (auto &sc : complexSCCs) {
          map<Vertex *, int> cmplxRelSchedTemp;
          for (auto &v : sc->getVerticesOfSCC()) {
              cmplxRelSchedTemp[v] = bigComplexSchedule.at(v); // % (int)II;
              auto rp = (Resource *) resourceModel.getResource(v);
              try {
                  usedFUsInModSlot.at({rp, bigComplexSchedule.at(v) % (int) II})++;
              } catch (std::out_of_range &) {
                  usedFUsInModSlot[{rp, bigComplexSchedule.at(v) % (int) II}] = 1;
              }
          }
          complexRelSchedules[sc] = cmplxRelSchedTemp;
      }
  }

  void SCCSchedulerTemplate::setSolverTimeout(double seconds) {
      this->solverTimeout = seconds;
  }

  void SCCSchedulerTemplate::sharedPointerCastAndSetup(shared_ptr<IterativeModuloSchedulerLayer> &ssptr) {
      if (finalScheduler == scheduler::SMT) {
#if defined(USE_Z3)
          auto ptr = std::dynamic_pointer_cast<SMTUnaryScheduler>(ssptr);
          ptr->setLatencySearchMethod(SMTUnaryScheduler::latSearchMethod::LINEAR);
#else
          throw (HatScheT::Exception("SCC-Template: SMT-Unary-Scheduler needs Z3... Z3 not linked"));
#endif
      }
  }

#if defined(USE_Z3)
  void SCCSchedulerTemplate::calcStarttimesPerComplexSCC() {

      map<Vertex *, Vertex *> scc2v;
      map<Vertex *, Vertex *> v2scc;

      int sccCounter = 0;
      // Creating maps and Generate Graph and ResourceModel and Inset Vertices
      for (auto &sc : complexSCCs) {
          Graph sccG;
          ResourceModel sccRM;
          for (auto &v : sc->getVerticesOfSCC()) {
              Vertex &newV = sccG.createVertex(v->getId());
              scc2v[&newV] = v;
              v2scc[v] = &newV;
          }
          // Create Resources
          resetResourceModel(); // Needed because of strange exception in constructor of resource class.
          for (auto &vinSccG : sccG.Vertices()) {
              auto res = resourceModel.getResource(scc2v.at(vinSccG));
              Resource *newRes;
              try {
                  newRes = sccRM.getResource(res->getName());
              } catch (HatScheT::Exception &) {
                  newRes = &sccRM.makeResource(res->getName(), res->getLimit(), res->getLatency(),
                                               res->getBlockingTime());
              }
              sccRM.registerVertex(vinSccG, newRes);
          }
          modifyResourceModel(); // Needed because of strange exception in constructor of resource class.
          auto edges = sc->getSCCEdges();
          for (auto &e : edges) {
              auto &newSrc = v2scc.at(&e->getVertexSrc());
              auto &newDst = v2scc.at(&e->getVertexDst());
              auto &newE = sccG.createEdge(*newSrc, *newDst, e->getDistance(), e->getDependencyType());
              newE.setDelay(e->getDelay());
          }

          auto sccStartTimes = Utility::getSDCAsapAndAlapTimes(&sccG, &sccRM, this->II, true);

          z3::context cLocal;
          z3::optimize opti(cLocal);

          map<Vertex *, z3::expr> tvars;
          map<Vertex *, z3::expr> svars;

          //Create T-Variables and allow only pos. integers:
          for (auto &v : sc->getVerticesOfSCC()) {
              std::stringstream name;
              name << v->getName();
              z3::expr tvar(cLocal.int_const(name.str().c_str()));
              z3::expr svar(cLocal.bool_const(name.str().c_str()));
              tvars.insert({v, tvar});
              svars.insert({v, svar});
              opti.add(tvar >= 0);
          }

          for (auto &e : sc->getSCCEdges()) {
              Vertex *src = &e->getVertexSrc();
              Vertex *dst = &e->getVertexDst();
              int delay = e->getDelay();
              int distance = e->getDistance();
              opti.add(
                  tvars.at(dst) - tvars.at(src) >=
                  resourceModel.getResource(src)->getLatency() + delay - (distance * (int) II));
          }

          z3::expr_vector zeroPoints(cLocal);
          z3::expr_vector timeVars(cLocal);
          for (auto &v : sc->getVerticesOfSCC()) {
              opti.add(z3::implies(svars.at(v), tvars.at(v) == 0));
              zeroPoints.push_back(svars.at(v));
              timeVars.push_back(tvars.at(v));
          }

          opti.add(z3::atleast(zeroPoints, 1));
          opti.maximize(sum(timeVars));

          z3::check_result sati = opti.check();
          if (!quiet) { cout << "Searching Max Latency of SCC (z3) ... " << sati << ": "; }

          auto m = opti.get_model();

          int max = 0;
          for (auto &v : sc->getVerticesOfSCC()) {
              if ((m.eval(tvars.at(v)).get_numeral_int() + resourceModel.getVertexLatency(v)) > max) {
                  max = m.eval(tvars.at(v)).get_numeral_int() + resourceModel.getVertexLatency(v);
              }
          }
          max += sccCounter;
          maxTimesSCC.insert(max);
          if (!quiet) { cout << max << endl; }
          opti.pop();
          while (!timeVars.empty()) {
              timeVars.pop_back();
          }
          while (!zeroPoints.empty()) {
              zeroPoints.pop_back();
          }
          tvars.clear();
          svars.clear();

          unordered_map<Vertex *, int> ltTemp;
          unordered_map<Vertex *, Vertex *> new2old;

          for (auto &v : sccStartTimes.first) {
              earliestStarttimes[scc2v.at(v.first)] = v.second;
              ltTemp[scc2v.at(v.first)] = sccStartTimes.second.at(v.first);
              new2old[scc2v.at(v.first)] = scc2v.at(v.first);
          }

          auto length = Utility::getSDCScheduleLength(ltTemp, new2old, &resourceModel);

          for (auto &v : sccStartTimes.second) {
              latestStarttimes[scc2v.at(v.first)] = max - (length - v.second);
          }

          sccCounter++;
      }
  }

#elif defined(USE_SCALP)
  void SCCSchedulerTemplate::calcStarttimesPerComplexSCCScaLP() {

      cout << "ScaLP Version!" << endl;

      map<Vertex *, Vertex *> scc2v;
      map<Vertex *, Vertex *> v2scc;

      int sccCounter = 0;
      // Creating maps and Generate Graph and ResourceModel and Inset Vertices
      for (auto &sc : complexSCCs) {
          Graph sccG;
          ResourceModel sccRM;
          for (auto &v : sc->getVerticesOfSCC()) {
              Vertex &newV = sccG.createVertex(v->getId());
              scc2v[&newV] = v;
              v2scc[v] = &newV;
          }
          // Create Resources
          resetResourceModel(); // Needed because of strange exception in constructor of resource class.
          for (auto &vinSccG : sccG.Vertices()) {
              auto res = resourceModel.getResource(scc2v.at(vinSccG));
              Resource *newRes;
              try {
                  newRes = sccRM.getResource(res->getName());
              } catch (HatScheT::Exception &) {
                  newRes = &sccRM.makeResource(res->getName(), res->getLimit(), res->getLatency(),
                                               res->getBlockingTime());
              }
              sccRM.registerVertex(vinSccG, newRes);
          }
          modifyResourceModel(); // Needed because of strange exception in constructor of resource class.
          auto edges = sc->getSCCEdges();
          for (auto &e : edges) {
              auto &newSrc = v2scc.at(&e->getVertexSrc());
              auto &newDst = v2scc.at(&e->getVertexDst());
              auto &newE = sccG.createEdge(*newSrc, *newDst, e->getDistance(), e->getDependencyType());
              newE.setDelay(e->getDelay());
          }

          auto sccStartTimes = Utility::getSDCAsapAndAlapTimes(&sccG, &sccRM, this->II, true);

          ScaLP::Solver s{"Gurobi", "CPLEX", "SCIP", "LPSolve"};

          s.quiet = true;

          map<Vertex *, ScaLP::Variable> _svars;
          map<Vertex *, ScaLP::Variable> _tvars;
          ScaLP::Term ssum;
          ScaLP::Term tsum;

          const double M = 100000;

          //Create T-Variables and allow only pos. integers:
          for (auto &v : sc->getVerticesOfSCC()) {
              std::stringstream name;
              name << v->getName();
              ScaLP::Variable _tvar = ScaLP::newIntegerVariable("t" + name.str());
              ScaLP::Variable _svar = ScaLP::newIntegerVariable("s" + name.str());
              _tvars.insert({v, _tvar});
              _svars.insert({v, _svar});
              s.addConstraint((M * (1 - _svar)) - _tvar >= 0);
              s.addConstraint(_tvar >= 0);
              tsum += _tvar;
              ssum += _svar;
          }

          s.addConstraint(ssum >= 1);

          for (auto &e : sc->getSCCEdges()) {
              Vertex *src = &e->getVertexSrc();
              Vertex *dst = &e->getVertexDst();
              int delay = e->getDelay();
              int distance = e->getDistance();
              s.addConstraint(_tvars.at(dst) - _tvars.at(src) >=
                              resourceModel.getResource(src)->getLatency() + delay - (distance * (int) II));
          }

          s.setObjective(ScaLP::maximize(tsum));
          s.solve();

          auto maxResult = s.getResult();
          int max = 0;

          for (auto &it : _tvars) {
              int startTimePlusLAtency =
                  (int) maxResult.values.at(it.second) + resourceModel.getResource(it.first)->getLatency();
              if (startTimePlusLAtency > max) {
                  max = startTimePlusLAtency;
              }
          }

          max += sccCounter;
          maxTimesSCC.insert(max);
          if (!quiet) { cout << "Searching Max Latency of SCC (ScaLP) ... " << max << endl; }

          sccCounter++;
          s.reset();

          unordered_map<Vertex *, int> ltTemp;
          unordered_map<Vertex *, Vertex *> new2old;

          for (auto &v : sccStartTimes.first) {
              earliestStarttimes[scc2v.at(v.first)] = v.second;
              ltTemp[scc2v.at(v.first)] = sccStartTimes.second.at(v.first);
              new2old[scc2v.at(v.first)] = scc2v.at(v.first);
          }

          auto length = Utility::getSDCScheduleLength(ltTemp, new2old, &resourceModel);

          for (auto &v : sccStartTimes.second) {
              latestStarttimes[scc2v.at(v.first)] = max - (length - v.second);
          }

      }
  }
#endif

}