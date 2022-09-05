//
// Created by bkessler on 8/11/22.
//

#ifdef USE_Z3

#include <cmath>
#include <chrono>
#include <algorithm>
#include <z3++.h>
#include <HatScheT/scheduler/ALAPScheduler.h>

#include "SMTSCCScheduler.h"
#include "HatScheT/scheduler/ASAPScheduler.h"
#include "HatScheT/utility/subgraphs/KosarajuSCC.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/Verifier.h"
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
      timeLimit = -1;

      iigiven = false;
      numOfCmplxSCCs = 0;
      mode = schedule_t::fast;

      for (auto &v : g.Vertices()){
          this->startTimes[v] = 0;
      }

      for (auto &r : resourceModel.Resources()) {
          for (int i = 0; i < II; i++) {
              usedFUsInModSlot[{r, i}] = 0;
          }
      }
  }

  SMTSCCScheduler::SMTSCCScheduler(Graph &g, ResourceModel &resourceModel, double II) : SchedulerBase(g, resourceModel) {
      this->timeouts = 0;
      startTimes.clear();
      scheduleFound = false;
      timeLimit = -1;

      this->iigiven = true;
      this->II = II;

      numOfCmplxSCCs = 0;
      mode = schedule_t::fast;

      for (auto &v : g.Vertices()){
          this->startTimes[v] = 0;
      }

      for (auto &r : resourceModel.Resources()) {
          for (int i = 0; i < II; i++) {
              usedFUsInModSlot[{r, i}] = 0;
          }
      }
  }

  void SMTSCCScheduler::schedule() {

      modifyResourceModel();
      // Get SCCs and sort them:
      computeSCCs();

      // Compute relative Schedules:
      deque<SCC*> complexSCCs;
      deque<SCC*> basicSCCs;
      for (auto &sc : topoSortedSccs){
          if(sc.first->getSccType() == unknown){
              //cout << sc.second << ": SCC_" << sc.first->getId() << " - " << "Unknown" << endl;
              throw(HatScheT::Exception("SMTSCCScheduler::schedule() : SCC-Type not determined!"));
          }
          if (sc.first->getSccType() == trivial){
              //cout << sc.second << ": SCC_" << sc.first->getId() << " - " << "Trivial - Vertices: " << sc.first->getNumberOfVertices() << endl;
              continue;
          }
          if (sc.first->getSccType() == basic){
              //cout << sc.second << ": SCC_" << sc.first->getId() << " - " << "Basic - Vertices: " << sc.first->getNumberOfVertices() << endl;
              basicSCCs.push_back(sc.first);
          }
          if (sc.first->getSccType() == complex){
              //cout << sc.second << ": SCC_" << sc.first->getId() << " - " << "Complex - Vertices: " << sc.first->getNumberOfVertices() << endl;
              complexSCCs.push_back(sc.first);
          }
      }
      numOfCmplxSCCs = complexSCCs.size();
      //Complex SCCs first to determine a feasible II
      if (!complexSCCs.empty()) {
          computeComplexSchedule(complexSCCs);
      }
      if(!basicSCCs.empty()){
          for (auto &sc : basicSCCs) {
              auto relSched = computeBasicSchedules(sc);
              basicRelSchedules[sc] = relSched;
          }
      }
      // Set optimal flags:
      secondObjectiveOptimal = false;
      timeouts > 0 ? firstObjectiveOptimal = false : firstObjectiveOptimal = true;

      // Combining relative Schedules:
      combineRelScheds();
      // Check if Schedule is Valid:
      scheduleFound = verifyModuloSchedule(g, resourceModel, startTimes, (int) II);

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
              r->setLimit(UNLIMITED, false);
          }
      }
  }

  void SMTSCCScheduler::resetResourceModel() {
      for (auto &r : resourceLimits){
          r.first->setLimit(resourceLimits.at(r.first), false);
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

  void SMTSCCScheduler::computeComplexSchedule(deque<SCC*> &complexSCCs) {
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
      resetResourceModel(); // Needed because of strange exception in constructor of resource class.
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
      modifyResourceModel(); // Needed because of strange exception in constructor of resource class.
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
      auto bigComplexShedule = smtSchedule(gr, rm, sccVertexToVertex);
      for (auto &sc : complexSCCs){
          map<Vertex*, int> cmplxRelSchedTemp;
          for (auto &v : sc->getVerticesOfSCC()){
              cmplxRelSchedTemp[v] = bigComplexShedule.at(v); // % (int)II;
              auto rp = (Resource*)resourceModel.getResource(v);
              try {
                  usedFUsInModSlot.at({rp, bigComplexShedule.at(v) % (int) II})++;
              }catch(std::out_of_range&){
                  usedFUsInModSlot[{rp, bigComplexShedule.at(v) % (int)II}] = 1;
              }
          }
          complexRelSchedules[sc] = cmplxRelSchedTemp;
      }
  }

  map<Vertex *, int> SMTSCCScheduler::smtSchedule(shared_ptr<Graph> &gr, shared_ptr<ResourceModel> &rm,
                                                  map<Vertex *, Vertex *> &sccVertexToVertex) {

      map<Vertex*, int> relSched;
      map<Vertex*, int> relSchedTemp;
      bool foundSchedule;
      if (!iigiven) {
          do {
              auto smt = new SMTBinaryScheduler(*gr, *rm, this->II);
              smt->setQuiet(quiet);
              int maxSCCslat;
              if (mode == schedule_t::fast){
                  maxSCCslat = std::max(expandSCC(gr, rm) + numOfCmplxSCCs - 1, (int) II);
                  smt->setMaxLatencyConstraint(maxSCCslat);
              }else if (mode == schedule_t::optimal){
                  maxSCCslat = expandSCC(gr, rm) + (int)II;
                  smt->setMaxLatencyConstraint(maxSCCslat);
              }else {
                  //Let Scheduler Search...
              }
              if (timeLimit > 0) { smt->setSolverTimeout(timeLimit); }
              smt->schedule();
              timeBudget = smt->getTimeBudget();
              if( smt->getTimeouts() > 0 ) { timeouts++; }
              if (!quiet) { cout << "Timeouts of SMT-Binary-Scheduler: " << timeouts << endl; }
              foundSchedule = smt->getScheduleFound();
              if (foundSchedule) {
                  relSchedTemp = smt->getSchedule();
                  this->II = smt->getII();
              }
              if (!foundSchedule) { this->II += 1; }
              delete smt;
          } while (!foundSchedule and ((int)this->II < maxII));
      }else{
          auto smt = new SMTBinaryScheduler(*gr, *rm, this->II);
          smt->setQuiet(quiet);
          int maxSCCslat;
          if (mode == schedule_t::fast){
              maxSCCslat = std::max(expandSCC(gr, rm) + numOfCmplxSCCs - 1, (int)II);
              smt->setMaxLatencyConstraint(maxSCCslat);
          }else if (mode == schedule_t::optimal){
              maxSCCslat = expandSCC(gr, rm) + (int)II;
              smt->setMaxLatencyConstraint(maxSCCslat);
          }else{
              //Let Scheduler search...
          }
          if (timeLimit > 0) { smt->setSolverTimeout(timeLimit); }
          smt->schedule();
          timeBudget = smt->getTimeBudget();
          if( smt->getTimeouts() > 0 ) { timeouts++; }
          if (!quiet) { cout << "Timeouts of SMT-Binary-Scheduler: " << timeouts << endl; }
          scheduleFound = smt->getScheduleFound();
          relSchedTemp = smt->getSchedule();
          delete smt;
      }

      if (!quiet){
          if (verifyModuloSchedule(*gr, *rm, relSchedTemp,(int)this->II)){
                cout << "Big Complex Relative Schedule is Valid!" << endl;
          } else {
                cout << "Big Complex Relative Schedule is NOT!!!!!!!111111 Valid!" << endl;
          }
      }

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

      // Topological Order with Asap Schedule:
      ALAPScheduler alap (*gr, *rm);
      map<SCC*, int> inverseSccPriority;
      alap.schedule();
      auto asapSched = alap.getSchedule();
      for (auto &vtPair: asapSched){
          inverseSccPriority[locVertexToScc.at(vtPair.first)] = vtPair.second;
      }

      return inverseSccPriority;
  }

  void SMTSCCScheduler::combineRelScheds() {
      //Going through topo sorted SCCs, and deciding based on sccType, what to do:
      for (auto &sc : topoSortedSccs) {
          auto sccType = sc.first->getSccType();
          if (sccType == unknown) {
              // Should never happen, throw Error.
              cout << "SCC_" << sc.first->getId() << " is Unknown!" << endl;
              throw (HatScheT::Exception("SMTSCCScheduler::combineRelScheds() SCC of Type 'Unknown' found!"));
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
                      "SMTSCCScheduler::combineRelScheds() Trivial SCC with now exactly 1 Vertex!"));
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
                      if (usedFUsInModSlot.at({rp, slot}) < rLim) {
                          offsetTime = i;
                          usedFUsInModSlot.at({rp, slot})++;
                          break;
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
                  auto minOffset = (int) std::ceil(((double) tSrc - tDstRel + lSrc + delay - (distance * this->II)) / (this->II));
                  minOffset = std::max(minOffset, 0);
                  maxMinOffset = std::max(maxMinOffset, minOffset);
              }
              // offset every vertex by the minimum offset
              for (auto &vtpair : complexRelSchedules.at(sc.first)) {
                  int tSrc = vtpair.second + (int)(maxMinOffset * II);
                  startTimes.at(vtpair.first) = tSrc;
              }
          }
      }
  }

  void SMTSCCScheduler::setSolverTimeout(int seconds) {
        timeLimit = seconds;
  }

  int SMTSCCScheduler::expandSCC(shared_ptr<Graph> &gr, shared_ptr<ResourceModel> &rm) {

      KosarajuSCC kscc(*gr);
      auto locSCCs = kscc.getSCCs();

      set<int> maxtimes;

      for (auto sc : locSCCs) {
          z3::context c;
          z3::optimize opti(c);
          opti.push();
          map<Vertex *, z3::expr> tvars;
          map<Vertex *, z3::expr> svars;

          //Create T-Variables and allow only pos. integers:
          for (auto &v : sc->getVerticesOfSCC()) {
              std::stringstream name;
              name << v->getName();
              z3::expr tvar(c.int_const(name.str().c_str()));
              z3::expr svar(c.bool_const(name.str().c_str()));
              tvars.insert({v, tvar});
              svars.insert({v, svar});
              opti.add(tvar >= 0);
          }

          //Dependecy Constraints
          for (auto &e : sc->getSCCEdges()) {
              Vertex *src = &e->getVertexSrc();
              Vertex *dst = &e->getVertexDst();
              int delay = e->getDelay();
              int distance = e->getDistance();
              opti.add(
                  tvars.at(dst) - tvars.at(src) >= rm->getResource(src)->getLatency() + delay - (distance * (int) II));
          }

          z3::expr_vector zeroPoints(c);
          z3::expr_vector timeVars(c);
          for (auto &v : sc->getVerticesOfSCC()) {
              opti.add(z3::implies(svars.at(v), tvars.at(v) == 0));
              zeroPoints.push_back(svars.at(v));
              timeVars.push_back(tvars.at(v));
          }

          opti.add(z3::atleast(zeroPoints, 1));
          opti.maximize(sum(timeVars));

          z3::check_result sati = opti.check();
          if (!quiet) { cout << "Searching Max Latency of SCC: " << sati << ": "; }

          auto m = opti.get_model();

          int max = 0;
          for (auto &v : sc->getVerticesOfSCC()) {
              //cout << v->getName() << ": " << m.eval(tvars.at(v)).get_numeral_int() + rm->getVertexLatency(v) << endl;
              if ((m.eval(tvars.at(v)).get_numeral_int() + rm->getVertexLatency(v)) > max) {
                  max = m.eval(tvars.at(v)).get_numeral_int() + rm->getVertexLatency(v);
              }
          }
          if (!quiet) { cout << max << endl; }
          maxtimes.insert(max);
          opti.pop();
          while (!timeVars.empty()) {
              timeVars.pop_back();
          }
          while (!zeroPoints.empty()) {
              zeroPoints.pop_back();
          }
          tvars.clear();
          svars.clear();
      }
      return *std::max_element(maxtimes.begin(), maxtimes.end());
  }

  void SMTSCCScheduler::setMode(schedule_t schedulemode) {
      if (schedulemode == schedule_t::fast){
          cout << "'Fast-Mode'" << endl;
      }else if (schedulemode == schedule_t::optimal){
          cout << "'Optimal-Mode'" << endl;
      }else if (schedulemode == schedule_t::automatic){
          cout << "'Automatic-Mode'" << endl;
      }
      this->mode = schedulemode;
  }

}

#endif //USE_Z3