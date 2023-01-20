//
// Created by bkessler on 5/7/22.
//

#ifdef USE_Z3
#include "SMTUnaryScheduler.h"
#include "HatScheT/utility/Exception.h"
#include "HatScheT/utility/Utility.h"

#include <z3++.h>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <deque>
#include <algorithm>
#include <unordered_map>

namespace HatScheT {

  SMTUnaryScheduler::SMTUnaryScheduler(Graph &g, ResourceModel &resourceModel, double II) : IterativeModuloSchedulerLayer(g, resourceModel, II){
      this->sPref = schedulePreference::MOD_ASAP;
      this->latSM = latSearchMethod::BINARY;
      this->latencySpaceIndex = 0;
      this->candidateLatency = -1;
      this->latencyConstraintOutOfRange = false;
      this->minLatency = 0;
      this->maxLatency = -1;
      this->targetLatency = -1;
      this->leftIndex = 0;
      this->rightIndex = 0;

      this->binarySearchInit = true;
      this->linearSearchInit = true;

      this->modAsapLength = 0;
      this->modAlapLength = 0;
      this->increment = 0;

      this->designName = "";
      firstObjectiveOptimal = true;
      secondObjectiveOptimal = true;

  }

  /*!---------------------------*
   * Main scheduling functions: *
   * ---------------------------*/
  void SMTUnaryScheduler::scheduleInit() {
      if(!quiet) {
          cout << "Scheduling with " << this->getName() <<":" << endl;
          cout << "Rec-Min-II: " << recMinII << endl;
          cout << "Res-Min-II: " << resMinII << endl;
          cout << "Max. II: " << maxII << endl;
          cout << "Vertices of G: " << g.getNumberOfVertices() << endl;
          cout << "Edges of G: " << g.getNumberOfEdges() << endl;
          this->printZ3Params();
      }
  }

  void SMTUnaryScheduler::scheduleIteration() {

      this->solvingTimePerIteration = 0;
      candidateLatency = 0;
      secondObjectiveOptimal = true;
      binarySearchInit = true;
      calcLatencyEstimation();
      if (latencyConstraintOutOfRange){
          scheduleFound = false;
          this->II = -1;
          startTimes.clear();
          return;
      }
      if ( scheduleFound ) { return; }
      calcLatencySpace();

      while (true) {
          z3Reset();
          if (!quiet) { cout << "*--------------------------*" << endl; }
          if (!quiet) { cout << "* Start scheduling attempt *" << endl; }
          if (!quiet) { cout << "*--------------------------*" << endl; }

          cout << "Solver Parameters: " << p << endl;

          int oldLatency = candidateLatency;
          if (latSM == latSearchMethod::LINEAR) { candidateLatency = latLinearSearch(this->getZ3Result()); }
          if (latSM == latSearchMethod::BINARY) { candidateLatency = latBinarySearch(this->getZ3Result()); }

          if (latSM == latSearchMethod::BINARY) {
              if (candidateLatency == -1 or oldLatency == candidateLatency) {
                  cout << "Bin. Search Break: " << candidateLatency << " / " << oldLatency << endl;
                  candidateLatency = 0;
                  break;
              }
          }
          if (latSM == latSearchMethod::LINEAR){
              if (candidateLatency == -1){
                  return;
              }
          }
          if (!quiet) { cout << "Trying II: " << this->II << " Trying Latency: " << candidateLatency << endl; }
          //Updating latest start times with candidate latency
          updateLatestStartTimes();
          //Generating dependency constraints for z3-prover.
          if (!quiet) { cout << "Generating b-variables... " << endl; }
          generateBvariables();
          //if (!quiet) { print_b_variables(); }

          //Reducing Seach Space.
          if (!quiet) { cout << "Prohibit early starts... " << endl; }
          prohibitToEarlyStartsAndAdd();
          if (!quiet) { cout << "Prohibit late starts... " << endl; }
          prohibitToLateStartsAndAdd();

          //Pushing assertions into z3-Prover
          if (!quiet) { cout << "Solving... " << endl; }

          if (!quiet) { cout << "Add one-slot-constraints... " << endl; }
          addOneSlotConstraintsToSolver();
          if (!quiet) { cout << ">>" << this->getZ3Result() << "<<" << endl; }

          if (!quiet) { cout << "Add resource-constraints... " << endl; }
          if (this->getZ3Result() != z3::unsat) {
              this->setZ3Timeout((uint32_t) timeRemaining);
              addResourceLimitConstraintToSolver((int)this->II);
              if (!quiet) { cout << ">>" << this->getZ3Result() << "<<" << endl; }
          }

          if (this->getZ3Result() != z3::unsat) {
              if (!quiet) { cout << "Add dependency-constraints... " << endl; }
              if (g.getNumberOfEdges() * candidateLatency > 5000000) {
                  if (!quiet) { cout << "Using Method for BIIIIG Problems... " << g.getNumberOfEdges() * candidateLatency << endl; }
                  setDependencyConstraintsAndAddToSolverBIG((int)this->II);
              } else {
                  setDependencyConstraintsAndAddToSolver((int)this->II);
              }
              if (!quiet) { cout << ">>" << "DONE" << "<<" << endl; }
          }

          //if (!quiet) { cout << "System of " << s.assertions().size() << " assertions... Final Check" << endl; }
          double t = 0;
          //Final Check, when all assertions are in.
          if (this->getZ3Result() != z3::unsat) {
              //this->setZ3Quiet(false);
              this->setZ3Timeout((uint32_t) timeRemaining);
              startTimeTracking();
              this->z3Check();
              endTimeTracking();
              //this->setZ3Quiet(true);
          }

          if (this->getZ3Result() == z3::unknown) {
              timeouts++;
          }
          //Saving solving times per latency attempt.
          if (!quiet) {
              cout << ">>" << this->getZ3Result() << "<<" << endl;
              cout << "Solving Time: " << solvingTimePerIteration << " sec " << endl;
              cout << "TimeRemaining: " << timeRemaining << endl;
              cout << "*-------------------------*" << endl;
              cout << "* Scheduling attempt done *" << endl;
              cout << "*-------------------------*" << endl;
              cout << endl;
          }

          if (timeouts > 0){
              firstObjectiveOptimal = false;
          }

          if (this->getZ3Result() == z3::sat) {
              parseSchedule(this->m);
              scheduleFound = verifyModuloScheduleSMT(this->g, this->resourceModel, this->startTimes, (int) this->II);
              if (disableSecObj and scheduleFound) {
                  return;
              }
              if (!scheduleFound) {
                  cerr << "SMTUnaryScheduler: Found Schedule is not valid!" << endl;
              }
              if (latSM == latSearchMethod::LINEAR) {
                  break;
              }
          } else if (this->getZ3Result() == z3::unknown) {
              secondObjectiveOptimal = false;
              break;
          }
      }
  }

  /*!-------------------*
   * Latency Estimation *
   * -------------------*/
  void SMTUnaryScheduler::calcLatencyEstimation() {

      this->maxLatency = 0;
      //Check if max latency was set by user

      int resMaxLat = 0;

      //Getting earliest and latest possible Starttimes for a modulo schedule
      auto aslap = calcAsapAndAlapModScheduleWithSdc(g, resourceModel);
      if (scheduleFound){ return; }
      earliestStartTimes = aslap.first;
      latestStartTimes = aslap.second;

      // Try to find best latency:
      if (targetLatency < 0) {
          if ( !quiet ) {
              cout << "*------------------------*" << endl;
              cout << "* Max Latency Estimation *" << endl;
              cout << "*------------------------*" << endl;
          }
          auto startMaxTime = std::chrono::high_resolution_clock::now();
          calcMaxLatencyEstimation((int)this->II);
          if (maxLatency < (int)this->II) {
              maxLatency = (int)this->II;
          }
          auto endMaxTime = std::chrono::high_resolution_clock::now();
          auto durationMaxTime = std::chrono::duration_cast<std::chrono::microseconds>(
              endMaxTime - startMaxTime).count();
          if (!quiet) { cout << "Done in " << (float) durationMaxTime / 1000000 << " seconds." << endl << endl; }
      }
      // Solve for a given latency:
      else{
          cout << "Latency set by User: " << targetLatency << endl;
          cout << "Trying this latency!" << endl;
          minLatency = targetLatency;
          maxLatency = targetLatency;
          return;
      }

      if ( !quiet ) {
          cout << "*------------------------*" << endl;
          cout << "* Min Latency Estimation *" << endl;
          cout << "*------------------------*" << endl;
      }

      //Min Latency Estimation with ILP:
      //auto startTimeClock = std::chrono::high_resolution_clock::now();
      //int t = Utility::getMinLatency(&g, &resourceModel, aslap.first, aslap.second, currentII, {"Gurobi"}, this->timeouts, this->maxLatency);
      //cout << "Tau: " << t << " SDC-Latency: " << minLatency << " MinLatency: " << t + minLatency << endl;
      //auto endTimeClock = std::chrono::high_resolution_clock::now();
      //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTimeClock - startTimeClock).count();
      //cout << "Done in " << (float)duration/1000000 << " seconds." << endl;
      //throw(HatScheT::Exception("Stop here..."));

      //Min Latency Estimation with custom 'nameless' Algorithm.
      auto startTimeClock2 = std::chrono::high_resolution_clock::now();
      if ((int)this->II > modAsapLength){
          minLatency = (int)this->II;
          for (auto &it : aslap.second){
              it.second = it.second + ((int)this->II - modAsapLength);
          }
      }

      while (true) {
          auto feasilbe = calcMinLatencyEstimation(aslap, (int)this->II);
          if (feasilbe or (minLatency >= maxLatency)){
              if (!quiet) { cout << "Min Latency: " << minLatency << endl; }
              break;
          }
          for (auto &it : aslap.second){
              it.second += increment;
          }
          minLatency += increment;
      }
      auto endTimeClock2 = std::chrono::high_resolution_clock::now();
      auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(endTimeClock2 - startTimeClock2).count();
      if (!quiet) { cout << "Done in " << (float)duration2/1000000 << " seconds." << endl << endl; }
      if (maxLatencyConstraint > 0 and maxLatencyConstraint < maxLatency){
          if (!this->quiet){ cout << "Max Latency Constraint: " << maxLatencyConstraint << endl; }
          maxLatency = maxLatencyConstraint;
      }
      if ((maxLatencyConstraint < minLatency) and (maxLatencyConstraint > 0)){
          if (!quiet) { cout << endl << "Not possible to schedule graph for given II and Max. Latency Constraint." << endl; }
          if (!quiet) { cout << "Stopping Scheduleattempt..." << endl << endl; }
          latencyConstraintOutOfRange = true;
      }
  }

  void SMTUnaryScheduler::calcMaxLatencyEstimation(int currentII) {

      //Algorithm to estimate max latency based on earliest and latest possible start times
      //as well as on ressource constraints.
      map<pair<Vertex*, int>, bool> vertexTimeslot;
      unordered_map<Vertex*, bool> checked;

      //Generates a matrix with the information wether a Vertex can be schedules in a timeslot or not,
      //based on dependency constraints.
      for (auto &v : g.Vertices()){
          checked.insert(std::make_pair(v,false));
          int tAsap = earliestStartTimes.at(v);
          int tAlap = latestStartTimes.at(v);
          for (int i = 0; i < tAsap; i++){
              vertexTimeslot[{v, i}] = false;
          }
          for (int i = tAsap; i <= tAlap; i++){
              vertexTimeslot[{v, i}] = true;
          }
          for (int i = tAlap + 1; i < minLatency; i++){
              vertexTimeslot[{v, i}] = false;
          }
      }

      //Trys to satisfy ressource constraints. If they can not be satisfied, there will be operations
      //that can not be scheduled.
      for (auto &r : resourceModel.Resources()){
          if (r->isUnlimited()){
              continue;
          }
          vector<int>usedFuInModslot;
          usedFuInModslot.resize(currentII);
          for (int i = 0; i < modAsapLength; i++){
              for (auto &cvp : resourceModel.getVerticesOfResource(r)){
                  auto v = (Vertex*) cvp;
                  if (!vertexTimeslot.at(std::make_pair(v, i))){
                      continue;
                  }
                  //Check conflicts with other mod-slots.
                  if (usedFuInModslot.at(i % currentII) < r->getLimit() and !checked.at(v)){
                      //Check vertex and count up used Resources:
                      checked.at(v) = true;
                      usedFuInModslot.at(i % currentII)++;
                      continue;
                  }
                  //Set vertices, of Resource in the same timeslot to 0, because there are no more FUs.
                  vertexTimeslot.at(std::make_pair(v, i)) = false;
              }
          }
          usedFuInModslot.clear();
          usedFuInModslot.shrink_to_fit();
      }
      //Counting the not scheduled operations to expand the schedule by that amount.
      deque<int>criticalRessources;
      for (auto &r : resourceModel.Resources()) {
          if (r->isUnlimited()){
              continue;
          }
          int count2 = 0;
          for (auto &v : resourceModel.getVerticesOfResource(r)) {
              int count = 0;
              for (int i = 0; i < modAsapLength; i++) {
                  count += (int)vertexTimeslot.at(std::make_pair((Vertex*)v, i));
              }
              if (count == 0){
                  count2++;
              }
          }
          criticalRessources.push_back(count2);
      }

      int maximum = 0;
      if (!criticalRessources.empty()) {
          maximum = *std::max_element(criticalRessources.begin(), criticalRessources.end());
      }

      maxLatency = currentII * (int)ceil(float(maximum)/float(currentII)) + modAsapLength;
      if ( !quiet ) { cout << "Max Latency Suggestion: " << currentII << " * "
                           << (int)ceil(float(maximum)/float(currentII)) << " + " << modAsapLength << " = "
                           << currentII * (int)ceil(float(maximum)/float(currentII)) + modAsapLength << endl; }
  }

  bool SMTUnaryScheduler::calcMinLatencyEstimation(pair<map<Vertex*, int>, map<Vertex*, int>> &aslap, int currentII){

      //Algorithm to estimate min latency based on earliest and latest possible start times
      //as well as on ressource constraints.
      map<pair<Vertex*, int>, bool> vertexTimeslot;
      unordered_map<Resource*, bool> checkedResource;

      for (auto &r : resourceModel.Resources()){
          checkedResource[r] = false;
      }

      //Generates a matrix with the information wether a Vertex can be schedules in a timeslot or not,
      //based on dependency constraints.
      for (auto &v : g.Vertices()){
          int t_asap = aslap.first.at(v);
          int t_alap = aslap.second.at(v);
          for (int i = 0; i < t_asap; i++){
              vertexTimeslot[{v, i}] = false;
          }
          for (int i = t_asap; i <= t_alap; i++){
              vertexTimeslot[{v, i}] = true;
          }
          for (int i = t_alap + 1; i < minLatency; i++){
              vertexTimeslot[{v, i}] = false;
          }
      }

      // Schedules all operations with satisfied resource constraints. So we can get a requiered min. latency
      // for a schedule
      for (auto &r : resourceModel.Resources()) {
          if (r->isUnlimited()) {
              continue;
          }

          auto verticesOfThisResource = resourceModel.getVerticesOfResource(r);

          //Preprocessing: Problem auf einen Iteration Interval Reduzieren.
          for (int x = 0; x < min(currentII, minLatency); x++) {
              for (auto &cvp : verticesOfThisResource) {
                  auto vp = (Vertex *) cvp;
                  if (vertexTimeslot.at({vp, x})) {
                      for (int i = x + 1; i < minLatency; i++) {
                          try {
                              if ((i % currentII) != x) {
                                  continue;
                              }
                              vertexTimeslot.at({vp, i}) = false;
                          } catch (std::out_of_range &) {
                              cout << "i: " << i << " Out of Range A" << endl;
                              cout << "SMTUnaryScheduler::calcMinLatencyEstimation: Out of Range, " << vp->getName()
                                   << " " << x << endl;
                              cout << "Min. Latency: " << minLatency << endl;
                              throw (HatScheT::Exception(
                                  "SMTUnaryScheduler::calcMinLatencyEstimation: Out of Range A"));
                          }
                      }
                  } else {
                      for (int i = x + 1; i < minLatency; i++) {
                          try {
                              if ((i % currentII) != x) {
                                  continue;
                              }
                              if (vertexTimeslot.at({vp, i})) {
                                  vertexTimeslot.at({vp, i}) = false;
                                  vertexTimeslot.at({vp, x}) = true;
                              }
                          } catch (std::out_of_range &) {
                              cout << "i: " << i << endl;
                              cout << "SMTUnaryScheduler::calcMinLatencyEstimation: Out of Range, " << vp->getName()
                                   << " " << x << endl;
                              cout << "Min. Latency: " << minLatency << endl;
                              throw (HatScheT::Exception(
                                  "SMTUnaryScheduler::calcMinLatencyEstimation: Out of Range B"));
                          }
                      }
                  }
              }
          }

          vector<int> usedFuInModslot;
          usedFuInModslot.resize(currentII);
          map<Vertex *, int> availibleSlots;

          //Count how many FUs are used in each timeslot.
          //Count how many timeslots exist for each vertex.
          for (auto &cvp : verticesOfThisResource) {
              auto vp = (Vertex *) cvp;
              availibleSlots[vp] = 0;
              for (int x = 0; x < minLatency; x++) {
                  if (!vertexTimeslot.at({vp, x})) {
                      continue;
                  }
                  usedFuInModslot.at(x % currentII)++;
                  availibleSlots.at(vp)++;
              }
          }
          //Checking Resourceconstraints
          for (int x = 0; x < min(currentII, minLatency); x++) {
              //cout << "Checking Slot: " << x << endl;
              if (usedFuInModslot.at(x) <= r->getLimit()) {
                  //No Conflict, lock opterations.
                  checkedResource.at(r) = true;
                  for (auto &cvp : verticesOfThisResource){
                      auto vp = (Vertex*)cvp;
                      if (vertexTimeslot.at({vp, x})){
                          for (int i = 0; i < min(currentII, minLatency); i++){
                              if (i == x){
                                  continue;
                              }
                              if (vertexTimeslot.at({vp, i})) {
                                  vertexTimeslot.at({vp, i}) = false;
                                  availibleSlots.at(vp)--;
                                  usedFuInModslot.at(i)--;
                              }
                          }
                      }
                  }
                  continue;
              }
              // Potential conflict:
              // If there is a potential conflict, we have to create a mapping between the operations in the
              // mod-slot and their "mobility".
              // These operation are now sorted in a priority queue based on their mobility (availible slots).
              // Higher mobility means an earlier position in the PQ.
              // Then we take the operation from the PQ and move them to the mod-slot with the fewest used FUs,
              // until the resource contraint for the actual slot id satisfied or there are no more available slots
              // where the operation can be moved.
              priority_queue<pair<Vertex *, int>, vector<pair<Vertex *, int>>, SmtVertexIntCompLess> pq;
              for (auto &cvp : verticesOfThisResource) {
                  auto vp = (Vertex *) cvp;
                  pq.push({vp, availibleSlots.at(vp)});
              }
              // We track used FUs for each mod-slot so we can compare the used FUs from the last iteration to the used
              // FUs of the current iteration. If used FUs of the previous iteration are the same as the used FUs in
              // Current iteration, we know that we can not satisfy the resource constraint, and we have to increase
              // min. Latency
              int usedFUsAtActualSlot = 0;

              // While-Loop to reduce used FUs until resource constraints are satisfied or no further reduction of used
              // FUs is possible.
              while (usedFuInModslot.at(x) > r->getLimit()) {
                  if (usedFuInModslot.at(x) == usedFUsAtActualSlot) {
                      increment = (int)floor(usedFuInModslot.at(x)/r->getLimit()) - r->getLimit();
                      if (increment < 1){
                          increment = 1;
                      }
                      cout << "break equal" << endl;
                      break;
                  }
                  usedFUsAtActualSlot = usedFuInModslot.at(x);
                  // Operation with most available slot will be moved away from the actual slot, to the slot with
                  // least amount uf used FUs.
                  while(!pq.empty()) {
                      if (usedFuInModslot.at(x) <= r->getLimit()){
                          break;
                      }
                      auto currOp = pq.top();
                      pq.pop();
                      Vertex *y = currOp.first;
                      if (!vertexTimeslot.at({y, x})){
                          continue;
                      }
                      priority_queue<pair<int, int>, vector<pair<int, int>>, SmtIntIntComp> fspq;
                      for (int i = 0; i < min((int)usedFuInModslot.size(), minLatency); i++) {
                          int temp = (r->getLimit() - usedFuInModslot.at(i)) * (int) vertexTimeslot.at({y, i});
                          fspq.push({i, temp});
                      }
                      while (!fspq.empty()) {
                          auto curr = fspq.top();
                          fspq.pop();
                          if (curr.first == x) {
                              continue;
                          }
                          if (vertexTimeslot.at({y, curr.first})) {
                              if (vertexTimeslot.at({y, x})) {
                                  vertexTimeslot.at({y, x}) = false;
                                  usedFuInModslot.at(x)--;
                                  availibleSlots.at(y)--;
                              }
                          }
                      }
                  }
              }
              // Check if resource constraints can be satisfied
              if (usedFuInModslot.at(x) > r->getLimit()){
                  //If not, break and increase latency
                  cout << "Conflict at " << x << " Used FUs: " << usedFuInModslot.at(x) << endl;
                  cout << "Not Feasiable... Increase Latency!" << endl;
                  return false;
              }else{
                  checkedResource.at(r) = true;
                  for (auto &cvp : verticesOfThisResource){
                      auto vp = (Vertex*)cvp;
                      if (vertexTimeslot.at({vp, x})){
                          for (int i = 0; i < min(currentII, minLatency); i++){
                              if (i == x){
                                  continue;
                              }
                              if (vertexTimeslot.at({vp, i})) {
                                  vertexTimeslot.at({vp, i}) = false;
                                  availibleSlots.at(vp)--;
                                  usedFuInModslot.at(i)--;
                              }
                          }
                      }
                  }
              }
          }
      }

      return true;
  }

  pair <map<Vertex*,int>, map<Vertex*, int>> SMTUnaryScheduler::calcAsapAndAlapModScheduleWithSdc(Graph &g, ResourceModel &resM) {

      // Create SDC-Graphs (Transposed with Helper for ALAP-Starttimes
      // and original with Helper for ASAP-Starttimes) as well as a mapping
      // to the Vertices of our original Graph:
      Graph sdcGraphAlap;
      Graph sdcGraphAsap;
      unordered_map<Vertex*, int> vertexLatencyAlap;
      unordered_map<Vertex*, int>vertexDistanceAlap;
      unordered_map<Vertex*, Vertex*> oldToNewVertexAlap;
      unordered_map<Vertex*, Vertex*> newToOldVertexAlap;
      unordered_map<Vertex*, int> vertexLatencyAsap;
      unordered_map<Vertex*, int> vertexDistanceAsap;
      unordered_map<Vertex*, Vertex*> oldToNewVertexAsap;
      unordered_map<Vertex*, Vertex*> newToOldVertexAsap;

      for (auto &v : g.Vertices()){
          Vertex* vSdcAlap = &sdcGraphAlap.createVertex(v->getId());
          Vertex* vSdcAsap = &sdcGraphAsap.createVertex(v->getId());
          vSdcAlap->setName(v->getName());
          vSdcAsap->setName(v->getName());
          vertexLatencyAlap[vSdcAlap] = resM.getVertexLatency(v);
          vertexLatencyAsap[vSdcAsap] = resM.getVertexLatency(v);
          oldToNewVertexAlap[v] = vSdcAlap;
          oldToNewVertexAsap[v] = vSdcAsap;
          newToOldVertexAlap[vSdcAlap] = v;
          newToOldVertexAsap[vSdcAsap] = v;
      }

      for (auto &e : g.Edges()){
          auto vSrcAlap = oldToNewVertexAlap.at(&e->getVertexSrc());
          auto vSrcAsap = oldToNewVertexAsap.at(&e->getVertexSrc());
          auto vDstAlap = oldToNewVertexAlap.at(&e->getVertexDst());
          auto vDstAsap = oldToNewVertexAsap.at(&e->getVertexDst());
          sdcGraphAlap.createEdge(*vDstAlap, *vSrcAlap, -vertexLatencyAlap.at(vSrcAlap)
                                                        - e->getDelay()
                                                        + e->getDistance() * (int)this->II);
          sdcGraphAsap.createEdge(*vSrcAsap, *vDstAsap, -vertexLatencyAsap.at(vSrcAsap)
                                                        - e->getDelay()
                                                        + e->getDistance() * (int)this->II);

      }

      //Bellmann-Ford Step 1:
      //Create Helper Vertex and connect it to all other Vertices with distance 0:
      //Initialization of Bellmann-Ford Algorithm with INT_MAX for all Vertices except Helper
      //to solve the SDC-System:
      Vertex& helperAlap = sdcGraphAlap.createVertex();
      Vertex& helperAsap = sdcGraphAsap.createVertex();
      helperAlap.setName("Helper" + to_string(helperAlap.getId()));
      helperAsap.setName("Helper" + to_string(helperAsap.getId()));
      for (auto &v : sdcGraphAlap.Vertices()){
          if (v == &helperAlap){
              vertexDistanceAlap[&helperAlap] = 0;
              continue;
          }
          sdcGraphAlap.createEdge(helperAlap, *v, -resourceModel.getVertexLatency(newToOldVertexAlap.at(v)));
          vertexDistanceAlap[v] = INT32_MAX;
      }
      for (auto &v : sdcGraphAsap.Vertices()){
          if (v == &helperAsap){
              vertexDistanceAsap[&helperAsap] = 0;
              continue;
          }
          sdcGraphAsap.createEdge(helperAsap, *v, 0);
          vertexDistanceAsap[v] = INT32_MAX;
      }

      //Bellmann-Ford Step 2:
      //Searching shortest path:
      //A simple shortest path from src to any other vertex can have
      //at-most |V| - 1 edges
      try {
          unsigned int numOfVerticesAlap = sdcGraphAlap.getNumberOfVertices();
          for (int i = 0; i < numOfVerticesAlap - 1; i++) {
              for (auto &e : sdcGraphAlap.Edges()) {
                  auto t = &e->getVertexSrc();
                  auto u = &e->getVertexDst();
                  int weight = e->getDistance();
                  if ((vertexDistanceAlap.at(t) != INT32_MAX) &&
                      (vertexDistanceAlap.at(t) + weight < vertexDistanceAlap.at(u))) {
                      vertexDistanceAlap.at(u) = vertexDistanceAlap.at(t) + weight;
                  }
              }
          }
      }catch (std::out_of_range&) {
          throw (std::out_of_range("SMT_Sheduler: calcAsapAndAlapModScheduleWithSdc(), Bellmann-Ford, Step 2, ALAP Loop"));
      }
      try{
          unsigned int numOfVerticesAsap = sdcGraphAsap.getNumberOfVertices();
          for (int i = 0; i < numOfVerticesAsap - 1; i++) {
              for (auto &e : sdcGraphAsap.Edges()) {
                  auto t = &e->getVertexSrc();
                  auto u = &e->getVertexDst();
                  int weight = e->getDistance();
                  if ((vertexDistanceAsap.at(t) != INT32_MAX) &&
                      (vertexDistanceAsap.at(t) + weight < vertexDistanceAsap.at(u))) {
                      vertexDistanceAsap.at(u) = vertexDistanceAsap.at(t) + weight;
                  }
              }
          }
      } catch (std::out_of_range&) {
          throw (std::out_of_range("SMT_Sheduler: calcAsapAndAlapModScheduleWithSdc(), Bellmann-Ford, Step 2, ASAP Loop"));
      }

      //Checking for negativ Cycles:
      for (auto &e : sdcGraphAlap.Edges()){
          auto t = &e->getVertexSrc();
          auto u = &e->getVertexDst();
          int weight = e->getDistance();
          if ((vertexDistanceAlap.at(t) != INT32_MAX) && (vertexDistanceAlap.at(t) + weight < vertexDistanceAlap.at(u))){
              cout << "Negative Cycle Detected ALAP:" << endl;
              cout << "Infeasibility without Ressource Constraints: Check if II is correct." << endl;
              throw(HatScheT::Exception("SMTUnaryScheduler::calcAsapAndAlapModScheduleWithSdc()"));
          }
      }
      for (auto &e : sdcGraphAsap.Edges()){
          auto t = &e->getVertexSrc();
          auto u = &e->getVertexDst();
          int weight = e->getDistance();
          if ((vertexDistanceAsap.at(t) != INT32_MAX) && (vertexDistanceAsap.at(t) + weight < vertexDistanceAsap.at(u))){
              cout << "Negative Cycle Detected ASAP" << endl;
              cout << "Infeasibility without Ressource Constraints: Check if II is correct." << endl;
              throw(HatScheT::Exception("SMTUnaryScheduler::calcAsapAndAlapModScheduleWithSdc()"));
          }
      }

      /*if ( !quiet ) {
          cout << "Transposed Helper Graph:" << endl << sdcGraphAlap << endl;
          cout << "Helper Graph:" << endl << sdcGraphAsap << endl;
      }*/

      auto min = std::min_element(vertexDistanceAlap.begin(), vertexDistanceAlap.end(), [](const pair<Vertex*,int> &x, const pair<Vertex*, int> &y) {
        return x.second < y.second;
      })->second;

      map<Vertex*, int> startTimes_alap;
      map<Vertex*, int> startTimesAsap;

      //if ( !quiet ) { cout << "Latest possible Starttimes:" << endl; }
      for (auto &it : vertexDistanceAlap) {
          //Removing HELPER_ALAP-Vertex from ALAP-Starttimes:
          if (it.first == &helperAlap){
              continue;
          }
          it.second += abs(min);
          startTimes_alap[newToOldVertexAlap[it.first]]=it.second;
          //if ( !quiet) { cout << it.first->getName() << ": " << it.second << endl; }
      }

      //Checking if we are lucky and MOD-ALAP-Starttimes are already a valig modulo schedule:
      auto validAlap = verifyModuloScheduleSMT(g, resM, startTimes_alap, (int)this->II);
      string validstr;
      if ( !quiet ) {
          if (validAlap) { validstr = "True"; } else { validstr = "False"; }
          cout << "---------------------------------------------------------" << endl;
          cout << "Valid Mod. Schedule for given Ressources: " << validstr << " / II: " << (int)this->II << endl;
          cout << "---------------------------------------------------------" << endl;
          //cout << "Earliest possible Starttimes:" << endl;
      }
      for (auto &it : vertexDistanceAsap) {
          //Removing HELPER_ASAP-Vertex from ALAP-Starttimes:
          if (it.first == &helperAsap) {
              continue;
          }
          it.second = abs(it.second);
          startTimesAsap[newToOldVertexAsap[it.first]] = it.second;
          //if (!quiet) { cout << it.first->getName() << ": " << it.second << endl; }
      }

      //Checking if we are lucky and MOD-ASAP-Starttimes are already a valig modulo schedule:
      auto validAsap = verifyModuloScheduleSMT(g, resM, startTimesAsap, (int)this->II);
      if (!quiet) {
          if (validAsap) { validstr = "True"; } else { validstr = "False"; }
          cout << "---------------------------------------------------------" << endl;
          cout << "Valid Mod. Schedule for given Ressources: " << validstr << " / II: " << (int)this->II << endl;
          cout << "---------------------------------------------------------" << endl;
      }

      auto success = vertexDistanceAlap.erase(&helperAlap);
      if ( !quiet and success){
          cout << "Successful deleted Helper_Vertex_ALAP: True" << endl;
      }else if ( !quiet ){
          cout << "Successful deleted Helper_Vertex_ALAP: False" << endl;
      }
      success = vertexDistanceAsap.erase(&helperAsap);
      if ( !quiet and success){
          cout << "Successful deleted Helper_Vertex_ASAP: True" << endl;
      }else if ( !quiet ){
          cout << "Successful deleted Helper_Vertex_ASAP: False" << endl;
      }

      //Saving schedule length
      if (!quiet) { cout << endl << "MOD_ALAP: " << endl; }
      modAlapLength = getScheduleLatency(vertexDistanceAlap, newToOldVertexAlap);
      if (!quiet) { cout << endl << "MOD_ASAP: " << endl; }
      modAsapLength = getScheduleLatency(vertexDistanceAsap, newToOldVertexAsap);
      if (!quiet) { cout << "SDC-ALAP-Latency: " << modAlapLength << endl; }
      if (!quiet) { cout << "SDC-ASAP-Latency: " << modAsapLength << endl; }
      if (!quiet) { cout << endl; }
      if (modAsapLength != modAlapLength ){
          throw (HatScheT::Exception("smtbased-Scheduler, calcAsapAndAlapModScheduleWithSdc(), modAlapLength and not equal"));
      }
      minLatency = modAsapLength;

      //If a valid schedule is already found, just returning it.
      if (validAsap and sPref == schedulePreference::MOD_ASAP) {
          scheduleFound = true;
          startTimes = startTimesAsap;
      } else if (validAlap and sPref == schedulePreference::MOD_ALAP) {
          scheduleFound = true;
          startTimes = startTimes_alap;
      } else if (validAsap){
          scheduleFound = true;
          startTimes = startTimesAsap;
      } else if (validAlap){
          scheduleFound = true;
          startTimes = startTimes_alap;
      }

      //If no valid schedule is found, we continue our latency estimation with the found
      //MOD-ASAP- and MOD-ALAP-Starttimes
      return {startTimesAsap, startTimes_alap};
  }

  int SMTUnaryScheduler::getScheduleLatency(unordered_map<Vertex *, int> &vertexLatency,
                                            unordered_map<Vertex *, Vertex *> &newToOld) {
      int maxTime = -1;
      for (std::pair<Vertex *, int> vtPair : vertexLatency) {
          try {
              Vertex *v = vtPair.first;
              if ((vtPair.second + resourceModel.getVertexLatency(newToOld.at(v))) > maxTime) {
                  maxTime = (vtPair.second + resourceModel.getVertexLatency(newToOld.at(v)));
                  //Debugging:
                  if (!quiet) {
                      cout << vtPair.first->getName() << ": " << vtPair.second << " + "
                           << resourceModel.getVertexLatency(newToOld.at(v)) << " = " << maxTime << endl;
                  }
              }
          }catch(std::out_of_range&){
              cout << vtPair.first->getName() << ": " << vtPair.second << endl;
              throw (HatScheT::Exception("SMTUnaryScheduler::getScheduleLatency() OUT_OF_RANGE"));
          }
      }
      return maxTime;
  }

  //Copied from Verifier to get rid of the CERR outputs. No one likes errors!
  bool SMTUnaryScheduler::verifyModuloScheduleSMT(Graph &g, ResourceModel &rm, std::map<Vertex *, int> &schedule, int II)
  {
      if(II<=0){
          if ( !quiet ) { cout << "HatScheT.verifyModuloSchedule Error invalid II passed to verifier: "  << II << endl; }
          return false;
      }
      if(schedule.empty()){
          if ( !quiet ) { cout << "HatScheT.verifyModuloSchedule Error empty schedule provided to verifier!"  << endl; }
          return false;
      }
      auto &S = schedule; // alias
      bool ok;

      /* 1) precedence edges */
      for (auto it = g.edgesBegin(), end = g.edgesEnd(); it != end; it++) {
          auto e = *it;
          auto i = &e->getVertexSrc();
          auto j = &e->getVertexDst();

          ok = S[i] + rm.getVertexLatency(i) + e->getDelay() <= S[j] + e->getDistance() * II;
          if (! ok) {
              if ( !quiet ) { cout << *e << " violated: " << S[i] << " + " << rm.getVertexLatency(i)
                                   << " + " << e->getDelay() << " <= " << S[j] << " + " << e->getDistance()
                                   << "*" << II << endl; }
              return false;
          }
      }

      /* 2) modulo resource constraints */
      vector<map<const Resource *, vector<Vertex *>>> congruenceClassesByRes(II);
      for (auto it = g.verticesBegin(), end = g.verticesEnd(); it != end; it++) {
          auto v = *it;
          congruenceClassesByRes[ /* modulo slot: */ S[v] % II         ]
          [ /* resource:    */ rm.getResource(v) ]
              .push_back(v);
      }
      for (int m = 0; m < II; ++m) {
          auto &cc = congruenceClassesByRes[m];
          for (auto &entry : cc) {
              auto res = entry.first;
              auto &vs  = entry.second;

              ok = res->getLimit() == UNLIMITED || vs.size() <= res->getLimit();
              if (! ok) {
                  if ( !quiet ) { cout << "The following " << vs.size() << " vertices violate the resource limit "
                                       << res->getLimit() << " for " << res->getName() << " in congruence class "
                                       << m << " (mod " << II << "):" << endl; }
                  for (auto v : vs){
                      if (!quiet) { cout << *v << " (t=" << S[v] << ")" << endl; }
                  }
                  return false;
              }
          }
      }

      /* 3) TODO: cycle-time constraints are obeyed */

      return true;
  }

  /*!----------------------------------------*
   * Functions to add constraints to solver: *
   * ----------------------------------------*/
  void SMTUnaryScheduler::generateBvariables() {
      bVariables.clear();
      for (auto &it : g.Vertices()){
          for (int i = 0; i <= candidateLatency; i++){
              auto key = std::make_pair(it, i);
              std::stringstream name;
              name << it->getName() << "->" << i;
              z3::expr e(c.bool_const(name.str().c_str()));
              bVariables.insert({key, e});
          }
      }
  }

  z3::expr *SMTUnaryScheduler::getBvariable(Vertex *v, int i) {
      auto key = std::make_pair(v, i);
      try {
          return &bVariables.at(key);
      }catch(std::out_of_range&){
          cout << "Out_of_Range: " << v->getName() << " - " << i << endl;
          throw (HatScheT::Exception("smtbased Scheduler: getBvariable std::out_of_range"));
      }
  }

  z3::check_result SMTUnaryScheduler::addOneSlotConstraintsToSolver() {
      for (auto &it : g.Vertices()) {

          vector<int> coefficients;
          coefficients.reserve(candidateLatency + 1 );
          z3::expr_vector b_expressions(c);
          for (int i = 0; i <= candidateLatency; i++) {
              if (!startTimesSimplification.at(std::make_pair((Vertex *)it, i))){
                  continue;
              }
              coefficients.push_back(1);
              b_expressions.push_back(*getBvariable(it, i));
          }
          if (!b_expressions.empty()) {
              this->s.add(z3::pbeq(b_expressions, &coefficients[0], 1));
          }
          coefficients.clear();
          coefficients.shrink_to_fit();
      }
      startTimeTracking();
      z3Check();
      endTimeTracking();
      return this->getZ3Result();
  }

  z3::check_result SMTUnaryScheduler::addResourceLimitConstraintToSolver(int candidateII) {
      for (auto &it : resourceModel.Resources()) {
          if (it->isUnlimited()) {
              continue;
          }
          for (int i = 0; i < candidateII; i++) {
              set<const Vertex *> vSet = resourceModel.getVerticesOfResource(it);
              z3::expr_vector b_expressions(c);
              for (auto &vIt : vSet) {
                  for (int j = 0; j <= candidateLatency; j++) {
                      if ((j % candidateII) != i) {
                          continue;
                      }
                      if (!startTimesSimplification.at(std::make_pair((Vertex*)vIt, j))){
                          continue;
                      }
                      b_expressions.push_back(*getBvariable((Vertex *) vIt, j));
                  }
              }
              if (!b_expressions.empty()) {
                  this->s.add(z3::atmost(b_expressions, it->getLimit()));
              }
          }
      }
      startTimeTracking();
      z3Check();
      endTimeTracking();
      return this->getZ3Result();
  }

  void SMTUnaryScheduler::prohibitToEarlyStartsAndAdd() {
      startTimesSimplification.clear();
      for (auto &v : g.Vertices()){
          for (int i = 0; i <= candidateLatency; i++){
              if (i < earliestStartTimes.at(v)){
                  startTimesSimplification[std::make_pair(v, i)] = false;
                  //s.add(!*getBvariable(v, i));
              }else{
                  startTimesSimplification[std::make_pair(v, i)] = true;
              }
          }
      }
  }

  void SMTUnaryScheduler::prohibitToLateStartsAndAdd() {
      for (auto &v : g.Vertices()){
          for (int i = 0; i <= candidateLatency; i++){
              if (i > latestStartTimesUpdated.at(v)){
                  startTimesSimplification.at(std::make_pair(v, i)) = false;
                  //s.add(!*getBvariable(v, i));
              }
          }
      }
  }

  void SMTUnaryScheduler::updateLatestStartTimes() {
      latestStartTimesUpdated.clear();
      for (auto &it: latestStartTimes) {
          this->latestStartTimesUpdated[it.first] = this->candidateLatency - (modAsapLength - it.second);
      }
  }

  void SMTUnaryScheduler::setDependencyConstraintsAndAddToSolver(const int &candidateII) {
      //tj + distance * this->candidateII - ti - lSrc - delay >= 0
      int i = 0;
      deque<long> sTimes;
      clock_t start, end;
      int x = 0;
      for (auto &e : g.Edges()) {
          x++;
          auto *vSrc = &e->getVertexSrc();
          auto *vDst = &e->getVertexDst();
          auto lSrc = this->resourceModel.getVertexLatency(vSrc);
          auto distance = e->getDistance();
          auto delay = e->getDelay();
/*          if (!this->quiet) {
              std::cout << "SMTScheduler: creating dependency constraint for edge '" << e->getId() << "': '" << vSrc->getName() << "' -> '" << vDst->getName() << "'" << std::endl;
              cout << "Source: " << earliestStartTimes.at(vSrc) << " / " << latestStartTimesUpdated.at(vSrc) << endl;
              cout << "Destination: " << earliestStartTimes.at(vDst) << " / " << latestStartTimesUpdated.at(vDst) << endl;
          }*/
          for (int ti = earliestStartTimes.at(vSrc); ti <= latestStartTimesUpdated.at(vSrc); ti++) {
              for (int tj = earliestStartTimes.at(vDst); tj <= latestStartTimesUpdated.at(vDst); tj++) {
                  if (tj + distance * candidateII - ti - lSrc - delay >= 0) {
                      //No Conflict... Do nothing
                      continue;
                  }
                  //Those cases are not needed since one of these conditions is already prohibited
                  //and we can use the law of absorption : (!a + !b) * !a = !a.
                  if (!startTimesSimplification.at(std::make_pair(vSrc, ti)) ||
                      !startTimesSimplification.at(std::make_pair(vDst, tj))) {
                      continue;
                  }
                  //Conflict... Not both Operations in this timeslot;
                  this->s.add(!*getBvariable(vSrc, ti) || !*getBvariable(vDst, tj));
              }
          }
      }
  }


  void SMTUnaryScheduler::setDependencyConstraintsAndAddToSolverBIG(const int &candidateII) {
      //tj + distance * this->candidateII - ti - lSrc - delay >= 0
      if (!quiet) { cout << g.getNumberOfEdges() << " Edges... " << endl; }
      deque<long> sTimes;
      z3::check_result satisfiable = z3::unknown;
      clock_t start, end;
      int eCount = 0;
      for (auto &e : g.Edges()) {
          eCount++;
          auto *vSrc = &e->getVertexSrc();
          auto *vDst = &e->getVertexDst();
          auto lSrc = this->resourceModel.getVertexLatency(vSrc);
          auto distance = e->getDistance();
          auto delay = e->getDelay();
          z3::expr_vector ev(c);
          vector<int> factors; // Gewichte für Dependency Constraint
          for (int i = 0; i <= candidateLatency; i++) {
              if (!startTimesSimplification.at({vSrc, i})) {
                  continue;
              }
              factors.push_back(-i);
              ev.push_back(*getBvariable(vSrc, i));
          }
          for (int j = 0; j <= candidateLatency; j++) {
              if (!startTimesSimplification.at({vDst, j})) {
                  continue;
              }
              factors.push_back(j);
              ev.push_back(*getBvariable(vDst, j));
          }
          if (ev.size() != factors.size() or ev.empty()) {
              cout << "ev is empty: " << ev.empty() << endl;
              throw (std::out_of_range("Vector sizes not equal!"));
          }
          this->s.add(z3::pbge(ev, &factors.at(0), (lSrc + delay - distance * candidateII))); // Dependency Constraint
      }

      //startTimesSimplification.clear();
  }

  /*!--------------------------*
   * Latency-Search algorithms *
   * --------------------------*/
  void SMTUnaryScheduler::calcLatencySpace() {
      latencySpace.clear();
      latencySpace.shrink_to_fit();
      if (minLatency < (int)minII){
          minLatency = (int)minII;
      }
      latencySpace.reserve((maxLatency - minLatency) + 1);
      for (int i = minLatency; i <= maxLatency; i++){
          latencySpace.push_back(i);
      }
  }

  int SMTUnaryScheduler::latLinearSearch(z3::check_result result) {
      try {
          if (linearSearchInit) {
              latencySpaceIndex = 0;
              linearSearchInit = false;
          }
          if (result == z3::sat) {
              return -1;
          }
          if (latencySpaceIndex == latencySpace.size()) {
              latencySpaceIndex = 0;
              return -1;
          }
          int temp = latencySpaceIndex;
          latencySpaceIndex++;
          return latencySpace.at(temp);
      }catch (std::out_of_range&){
          cout << "Index: " << latencySpaceIndex << endl;
          cout << "Size: " << latencySpace.size() << endl;
          int i = 0;
          for (auto &it : latencySpace){
              cout << i << ": " << it << endl;
              i++;
          }
          throw(HatScheT::Exception("SMTUnaryScheduler::latLinearSearch() -- Out of Range"));
      }
  }

  int SMTUnaryScheduler::latBinarySearch(z3::check_result result) {
      try {
          if (binarySearchInit) {
              if (!quiet) { cout << "Init Binary Search..." << endl; }
              binarySearchInit = false;
              latencySpaceIndex = (int) latencySpace.size() / 2;
              leftIndex = 0;
              rightIndex = (int) latencySpace.size() - 1;
              return latencySpace.at(latencySpaceIndex);
          }
          if (leftIndex < rightIndex) {
              switch (result) {
                  case z3::sat:
                      rightIndex = latencySpaceIndex;
                      latencySpaceIndex = leftIndex + (rightIndex - leftIndex) / 2;
                      break;
                  case z3::unsat:
                      leftIndex = latencySpaceIndex + 1;
                      latencySpaceIndex = leftIndex + (rightIndex - leftIndex) / 2;
                      if (latencySpaceIndex > (int)latencySpace.size() - 1){
                          latencySpaceIndex = (int)latencySpace.size() - 1;
                      }
                      break;
                  case z3::unknown:
                      leftIndex = latencySpaceIndex + 1;
                      latencySpaceIndex = leftIndex + (rightIndex - leftIndex) / 2;
                      break;
              }
              return latencySpace.at(latencySpaceIndex);
          }
          binarySearchInit = true;
          return -1;
      }catch (std::out_of_range&){
          for (auto &it : latencySpace){
              cout << it << " ";
          }
          cout << endl << "LAT Space Index: " << latencySpaceIndex << endl;
          throw (HatScheT::Exception("SMTUnaryScheduler: Binary Latency Search: STD::OUT_OF_RANGE"));
      }
  }

  /*!-------------------------------*
   * Parsing schedule from z3-model *
   *--------------------------------*/
  void SMTUnaryScheduler::parseSchedule(z3::model &m) {
      for (auto &it : g.Vertices()){
          for (int i = 0; i <= candidateLatency; i++) {
              auto val = m.eval(*getBvariable(it, i));
              if (val.is_true()){
                  this->startTimes[it] = i;
              }
          }
      }
  }

  /*!-------------*
   * Solver Setup *
   *--------------*/
  void SMTUnaryScheduler::setSolverTimeout(double seconds) {
      this->solverTimeout = seconds;
      this->setZ3Timeout((uint32_t)seconds);
  }

  /*!--------------------------------*
   * Print and Debugging Functions:  *
   * --------------------------------*/
  void SMTUnaryScheduler::print_b_variables() {
      for (auto &it:g.Vertices()){
          for (int i = 0; i <= candidateLatency; i++){
              cout << *getBvariable(it, i) << endl;
          }
      }
  }

  void SMTUnaryScheduler::print_latency_space(int l_index, int r_index) {
      for (int i = l_index; i < r_index; i++){
          cout << latencySpace.at(i) << " ";
      }
      cout << endl;
  }

  void SMTUnaryScheduler::print_solution(z3::model &m) {
      for (auto &it : g.Vertices()){
          for (int i = 0; i <= candidateLatency; i++) {
              auto val = m.eval(*getBvariable(it, i));
              if (val.is_true()){
                  cout << it->getName() << " - " << i << ": " <<  val << endl;
              }
          }
      }
  }

  void SMTUnaryScheduler::print_ASAP_ALAP_restictions() {
      for (auto &it : startTimesSimplification){
          cout << it.first.first->getName() << " <" << it.first.second << "> " << it.second << endl;
      }
  }

  void SMTUnaryScheduler::writeSolvingTimesToFile(deque<double> &times, int x) {
      if (!designName.empty()) {
          bool skip = true;
          reverse(designName.begin(), designName.end());
          cout << designName << endl;
          for (int i = 0; i < designName.size(); i++) {
              cout << designName.at(i) << endl;
              if (designName.at(i) == '/' and skip) {
                  designName.at(i) = '_';
                  skip = false;
              } else if (designName.at(i) == '/' and !skip) {
                  designName = designName.substr(4, i - 4);
                  break;
              }
          }
          reverse(designName.begin(), designName.end());

          string path = "SMTBenchmark/Solving_Times_" + designName + "_" + to_string(x) + ".csv";
          cout << path << endl;
          try {
              ofstream of;
              of.open(path);
              if (of.is_open()) {
                  of << "Latency,SolvingTime,\n";
                  int j = 0;
                  for (auto &it : times) {
                      of << j << "," << it << ",\n";
                      j++;
                  }
                  of.close();
              } else {
                  throw (HatScheT::Exception("File not created"));
              }
          } catch (std::ofstream::failure &writeErr) {
              cout << path << endl;
              throw (HatScheT::Exception("File not created"));
          }
      }
  }

  void SMTUnaryScheduler::printPossibleStarttimes(map<pair<Vertex*, int>, bool>& vertex_timeslot) {
      cout << "MinLAT: " << minLatency << " modASAPLength: " << modAsapLength << endl;
      for (auto &r : resourceModel.Resources()) {
          if (r->isUnlimited()) {
              continue;
          }
          for (auto &v : resourceModel.getVerticesOfResource(r)) {
              cout << setw(18) << v->getName() << ": ";
              for (int i = 0; i < minLatency; i++) {
                  if (i % (int)this->II == 0) {
                      cout << " ";
                  }
                  cout << vertex_timeslot.at(std::make_pair((Vertex *) v, i));
              }
              cout << endl;
          }
          cout << endl;
      }
  }

  z3::check_result SMTUnaryScheduler::test_binary_search(int value_to_check, int target_value) {
      if (value_to_check < target_value){
          return z3::unsat;
      }
      return z3::sat;
  }

  void SMTUnaryScheduler::endTimeTracking() {
      this->end_t = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_t - start_t).count();
      solvingTimePerIteration += (double) duration / 1000000;
      timeRemaining = solverTimeout - solvingTimePerIteration;
  }

  map <Vertex*, pair<int, int>> SMTUnaryScheduler::printVertexStarttimes() {

      map <Vertex*, pair<int, int>> returnmap;

      cout << "Earliest and Latest Starttimes:" << endl;
      for (auto v : g.Vertices()){
          try {
              //cout << v->getName() << ": " << earliestStartTimes.at(v) << " / " << latestStartTimesUpdated.at(v)
              //     << endl;
              returnmap.insert({v, {earliestStartTimes.at(v), latestStartTimesUpdated.at(v)}});
          }catch (std::out_of_range&){
              cout << "Who Cares!" << endl;
          }
      }
      return returnmap;
  }

}
#endif

