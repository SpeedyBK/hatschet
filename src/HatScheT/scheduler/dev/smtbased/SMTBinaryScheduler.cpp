//
// Created by bkessler on 5/7/22.
//

#ifdef USE_Z3
#include "SMTBinaryScheduler.h"
#include "HatScheT/utility/Exception.h"
#include "HatScheT/utility/Utility.h"

#include "HatScheT/scheduler/ASAPScheduler.h"

#include <z3++.h>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <chrono>
#include <cmath>
#include <deque>
#include <algorithm>
#include <unordered_map>

namespace HatScheT {

  SMTBinaryScheduler::SMTBinaryScheduler(Graph &g, ResourceModel &resourceModel) : SchedulerBase(g, resourceModel) {
      // reset previous solutions
      this->timeouts = 0;
      startTimes.clear();
      scheduleFound = false;
      computeMinII(&g, &resourceModel);
      minII = ceil(minII);
      computeMaxII(&g, &resourceModel);

      for (int i = (int)minII; i <= (int)maxII+1; i++){
          this->iiSpace.push_back(i);
      }
      this->iiSpaceIndex = 0;

      this->sPref = schedulePreference::MOD_ASAP;
      this->latSM = latSearchMethod::LINEAR;
      this->quiet = true;
      this->latencySpaceIndex = 0;
      this->candidateLatency = -1;
      this->minLatency = 0;
      this->maxLatency = -1;
      this->maxRuns = INT32_MAX;
      this->leftIndex = 0;
      this->rightIndex = 0;

      this->binarySearchInit = true;
      this->linearSearchInit = true;
      this->iiSearchInit = true;

      this->modAsapLength = 0;
      this->modAlapLength = 0;
      this->increment = 0;

      this->designName = "";
      this->timeLimit = -1;
      this->timeBudget = INT32_MAX;
      firstObjectiveOptimal = true;
      secondObjectiveOptimal = true;

  }

  /*!--------------------------*
   * Main scheduling function: *
   * --------------------------*/
  void SMTBinaryScheduler::schedule() {

      if(!quiet) {
          cout << "Rec-Min-II: " << recMinII << endl;
          cout << "Res-Min-II: " << resMinII << endl;
          cout << "Max. II: " << maxII << endl;
      }

      //Saving times:
      auto start_t = std::chrono::high_resolution_clock::now();
      auto end_t = std::chrono::high_resolution_clock::now();
      deque<double>solving_times;

      //Setting up z3:
      z3::solver s(c);
      auto sati = z3::unknown;
      z3::model m(c);
      z3::params p(c);
      if (timeLimit > 0){
          timeBudget = timeLimit;
      }
      if (timeBudget > 0) { p.set("timeout", (uint32_t)timeBudget); }
      //p.set("arith.print_stats", true);
      if (!quiet) { cout << p << endl; }
      s.set(p);

      //Setting up stuff for scheduling loop
      int candidateII = 0;
      int candidateIIOLD;
      int i = 0;

      //Scheduling loop incremataly searching for a feasible II.
      //Stops if a feasible II is found, or a limit set by the user is reached.
      while (i < maxRuns) {
          //Incremental searching for II
          candidateIIOLD = candidateII;
          candidateII = iiLinearSearch(sati);
          secondObjectiveOptimal = true;

          //If feasible or iiSpace exhausted -> break
          if (candidateII == -1){
              break;
          }

          //Calculating minimum latency and maximum latency-estimation:
          calcLatencyEstimation();
          //If valid schedule already found, just return.
          if (scheduleFound){ return; }
          //Setting up latency-search-space for latency-search-loop
          calcLatencySpace();

          //Latency-search-loop
          while (true){
              if ( !quiet ) { cout << "*--------------------------*" << endl; }
              if ( !quiet ) { cout << "* Start scheduling attempt *" << endl; }
              if ( !quiet ) { cout << "*--------------------------*" << endl; }
              int oldLatency = candidateLatency;
              if (latSM == latSearchMethod::LINEAR) { candidateLatency = latLinearSearch(sati); }
              if (latSM == latSearchMethod::BINARY) { candidateLatency = latBinarySearch(sati); }

              if (candidateLatency == -1 or oldLatency == candidateLatency ) {
                  candidateLatency = 0;
                  break;
              }

              if (!quiet) { cout << "Trying II: " << candidateII << " Trying Latency: " << candidateLatency << endl; }
              //Updating latest start times with candidate latency
              updateLatestStartTimes(oldLatency);
              //Generating debendency constraints for z3-prover.
              if (!quiet) { cout << "Generating b-variables... " << endl; }
              generateBvariables();
              //if (!quiet) { print_b_variables(); }

              //Reseting solver and add timeout, before pushing in stuff.
              s.reset();
              s.set(p);

              //Reducing Seach Space.
              if (!quiet) { cout << "Prohibit early starts... " << endl; }
              prohibitToEarlyStartsAndAdd(s);
              if (!quiet) { cout << "Prohibit late starts... " << endl; }
              prohibitToLateStartsAndAdd(s);

              //Takeing a shortcut, probably nonsence...
              /*if (unsatCheckShortcut()){
                  sati = z3::unsat;
                  continue;
              }*/

              //Pushing assertions into z3-Prover
              if (!quiet) { cout << "Solving... " << endl; }

              if (!quiet) { cout << "Add one-slot-constraints... " << endl; }
              sati = addOneSlotConstraintsToSolver(s);
              if (!quiet) { cout << ">>" << sati << "<<" << endl; }

              if (!quiet) { cout << "Add resource-constraints... " << endl; }
              if (sati != z3::unsat) {
                  sati = addResourceLimitConstraintToSolver(s, candidateII);
                  if (!quiet) { cout << ">>" << sati << "<<" << endl; }
              }

              if (sati != z3::unsat) {
                  if (!quiet) { cout << "Add dependency-constraints... " << endl; }
                  sati = setDependencyConstraintsAndAddToSolver(s, candidateII);
                  if (!quiet) { cout << ">>" << sati << "<<" << endl; }
              }

              if (!quiet) { cout << "System of " << s.assertions().size() << " assertions... Final Check" << endl; }
              double t = 0;
              //Final Check, when all assertions are in.
              if (sati != z3::unsat) {
                  start_t = std::chrono::high_resolution_clock::now();
                  sati = s.check();
                  end_t = std::chrono::high_resolution_clock::now();
                  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_t - start_t).count();
                  t = (int)((double)duration / 1000);
                  timeBudget = timeBudget - t;
              }
              //Saving solving times per latency attempt.
              solving_times.push_back((double)t/1000);
              if (!quiet) {
                  cout << "-->" << sati << "<--" << endl;
                  cout << "Solving Time: " << fixed << (double) t / 1000 << setprecision(5) << " sec " << endl;
                  cout << "TimeBudget: " << timeBudget << endl;
                  cout << "*-------------------------*" << endl;
                  cout << "* Scheduling attempt done *" << endl;
                  cout << "*-------------------------*" << endl;
                  cout << endl;
              }
              if (timeBudget < 0){
                  if (sati == z3::sat){
                      //A schedule for given II is found, and II can only get worse,
                      //so we return the schedule and we are done.
                      m = s.get_model();
                      this->scheduleFound = true;
                      this->II = iiSpace.at(iiSpaceIndex);
                      parseSchedule(m);
                      return;
                  }else{
                      if (scheduleFound){
                          //If there is a schedule from an earlier iteration,
                          //We return this schedule.
                          return;
                      }
                      //No Schedule is found on this itteration and before, so we increment
                      //II and try again, time budget is set back to the original timeout.
                      if (timeLimit > 0) {
                          timeBudget = timeLimit;
                      }
                      firstObjectiveOptimal = false;
                      break;
                  }
              }
              //If a schedule has been found on the current itteration, we save it.
              //If we are on binary seach mode, we try to get a better schedule.
              //If we are on linear mode, the found schedule is the best one we can find, so we do a break here.
              if (sati == z3::sat){
                  m = s.get_model();
                  this->scheduleFound = true;
                  this->II = iiSpace.at(iiSpaceIndex);
                  parseSchedule(m);
                  if (latSM == latSearchMethod::LINEAR) {
                      break;
                  }
              }
          }
          i++;
      }

      II = candidateIIOLD;

      try {
          if (!verifyModuloScheduleSMT(this->g, this->resourceModel, startTimes, (int) II)) {
              throw (HatScheT::Exception("Schedule not valid!"));
          }
      }catch (HatScheT::Exception&){
          this->II = -1;
          cerr << "SMTBinaryScheduler::Schedule() : Schedule not valid!" << endl;
      }
  }

  /*!-------------------*
   * Latency Estimation *
   * -------------------*/
  void SMTBinaryScheduler::calcLatencyEstimation() {

      //Check if max latency was set by user
      if (this->maxLatencyConstraint >= 0) {
          maxLatency = maxLatencyConstraint;
          return;
      }
      this->maxLatency = 0;
      int resMaxLat = 0;

      //Getting the current II mainly for better readability
      int currentII;
      try {
          currentII = iiSpace.at(iiSpaceIndex);
      }catch (std::out_of_range&){
          for (auto &it : iiSpace){
              cout << it << " ";
          }
          cout << endl << "iiSpaceIndex:" << iiSpaceIndex << endl;
          throw (HatScheT::Exception("SMTbinaryScheduler: calcMaxLatencyEstimation() ... STD::OUT_OF_RANGE.."));
      }

      //Getting earliest and latest possible Starttimes for a modulo schedule
      auto aslap = calcAsapAndAlapModScheduleWithSdc(g, resourceModel);
      if (scheduleFound){ return; }
      earliestStartTimes = aslap.first;
      latestStartTimes = aslap.second;

      if ( !quiet ) {
          cout << "*------------------------*" << endl;
          cout << "* Max Latency Estimation *" << endl;
          cout << "*------------------------*" << endl;
      }

      auto startMaxTime = std::chrono::high_resolution_clock::now();
      calcMaxLatencyEstimation(currentII);
      if (maxLatency < currentII){
          maxLatency = currentII;
      }
      auto endMaxTime = std::chrono::high_resolution_clock::now();
      auto durationMaxTime = std::chrono::duration_cast<std::chrono::microseconds>(endMaxTime - startMaxTime).count();
      if (!quiet) { cout << "Done in " << (float)durationMaxTime/1000000 << " seconds." << endl << endl; }

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
      if (currentII > modAsapLength){
          minLatency = currentII;
          for (auto &it : aslap.second){
              it.second = it.second + (currentII - modAsapLength);
          }
      }

      while (true) {
          auto feasilbe = calcMinLatencyEstimation(aslap, currentII);
          if (feasilbe or (minLatency >= maxLatency)){
              cout << "Min Latency: " << minLatency << endl;
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
  }

  void SMTBinaryScheduler::calcMaxLatencyEstimation(int currentII) {

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

      maxLatency = maximum + modAsapLength;
      if ( !quiet ) { cout << "Max Latency Suggestion: " << maximum + modAsapLength << endl; }
  }

  bool SMTBinaryScheduler::calcMinLatencyEstimation(pair<map<Vertex*, int>, map<Vertex*, int>> &aslap, int currentII){

      //Algorithm to estimate min latency based on earliest and latest possible start times
      //as well as on ressource constraints.
      map<pair<Vertex*, int>, bool> vertexTimeslot;

      for (auto &r : resourceModel.Resources()){
          this->checkedResource[r] = false;
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

      // Schedules all operation with satisfied resource constraints. So we can get a requiered min. latency
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
                              cout << "SMTBinaryScheduler::calcMinLatencyEstimation: Out of Range, " << vp->getName()
                                   << " " << x << endl;
                              cout << "Min. Latency: " << minLatency << endl;
                              throw (HatScheT::Exception(
                                  "SMTBinaryScheduler::calcMinLatencyEstimation: Out of Range A"));
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
                              cout << "SMTBinaryScheduler::calcMinLatencyEstimation: Out of Range, " << vp->getName()
                                   << " " << x << endl;
                              cout << "Min. Latency: " << minLatency << endl;
                              throw (HatScheT::Exception(
                                  "SMTBinaryScheduler::calcMinLatencyEstimation: Out of Range B"));
                          }
                      }
                  }
              }
          }

          vector<int> usedFuInModslot;
          usedFuInModslot.resize(currentII);
          map<Vertex *, int> availibleSlots;

          //Zählen, wie viele FUs in jedem Zeitslot benutzt werden. (Potentiell)
          //Zählen wie viele potentielle Zeitslots existieren.
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
          //Überprüfen, ob Resourcenbeschränkungen eingehalten werden.
          for (int x = 0; x < min(currentII, minLatency); x++) {
              //cout << "Checking Slot: " << x << endl;
              if (usedFuInModslot.at(x) <= r->getLimit()) {
                  //Kein Konflikt alles gut.
                  continue;
              }
              //Potentieller Konflikt:
              //Mappings zwischen Operationen und deren Priorität herstellen.
              //Zudem werden die Operationen in einer Prioritätswarteschlange sortiert.
              //Je mehr mögliche slots es gibt, desto einfacher lässt sich diese operation wegschedulen.
              //Es wird also die Operation mit den meisten möglichen Slots gesucht und diese Operation dann
              //in den Slot mit der geringsten Resourcenauslastung einsortiert.
              priority_queue<pair<Vertex *, int>, vector<pair<Vertex *, int>>, SmtVertexIntCompLess> pq;
              for (auto &cvp : verticesOfThisResource) {
                  auto vp = (Vertex *) cvp;
                  pq.push({vp, availibleSlots.at(vp)});
              }
              //Benutzte Resourcen im aktuellen Zeitlot merken um festzustellen ob sich dieser weiter verringern lässt.
              //Ist dies nicht der Fall, kann das als Abbruchbedingung genutzt werden.
              int usedFUsAtActualSlot = 0;

              //While-Schleife zur Resourcenminimierung. Diese läuft entweder, bis sich keine Minimierung mehr erziehlen lässt,
              //oder bis die Resourcenlimits erreicht sind.
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
                  //Operation mit der größten Anzahl an möglichen Slots wird gesucht und aus dem aktuellen Slot weggescheduled.
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
              //Überprüfen of Resourcenbeschränkungen eingehalten werden können.
              if (usedFuInModslot.at(x) > r->getLimit()){
                  //Können nicht eingehalten werden, dann abbrechen und latenz erhöhen.
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

  pair <map<Vertex*,int>, map<Vertex*, int>> SMTBinaryScheduler::calcAsapAndAlapModScheduleWithSdc(Graph &g, ResourceModel &resM) {

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
                                                        + e->getDistance() * this->iiSpace.at(iiSpaceIndex));
          sdcGraphAsap.createEdge(*vSrcAsap, *vDstAsap, -vertexLatencyAsap.at(vSrcAsap)
                                                        - e->getDelay()
                                                        + e->getDistance() * this->iiSpace.at(iiSpaceIndex));

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
              throw(HatScheT::Exception("SMTBinaryScheduler::calcAsapAndAlapModScheduleWithSdc()"));
          }
      }
      for (auto &e : sdcGraphAsap.Edges()){
          auto t = &e->getVertexSrc();
          auto u = &e->getVertexDst();
          int weight = e->getDistance();
          if ((vertexDistanceAsap.at(t) != INT32_MAX) && (vertexDistanceAsap.at(t) + weight < vertexDistanceAsap.at(u))){
              cout << "Negative Cycle Detected ASAP" << endl;
              cout << "Infeasibility without Ressource Constraints: Check if II is correct." << endl;
              throw(HatScheT::Exception("SMTBinaryScheduler::calcAsapAndAlapModScheduleWithSdc()"));
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
      auto validAlap = verifyModuloScheduleSMT(g, resM, startTimes_alap, iiSpace.at(iiSpaceIndex));
      string validstr;
      if ( !quiet ) {
          if (validAlap) { validstr = "True"; } else { validstr = "False"; }
          cout << "---------------------------------------------------------" << endl;
          cout << "Valid Mod. Schedule for given Ressources: " << validstr << " / II: " << iiSpace.at(iiSpaceIndex) << endl;
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
      auto validAsap = verifyModuloScheduleSMT(g, resM, startTimesAsap, iiSpace.at(iiSpaceIndex));
      if (!quiet) {
          if (validAsap) { validstr = "True"; } else { validstr = "False"; }
          cout << "---------------------------------------------------------" << endl;
          cout << "Valid Mod. Schedule for given Ressources: " << validstr << " / II: " << iiSpace.at(iiSpaceIndex) << endl;
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
          II = (double)iiSpace.at(iiSpaceIndex);
          scheduleFound = true;
          startTimes = startTimesAsap;
      } else if (validAlap and sPref == schedulePreference::MOD_ALAP) {
          II = (double)iiSpace.at(iiSpaceIndex);
          scheduleFound = true;
          startTimes = startTimes_alap;
      } else if (validAsap){
          II = (double)iiSpace.at(iiSpaceIndex);
          scheduleFound = true;
          startTimes = startTimesAsap;
      } else if (validAlap){
          II = (double)iiSpace.at(iiSpaceIndex);
          scheduleFound = true;
          startTimes = startTimes_alap;
      }

      //If no valid schedule is found, we continue our latency estimation with the found
      //MOD-ASAP- and MOD-ALAP-Starttimes
      return {startTimesAsap, startTimes_alap};
  }

  int SMTBinaryScheduler::getScheduleLatency(unordered_map<Vertex *, int> &vertexLatency,
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
              throw (HatScheT::Exception("SMTBinaryScheduler::getScheduleLatency() OUT_OF_RANGE"));
          }
      }
      return maxTime;
  }

  //Copied from Verifier to get rid of the CERR outputs. No one likes errors!
  bool SMTBinaryScheduler::verifyModuloScheduleSMT(Graph &g, ResourceModel &rm, std::map<Vertex *, int> &schedule, int II)
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
  void SMTBinaryScheduler::generateBvariables() {
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

  z3::expr *SMTBinaryScheduler::getBvariable(Vertex *v, int i) {
      auto key = std::make_pair(v, i);
      try {
          return &bVariables.at(key);
      }catch(std::out_of_range&){
          cout << "Out_of_Range: " << v->getName() << " - " << i << endl;
          throw (HatScheT::Exception("smtbased Scheduler: getBvariable std::out_of_range"));
      }
  }

  z3::check_result SMTBinaryScheduler::addOneSlotConstraintsToSolver(z3::solver &s) {
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
              s.add(z3::pbeq(b_expressions, &coefficients[0], 1));
          }
          coefficients.clear();
          coefficients.shrink_to_fit();
      }
      return s.check();
  }

  z3::check_result SMTBinaryScheduler::addResourceLimitConstraintToSolver(z3::solver &s, int candidateII) {
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
                  s.add(z3::atmost(b_expressions, it->getLimit()));
              }
          }
      }

//      if (latSM == latSearchMethod::LINEAR){
//          return z3::sat;
//      }
      return s.check();
  }

  void SMTBinaryScheduler::prohibitToEarlyStartsAndAdd(z3::solver &s) {
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

  void SMTBinaryScheduler::prohibitToLateStartsAndAdd(z3::solver &s) {
      for (auto &v : g.Vertices()){
          for (int i = 0; i <= candidateLatency; i++){
              if (i > latestStartTimesUpdated.at(v)){
                  startTimesSimplification.at(std::make_pair(v, i)) = false;
                  //s.add(!*getBvariable(v, i));
              }
          }
      }
  }

  void SMTBinaryScheduler::updateLatestStartTimes(int oldLatency) {
      latestStartTimesUpdated.clear();
      for (auto &it: latestStartTimes) {
          //cout << it.first->getName() << ": " << it.second << " + "<< this->candidateLatency << " - " << modAsapLength << endl;
          this->latestStartTimesUpdated[it.first] = it.second + this->candidateLatency - modAsapLength;
          //cout << it.first->getName() << ": " << latestStartTimesUpdated.at(it.first) << " + "<< this->candidateLatency << " - " << modAsapLength << endl;
      }
  }

  z3::check_result SMTBinaryScheduler::setDependencyConstraintsAndAddToSolver(z3::solver &s, const int &candidateII) {
      //tj + distance * this->candidateII - ti - lSrc - delay >= 0
      int i = 0;
      deque<long> sTimes;
      z3::check_result satisfiable;
      clock_t start, end;
      for (auto &e : g.Edges()) {
          auto *vSrc = &e->getVertexSrc();
          auto *vDst = &e->getVertexDst();
          auto lSrc = this->resourceModel.getVertexLatency(vSrc);
          auto distance = e->getDistance();
          auto delay = e->getDelay();
          for (int ti = 0; ti <= candidateLatency; ti++) {
              for (int tj = 0; tj <= candidateLatency; tj++) {
                  if (tj + distance * candidateII - ti - lSrc - delay >= 0) {
                      //No Conflict... Do nothing
                      continue;
                  } else {
                      //Those cases are not needed since one of these conditions is already prohibited
                      //and we can use the law of absorption : (!a + !b) * !a = !a.
                      if (!startTimesSimplification.at(std::make_pair(vSrc, ti)) ||
                          !startTimesSimplification.at(std::make_pair(vDst, tj))) {
                          continue;
                      }
                      //Conflict... Not both Operations in this timeslot;
                      s.add(!*getBvariable(vSrc, ti) || !*getBvariable(vDst, tj));
                      if (i % 10000 == 0) {
                          auto start_time = std::chrono::high_resolution_clock::now();
                          satisfiable = s.check();
                          auto end_time = std::chrono::high_resolution_clock::now();
                          auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
                          sTimes.push_back((double)duration);
                          timeBudget -= (int)(sTimes.back());
                          if( !quiet ) { cout << i << ": >>" << satisfiable << "<< " << "Solving Time: " << fixed
                                              << (double)sTimes.back()/1000 << setprecision(5) << " sec "
                                              << "Time Budget: " << timeBudget << endl; }
                          if (satisfiable == z3::unsat){
                              return satisfiable;
                          }
                          if (satisfiable == z3::unknown){
                              timeBudget = -1;
                          }
                          if (timeBudget < 0){
                              cerr << "Timeout! Candidate II: " << candidateII
                                   << " Candidate Latency: " << candidateLatency << endl;
                              linearSearchInit = true;
                              secondObjectiveOptimal = false;
                              return z3::unsat;
                          }
                      }
                      i++;
                      //Debugging:
                      /* if (i % 250000 == 0){
                          writeSolvingTimesToFile(sTimes, i);
                      } */
                  }
              }
          }
      }
      startTimesSimplification.clear();
      return satisfiable;
      //throw(HatScheT::Exception("Abort!"));
  }

  bool SMTBinaryScheduler::unsatCheckShortcut() {
      for (auto &vIt : g.Vertices()){
          int count = 0;
          for(int lIt = 0; lIt <= candidateLatency; lIt++){
              count += (int)startTimesSimplification.at(std::make_pair(vIt, lIt));
          }
          if (count == 0){
              return true;
          }
      }
      return false;
  }

  /*!-----------------------*
   *  II-Search 'algorithm' *
   * -----------------------*/
  int SMTBinaryScheduler::iiLinearSearch(z3::check_result result) {
      if (result == z3::sat) {
          return -1;
      }else if (result == z3::unknown){
          if (iiSearchInit) {
              iiSearchInit = false;
              return iiSpace.at(0);
          }
          if (iiSpaceIndex < iiSpace.size()) {
              iiSpaceIndex++;
              if (iiSpaceIndex == iiSpace.size()){
                  return -1;
              }
          }
          return iiSpace.at(iiSpaceIndex);
      }else{
          if (iiSpaceIndex < iiSpace.size()) {
              iiSpaceIndex++;
              if (iiSpaceIndex == iiSpace.size()){
                  return -1;
              }
          }
          return iiSpace.at(iiSpaceIndex);
      }
  }

  /*!--------------------------*
   * Latency-Search algorithms *
   * --------------------------*/
  void SMTBinaryScheduler::calcLatencySpace() {
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

  int SMTBinaryScheduler::latLinearSearch(z3::check_result result) {
      if (linearSearchInit){
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
  }

  int SMTBinaryScheduler::latBinarySearch(z3::check_result result) {
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
          throw (HatScheT::Exception("SMTBinaryScheduler: Binary Latency Search: STD::OUT_OF_RANGE"));
      }
  }

  /*!-------------------------------*
   * Parsing schedule from z3-model *
   *--------------------------------*/
  void SMTBinaryScheduler::parseSchedule(z3::model &m) {
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
  void SMTBinaryScheduler::setSolverTimeout(unsigned int seconds) {
      this->timeLimit = seconds * 1000;
  }

  /*!--------------------------------*
   * Print and Debugging Functions:  *
   * --------------------------------*/
  void SMTBinaryScheduler::print_b_variables() {
      for (auto &it:g.Vertices()){
          for (int i = 0; i <= candidateLatency; i++){
              cout << *getBvariable(it, i) << endl;
          }
      }
  }

  void SMTBinaryScheduler::print_latency_space(int l_index, int r_index) {
      for (int i = l_index; i < r_index; i++){
          cout << latencySpace.at(i) << " ";
      }
      cout << endl;
  }

  void SMTBinaryScheduler::print_solution(z3::model &m) {
      for (auto &it : g.Vertices()){
          for (int i = 0; i <= candidateLatency; i++) {
              auto val = m.eval(*getBvariable(it, i));
              if (val.is_true()){
                  cout << it->getName() << " - " << i << ": " <<  val << endl;
              }
          }
      }
  }

  void SMTBinaryScheduler::print_ASAP_ALAP_restictions() {
      for (auto &it : startTimesSimplification){
          cout << it.first.first->getName() << " <" << it.first.second << "> " << it.second << endl;
      }
  }

  void SMTBinaryScheduler::writeSolvingTimesToFile(deque<double> &times, int x) {
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

  void SMTBinaryScheduler::printPossibleStarttimes(map<pair<Vertex*, int>, bool>& vertex_timeslot) {
      cout << "MinLAT: " << minLatency << " modASAPLength: " << modAsapLength << endl;
      for (auto &r : resourceModel.Resources()) {
          if (r->isUnlimited()) {
              continue;
          }
          for (auto &v : resourceModel.getVerticesOfResource(r)) {
              cout << setw(18) << v->getName() << ": ";
              for (int i = 0; i < minLatency; i++) {
                  if (i % iiSpace.at(iiSpaceIndex) == 0) {
                      cout << " ";
                  }
                  cout << vertex_timeslot.at(std::make_pair((Vertex *) v, i));
              }
              cout << endl;
          }
          cout << endl;
      }
  }

  z3::check_result SMTBinaryScheduler::test_binary_search(int value_to_check, int target_value) {
      if (value_to_check < target_value){
          return z3::unsat;
      }
      return z3::sat;
  }

}
#endif

