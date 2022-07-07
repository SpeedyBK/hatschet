//
// Created by bkessler on 5/7/22.
//

#ifdef USE_Z3
#include "SMTBinaryScheduler.h"
#include "HatScheT/utility/Exception.h"

#include "HatScheT/scheduler/ASAPScheduler.h"

#include <z3++.h>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <cmath>
#include <deque>
#include <algorithm>
#include <unordered_map>

namespace HatScheT {

  SMTBinaryScheduler::SMTBinaryScheduler(Graph &g, ResourceModel &resourceModel) : SchedulerBase(g, resourceModel) {
      // reset previous solutions
      this->timeouts = -1;
      startTimes.clear();
      scheduleFound = false;
      computeMinII(&g, &resourceModel);
      minII = ceil(minII);
      computeMaxII(&g, &resourceModel);

      quiet = true;
      for (int i = (int)minII; i <= (int)maxII; i++){
          this->iiSpace.push_back(i);
      }
      iiSpaceIndex = 0;

      sPref = schedulePreference::MOD_ASAP;
      latSM = latSearchMethod::binary;
      this->latencySpaceIndex = 0;
      this->candidateLatency = -1;
      this->minLatency = 0;
      this->maxLatency = -1;
      this->maxRuns = INT32_MAX;
      this->leftIndex = 0;
      this->rightIndex = 0;

      binarySearchInit = true;
      reverseSearchInit = true;
      iiSearchInit = true;

      modAsapLength = 0;
      modAlapLength = 0;

      designName = "";
      timeBudget = INT32_MAX;

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
      clock_t start, end;
      deque<double>solving_times;

      //Setting up z3:
      z3::solver s(c);
      auto sati = z3::unknown;
      z3::model m(c);
      z3::params p(c);
      //if (timeouts > 0) { p.set(":timeout", timeouts); }
      if (!quiet) { cout << p << endl; }
      //s.set(p);
      if ((int) timeouts > 0) {
          timeBudget = (int) timeouts;
      }
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

          //If feasible -> break
          if (candidateII == -1){
              break;
          }

          if (!quiet) { cout << "Trying II: " << candidateII << endl; }

          //Calculating minimum latency and maximum latency-estimation:
          calcMaxLatencyEstimation();
          //If valid schedule already found, just return.
          if (scheduleFound){ return; }

          if (!quiet) { cout << "Max-Latency : " << maxLatency << endl; }
          if (!quiet) { cout << "Min-Latency : " << minLatency << endl; }
          //Setting up latency-search-space for latency-search-loop
          calcLatencySpace();

          //Latency-search-loop
          while (true){
              if ( !quiet ) { cout << " -------------------------------- " << endl; }
              int old_latency = candidateLatency;
              if (latSM == latSearchMethod::linear) { candidateLatency = latLinearSearch(sati); }
              if (latSM == latSearchMethod::binary) { candidateLatency = latBinarySearch(sati); }
              if (latSM == latSearchMethod::reverse_linear) { candidateLatency = latReverseLinearSearch(sati); }//toDo: Does not work yet

              if (candidateLatency == -1 or old_latency == candidateLatency ) {
                  candidateLatency = 0;
                  break;
              }

              if (!quiet) { cout << "Trying Latency: " << candidateLatency << endl; }
              //Updating latest start times with candidate latency
              updateLatestStartTimes();
              //Generating debendency constraints for z3-prover.
              if (!quiet) { cout << "Generating b-variables... " << endl; }
              generateBvariables();
              //if (!quiet) { print_b_variables(); }

              //Reseting solver and add timeout, before pushing in stuff.
              s.reset();
              //s.set(p);

              //Reducing Seach Space.
              if (!quiet) { cout << "Prohibit early starts... " << endl; }
              prohibitToEarlyStartsAndAdd(s);
              if (!quiet) { cout << "Prohibit late starts... " << endl; }
              prohibitToLateStartsAndAdd(s);

              //Takeing a shortcut, probably nonsence...
              if (unsatCheckShortcut()){
                  sati = z3::unsat;
                  continue;
              }

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
              if (sati != z3::unsat) {
                  start = clock();
                  sati = s.check();
                  end = clock();
                  t = (int)((double(end - start) / double(CLOCKS_PER_SEC)) * 1000);
                  timeBudget = timeBudget - (int)((double(end - start) / double(CLOCKS_PER_SEC)) * 1000);
              }
              //Saving solving times per latency attempt.
              solving_times.push_back(double(end - start)/ double(CLOCKS_PER_SEC));
              if (!quiet) {
                  cout << "-->" << sati << "<--" << endl;
                  //cout << "reason unknown: " << s.reason_unknown() << endl;
                  cout << "Solving Time: " << fixed << t
                       << setprecision(5) << " sec " << endl;
              }
              if (!quiet) { cout << "TimeBudget: " << timeBudget << endl; }
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
                      if ((int)timeouts > 0) {
                          timeBudget = (int) timeouts;
                      }
                      break;
                  }
              }
              if (sati == z3::sat){
                  m = s.get_model();
                  this->scheduleFound = true;
                  this->II = iiSpace.at(iiSpaceIndex);
                  parseSchedule(m);
                  if (latSM == latSearchMethod::linear) {
                      break;
                  }
              }
          }
          i++;
      }

      II = candidateIIOLD;

      if (!verifyModuloScheduleSMT(this->g, this->resourceModel, startTimes, II)){
          throw (HatScheT::Exception("Schedule not valid!"));
      }
  }

  /*!-------------------*
   * Latency Estimation *
   * -------------------*/
  void SMTBinaryScheduler::calcMaxLatencyEstimation() {

      //Check if max latency was set by user
      if (this->maxLatencyConstraint >= 0) {
          maxLatency = maxLatencyConstraint;
          return;
      }
      this->maxLatency = 0;
      int resMaxLat = 0;

      //Getting the current II mainly for better readability
      int current_II;
      try {
          current_II = iiSpace.at(iiSpaceIndex);
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

      //Algorithm to estimate max latency based on earliest and latest possible start times
      //as well as on ressource constraints.
      map<pair<Vertex*, int>, bool> vertex_timeslot;
      unordered_map<Vertex*, bool> checked;

      //Generates a matrix with the information wether a Vertex can be schedules in a timeslot or not,
      //based on dependency constraints.
      for (auto &v : g.Vertices()){
          checked.insert(std::make_pair(v,false));
          int t_asap = earliestStartTimes.at(v);
          int t_alap = latestStartTimes.at(v);
          for (int i = 0; i < t_asap; i++){
              vertex_timeslot[{v, i}] = false;
          }
          for (int i = t_asap; i <= t_alap; i++){
              vertex_timeslot[{v, i}] = true;
          }
          for (int i = t_alap + 1; i < modAsapLength; i++){
              vertex_timeslot[{v, i}] = false;
          }
      }

      if ( !quiet ) {
          print_possible_starttimes(vertex_timeslot);
          cout << endl << "------------------------------------------------------------------" << endl;
      }

      //Trys to satisfy ressource constraints. If they can not be satisfied, there will be operations
      //that can not be scheduled.
      for (auto &r : resourceModel.Resources()){
          if (r->isUnlimited()){
              continue;
          }
          vector<int>used_FU_in_Modslot;
          used_FU_in_Modslot.resize(current_II);
          for (int i = 0; i < modAsapLength; i++){
              for (auto &cvp : resourceModel.getVerticesOfResource(r)){
                  auto v = (Vertex*) cvp;
                  if (!vertex_timeslot.at(std::make_pair(v, i))){
                      continue;
                  }
                  //Check conflicts with other mod-slots.
                  if (used_FU_in_Modslot.at(i%current_II) < r->getLimit() and !checked.at(v)){
                      //Check vertex and count up used Resources:
                      checked.at(v) = true;
                      used_FU_in_Modslot.at(i % current_II)++;
                      continue;
                  }
                  //Set vertices, of Resource in the same timeslot to 0, because there are no more FUs.
                  vertex_timeslot.at(std::make_pair(v, i)) = false;
              }
          }
          used_FU_in_Modslot.clear();
          used_FU_in_Modslot.shrink_to_fit();
      }
      //Counting the not scheduled operations to expand the schedule by that amount.
      deque<int>critical_ressources;
      for (auto &r : resourceModel.Resources()) {
          if (r->isUnlimited()){
              continue;
          }
          int count2 = 0;
          for (auto &v : resourceModel.getVerticesOfResource(r)) {
              int count = 0;
              for (int i = 0; i < modAsapLength; i++) {
                  count += (int)vertex_timeslot.at(std::make_pair((Vertex*)v, i));
              }
              if (count == 0){
                  count2++;
              }
          }
          critical_ressources.push_back(count2);
      }

      int maximum = *std::max_element(critical_ressources.begin(),critical_ressources.end());

      if ( !quiet ) {
          print_possible_starttimes(vertex_timeslot);
      }

      maxLatency = maximum + modAsapLength;
      if ( !quiet ) { cout << "Max Latency Suggestion: " << maximum + modAsapLength << endl; }
  }

  pair <map<Vertex*,int>, map<Vertex*, int>> SMTBinaryScheduler::calcAsapAndAlapModScheduleWithSdc(Graph &g, ResourceModel &resM) {

      // Create SDC-Graphs (Transposed with Helper for ALAP-Starttimes
      // and original with Helper for ASAP-Starttimes) as well as a mapping
      // to the Vertices of our original Graph:
      Graph sdc_graph_alap;
      Graph sdc_graph_asap;
      unordered_map<Vertex*, int> vertex_Latency_alap;
      unordered_map<Vertex*, int>vertex_distance_alap;
      unordered_map<Vertex*, Vertex*> old_to_new_vertex_alap;
      unordered_map<Vertex*, Vertex*> new_to_old_vertex_alap;
      unordered_map<Vertex*, int> vertex_Latency_asap;
      unordered_map<Vertex*, int> vertex_distance_asap;
      unordered_map<Vertex*, Vertex*> old_to_new_vertex_asap;
      unordered_map<Vertex*, Vertex*> new_to_old_vertex_asap;

      for (auto &v : g.Vertices()){
          Vertex* v_sdc_alap = &sdc_graph_alap.createVertex(v->getId());
          Vertex* v_sdc_asap = &sdc_graph_asap.createVertex(v->getId());
          v_sdc_alap->setName(v->getName());
          v_sdc_asap->setName(v->getName());
          vertex_Latency_alap[v_sdc_alap] = resM.getVertexLatency(v);
          vertex_Latency_asap[v_sdc_asap] = resM.getVertexLatency(v);
          old_to_new_vertex_alap[v] = v_sdc_alap;
          old_to_new_vertex_asap[v] = v_sdc_asap;
          new_to_old_vertex_alap[v_sdc_alap] = v;
          new_to_old_vertex_asap[v_sdc_asap] = v;
      }

      for (auto &e : g.Edges()){
          auto v_src_alap = old_to_new_vertex_alap.at(&e->getVertexSrc());
          auto v_src_asap = old_to_new_vertex_asap.at(&e->getVertexSrc());
          auto v_dst_alap = old_to_new_vertex_alap.at(&e->getVertexDst());
          auto v_dst_asap = old_to_new_vertex_asap.at(&e->getVertexDst());
          sdc_graph_alap.createEdge(*v_dst_alap, *v_src_alap, -vertex_Latency_alap.at(v_src_alap)
                                                              - e->getDelay()
                                                              + e->getDistance() * this->iiSpace.at(iiSpaceIndex));
          sdc_graph_asap.createEdge(*v_src_asap, *v_dst_asap, -vertex_Latency_asap.at(v_src_asap)
                                                              - e->getDelay()
                                                              + e->getDistance() * this->iiSpace.at(iiSpaceIndex));

      }

      //Bellmann-Ford Step 1:
      //Create Helper Vertex and connect it to all other Vertices with distance 0:
      //Initialization of Bellmann-Ford Algorithm with INT_MAX for all Vertices except Helper
      //to solve the SDC-System:
      Vertex& helper_alap = sdc_graph_alap.createVertex();
      Vertex& helper_asap = sdc_graph_asap.createVertex();
      helper_alap.setName("Helper" + to_string(helper_alap.getId()));
      helper_asap.setName("Helper" + to_string(helper_asap.getId()));
      for (auto &v : sdc_graph_alap.Vertices()){
          if (v == &helper_alap){
              vertex_distance_alap[&helper_alap] = 0;
              continue;
          }
          sdc_graph_alap.createEdge(helper_alap, *v, 0);
          vertex_distance_alap[v] = INT32_MAX;
      }
      for (auto &v : sdc_graph_asap.Vertices()){
          if (v == &helper_asap){
              vertex_distance_asap[&helper_asap] = 0;
              continue;
          }
          sdc_graph_asap.createEdge(helper_asap, *v, 0);
          vertex_distance_asap[v] = INT32_MAX;
      }

      //Bellmann-Ford Step 2:
      //Searching shortest path:
      //A simple shortest path from src to any other vertex can have
      //at-most |V| - 1 edges
      try {
          unsigned int num_of_vertices_alap = sdc_graph_alap.getNumberOfVertices();
          for (int i = 0; i < num_of_vertices_alap - 1; i++) {
              for (auto &e : sdc_graph_alap.Edges()) {
                  auto t = &e->getVertexSrc();
                  auto u = &e->getVertexDst();
                  int weight = e->getDistance();
                  if ((vertex_distance_alap.at(t) != INT32_MAX) &&
                      (vertex_distance_alap.at(t) + weight < vertex_distance_alap.at(u))) {
                      vertex_distance_alap.at(u) = vertex_distance_alap.at(t) + weight;
                  }
              }
          }
      }catch (std::out_of_range&) {
          throw (std::out_of_range("SMT_Sheduler: calcAsapAndAlapModScheduleWithSdc(), Bellmann-Ford, Step 2, ALAP Loop"));
      }
      try{
          unsigned int num_of_vertices_asap = sdc_graph_asap.getNumberOfVertices();
          for (int i = 0; i < num_of_vertices_asap - 1; i++) {
              for (auto &e : sdc_graph_asap.Edges()) {
                  auto t = &e->getVertexSrc();
                  auto u = &e->getVertexDst();
                  int weight = e->getDistance();
                  if ((vertex_distance_asap.at(t) != INT32_MAX) &&
                      (vertex_distance_asap.at(t) + weight < vertex_distance_asap.at(u))) {
                      vertex_distance_asap.at(u) = vertex_distance_asap.at(t) + weight;
                  }
              }
          }
      } catch (std::out_of_range&) {
          throw (std::out_of_range("SMT_Sheduler: calcAsapAndAlapModScheduleWithSdc(), Bellmann-Ford, Step 2, ASAP Loop"));
      }

      //Checking for negativ Cycles:
      for (auto &e : sdc_graph_alap.Edges()){
          auto t = &e->getVertexSrc();
          auto u = &e->getVertexDst();
          int weight = e->getDistance();
          if ((vertex_distance_alap.at(t) != INT32_MAX) && (vertex_distance_alap.at(t) + weight < vertex_distance_alap.at(u))){
              cout << "Negative Cycle Detected ALAP:" << endl;
              cout << "Infeasibility without Ressource Constraints: Check if II is correct." << endl;
              throw(HatScheT::Exception("SMTBinaryScheduler::calcAsapAndAlapModScheduleWithSdc()"));
          }
      }
      for (auto &e : sdc_graph_asap.Edges()){
          auto t = &e->getVertexSrc();
          auto u = &e->getVertexDst();
          int weight = e->getDistance();
          if ((vertex_distance_asap.at(t) != INT32_MAX) && (vertex_distance_asap.at(t) + weight < vertex_distance_asap.at(u))){
              cout << "Negative Cycle Detected ASAP" << endl;
              cout << "Infeasibility without Ressource Constraints: Check if II is correct." << endl;
              throw(HatScheT::Exception("SMTBinaryScheduler::calcAsapAndAlapModScheduleWithSdc()"));
          }
      }

      if ( !quiet ) {
          cout << "Transposed Helper Graph:" << endl << sdc_graph_alap << endl;
          cout << "Helper Graph:" << endl << sdc_graph_asap << endl;
      }

      auto min = std::min_element(vertex_distance_alap.begin(), vertex_distance_alap.end(), [](const pair<Vertex*,int> &x, const pair<Vertex*, int> &y) {
        return x.second < y.second;
      })->second;

      map<Vertex*, int> startTimes_alap;
      map<Vertex*, int> startTimes_asap;

      if ( !quiet ) { cout << "Latest possible Starttimes:" << endl; }
      for (auto &it : vertex_distance_alap) {
          //Removing HELPER_ALAP-Vertex from ALAP-Starttimes:
          if (it.first == &helper_alap){
              continue;
          }
          it.second += abs(min);
          startTimes_alap[new_to_old_vertex_alap[it.first]]=it.second;
          if ( !quiet) { cout << it.first->getName() << ": " << it.second << endl; }
      }

      //Checking if we are lucky and MOD-ALAP-Starttimes are already a valig modulo schedule:
      auto valid_alap = verifyModuloScheduleSMT(g, resM, startTimes_alap, iiSpace.at(iiSpaceIndex));
      string validstr;
      if ( !quiet ) {
          if (valid_alap) { validstr = "True"; } else { validstr = "False"; }
          cout << "---------------------------------------------------------" << endl;
          cout << "Valid Mod. Schedule for given Ressources: " << validstr << " / II: " << iiSpace.at(iiSpaceIndex) << endl;
          cout << "---------------------------------------------------------" << endl;
          cout << "Earliest possible Starttimes:" << endl;
      }
      for (auto &it : vertex_distance_asap) {
          //Removing HELPER_ASAP-Vertex from ALAP-Starttimes:
          if (it.first == &helper_asap) {
              continue;
          }
          it.second = abs(it.second);
          startTimes_asap[new_to_old_vertex_asap[it.first]] = it.second;
          if (!quiet) { cout << it.first->getName() << ": " << it.second << endl; }
      }

      //Checking if we are lucky and MOD-ASAP-Starttimes are already a valig modulo schedule:
      auto valid_asap = verifyModuloScheduleSMT(g, resM, startTimes_asap, iiSpace.at(iiSpaceIndex));
      if (!quiet) {
          if (valid_asap) { validstr = "True"; } else { validstr = "False"; }
          cout << "---------------------------------------------------------" << endl;
          cout << "Valid Mod. Schedule for given Ressources: " << validstr << " / II: " << iiSpace.at(iiSpaceIndex) << endl;
          cout << "---------------------------------------------------------" << endl;
      }

      auto success = vertex_distance_alap.erase(&helper_alap);
      if ( !quiet and success){
          cout << "Successful deleted Helper_Vertex_ALAP: True" << endl;
      }else if ( !quiet ){
          cout << "Successful deleted Helper_Vertex_ALAP: False" << endl;
      }
      success = vertex_distance_asap.erase(&helper_asap);
      if ( !quiet and success){
          cout << "Successful deleted Helper_Vertex_ASAP: True" << endl;
      }else if ( !quiet ){
          cout << "Successful deleted Helper_Vertex_ASAP: False" << endl;
      }

      //Saving schedule length
      if (!quiet) { cout << endl << "MOD_ALAP: " << endl; }
      modAlapLength = getScheduleLatency(vertex_distance_alap, new_to_old_vertex_alap);
      if (!quiet) { cout << endl << "MOD_ASAP: " << endl; }
      modAsapLength = getScheduleLatency(vertex_distance_asap, new_to_old_vertex_asap);
      if (!quiet) { cout << "SDC-ALAP-Latency: " << modAlapLength << endl; }
      if (!quiet) { cout << "SDC-ASAP-Latency: " << modAsapLength << endl; }
      if (modAsapLength != modAlapLength ){
          throw (HatScheT::Exception("smtbased-Scheduler, calcAsapAndAlapModScheduleWithSdc(), modAlapLength and not equal"));
      }
      minLatency = modAsapLength;

      //If a valid schedule is already found, just returning it.
      if (valid_asap and sPref == schedulePreference::MOD_ASAP) {
          II = (double)iiSpace.at(iiSpaceIndex);
          scheduleFound = true;
          startTimes = startTimes_asap;
      } else if (valid_alap and sPref == schedulePreference::MOD_ALAP) {
          II = (double)iiSpace.at(iiSpaceIndex);
          scheduleFound = true;
          startTimes = startTimes_alap;
      } else if (valid_asap){
          II = (double)iiSpace.at(iiSpaceIndex);
          scheduleFound = true;
          startTimes = startTimes_asap;
      } else if (valid_alap){
          II = (double)iiSpace.at(iiSpaceIndex);
          scheduleFound = true;
          startTimes = startTimes_alap;
      }

      //If no valid schedule is found, we continue our latency estimation with the found
      //MOD-ASAP- and MOD-ALAP-Starttimes
      return {startTimes_asap, startTimes_alap};
  }

  int SMTBinaryScheduler::getScheduleLatency(unordered_map<Vertex *, int> &vertex_latency,
                                             unordered_map<Vertex *, Vertex *> &newtoold) {
      int maxTime = -1;
      for (std::pair<Vertex *, int> vtPair : vertex_latency) {
          try {
              Vertex *v = vtPair.first;
              if ((vtPair.second + resourceModel.getVertexLatency(newtoold.at(v))) > maxTime) {
                  maxTime = (vtPair.second + resourceModel.getVertexLatency(newtoold.at(v)));
                  //Debugging:
                  if (!quiet) {
                      cout << vtPair.first->getName() << ": " << vtPair.second << " + "
                           << resourceModel.getVertexLatency(newtoold.at(v)) << " = " << maxTime << endl;
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
      int count = 0;
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

      if (latSM == latSearchMethod::linear){
          return z3::sat;
      }
      return s.check();
  }

  void SMTBinaryScheduler::prohibitToEarlyStartsAndAdd(z3::solver &s) {
      startTimesSimplification.clear();
      for (auto &v : g.Vertices()){
          for (int i = 0; i <= candidateLatency; i++){
              if (i < earliestStartTimes.at(v)){
                  startTimesSimplification[std::make_pair(v, i)] = false;
                  s.add(!*getBvariable(v, i));
              }else{
                  startTimesSimplification[std::make_pair(v, i)] = true;
              }
          }
      }
  }

  void SMTBinaryScheduler::prohibitToLateStartsAndAdd(z3::solver &s) {
      for (auto &v : g.Vertices()){
          for (int i = 0; i <= candidateLatency; i++){
              if (i > latestStartTimes.at(v)){
                  startTimesSimplification.at(std::make_pair(v, i)) = false;
                  s.add(!*getBvariable(v, i));
              }
          }
      }
  }

  void SMTBinaryScheduler::updateLatestStartTimes() {
      for (auto &it: latestStartTimes) {
          this->latestStartTimes.at(it.first) = it.second - modAlapLength + this->candidateLatency;
      }
  }

  z3::check_result SMTBinaryScheduler::setDependencyConstraintsAndAddToSolver(z3::solver &s, const int &candidateII) {
      //tj + distance * this->candidateII - ti - lSrc - delay >= 0
      int i = 0;
      deque<double> sTimes;
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
                          start = clock();
                          if (latSM != latSearchMethod::linear) {
                              satisfiable = s.check();
                          }else{
                              satisfiable = z3::sat;
                          }
                          end = clock();
                          sTimes.push_back(double(end - start) / double(CLOCKS_PER_SEC));
                          timeBudget -= (int)(sTimes.back() * 1000);
                          if( !quiet ) { cout << i << ": >>" << satisfiable << "<< " << "Solving Time: " << fixed
                                              << sTimes.back() << setprecision(5) << " sec "
                                              << "Time Budget: " << timeBudget << endl; }
                          if (satisfiable == z3::unsat){
                              return satisfiable;
                          }
                          if (timeBudget < 0){
                              cerr << "Timeout! Candidate II: " << candidateII
                                   << " Candidate Latency: " << candidateLatency << endl;
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
          int index = iiSpaceIndex;
          if (iiSpaceIndex < iiSpace.size() - 1) {
              iiSpaceIndex++;
          }
          return iiSpace.at(index);
      }else{
          int index = iiSpaceIndex;
          if (iiSpaceIndex < iiSpace.size() - 1) {
              iiSpaceIndex++;
          }
          return iiSpace.at(index);
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

  int SMTBinaryScheduler::latReverseLinearSearch(z3::check_result result) {
      if (reverseSearchInit) {
          latencySpaceIndex = (int) latencySpace.size() - 1;
          reverseSearchInit = false;
      }
      //todo get this done
      return 42;
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
      this->timeouts = seconds*1000;
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

  void SMTBinaryScheduler::print_possible_starttimes(map<pair<Vertex*, int>, bool>& vertex_timeslot) {
      for (auto &r : resourceModel.Resources()) {
          if (r->isUnlimited()) {
              continue;
          }
          for (auto &v : resourceModel.getVerticesOfResource(r)) {
              cout << setw(18) << v->getName() << ": ";
              for (int i = 0; i < modAsapLength; i++) {
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
