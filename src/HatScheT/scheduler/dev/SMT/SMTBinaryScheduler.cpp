//
// Created by bkessler on 5/7/22.
//

#ifdef USE_Z3
#include "SMTBinaryScheduler.h"
#include "HatScheT/utility/Exception.h"
#include "HatScheT/utility/Utility.h"

#include "HatScheT/scheduler/ASAPScheduler.h"
#include "HatScheT/scheduler/ALAPScheduler.h"

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
          this->II_space.push_back(i);
      }
      II_space_index = 0;

      s_pref = schedule_preference::MOD_ASAP;
      latSM = latSearchMethod::linear;
      this->latency_space_index = 0;
      this->candidateLatency = -1;
      this->min_Latency = 0;
      this->max_Latency = -1;
      this->maxRuns = INT32_MAX;
      this->left_index = 0;
      this->right_index = 0;

      binary_search_init = true;
      reverse_search_init = true;
      ii_search_init = true;

      design_name = "";

  }

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
      if (timeouts > 0) { p.set(":timeout", timeouts); }
      cout << p << endl;
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
          candidateII = ii_linear_search(sati);

          //If feasible -> break
          if (candidateII == -1){
              break;
          }

          if (!quiet) { cout << "Trying II: " << candidateII << endl; }

          //Calculating minimum latency and maximum latency-estimation:
          calc_Max_Latency_Estimation();
          //If valid schedule already found, just return.
          if (scheduleFound){ return; }

          if (!quiet) { cout << "Max-Latency : " << max_Latency << endl; }
          if (!quiet) { cout << "Min-Latency : " << min_Latency << endl; }
          //Setting up latency-search-space for latency-search-loop
          calcLatencySpace();

          //Latency-search-loop
          while (true){
              if ( !quiet ) { cout << " -------------------------------- " << endl; }
              int old_latency = candidateLatency;
              if (latSM == latSearchMethod::linear) { candidateLatency = lat_linear_search(sati); }
              if (latSM == latSearchMethod::binary) { candidateLatency = lat_binary_search(sati); }
              if (latSM == latSearchMethod::reverse_linear) { candidateLatency = lat_reverse_linear_search(sati); }//toDo: Does not work yet

              if (candidateLatency == -1 or old_latency == candidateLatency ) {
                  candidateLatency = 0;
                  break;
              }

              if (!quiet) { cout << "Trying Latency: " << candidateLatency << endl; }
              //Updating latest start times with candidate latency
              update_latest_start_times();
              //Generating debendency constraints for z3-prover.
              if (!quiet) { cout << "Generating b-variables... " << endl; }
              generate_b_variables();
              //if (!quiet) { print_b_variables(); }

              //Reseting solver and add timeout, before pushing in stuff.
              s.reset();
              s.set(p);

              //Reducing Seach Space.
              if (!quiet) { cout << "Prohibit early starts... " << endl; }
              prohibit_to_early_starts_and_add(s);
              if (!quiet) { cout << "Prohibit late starts... " << endl; }
              prohibit_to_late_starts_and_add(s);

              //Takeing a shortcut, probably nonsence...
              if (unsat_check_shortcut()){
                  sati = z3::unsat;
                  continue;
              }

              //Pushing assertions into z3-Prover
              if (!quiet) { cout << "Add one-slot-constraints... " << endl; }
              add_one_slot_constraints_to_solver(s);

              if (!quiet) { cout << "Add resource-constraints... " << endl; }
              add_resource_limit_constraint_to_solver(s, candidateII);

              if (!quiet) { cout << "Add dependency-constraints... " << endl; }
              set_b_variables(s, candidateII);

              //Start Solving
              if (!quiet) { cout << "System of " << s.assertions().size() << " assertions..." << endl; }
              if (!quiet) { cout << "Solving... " << endl; }
              start = clock();
              sati = s.check();
              end = clock();
              //Saving solving times per latency attempt.
              solving_times.push_back(double(end - start)/ double(CLOCKS_PER_SEC));
              if (!quiet) {
                  cout << "-->" << sati << "<--" << endl << "Solving Time: " << fixed
                       << double(end - start) / double(CLOCKS_PER_SEC)
                       << setprecision(5) << " sec " << endl;
              }
              if (sati == z3::sat){
                  m = s.get_model();
                  this->scheduleFound = true;
                  parse_schedule(m);
              }
              if (latSM == latSearchMethod::linear) {
                  if (sati == z3::sat) {
                      break;
                  }
              }
          }
          i++;
      }

      II = candidateIIOLD;

      write_solving_times_to_file(solving_times);

      //cout << s << endl << "--------------------------------" <<endl;
      //m = s.get_model();
      //print_solution(m);
      //parse_schedule(m);
      //this->scheduleFound = true;
  }

  void SMTBinaryScheduler::update_latest_start_times() {
      for (auto &it: latest_start_times) {
          this->latest_start_times.at(it.first) = it.second - mod_alap_length + this->candidateLatency;
      }
  }

  void SMTBinaryScheduler::prohibit_to_early_starts_and_add(z3::solver &s) {
      start_times_simplification.clear();
      for (auto &v : g.Vertices()){
          for (int i = 0; i < candidateLatency; i++){
              if (i < earliest_start_times.at(v)){
                  start_times_simplification.insert(std::make_pair(std::make_pair(v, i), false));
                  s.add(!*get_b_variable(v, i));
              }else{
                  start_times_simplification.insert(std::make_pair(std::make_pair(v, i), true));
              }
          }
      }
  }

  void SMTBinaryScheduler::prohibit_to_late_starts_and_add(z3::solver &s) {
      for (auto &v : g.Vertices()){
          for (int i = 0; i < candidateLatency; i++){
              if (i > latest_start_times.at(v)){
                  start_times_simplification.at(std::make_pair(v, i)) = false;
                  s.add(!*get_b_variable(v, i));
              }
          }
      }
  }

  void SMTBinaryScheduler::generate_b_variables() {
      b_variables.clear();
      for (auto &it : g.Vertices()){
          for (int i = 0; i < candidateLatency; i++){
              auto key = std::make_pair(it, i);
              std::stringstream name;
              name << it->getName() << "->" << i;
              z3::expr e(c.bool_const(name.str().c_str()));
              b_variables.insert({key, e});
          }
      }
  }

  z3::expr *SMTBinaryScheduler::get_b_variable(Vertex *v, int i) {
      auto key = std::make_pair(v, i);
      try {
          return &b_variables.at(key);
      }catch(std::out_of_range&){
          cout << "Out_of_Range: " << v->getName() << " - " << i << endl;
          throw (HatScheT::Exception("SMT Scheduler: get_b_variable std::out_of_range"));
      }
  }

  void SMTBinaryScheduler::set_b_variables(z3::solver &s, const int &candidateII) {
      //tj + distance * this->candidateII - ti - lSrc - delay >= 0

      for (auto &e : g.Edges()) {
          auto *vSrc = &e->getVertexSrc();
          auto *vDst = &e->getVertexDst();
          auto lSrc = this->resourceModel.getVertexLatency(vSrc);
          auto distance = e->getDistance();
          auto delay = e->getDelay();
          for (int ti = 0; ti < candidateLatency; ti++) {
              for (int tj = 0; tj < candidateLatency; tj++) {
                  if (tj + distance * candidateII - ti - lSrc - delay >= 0) {
                      //No Conflict... Do nothing
                      continue;
                  } else {
                      //Those cases are not needed since one of these conditions is already prohibited
                      //and we can use the law of absorption : (!a + !b) * !a = !a.
                      if (!start_times_simplification.at(std::make_pair(vSrc, ti)) ||
                          !start_times_simplification.at(std::make_pair(vDst, tj))) {
                          continue;
                      }
                      //Conflict... Not both Operations in this timeslot;
                      s.add(!*get_b_variable(vSrc, ti) || !*get_b_variable(vDst, tj));
                  }
              }
          }
      }
  }

  void SMTBinaryScheduler::add_one_slot_constraints_to_solver(z3::solver &s) {
      for (auto &it : g.Vertices()) {

          vector<int> coefficients;
          coefficients.reserve(candidateLatency);
          z3::expr_vector b_expressions(c);
          for (int i = 0; i < candidateLatency; i++) {
              if (!start_times_simplification.at(std::make_pair((Vertex *)it, i))){
                  continue;
              }
              coefficients.push_back(1);
              b_expressions.push_back(*get_b_variable(it, i));
          }
          if (!b_expressions.empty()) {
              s.add(z3::pbeq(b_expressions, &coefficients[0], 1));
          }
          coefficients.clear();
          coefficients.shrink_to_fit();
      }
  }

  void SMTBinaryScheduler::add_resource_limit_constraint_to_solver(z3::solver &s, int candidateII) {
      for (auto &it : resourceModel.Resources()) {
          if (it->isUnlimited()) {
              continue;
          }
          for (int i = 0; i < candidateII; i++) {
              set<const Vertex *> vSet = resourceModel.getVerticesOfResource(it);
              z3::expr_vector b_expressions(c);
              for (auto &vIt : vSet) {
                  for (int j = 0; j < candidateLatency; j++) {
                      if ((j % candidateII) != i) {
                          continue;
                      }
                      if (!start_times_simplification.at(std::make_pair((Vertex*)vIt, j))){
                          continue;
                      }
                      b_expressions.push_back(*get_b_variable((Vertex *) vIt, j));
                  }
              }
              /*cout << b_expressions.size() << " Type: " << it->getName() << " Limit: " << it->getLimit() << " i: " << i
                   << endl;*/
              if (!b_expressions.empty()) {
                  s.add(z3::atmost(b_expressions, it->getLimit()));
              }
          }
      }
  }

  void SMTBinaryScheduler::print_b_variables() {
      for (auto &it:g.Vertices()){
          for (int i = 0; i < candidateLatency; i++){
              cout << *get_b_variable(it, i) << endl;
          }
      }
  }

  void SMTBinaryScheduler::print_solution(z3::model &m) {
      for (auto &it : g.Vertices()){
          for (int i = 0; i < candidateLatency; i++) {
              auto val = m.eval(*get_b_variable(it, i));
              if (val.is_true()){
                  cout << it->getName() << " - " << i << ": " <<  val << endl;
              }
          }
      }
  }

  void SMTBinaryScheduler::parse_schedule(z3::model &m) {
      for (auto &it : g.Vertices()){
          for (int i = 0; i < candidateLatency; i++) {
              auto val = m.eval(*get_b_variable(it, i));
              if (val.is_true()){
                  this->startTimes[it] = i;
              }
          }
      }
  }

  int SMTBinaryScheduler::ii_linear_search(z3::check_result result) {
      if (result == z3::sat) {
          return -1;
      }else if (result == z3::unknown){
          if (ii_search_init) {
              ii_search_init = false;
              return II_space.at(0);
          }
          int index = II_space_index;
          II_space_index++;
          return II_space.at(index);
      }else{
          int index = II_space_index;
          II_space_index++;
          return II_space.at(index);
      }
  }

  void SMTBinaryScheduler::setSolverTimeout(unsigned int seconds) {
      this->timeouts = seconds*1000;
  }

  int SMTBinaryScheduler::lat_linear_search(z3::check_result result) {
      if (result == z3::sat) {
          return -1;
      }
      if (latency_space_index == latency_Space.size()) {
          latency_space_index = 0;
          return -1;
      }
      int temp = latency_space_index;
      latency_space_index++;
      return latency_Space.at(temp);
  }

  int SMTBinaryScheduler::lat_reverse_linear_search(z3::check_result result) {
      if (reverse_search_init) {
          latency_space_index = (int) latency_Space.size() - 1;
          reverse_search_init = false;
      }
      //todo get this done
      return 42;
  }

  void SMTBinaryScheduler::calcLatencySpace() {
      latency_Space.clear();
      latency_Space.shrink_to_fit();
      if (min_Latency < (int)minII){
          min_Latency = (int)minII;
      }
      latency_Space.reserve((max_Latency - min_Latency)+1);
      for (int i = min_Latency; i <= max_Latency; i++){
          latency_Space.push_back(i);
      }
  }

  void SMTBinaryScheduler::print_ASAP_ALAP_restictions() {
      for (auto &it : start_times_simplification){
          cout << it.first.first->getName() << " <" << it.first.second << "> " << it.second << endl;
      }
  }

  bool SMTBinaryScheduler::unsat_check_shortcut() {
      for (auto &vIt : g.Vertices()){
          int count = 0;
          for(int lIt = 0; lIt < candidateLatency; lIt++){
              //cout << vIt->getName() << " <" << lIt << "> " << endl; //(int)start_times_simplification.at(std::make_pair((Vertex*)vIt, lIt)) << endl;
              count += (int)start_times_simplification.at(std::make_pair(vIt, lIt));
          }
          if (count == 0){
              return true;
          }
      }
      return false;
  }

  int SMTBinaryScheduler::lat_binary_search(z3::check_result satisfiable) {
      //ToDo: Some Bugs with latency space smaler or equal 2.
      //if (!quiet) { print_latency_space(0, (int)latency_Space.size()); }
      if (binary_search_init) {
          if (!quiet) { cout << "Init Binary Search..." << endl; }
          binary_search_init = false;
          latency_space_index = (int)latency_Space.size()/2;
          left_index = 0;
          right_index = (int)latency_Space.size()-1;
          return latency_Space.at(latency_space_index);
      }
      if (left_index < right_index) {
          switch (satisfiable) {
              case z3::sat:
                  //cout << "--> Sat <--" << endl;
                  right_index = latency_space_index;
                  latency_space_index = left_index + (right_index-left_index)/2;
                  break;
              case z3::unsat:
                  //cout << "--> Unsat <--" << endl;
                  left_index = latency_space_index + 1;
                  latency_space_index = left_index + (right_index-left_index)/2;
                  break;
              case z3::unknown:
                  //cout << "--> Unknown <--" << endl;
                  left_index = latency_space_index + 1;
                  latency_space_index = left_index + (right_index-left_index)/2;
                  break;
          }
          //if (!quiet) { cout << "Left Index: " << left_index << " Right Index: " << right_index << endl; }
          //if (!quiet) { cout << "Index: " << latency_space_index << endl; }
          //if (!quiet) { cout << "Returnvalue: " << latency_Space.at(latency_space_index) << endl; }
          return latency_Space.at(latency_space_index);
      }
      binary_search_init = true;
      return -1;
  }

  void SMTBinaryScheduler::print_latency_space(int l_index, int r_index) {
      for (int i = l_index; i < r_index; i++){
          cout << latency_Space.at(i) << " ";
      }
      cout << endl;
  }

  z3::check_result SMTBinaryScheduler::test_binary_search(int value_to_check, int target_value) {
      if (value_to_check < target_value){
          return z3::unsat;
      }
      return z3::sat;
  }

  void SMTBinaryScheduler::write_solving_times_to_file(deque<double> &times) {
      if (!design_name.empty()) {
          bool skip = true;
          reverse(design_name.begin(), design_name.end());
          cout << design_name << endl;
          for (int i = 0; i < design_name.size(); i++) {
              cout << design_name.at(i) << endl;
              if (design_name.at(i) == '/' and skip) {
                  design_name.at(i) = '_';
                  skip = false;
              } else if (design_name.at(i) == '/' and !skip) {
                  design_name = design_name.substr(4, i - 4);
                  break;
              }
          }
          reverse(design_name.begin(), design_name.end());

          string path = "SMTBenchmark/Solving_Times_" + design_name + ".csv";
          cout << path << endl;
          try {
              ofstream of;
              of.open(path);
              if (of.is_open()) {
                  of << "Latency,SolvingTime,\n";
                  int j = 0;
                  for (auto &it : times) {
                      of << latency_Space.at(j) << "," << it << ",\n";
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

  void SMTBinaryScheduler::calc_Max_Latency_Estimation() {

      //Check if max latency was set by user
      if (this->maxLatencyConstraint >= 0) {
          max_Latency = maxLatencyConstraint;
          return;
      }
      this->max_Latency = 0;
      int resMaxLat = 0;

      //Getting the current II mainly for better readability
      int current_II = II_space.at(II_space_index);

      //Getting earliest and latest possible Starttimes for a modulo schedule
      auto aslap = calc_ASAP_and_ALAP_mod_Schedule_with_sdc(g, resourceModel);
      if (scheduleFound){ return; }
      earliest_start_times = aslap.first;
      latest_start_times = aslap.second;

      //Algorithm to estimate max latency based on earliest and latest possible start times
      //as well as on ressource constraints.
      map<pair<Vertex*, int>, bool> vertex_timeslot;
      unordered_map<Vertex*, bool> checked;

      //Generates a matrix with the information wether a Vertex can be schedules in a timeslot or not,
      //based on dependency constraints.
      for (auto &v : g.Vertices()){
          checked.insert(std::make_pair(v,false));
          int t_asap = earliest_start_times.at(v);
          int t_alap = latest_start_times.at(v);
          for (int i = 0; i < t_asap; i++){
              vertex_timeslot[{v, i}] = false;
          }
          for (int i = t_asap; i <= t_alap; i++){
              vertex_timeslot[{v, i}] = true;
          }
          for (int i = t_alap + 1; i < mod_asap_length; i++){
              vertex_timeslot[{v, i}] = false;
          }
      }

      /*!--------------------
       * Print Methode
       *--------------------*/
      if ( !quiet ) {
          for (auto &r : resourceModel.Resources()) {
              if (r->isUnlimited()) {
                  continue;
              }
              for (auto &v : resourceModel.getVerticesOfResource(r)) {
                  cout << setw(18) << v->getName() << ": ";
                  for (int i = 0; i < mod_asap_length; i++) {
                      if (i % current_II == 0) {
                          cout << " ";
                      }
                      cout << vertex_timeslot.at({(Vertex *) v, i});
                  }
                  cout << endl;
              }
              cout << endl;
          }
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
          for (int i = 0; i < mod_asap_length; i++){
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
              for (int i = 0; i < mod_asap_length; i++) {
                  count += (int)vertex_timeslot.at(std::make_pair((Vertex*)v, i));
              }
              if (count == 0){
                  count2++;
              }
          }
          critical_ressources.push_back(count2);
      }

      int maximum = *std::max_element(critical_ressources.begin(),critical_ressources.end());

      /*!--------------------
       * Print Methode
       *--------------------*/
      if ( !quiet ) {
          for (auto &r : resourceModel.Resources()) {
              if (r->isUnlimited()) {
                  continue;
              }
              for (auto &v : resourceModel.getVerticesOfResource(r)) {
                  cout << setw(18) << v->getName() << ": ";
                  for (int i = 0; i < mod_asap_length; i++) {
                      if (i % II_space.at(II_space_index) == 0) {
                          cout << " ";
                      }
                      cout << vertex_timeslot.at(std::make_pair((Vertex *) v, i));
                  }
                  cout << endl;
              }
              cout << endl;
          }
      }

      max_Latency = maximum + mod_asap_length;
      if ( !quiet ) { cout << "Max Latency Suggestion: " << maximum + mod_asap_length << endl; }
  }

  pair <map<Vertex*,int>, map<Vertex*, int>> SMTBinaryScheduler::calc_ASAP_and_ALAP_mod_Schedule_with_sdc(Graph &gr, ResourceModel &resM) {

      //for (auto &r : resM.Resources()){
      //    r->setLimit(UNLIMITED, false);
      //}

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
      unordered_map<Vertex*, int>vertex_distance_asap;
      unordered_map<Vertex*, Vertex*> old_to_new_vertex_asap;
      unordered_map<Vertex*, Vertex*> new_to_old_vertex_asap;

      for (auto &v : gr.Vertices()){
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

      for (auto &e : gr.Edges()){
          auto v_src_alap = old_to_new_vertex_alap.at(&e->getVertexSrc());
          auto v_src_asap = old_to_new_vertex_asap.at(&e->getVertexSrc());
          auto v_dst_alap = old_to_new_vertex_alap.at(&e->getVertexDst());
          auto v_dst_asap = old_to_new_vertex_asap.at(&e->getVertexDst());
          sdc_graph_alap.createEdge(*v_dst_alap, *v_src_alap, -vertex_Latency_alap.at(v_src_alap)
                                                                              - e->getDelay()
                                                                              + e->getDistance() * this->II_space.at(II_space_index));
          sdc_graph_asap.createEdge(*v_src_asap, *v_dst_asap, -vertex_Latency_asap.at(v_src_asap)
                                                                              - e->getDelay()
                                                                              + e->getDistance() * this->II_space.at(II_space_index));

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
          throw (std::out_of_range("SMT_Sheduler: calc_ASAP_and_ALAP_mod_Schedule_with_sdc(), Bellmann-Ford, Step 2, ALAP Loop"));
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
          throw (std::out_of_range("SMT_Sheduler: calc_ASAP_and_ALAP_mod_Schedule_with_sdc(), Bellmann-Ford, Step 2, ASAP Loop"));
      }

      //Checking for negativ Cycles:
      for (auto &e : sdc_graph_alap.Edges()){
          auto t = &e->getVertexSrc();
          auto u = &e->getVertexDst();
          int weight = e->getDistance();
          if ((vertex_distance_alap.at(t) != INT32_MAX) && (vertex_distance_alap.at(t) + weight < vertex_distance_alap.at(u))){
              cout << "Negative Cycle Detected ALAP" << endl;
              // toDo II not feasible?
          }
      }
      for (auto &e : sdc_graph_asap.Edges()){
          auto t = &e->getVertexSrc();
          auto u = &e->getVertexDst();
          int weight = e->getDistance();
          if ((vertex_distance_asap.at(t) != INT32_MAX) && (vertex_distance_asap.at(t) + weight < vertex_distance_asap.at(u))){
              cout << "Negative Cycle Detected ASAP" << endl;
              // toDo II not feasible?
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
      auto valid_alap = verifyModuloScheduleSMT(gr, resM, startTimes_alap, II_space.at(II_space_index));
      string validstr;
      if ( !quiet ) {
          if (valid_alap) { validstr = "True"; } else { validstr = "False"; }
          cout << "---------------------------------------------------------" << endl;
          cout << "Valid Mod. Schedule for given Ressources: " << validstr << " / II: " << II_space.at(II_space_index) << endl;
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

      int j = 0;
      for (auto &it : startTimes_alap){
          cout << j++ << ": ";
          cout << it.first->getName() << " : " << it.second << endl;
      }

      //Checking if we are lucky and MOD-ASAP-Starttimes are already a valig modulo schedule:
      auto valid_asap = verifyModuloScheduleSMT(gr, resM, startTimes_asap, II_space.at(II_space_index));
      if (!quiet) {
          if (valid_asap) { validstr = "True"; } else { validstr = "False"; }
          cout << "---------------------------------------------------------" << endl;
          cout << "Valid Mod. Schedule for given Ressources: " << validstr << " / II: " << II_space.at(II_space_index) << endl;
          cout << "---------------------------------------------------------" << endl;
      }

      if ( !quiet and vertex_distance_alap.erase(&helper_alap) ){
          cout << "Successful deleted Helper_Vertex_ALAP: True" << endl;
      }else if ( !quiet ){
          cout << "Successful deleted Helper_Vertex_ALAP: False" << endl;
      }
      if ( !quiet and vertex_distance_asap.erase(&helper_asap) ){
          cout << "Successful deleted Helper_Vertex_ASAP: True" << endl;
      }else if ( !quiet ){
          cout << "Successful deleted Helper_Vertex_ASAP: False" << endl;
      }

      //Saving schedule length
      mod_alap_length = getScheduleLatency(vertex_distance_alap, new_to_old_vertex_alap);
      mod_asap_length = getScheduleLatency(vertex_distance_asap, new_to_old_vertex_asap);
      if (!quiet) { cout << "SDC-ALAP-Latency: " << mod_alap_length << endl; }
      if (!quiet) { cout << "SDC-ASAP-Latency: " << mod_asap_length << endl; }
      if ( mod_asap_length != mod_alap_length ){
          throw (HatScheT::Exception("SMT-Scheduler, calc_ASAP_and_ALAP_mod_Schedule_with_sdc(), mod_alap_length and not equal"));
      }
      min_Latency = mod_asap_length;

      //If a valid schedule is already found, just returning it.
      if (valid_asap and s_pref == schedule_preference::MOD_ASAP) {
          II = (double)II_space.at(II_space_index);
          scheduleFound = true;
          startTimes = startTimes_asap;
      } else if (valid_alap and s_pref == schedule_preference::MOD_ALAP) {
          II = (double)II_space.at(II_space_index);
          scheduleFound = true;
          startTimes = startTimes_alap;
      } else if (valid_asap){
          II = (double)II_space.at(II_space_index);
          scheduleFound = true;
          startTimes = startTimes_asap;
      } else if (valid_alap){
          II = (double)II_space.at(II_space_index);
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
          Vertex *v = vtPair.first;
          if ((vtPair.second + resourceModel.getVertexLatency(newtoold.at(v))) > maxTime) {
              maxTime = (vtPair.second + resourceModel.getVertexLatency(newtoold.at(v)));
          }
      }
      return maxTime;
  }

  //Copied from Verifier to get rid of the CERR outputs. No one likes errors!
  bool SMTBinaryScheduler::verifyModuloScheduleSMT(Graph &g, ResourceModel &rm, std::map<Vertex *, int> &schedule, int II)
  {
      if(II<=0){
          cout << "HatScheT.verifyModuloSchedule Error invalid II passed to verifier: "  << II << endl;
          return false;
      }
      if(schedule.empty()){
          cout << "HatScheT.verifyModuloSchedule Error empty schedule provided to verifier!"  << endl;
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
              cout << *e << " violated: " << S[i] << " + " << rm.getVertexLatency(i) << " + " << e->getDelay() << " <= " << S[j] << " + " << e->getDistance() << "*" << II << endl;
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
              auto  res = entry.first;
              auto &vs  = entry.second;

              ok = res->getLimit() == UNLIMITED || vs.size() <= res->getLimit();
              if (! ok) {
                  cout << "The following " << vs.size() << " vertices violate the resource limit " << res->getLimit() << " for " << res->getName() << " in congruence class " << m << " (mod " << II << "):" << endl;
                  for (auto v : vs){
                      cout << *v << " (t=" << S[v] << ")" << endl;
                  }
                  return false;
              }
          }
      }

      /* 3) TODO: cycle-time constraints are obeyed */

      return true;
  }

}
#endif
