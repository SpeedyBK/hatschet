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
#include <HatScheT/utility/writer/DotWriter.h>
#include "HatScheT/utility/Verifier.h"

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
      latSM = latSearchMethod::binary;
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

      cout << "Rec-Min-II: " << recMinII << endl;
      cout << "Res-Min-II: " << resMinII << endl;
      cout << "Max. II: " << maxII << endl;

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

      //Calculating Latency-Space:
      find_earliest_start_times();
      calcMaxLatency();
      if (scheduleFound){ return; }
      calcLatencySpace();
      find_latest_start_times();

      candidateLatency = min_Latency;
      if (!quiet) { cout << "Max-Latency : " << max_Latency << endl; }
      if (!quiet) { cout << "Min-Latency : " << min_Latency << endl; }
      if (!quiet) { cout << "First-Try-Latency : " << candidateLatency << endl; }

      int candidateII=0;
      int candidateIIOLD;
      int i = 0;
      while (i < maxRuns) {
          candidateIIOLD = candidateII;
          candidateII = ii_linear_search(sati);

          if (candidateII == -1){
              break;
          }
          if (!quiet) { cout << "Trying II: " << candidateII << endl; }
          calcLatencySpace();
          while (true){
              cout << " -------------------------------- " << endl;
              int old_latency = candidateLatency;
              if (latSM == latSearchMethod::linear) { candidateLatency = lat_linear_search(sati); }
              if (latSM == latSearchMethod::binary) { candidateLatency = lat_binary_search(sati); }
              if (latSM == latSearchMethod::reverse_linear) { candidateLatency = lat_reverse_linear_search(sati); }

              if (candidateLatency == -1 or old_latency == candidateLatency ) {
                  candidateLatency = 0;
                  break;
              }

              if (!quiet) { cout << "Trying Latency: " << candidateLatency << endl; }
              find_latest_start_times();
              if (!quiet) { cout << "Generating b-variables... " << endl; }
              generate_b_variables();
              //if (!quiet) { print_b_variables(); }

              s.reset();
              s.set(p);

              if (!quiet) { cout << "Prohibit early starts... " << endl; }
              prohibit_to_early_starts_and_add(s);
              if (!quiet) { cout << "Prohibit late starts... " << endl; }
              prohibit_to_late_starts_and_add(s);

              if (unsat_check_shortcut()){
                  sati = z3::unsat;
                  continue;
              }

              //if (!quiet) { print_ASAP_ALAP_restictions(); }

              if (!quiet) { cout << "Add one-slot-constraints... " << endl; }
              add_one_slot_constraints_to_solver(s);

              if (!quiet) { cout << "Add resource-constraints... " << endl; }
              add_resource_limit_constraint_to_solver(s, candidateII);

              if (!quiet) { cout << "Add dependency-constraints... " << endl; }
              set_b_variables(s, candidateII);

              if (!quiet) { cout << "System of " << s.assertions().size() << " assertions..." << endl; }
              if (!quiet) { cout << "Solving... " << endl; }
              start = clock();
              sati = s.check();
              end = clock();
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

  void SMTBinaryScheduler::find_earliest_start_times() {

      map<Resource*, int> original_limits;
      for(auto &r : resourceModel.Resources()){
          original_limits[r] = r->getLimit();
          r->setLimit(UNLIMITED, false);
      }

      ASAPScheduler asap(g, resourceModel);
      asap.schedule();
      this->earliest_start_times = asap.getSchedule();
      this->min_Latency = asap.getScheduleLength();

      for(auto &r : resourceModel.Resources()){
          r->setLimit(original_limits.at(r), false);
      }
  }

  void SMTBinaryScheduler::find_latest_start_times() {
      map<Resource*, int> original_limits;
      for(auto &r : resourceModel.Resources()){
          original_limits[r] = r->getLimit();
          r->setLimit(UNLIMITED, false);
      }

      ALAPScheduler alap(g, resourceModel);
      alap.schedule();
      auto alap_schedule = alap.getSchedule();
      auto alap_schedule_length = alap.getScheduleLength();
      latest_start_times = alap_schedule;

      for (auto &it: latest_start_times) {
          this->latest_start_times.at(it.first) = it.second - alap_schedule_length + this->candidateLatency;
      }

      for (auto &r : resourceModel.Resources()) {
          r->setLimit(original_limits.at(r), false);
      }
  }

  void SMTBinaryScheduler::calcMaxLatency() {

      calc_max_latency_with_sdc(g, resourceModel);
      if (scheduleFound){ return; }

      // check if max latency was set by user
      if (this->maxLatencyConstraint >= 0) {
          max_Latency = maxLatencyConstraint;
          return;
      }
      this->max_Latency = 0;
      int resMaxLat = 0;

      int current_II = II_space.at(II_space_index);

      map<Resource*, int> original_limits;
      for(auto &r : resourceModel.Resources()){
          original_limits[r] = r->getLimit();
          r->setLimit(UNLIMITED, false);
      }

      ASAPScheduler asap (this->g, this->resourceModel);
      asap.schedule();
      auto asap_schedule = asap.getSchedule();

      if (!quiet) { cout << "ASAP-Shedule Lenghth: " << asap.getScheduleLength() << endl; }

      ALAPScheduler alap (this->g, this->resourceModel);
      alap.schedule();
      auto alap_schedule = alap.getSchedule();

      if (!quiet) { cout << "ALAP-Shedule Lenghth: " << alap.getScheduleLength() << endl; }

      for(auto &r : resourceModel.Resources()){
          r->setLimit(original_limits.at(r), false);
      }

      map<pair<Vertex*, int>, bool> vertex_timeslot;
      unordered_map<Vertex*, bool> checked;


      int schedule_length = asap.getScheduleLength();
      for (auto &v : g.Vertices()){
          checked.insert(std::make_pair(v,false));
          int t_asap = asap_schedule.at(v);
          int t_alap = alap_schedule.at(v);
          for (int i = 0; i < t_asap; i++){
              vertex_timeslot[{v, i}] = false;
          }
          for (int i = t_asap; i <= t_alap; i++){
              vertex_timeslot[{v, i}] = true;
          }
          for (int i = t_alap + 1; i < schedule_length; i++){
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
                  for (int i = 0; i < schedule_length; i++) {
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

      for (auto &r : resourceModel.Resources()){
          if (r->isUnlimited()){
              continue;
          }
          vector<int>used_FU_in_Modslot;
          used_FU_in_Modslot.resize(current_II);
          for (int i = 0; i < schedule_length; i++){
              for (auto &cvp : resourceModel.getVerticesOfResource(r)){
                  auto v = (Vertex*) cvp;
                  if (!vertex_timeslot.at(std::make_pair(v, i))){
                      continue;
                  }
                  //Check conflicts with other mod-slots.
                  if (used_FU_in_Modslot.at(i%current_II) < r->getLimit() and !checked.at(v)){
                      cout << v->getName() << endl;
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
      deque<int>critical_ressources;
      for (auto &r : resourceModel.Resources()) {
          if (r->isUnlimited()){
              continue;
          }
          int count2 = 0;
          for (auto &v : resourceModel.getVerticesOfResource(r)) {
              int count = 0;
              for (int i = 0; i < schedule_length; i++) {
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
                  for (int i = 0; i < schedule_length; i++) {
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

      max_Latency = maximum + schedule_length + 1;
      if ( !quiet ) { cout << "Max Latency Suggestion: " << maximum + schedule_length + 1 << endl; }
      //throw(HatScheT::Exception("Abort"));
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

  z3::expr *SMTBinaryScheduler::get_b_variable(Vertex *v, int i) {
      auto key = std::make_pair(v, i);
      try {
          return &b_variables.at(key);
      }catch(std::out_of_range&){
          cout << "Out_of_Range: " << v->getName() << " - " << i << endl;
          throw (HatScheT::Exception("SMT Scheduler: get_b_variable std::out_of_range"));
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

  void SMTBinaryScheduler::calc_max_latency_with_sdc(Graph &gr, ResourceModel &resM) {

      //for (auto &r : resM.Resources()){
      //    r->setLimit(UNLIMITED);
      //}

      //Create SDC-Graphs:
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
          sdc_graph_alap.createEdge(*v_dst_alap, *v_src_alap, -vertex_Latency_alap.at(v_src_alap)+e->getDistance() * this->II_space.at(II_space_index)); //Evtl e->Distance * II
          sdc_graph_asap.createEdge(*v_src_asap, *v_dst_asap, -vertex_Latency_asap.at(v_src_asap)+e->getDistance() * this->II_space.at(II_space_index));
      }

      //Bellmann-Ford Step 1:
      //Create Helper Vertex and connect to all other Vertices with distance 0:
      //Initialization of Bellmann-Ford Algorithm to solve SDC:
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
          throw (std::out_of_range("SMT_Sheduler: calc_max_latency_with_sdc(), Bellmann-Ford, Step 2, ALAP Loop"));
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
          throw (std::out_of_range("SMT_Sheduler: calc_max_latency_with_sdc(), Bellmann-Ford, Step 2, ASAP Loop"));
      }

      //Checking for negativ Cycles:
      for (auto &e : sdc_graph_alap.Edges()){
          auto t = &e->getVertexSrc();
          auto u = &e->getVertexDst();
          int weight = e->getDistance();
          if ((vertex_distance_alap.at(t) != INT32_MAX) && (vertex_distance_alap.at(t) + weight < vertex_distance_alap.at(u))){
              cout << "Negative Cycle Detected ALAP" << endl;
          }
      }
      for (auto &e : sdc_graph_asap.Edges()){
          auto t = &e->getVertexSrc();
          auto u = &e->getVertexDst();
          int weight = e->getDistance();
          if ((vertex_distance_asap.at(t) != INT32_MAX) && (vertex_distance_asap.at(t) + weight < vertex_distance_asap.at(u))){
              cout << "Negative Cycle Detected ASAP" << endl;
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

      startTimes_alap.clear();
      startTimes_asap.clear();

      if ( !quiet ) { cout << "Latest possible Starttimes:" << endl; }
      for (auto &it : vertex_distance_alap) {
          if (it.first == &helper_alap){
              continue;
          }
          it.second += abs(min);
          startTimes_alap[new_to_old_vertex_alap[it.first]]=it.second;
          if ( !quiet) { cout << it.first->getName() << ": " << it.second << endl; }
      }

      auto valid_alap = verifyModuloSchedule(gr, resM, startTimes_alap, II_space.at(II_space_index));
      string validstr;
      if ( !quiet ) {
          if (valid_alap) { validstr = "True"; } else { validstr = "False"; }
          cout << "------------------------------------------------------------" << endl;
          cout << "Valid Mod. Schedule for unlimited Ressources: " << validstr << " / II: " << II_space.at(II_space_index) << endl;
          cout << "------------------------------------------------------------" << endl;
          cout << "Earliest possible Starttimes:" << endl;
      }
      for (auto &it : vertex_distance_asap) {
          if (it.first == &helper_asap) {
              continue;
          }
          it.second = abs(it.second);
          startTimes_asap[new_to_old_vertex_asap[it.first]] = it.second;
          if (!quiet) { cout << it.first->getName() << ": " << it.second << endl; }
      }

      auto valid_asap = verifyModuloSchedule(gr, resM, startTimes_asap, II_space.at(II_space_index));
      if (!quiet) {
          if (valid_asap) { validstr = "True"; } else { validstr = "False"; }
          cout << "------------------------------------------------------------" << endl;
          cout << "Valid Mod. Schedule for unlimited Ressources: " << validstr << " / II: " << II_space.at(II_space_index) << endl;
          cout << "------------------------------------------------------------" << endl;

          cout << "Successful deleted Helper_Vertex_ALAP: " << vertex_distance_alap.erase(&helper_alap) << endl;
          cout << "Successful deleted Helper_Vertex_ASAP: " << vertex_distance_asap.erase(&helper_asap) << endl;

          cout << "SDC-ALAP-Latency: " << getScheduleLatency(vertex_distance_alap, new_to_old_vertex_alap) << endl;
          cout << "SDC-ASAP-Latency: " << getScheduleLatency(vertex_distance_asap, new_to_old_vertex_asap) << endl;
      }


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

}
#endif
