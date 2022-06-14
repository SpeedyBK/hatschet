//
// Created by bkessler on 5/7/22.
//

#ifdef USE_Z3
#include "SMTBinaryScheduler.h"
#include "HatScheT/utility/Exception.h"

#include "HatScheT/scheduler/ASAPScheduler.h"
#include "HatScheT/scheduler/ALAPScheduler.h"

#include <z3++.h>

#include <iostream>
#include <iomanip>
#include <ctime>
#include <cmath>

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

      latSM = latSearchMethod::linear;
      if (latSM == latSearchMethod::linear) { this->latency_space_index = 0; }
      this->candidateLatency = -1;
      this->min_Latency = 0;
      this->max_Latency = -1;
      this->maxRuns = INT32_MAX;

  }

  void SMTBinaryScheduler::schedule() {

      cout << "Rec-Min-II: " << recMinII << endl;
      cout << "Res-Min-II: " << resMinII << endl;

    /*!
      IN : 1
      SUM_0 : 2
      PROD_1 : 2
      PROD_0 : 1
      CONST_A : 0
      CONST_B : 0
      SUM_1 : 3
      OUT : 4
    */

      clock_t start, end;

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
              if (latSM == latSearchMethod::linear) { candidateLatency = lat_linear_search(sati); }
              if (latSM == latSearchMethod::binary) { candidateLatency = lat_binary_search(sati); }

              if (candidateLatency == -1) {
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
              if (!quiet) {
                  cout << "-->" << sati << "<--" << endl << "Solving Time: " << fixed
                       << double(end - start) / double(CLOCKS_PER_SEC)
                       << setprecision(5) << " sec " << endl;
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
      //cout << s << endl << "--------------------------------" <<endl;
      m = s.get_model();
      //print_solution(m);
      parse_schedule(m);
      this->scheduleFound = true;
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
      // check if max latency was set by user
      if (this->max_Latency >= 0) return;
      this->max_Latency = 0;
      int resMaxLat = 0;
      for (auto &r : resourceModel.Resources()){
          if (r->getLatency() > resMaxLat){
              resMaxLat = r->getLatency();
          }
      }

      max_Latency = (resMaxLat+1) * (int)g.getNumberOfVertices();
      if (!quiet) { cout << "Max Latency: " << max_Latency << endl; }
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
              //if (i > latest_start_times.at(v)+offset){
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
          return II_space.at(0);
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

  void SMTBinaryScheduler::calcLatencySpace() {
      latency_Space.clear();
      latency_Space.shrink_to_fit();
      if (min_Latency < (int)minII){
          min_Latency = (int)minII;
      }
      latency_Space.reserve(max_Latency - min_Latency);
      for (int i = min_Latency; i < max_Latency; i++){
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
      return -1;
  }

  void SMTBinaryScheduler::print_latency_space(int l_index, int r_index) {
      for (int i = l_index; i < r_index; i++){
          cout << latency_Space.at(i) << " ";
      }
      cout << endl;
  }

}
#endif

