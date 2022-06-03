//
// Created by bkessler on 5/7/22.
//

#ifdef USE_Z3
#include "SMTSmartieScheduler.h"
#include "HatScheT/utility/Exception.h"

#include "HatScheT/scheduler/ASAPScheduler.h"
#include "HatScheT/scheduler/ALAPScheduler.h"

#include <z3++.h>

#include <iostream>
#include <iomanip>
#include <ctime>
#include <cmath>

namespace HatScheT {

  SMTSmartieScheduler::SMTSmartieScheduler(Graph &g, ResourceModel &resourceModel) : SchedulerBase(g, resourceModel) {
      // reset previous solutions
      II = -1;
      this->timeouts = 0;
      startTimes.clear();
      scheduleFound = false;
      computeMinII(&g, &resourceModel);
      minII = ceil(minII);
      computeMaxII(&g, &resourceModel);

      quiet = true;
      iiSM = iiSearchMethod::binary;

      for (int i = (int)minII; i <= (int)maxII; i++){
          this->II_space.push_back(i);
      }
      II_space_index = 0;

      this->upperBound = 0;
      this->min_Latency = 0;
      this->offset = 125;
      this->maxRuns = INT32_MAX;

  }

  void SMTSmartieScheduler::schedule() {

      clock_t start, end;

      z3::solver s(c);
      auto sati = z3::unknown;
      z3::model m(c);

      find_earliest_start_times();
      find_latest_start_times();
      upperBound = min_Latency + offset;
      cout << upperBound << endl;

      int candidateII;
      int candidateIIOLD;
      if (iiSM == iiSearchMethod::binary) { candidateII = ii_binary_search(sati); }
      if (iiSM == iiSearchMethod::linear) { candidateII = ii_linear_search(sati); }

      int i = 0;
      while (i < maxRuns) {
          candidateIIOLD = candidateII;
          if (iiSM == iiSearchMethod::binary) { candidateII = ii_binary_search(sati); }
          if (iiSM == iiSearchMethod::linear) { candidateII = ii_linear_search(sati); }

          if (candidateII == -1){
              break;
          }
          if (!quiet) { cout << "Trying II: " << candidateII << endl; }
          s.reset();
          generate_b_variables();
          //if (!quiet) { print_b_variables(); }

          prohibit_to_early_starts_and_add(s);
          prohibit_to_late_starts_and_add(s);

          add_one_slot_constraints_to_solver(s);
          add_resource_limit_constraint_to_solver(s, candidateII);

          set_b_variables(s, candidateII);
          start = clock();
          sati = s.check();
          end = clock();
          if(!quiet) {
              cout << "-->" << sati << "<--" << endl << "Solving Time: " << fixed
                   << double(end - start) / double(CLOCKS_PER_SEC)
                   << setprecision(5) << " sec " << endl;
          }
          i++;
      }

      II = candidateIIOLD;
      //cout << s << endl << "--------------------------------" <<endl;
      m = s.get_model();
      print_solution(m);
      parse_schedule(m);
  }

  void SMTSmartieScheduler::find_earliest_start_times() {

      map<Resource*, int> original_limits;
      for(auto &r : resourceModel.Resources()){
          original_limits[r] = r->getLimit();
          r->setLimit(UNLIMITED);
      }

      ASAPScheduler asap(g, resourceModel);
      asap.schedule();
      this->earliest_start_times = asap.getSchedule();
      this->min_Latency = asap.getScheduleLength();

      for(auto &r : resourceModel.Resources()){
          r->setLimit(original_limits.at(r));
      }
  }

  void SMTSmartieScheduler::find_latest_start_times() {
      map<Resource*, int> original_limits;
      for(auto &r : resourceModel.Resources()){
          original_limits[r] = r->getLimit();
          r->setLimit(UNLIMITED);
      }

      ALAPScheduler alap(g, resourceModel);
      alap.schedule();
      this->latest_start_times = alap.getSchedule();

      for (auto &it: latest_start_times) {
          it.second += offset;
      }

      for (auto &r : resourceModel.Resources()) {
          r->setLimit(original_limits.at(r));
      }
  }

  void SMTSmartieScheduler::prohibit_to_early_starts_and_add(z3::solver &s) {
      for (auto &v : g.Vertices()){
          for (int i = 0; i < upperBound; i++){
              if (i < earliest_start_times.at(v)){
                  s.add(!*get_b_variable(v, i));
              }
          }
      }
  }

  void SMTSmartieScheduler::prohibit_to_late_starts_and_add(z3::solver &s) {
      for (auto &v : g.Vertices()){
          for (int i = 0; i < upperBound; i++){
              if (i > latest_start_times.at(v)+offset){
                  s.add(!*get_b_variable(v, i));
              }
          }
      }
  }

  z3::expr *SMTSmartieScheduler::get_b_variable(Vertex *v, int i) {
      auto key = std::make_pair(v, i);
      return &b_variables.at(key);
  }

  void SMTSmartieScheduler::generate_b_variables() {
      for (auto &it : g.Vertices()){
          for (int i = 0; i < upperBound; i++){
              auto key = std::make_pair(it, i);
              std::stringstream name;
              name << it->getName() << "->" << i;
              z3::expr e(c.bool_const(name.str().c_str()));
              b_variables.insert({key, e});
          }
      }
  }

  void SMTSmartieScheduler::set_b_variables(z3::solver& s, const int &candidateII) {
      //tj + distance * this->candidateII - ti - lSrc - delay >= 0

      for(auto &e : g.Edges()){
          auto *vSrc = &e->getVertexSrc();
          auto *vDst = &e->getVertexDst();
          auto lSrc = this->resourceModel.getVertexLatency(vSrc);
          auto distance = e->getDistance();
          auto delay = e->getDelay();
          for(int ti = 0; ti < upperBound; ti++){
              for (int tj = 0; tj < upperBound; tj++){
                  if (tj + distance * candidateII - ti - lSrc - delay >= 0){
                      //No Conflict... Do nothing
                      continue;
                  }else{
                      //Conflict... Not both Operations in this timeslot;
                      s.add(!*get_b_variable(vSrc, ti) || !*get_b_variable(vDst, tj));
                  }
              }
          }
      }
  }

  void SMTSmartieScheduler::add_one_slot_constraints_to_solver(z3::solver &s) {
      for (auto &it : g.Vertices()){
          if (!resourceModel.getResource(it)->isUnlimited()) {
              vector<int> coefficients;
              coefficients.reserve(upperBound);
              z3::expr_vector b_expressions(c);
              for (int i = 0; i < upperBound; i++) {
                  coefficients.push_back(1);
                  b_expressions.push_back(*get_b_variable(it, i));
              }
              s.add(z3::pbeq(b_expressions, &coefficients[0], 1));
              coefficients.clear();
              coefficients.shrink_to_fit();
          }
      }
  }

  void SMTSmartieScheduler::add_resource_limit_constraint_to_solver(z3::solver &s, int candidateII) {
      for (auto &it : resourceModel.Resources()) {
          if (it->isUnlimited()) {
              continue;
          }
          for (int i = 0; i < candidateII; i++) {
              set<const Vertex *> vSet = resourceModel.getVerticesOfResource(it);
              z3::expr_vector b_expressions(c);
              for (auto &vIt : vSet) {
                  for (int j = 0; j < upperBound; j++) {
                      if ((j % candidateII) != i) {
                          continue;
                      }
                      b_expressions.push_back(*get_b_variable((Vertex *) vIt, j));
                  }
              }
              s.add(z3::atmost(b_expressions, it->getLimit()));
          }
      }
  }

  void SMTSmartieScheduler::print_b_variables() {
      for (auto &it:g.Vertices()){
          for (int i = 0; i < upperBound; i++){
              cout << *get_b_variable(it, i) << endl;
          }
      }
  }

  void SMTSmartieScheduler::print_solution(z3::model &m) {
      for (auto &it : g.Vertices()){
          for (int i = 0; i < upperBound; i++) {
              auto val = m.eval(*get_b_variable(it, i));
              if (val.is_true()){
                  cout << it->getName() << " - " << i << ": " <<  val << endl;
              }
          }
      }
  }

  void SMTSmartieScheduler::parse_schedule(z3::model &m) {
      for (auto &it : g.Vertices()){
          for (int i = 0; i < upperBound; i++) {
              auto val = m.eval(*get_b_variable(it, i));
              if (val.is_true()){
                  this->startTimes[it] = i;
              }
          }
      }
  }

  int SMTSmartieScheduler::ii_binary_search(z3::check_result result) {
      if (II_space.size() > 2) {
          if (result == z3::unknown) {
              II_space_index = (int) (II_space.size() / 2);
              return II_space.at(II_space_index);
          } else if (result == z3::unsat) {
              //In this case, we know, that we have to search in the right half of our Array. Excluding the last Value.
              vector<int> temp;
              temp.resize(II_space.size() - (II_space_index));
              for (int i = II_space_index; i < II_space.size(); i++) {
                  temp.at(i - (II_space_index)) = II_space.at(i);
              }
              II_space.clear();
              II_space.resize(temp.size());
              II_space = temp;
              II_space_index = floor(II_space.size() / 2);
              return II_space.at(II_space_index);
          } else if (result == z3::sat) {
              //In this case, we know, that we have to search in the left half of our Array. Including the last Value.
              vector<int> temp;
              temp.resize(II_space_index + 1);
              for (int i = 0; i <= II_space_index; i++) {
                  temp.at(i) = II_space.at(i);
              }
              II_space.clear();
              II_space.resize(temp.size());
              II_space = temp;
              II_space_index = floor(II_space.size() / 2);
              return II_space.at(II_space_index);
          }
      } else if (II_space.size() == 2) {
          int val = 0;
          if (result == z3::sat) {
              val = II_space.at(0);
          } else {
              val = II_space.at(1);
          }
          II_space.clear();
          II_space.shrink_to_fit();
          return val;
      }else if(II_space.size() == 1){
          int val = II_space.at(0);
          II_space.clear();
          II_space.shrink_to_fit();
          return val;
      }else{
          return -1;
      }
      throw (HatScheT::Exception("SMT-Scheduler, II-Binary search, this should never happen!"));
  }

  int SMTSmartieScheduler::ii_linear_search(z3::check_result result) {
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
}
#endif

