//
// Created by bkessler on 5/7/22.
//

#ifdef USE_Z3
#include "SMTModScheduler.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/Exception.h"

#include "HatScheT/scheduler/ASAPScheduler.h"

#include <z3++.h>

#include <iostream>
#include <map>
#include <cmath>

namespace HatScheT {

  SMTModScheduler::SMTModScheduler(Graph &g, ResourceModel &resourceModel) : SchedulerBase(g, resourceModel) {
      // reset previous solutions
      II = -1;
      this->timeouts = 0;
      startTimes.clear();
      scheduleFound = false;
      computeMinII(&g, &resourceModel);
      minII = ceil(minII);
      computeMaxII(&g, &resourceModel);
      quiet = true;

      encode = encodingMode::pbeq;
      mode = 7;

      auto as = ASAPScheduler(g, resourceModel);
      as.schedule();
      this->maxLatency = as.getScheduleLength();
  }

  void SMTModScheduler::build_Data_Structure() {
      if(!quiet){ cout << "Creating data-structure..." << endl; }
      create_t_Variables();
      this->data_Dependency_Constraints = build_Dependency_Constraints();
      create_b_variables();
  }

  void SMTModScheduler::create_t_Variables() {
      if (!quiet) { cout << "\nCreating t - variables:" << endl;
                    cout << "Index: vertex: expression:" << endl;}

      int i = 0;
      for (auto &it : g.Vertices()) {
          if (!quiet) { cout << i << ": " << it->getName() << " : "; }
          std::stringstream expr_name;
          expr_name << it->getName();
          t_Variables.push_back(c.int_const(expr_name.str().c_str()));
          vertex_t_Variables_Map.insert({it, i});
          i++;
          if (!quiet) { cout << t_Variables.back() << endl; }
      }
  }

  pair <deque<z3::expr>, deque<z3::expr>> SMTModScheduler::build_Dependency_Constraints(){

      if (!quiet) { cout << "\nCreating dependency contraints..." << endl; }

      deque<z3::expr> edgesWithoutDistance;
      deque<z3::expr> edgesWithDistance;

      int i = 0;
      for(auto &it : g.Edges()){
          if (it->getDistance() == 0){
              std::stringstream expr_name;
              expr_name << "Edge_" << it->getId();
              z3::expr e (c.int_const(expr_name.str().c_str()));
              e = (  t_Variables.at(vertex_t_Variables_Map.at(&it->getVertexDst()))
                   - t_Variables.at(vertex_t_Variables_Map.at(&it->getVertexSrc()))
                   >= resourceModel.getVertexLatency(&it->getVertexSrc()));
              //if (!quiet) { cout << e << endl; }
              edgesWithoutDistance.push_back(e);
              expr_name.clear();
          }else{
              std::stringstream expr_name;
              expr_name << "Edge_" << it->getId();
              z3::expr e (c.int_const(expr_name.str().c_str()));
              e = (  t_Variables.at(vertex_t_Variables_Map.at(&it->getVertexDst()))
                   - t_Variables.at(vertex_t_Variables_Map.at(&it->getVertexSrc()))
                   + (int) II * it->getDistance()
                   >= resourceModel.getVertexLatency(&it->getVertexSrc()));
              //if (!quiet) { cout << e << endl; }
              edgesWithDistance.push_back(e);
              exprToEdgeMap.insert({i, it});
              i++;
              expr_name.clear();
          }
      }
      return {edgesWithoutDistance, edgesWithDistance};
  }

  void SMTModScheduler::create_b_variables() {

      b_variables.clear();

      deque<deque<b_variable>> collumn;
      deque<b_variable> line;

      if (!quiet) { cout << "\nCreating b_variables:" << endl; }
      if (!quiet) { cout << "Vertex: Moduloslot: Resource:\n"; }

      int r = 0;
      for (const auto &res : resourceModel.Resources()){
          if (res->getLimit() == UNLIMITED){
              continue;
          }
          resource_to_b_vairable_index[res] = r;
          r++;
          for (int tau = 0; tau < this->II; tau++){
              int v = 0;
              for(const auto &ver : g.Vertices()){
                    if(resourceModel.getResource(ver) == res) {
                        vertex_to_b_vairable_index[ver] = v;
                        std::stringstream expr_name;
                        expr_name.clear();
                        expr_name << "[" << ver->getName() << "] [" << tau << "] [" << res->getName() << "]";
                        z3::expr e(c.bool_const(expr_name.str().c_str()));
                        b_variable b(tau, e);
                        b.vertexname = ver->getName();
                        b.resourcename = res->getName();
                        line.push_back(b);
                        v++;
                    }
              }
              collumn.push_back(line);
              line.clear();
          }
          b_variables.push_back(collumn);
          collumn.clear();
      }
  }

  SMTModScheduler::b_variable *SMTModScheduler::get_b_var(Vertex *v, const Resource *r, int slot) {
      try {
          return &b_variables.at(resource_to_b_vairable_index.at(const_cast<Resource *>(r))).at(slot).at(
              vertex_to_b_vairable_index.at(v));
      } catch (std::out_of_range &) {
          cout << "\nTrying to get non existing b_variable" << endl;
          cout << "[" << v->getName() << "] [" << r->getName() << "] [" << slot << "]" << endl;
          throw HatScheT::Exception("SMTModScheduler::b_variable *SMTModScheduler::get_b_var");
      }
  }

  void SMTModScheduler::print_b_variables() {
      for (const auto &oit : b_variables){
          for (const auto &mit : oit){
              for(auto &iit : mit){
                  cout << iit.b_var << endl;
              }
          }
      }
  }

  void SMTModScheduler::print_data_dependency_Constraints() {
      cout << "\nDependency Constraints (not II-Dependent):" << endl;
      for (auto &it : data_Dependency_Constraints.first) {
          cout << it << endl;
      }
      cout << "\nDependency Constraints (II-Dependent):" << endl;
      for (auto &it : data_Dependency_Constraints.second) {
          cout << it << endl;
      }
  }

  void SMTModScheduler::add_Constraints_to_solver(z3::solver &s, deque<z3::expr, allocator<z3::expr>> &eVec) {
      for (auto &it : eVec) {
          s.add(it.simplify());
      }
  }

  void SMTModScheduler::add_one_slot_constraints_to_solver(z3::solver &s) {

      for (auto &it : g.Vertices()) {
          z3::expr_vector one_Slot_Constraint(c);
          for (int i = 0; i < II; i++) {
              if (!resourceModel.getResource(it)->isUnlimited()) {
                  one_Slot_Constraint.push_back(get_b_var(it, resourceModel.getResource(it), i)->b_var);
              }
          }
          if (!quiet) { cout << "One Slot Constraint Size:" << one_Slot_Constraint.size() << endl; }
          if (!resourceModel.getResource(it)->isUnlimited()) {
              if (!one_Slot_Constraint.empty()) {
                  if (encode == encodingMode::pbeq){
                      vector<int> coeffs;
                      coeffs.reserve(one_Slot_Constraint.size());
                      for (int i = 0; i < one_Slot_Constraint.size(); i++) {
                          coeffs.push_back(1);
                      }
                      z3::expr e = pbeq(one_Slot_Constraint, &coeffs[0], 1);
                      if (!quiet) { cout << e << endl; }
                      s.add(e.simplify());
                      coeffs.clear();
                  }else if (encode == encodingMode::ite) {
                      z3::expr e = sum(one_Slot_Constraint) == 1;
                      if (!quiet) { cout << e << endl; }
                      s.add(e.simplify());
                  }
              }
          }
      }
  }

  void SMTModScheduler::add_resource_limit_constraint_to_solver(z3::solver &s){
      for (auto &rit : resourceModel.Resources()) {
          if (!rit->isUnlimited()) {
              for (int i = 0; i < II; i++) {
                  z3::expr_vector resource_limit_constraint(c);
                  for (auto &it : g.Vertices()) {
                      if (resourceModel.getResource(it)==rit) { //Du alte drecksau... fick dich!
                          resource_limit_constraint.push_back(
                              get_b_var(it, resourceModel.getResource(it), i)->b_var);
                      }
                  }
                  z3::expr e = c.bool_val("Sum"); //TODO ...

                  if (encode == encodingMode::ite) {
                      e = sum(resource_limit_constraint) <= rit->getLimit();
                  }else if(encode == encodingMode::pbeq) {
                      e = z3::atmost(resource_limit_constraint, rit->getLimit());
                  }
                  if (!quiet) { cout << e << endl; }
                  s.add(e.simplify());
              }
          }
      }
  }

  void SMTModScheduler::add_linking_constraints_to_solver(z3::solver &s) {

      int count = 0;
      for(auto &it : g.Vertices()) {
          if (!resourceModel.getResource(it)->isUnlimited()) {
              std::stringstream expr_name;
              expr_name << "y_" << it->getId();
              z3::expr y(c.int_const(expr_name.str().c_str()) * (int) II);
              //if (encode == encodingMode::ite) {
                  z3::expr_vector b_times_tau_vec(c);
                  z3::expr zero(c.int_val(0));
                  z3::expr one(c.int_val(1));

                  for (int i = 1; i < (int) II; i++) {
                      std::stringstream name;
                      name << "Li_" << count;
                      count++;
                      z3::expr bi(c.int_const(name.str().c_str()));
                      bi = (z3::ite(get_b_var(it, resourceModel.getResource(it), i)->b_var, one, zero) * i);
                      b_times_tau_vec.push_back(bi);
                  }
                  z3::expr sum_of_b_times_tau = sum(b_times_tau_vec);
                  if (!quiet) { cout << "-- " << sum_of_b_times_tau << endl; }
                  s.add(t_Variables.at(vertex_t_Variables_Map.at(it)) == y + sum_of_b_times_tau);
              //}
              /*else if( encode == encodingMode::pbeq ){
                  z3::expr_vector b_vec(c);
                  vector<int> coeffs;
                  coeffs.reserve((int)II);
                  for (int i = 0; i < (int)II; i++ ){
                      coeffs.push_back(i);
                      b_vec.push_back(get_b_var(it, resourceModel.getResource(it), i)->b_var);
                  }
                  z3::expr t_y = (t_Variables.at(vertex_t_Variables_Map.at(it))) - y;
                  z3::expr linkingConstraint = z3::pbeq(b_vec, &coeffs[0], t_y.get_numeral_int());
                  s.add(linkingConstraint);
                  coeffs.clear();
                  coeffs.shrink_to_fit();
              }*/
          }
      }
  }

  void SMTModScheduler::set_max_latency(z3::solver &s, int maxLat) {
      for (auto &it : g.Vertices()){
          s.add(t_Variables.at(vertex_t_Variables_Map.at(it)) < maxLat && t_Variables.at(vertex_t_Variables_Map.at(it)) >= 0);
      }
  }

  void SMTModScheduler::create_and_add_latency_constraints(z3::solver &s) {
      int maxlat = INT32_MIN;
      int minlat = INT32_MAX;
      for (auto &it : startTimes){
          it.second > maxlat ? maxlat = it.second : maxlat = maxlat;
          it.second < minlat ? minlat = it.second : minlat = minlat;
      }
      for(auto &it : t_Variables){
          s.add(it < maxlat && it >= minlat);
      }
  }

  void SMTModScheduler::schedule() {

      z3::solver s(c);

      II = minII;

      build_Data_Structure();

      if (!quiet) {
          print_b_variables();
          print_data_dependency_Constraints();
      }

      if (!quiet) { cout << "\nAdding dependency constraints (not II-Dependent) to solver." << endl; }
      add_Constraints_to_solver(s, data_Dependency_Constraints.first);

      auto satisfiable = s.check();
      if (!quiet) { cout << "\n" << s.get_model() << "\n"; }
      if (!quiet) { cout << "\nsolving... " << satisfiable << endl; }
      if (!quiet) { cout << "Creating fallback-point." << endl; }
      s.push();

      if (!quiet) { cout << "\nAdding dependency constraints (not II-Dependent) to solver." << endl; }
      add_Constraints_to_solver(s, data_Dependency_Constraints.second);

      if (((this->mode) % 2) == 1) {
          if (!quiet) { cout << "\nAdding 'only-one-mod-slot-per-resource-constrained-operation'-constraints to solver" << endl; }
          add_one_slot_constraints_to_solver(s);
      }

      if (((this->mode/2) % 2) == 1) {
          if (!quiet) { cout << "\nAdding resource-limit-constraints to solver" << endl; }
          add_resource_limit_constraint_to_solver(s);
      }

      if (((this->mode/4) % 2) == 1) {
          if (!quiet) { cout << "\nAdding linking-constraints to solver" << endl; }
          add_linking_constraints_to_solver(s);
      }

      set_max_latency(s, maxLatency);

      if (!quiet) { cout << "II:" << II << endl; }
      clock_t start, end;
      start = clock();
      cout << "------------------------------------" << endl;
      satisfiable = s.check();
      cout << "------------------------------------" << endl;
      end = clock();
      if (!quiet) { cout << "Solving Time: " << double(end - start) / double(CLOCKS_PER_SEC) << " sec " << endl; }
      if (!quiet) { cout << s << "\n" << "solving... " << satisfiable << "\n"; }

      while (satisfiable != z3::sat) {
          s.pop();
          s.push();
          if (!quiet) { cout << s << "\n" << "solving... " << satisfiable << "\n"; }
          II = II + 1;
          if (!quiet) { cout << "II:" << II << endl; }
          create_b_variables();
          if ((this->mode % 2) == 1) { add_one_slot_constraints_to_solver(s); }
          if (((this->mode/2) % 2) == 1) { add_resource_limit_constraint_to_solver(s); }
          if (((this->mode/4) % 2) == 1) { add_linking_constraints_to_solver(s); }
          set_max_latency(s, maxLatency);
          if (!quiet) { cout << endl; }
          for (auto &it : exprToEdgeMap){
              z3::expr e = (  t_Variables[it.second->getVertexDst().getId()]
                            - t_Variables[it.second->getVertexSrc().getId()]
                            + (int) II * it.second->getDistance()
                            >= resourceModel.getVertexLatency(&it.second->getVertexSrc()));
              s.add(e.simplify());
              satisfiable = s.check();
              if (!quiet) { cout << satisfiable << endl; }
              if (satisfiable == z3::unsat){ break; }
          }
          if(!quiet){ cout << s << "\n" << "solving... " << satisfiable << "\n"; }
          if (II > maxII) { throw(HatScheT::Exception("No Schedule found.")); }
      }

      z3::model m = s.get_model();
      if(!quiet){ std::cout << "solution\n" << m << "\n \n"; }

      for (auto &it : vertex_t_Variables_Map){
          startTimes.insert({it.first , m.eval(t_Variables[it.second]).get_numeral_int()});
      }

      if(!quiet){ cout << "Schedule found... mimimizing latency... " << "\n"; }
      s.push();
      while (satisfiable == z3::sat){
          m = s.get_model();
          startTimes.clear();
          for (auto &it : vertex_t_Variables_Map){
              startTimes.insert({it.first , m.eval(t_Variables[it.second]).get_numeral_int()});
              if (!quiet) { cout << startTimes.at(it.first) << endl; }
          }
          if (!quiet) { cout << endl; }
          s.pop();
          s.push();
          create_and_add_latency_constraints(s);
          satisfiable = s.check();
      }

      int min = INT32_MAX;
      for (auto &it : startTimes) {
          if (it.second < min) {
              min = it.second;
          }
      }
      if (min < 0) {
          for (auto &it: startTimes) {
              it.second += abs(min);
          }
      }else{
          for (auto &it: startTimes) {
              it.second -= abs(min);
          }
      }
  }
}

#endif

