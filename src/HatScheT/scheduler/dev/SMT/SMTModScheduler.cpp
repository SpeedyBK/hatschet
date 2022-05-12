//
// Created by bkessler on 5/7/22.
//

#ifdef USE_Z3
#include "SMTModScheduler.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/Exception.h"

#include <z3++.h>

#include <iostream>
#include <map>
#include <cmath>

namespace HatScheT {

  SMTModScheduler::SMTModScheduler(Graph &g, ResourceModel &resourceModel)
      : SchedulerBase(g, resourceModel) {
      // reset previous solutions
      II = -1;
      this->timeouts = 0;
      startTimes.clear();
      scheduleFound = false;
      computeMinII(&g, &resourceModel);
      minII = ceil(minII);
      computeMaxII(&g, &resourceModel);
      quiet = true;
  }

  void SMTModScheduler::build_Data_Structure() {
      if(!quiet){ cout << "Creating data-structure..." << endl; }
      create_t_Variables();
      this->data_Dependency_Constraints = build_Dependency_Constraints();
      create_b_variables();

  }

  void SMTModScheduler::create_t_Variables() {
      if (!quiet) { cout << "\nCreating vertex expressions (t_i - variables):" << endl;
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

  pair <vector<z3::expr>, vector<z3::expr>> SMTModScheduler::build_Dependency_Constraints(){

      if (!quiet) { cout << "\nCreating dependency contraints:" << endl; }

      vector<z3::expr> edgesWithoutDistance;
      vector<z3::expr> edgesWithDistance;

      int i = 0;
      for(auto &it : g.Edges()){
          if (!quiet) {
              cout << t_Variables[vertex_t_Variables_Map[&it->getVertexDst()]] << " - "
                   << t_Variables[vertex_t_Variables_Map[&it->getVertexSrc()]] << " + "
                   << (int) II << " * " << it->getDistance() << " >= "
                   << resourceModel.getVertexLatency(&it->getVertexSrc()) << endl;
          }
          if (it->getDistance() == 0){
              z3::expr e = (  t_Variables[vertex_t_Variables_Map[&it->getVertexDst()]]
                              - t_Variables[vertex_t_Variables_Map[&it->getVertexSrc()]]
                              >= resourceModel.getVertexLatency(&it->getVertexSrc()));
              edgesWithoutDistance.push_back(e);
          }else{
              z3::expr e = (  t_Variables[vertex_t_Variables_Map[&it->getVertexDst()]]
                              - t_Variables[vertex_t_Variables_Map[&it->getVertexSrc()]]
                              + (int) II * it->getDistance()
                              >= resourceModel.getVertexLatency(&it->getVertexSrc()));
              edgesWithDistance.push_back(e);
              exprToEdgeMap.insert({i, it});
              i++;
          }
      }
      return {edgesWithoutDistance, edgesWithDistance};
  }

  void SMTModScheduler::create_b_variables() {

      b_variables.clear();

      vector<vector<b_variable>> collumn;
      vector<b_variable> line;

      if (!quiet) { cout << "\nCreating b_variables:" << endl; }

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
              vertex_to_b_vairable_index.at(v)); //LOL
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
      cout << "\nExpressions without distance:" << endl;
      for (auto &it : data_Dependency_Constraints.first) {
          cout << it << endl;
      }
      cout << "\nExpressions with distance:" << endl;
      for (auto &it : data_Dependency_Constraints.second) {
          cout << it << endl;
      }
  }

  void SMTModScheduler::add_Constraints_to_solver(z3::solver &s, vector<z3::expr, allocator<z3::expr>> &eVec) {
      for (auto &it : eVec) {
          s.add(it);
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
          z3::expr e = sum(one_Slot_Constraint) == 1;
          if(!quiet){ cout << e << endl; }
          s.add(e);
      }
  }

  void SMTModScheduler::add_resource_limit_constraint_to_solver(z3::solver &s){
      //TODO: Seems like there is something wrong.
      for (auto &rit : resourceModel.Resources()) {
          if (!rit->isUnlimited()) {
              for (int i = 0; i < II; i++) {
                  z3::expr_vector resource_limit_constraint(c);
                  for (auto &it : g.Vertices()) {
                      if (!resourceModel.getResource(it)->isUnlimited()) {
                          resource_limit_constraint.push_back(
                              get_b_var(it, resourceModel.getResource(it), i)->b_var);
                      }
                  }
                  z3::expr e = sum(resource_limit_constraint) <= rit->getLimit();
                  if(!quiet){ cout << e << endl; }
                  s.add(e);
              }
          }
      }
  }

  void SMTModScheduler::add_linking_constraints_to_solver(z3::solver &s) {
      for(auto &it : g.Vertices()){
          std::stringstream expr_name;

          expr_name << "y_" << it->getId();
          z3::expr y(c.int_const(expr_name.str().c_str()));
          z3::expr yII = (y * (int)II);
          z3::expr_vector b_times_tau_vec(c);
          z3::expr zero(c.int_val(0));
          z3::expr one(c.int_val(1));

          for(int i = 0; i < (int)II; i++){
              z3::expr b = get_b_var(it, resourceModel.getResource(it), i)->b_var;
              z3::expr bi = z3::ite(b, one, zero);
              z3::expr bii = (bi * i);// Wirft error.
              b_times_tau_vec.push_back(bii);
          }

          z3::expr sum_of_b_times_tau = sum(b_times_tau_vec);
          s.add(t_Variables.at(vertex_t_Variables_Map.at(it)) == yII + sum_of_b_times_tau );
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

      if (!quiet) { cout << "\nAdding dependency constraints without distance to solver." << endl; }
      add_Constraints_to_solver(s, data_Dependency_Constraints.first);

      auto satisfiable = s.check();
      if (!quiet) { cout << "\nsolving... " << satisfiable << endl; }
      if (!quiet) { cout << "Creating fallback-point." << endl; }
      s.push();

      if (!quiet) { cout << "\nAdding dependency constraints with distance to solver." << endl; }
      add_Constraints_to_solver(s, data_Dependency_Constraints.second);

      if (!quiet) { cout << "\nAdding 'only-one-mod-slot-per-resource-constrained-operation'-constraints to solver" << endl; }
      add_one_slot_constraints_to_solver(s);

      if (!quiet) { cout << "\nAdding resource-limit-constraints to solver" << endl; }
      add_resource_limit_constraint_to_solver(s);

      if (!quiet) { cout << "\nAdding linking-constraints to solver" << endl; }
      add_linking_constraints_to_solver(s);

      satisfiable = s.check();
      if(!quiet){ cout << "\nsolving... " << satisfiable << endl;  }

      while (satisfiable != z3::sat) {
          s.pop();
          if(!quiet){ cout << s << "\n" << "solving... " << satisfiable << "\n"; }
          II = II + 1;
          if(!quiet){ cout << "II:" << II << endl; }
          s.push();
          for (auto &it : exprToEdgeMap){
              cout << "Edge ID : "<< it.second->getId() << endl;
              z3::expr e = (  t_Variables[it.second->getVertexDst().getId()]
                            - t_Variables[it.second->getVertexSrc().getId()]
                            + (int) II * it.second->getDistance()
                            >= resourceModel.getVertexLatency(&it.second->getVertexSrc()));
              s.add(e);
          }
          create_b_variables();
          add_one_slot_constraints_to_solver(s);
          add_resource_limit_constraint_to_solver(s);
          add_linking_constraints_to_solver(s);
          satisfiable = s.check();
          if(!quiet){ cout << s << "\n" << "solving... " << satisfiable << "\n"; }
          if (II > maxII) { break; }
      }

      z3::model m = s.get_model();
      if(!quiet){ std::cout << "solution\n" << m << "\n \n"; }

      for (auto &it : vertex_t_Variables_Map){
          startTimes.insert({it.first , m.eval(t_Variables[it.second]).get_numeral_int()});
      }

      int min = INT32_MAX;
      for (auto &it : startTimes) {
          if (it.second < min) {
              min = it.second;
          }
      }

      for (auto &it: startTimes) {
          it.second += abs(min);
      }
  }
}

#endif

