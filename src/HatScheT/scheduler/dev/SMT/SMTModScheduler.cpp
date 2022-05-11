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
      if(!quiet){ cout << "Creating Data Structure..." << endl; }
      create_t_Variables();
      this->data_Dependency_Constraints = build_Dependency_Constraints();
      create_b_variables();

  }

  void SMTModScheduler::create_t_Variables() {
      if (!quiet) { cout << "Creating Vertex Expressions (t_i - Variables):" << endl;
                    cout << "Index: Vertex: Expression:" << endl;}

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

      if (!quiet) { cout << "Creating Dependency Contraints:" << endl; }

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

      vector<vector<b_variable>> collumn;
      vector<b_variable> line;

      if (!quiet) { cout << "Creating b_variables:" << endl; }

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
              collumn.push_back(line);
              line.clear();
          }
          b_variables.push_back(collumn);
          collumn.clear();
      }
  }

  SMTModScheduler::b_variable *SMTModScheduler::get_b_var(Vertex *v, Resource *r, int slot) {
      return &b_variables.at(resource_to_b_vairable_index[r]).at(slot).at(vertex_to_b_vairable_index[v]); //LOL
  }

  void SMTModScheduler::print_b_variables() {
      for (const auto &res : resourceModel.Resources()){
          if (res->getLimit() == UNLIMITED){
              continue;
          }
          for (int tau = 0; tau < this->II; tau++){
              for(const auto &ver : g.Vertices()){
                  cout << get_b_var(ver, res, tau)->b_var << endl;
              }
          }
      }
  }

  void SMTModScheduler::schedule() {

      z3::solver s(c);

      II = minII;

      build_Data_Structure();

      if (!quiet) {
          print_b_variables();
          cout << "Expressions without weight:" << endl;
          for (auto &it : data_Dependency_Constraints.first) {
              cout << it << endl;
          }
          cout << "Expressions with weight:" << endl;
          for (auto &it : data_Dependency_Constraints.second) {
              cout << it << endl;
          }
      }

      //TODO Mach weg, den Quatsch
      z3::expr_vector bums (c);
      for (auto &it : g.Vertices()){
          bums.push_back(get_b_var(it, resourceModel.getResource("green"), 0)->b_var);
      }

      s.add(sum(bums) > 5);

      if(!quiet){ cout << "Adding Expressions without weight to solver." << endl; }
      for (auto &it : data_Dependency_Constraints.first){
          s.add(it);
      }

      if(!quiet){ cout << "Adding Expressions with weight to solver." << endl; }
      auto satisfiable = s.check();
      if(!quiet){ cout << "solving... " << satisfiable << endl; }
      if(!quiet){ cout << "Creating Fallback-Point." << endl; }
      s.push();

      if(!quiet){ cout << "Adding Expressions with weight to solver." << endl; }
      for (auto &it : data_Dependency_Constraints.second){
          s.add(it);
      }
      satisfiable = s.check();
      if(!quiet){ cout << "solving... " << satisfiable << endl;  }

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

