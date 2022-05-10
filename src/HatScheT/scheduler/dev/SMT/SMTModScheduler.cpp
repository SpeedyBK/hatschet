//
// Created by bkessler on 5/7/22.
//

#ifdef USE_Z3
#include "SMTModScheduler.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/Exception.h"

#include "HatScheT/scheduler/ilpbased/EichenbergerDavidson97Scheduler.h"
#include <z3++.h>

#include <iostream>
#include <map>
#include <limits>
#include <cmath>

namespace HatScheT {

  SMTModScheduler::SMTModScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string> solverWishlist)
      : SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist) {
      // reset previous solutions
      II = -1;
      this->timeouts = 0;
      startTimes.clear();
      scheduleFound = false;
      optimalResult = true;
      computeMinII(&g, &resourceModel);
      minII = ceil(minII);
      computeMaxII(&g, &resourceModel);
      quiet = true;
  }

  void SMTModScheduler::buildDataStructure() {

      this->t_Variables = creating_t_Variables();
      this->data_Dependency_Constraints = build_Dependency_Constraints();

  }

  vector<z3::expr> SMTModScheduler::creating_t_Variables() {
      if (!quiet) { cout << "Creating Vertex Expressions (t_i - Variables):" << endl;
                    cout << "Index: Vertex: Expression:" << endl;}

      vector <z3::expr> t_VariablesVector;
      int i = 0;
      for (auto &it : g.Vertices()) {
          if (!quiet) { cout << i << ": " << it->getName() << " : "; }
          std::stringstream expr_name;
          expr_name << it->getName();
          t_VariablesVector.push_back(c.int_const(expr_name.str().c_str()));
          vertex_t_Variables_Map.insert({it, i});
          i++;
          if (!quiet) { cout << t_VariablesVector.back() << endl; }
      }

      return t_VariablesVector;
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




  void SMTModScheduler::schedule() {

      II = minII;

      buildDataStructure();

      for (const auto &[ExpressionIndex, Edge] : exprToEdgeMap){
          cout << ExpressionIndex << " : " << Edge->getId() << endl;
      }

      throw(HatScheT::Exception("Bums"));

      if(!quiet){ cout << "Starting with II = " << II << endl; }
/*

      auto dependencies = createDependencyConstraints(startTimesVector);

      if (!quiet) {
          cout << "Expressions without weight:" << endl;
          for (auto it : dependencies.first) {
              cout << it << endl;
          }
          cout << "Expressions with weight:" << endl;
          for (auto it : dependencies.second) {
              cout << it << endl;
          }
      }

      if(!quiet){ cout << "Adding Expressions without weight to solver." << endl; }
      z3::solver s(c);

      for (auto it : dependencies.first){
          s.add(it);
      }

      if(!quiet){ cout << "Adding Expressions without weight to solver." << endl; }
      auto satisfiable = s.check();
      if(!quiet){ cout << "solving... " << satisfiable << endl; }
      if(!quiet){ cout << "Creating Fallback-Point." << endl; }
      s.push();

      if(!quiet){ cout << "Adding Expressions with weight to solver." << endl; }
      for (auto it : dependencies.second){
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
              z3::expr e = (  startTimesVector[it.second->getVertexDst().getId()]
                            - startTimesVector[it.second->getVertexSrc().getId()]
                            + (int) II * it.second->getDistance()
                            >= resourceModel.getVertexLatency(&it.second->getVertexSrc()));
              s.add(e);
          }
          satisfiable = s.check();
          if(!quiet){ cout << s << "\n" << "solving... " << satisfiable << "\n"; }
          //TODO Has to be changed;
          if (II > maxII) { break; }
      }

      z3::model m = s.get_model();
      if(!quiet){ std::cout << "solution\n" << m << "\n \n"; }

      for (auto &it : vertexToExprMap){
          startTimes.insert({it.first , m.eval(startTimesVector[it.second]).get_numeral_int()});
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

*/

  }

/*  z3::expr_vector SMTModScheduler::creatingStartTimesVec() {
      if (!quiet) { cout << "Creating Vertex Expressions:" << endl; }

      //Creating one z3::expr per Vertex and storing in an expr_vector;
      z3::expr_vector startTimesVector (c);
      int i = 0;
      for (auto &it : g.Vertices()) {
          if (!quiet) { cout << i << ": " << it->getName() << " : "; }
          std::stringstream expr_name;
          expr_name << it->getName();
          startTimesVector.push_back(c.int_const(expr_name.str().c_str()));
          vertexToExprMap.insert({it, i});
          i++;
          if (!quiet) { cout << startTimesVector.back() << endl; }
      }

      return startTimesVector;
  }*/

}

#endif

