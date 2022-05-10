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


  void SMTModScheduler::schedule() {

      //II = minII;

      auto startTimesVector = creatingStartTimesVec();

      if(!quiet){ cout << "Starting with II = " << II << endl; }

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
      if(!quiet){ cout << satisfiable << endl; }
      if(!quiet){ cout << "Creating Fallback-Point." << endl; }
      s.push();

      if(!quiet){ cout << "Adding Expressions with weight to solver." << endl; }
      for (auto it : dependencies.second){
          s.add(it);
      }
      satisfiable = s.check();
      if(!quiet){ cout << satisfiable << endl; }

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
          if (II > 10) { break; }
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

      //----------------------------------------------------------------------------------------------------//
      /*auto es = EichenbergerDavidson97Scheduler(g, resourceModel, {"CPLEX", "Gurobi", "SCIP", "LPSolve"});
      es.schedule();
      this->startTimes = es.getSchedule();
      this->II = es.getII();*/

  }


  void SMTModScheduler::handmadeSchedule() {

      cout << "Starting with II = " << II << endl;
      //z3::context c;
      z3::solver _s(c);

      z3::expr z = c.int_const("vertex_0");
      z3::expr y = c.int_const("vertex_1");
      z3::expr x = c.int_const("vertex_2");
      z3::expr w = c.int_const("vertex_3");
      z3::expr v = c.int_const("vertex_4");
      z3::expr u = c.int_const("vertex_5");
      z3::expr t = c.int_const("vertex_6");
      z3::expr s = c.int_const("vertex_7");

      z3::expr e = (y + x == 4);
      _s.add(y - z >= 1);
      _s.add(x - y >= 1);
      _s.add(w - x >= 1);
      _s.add(u - z >= 1);
      _s.add(u - v >= 1);
      _s.add(t - u >= 1);
      _s.add(w - t >= 1);
      _s.add(u - s >= 1);
      _s.add(y - v >= 1);
      _s.check(); //Scheint der Solver zu brauchen.
      _s.push();
      _s.add(y - w + (int) II * 2 >= 1);
      _s.add(u - t + (int) II * 2 >= 1);
      auto satisfiable = _s.check(); //Scheint der Solver zu brauchen.

      while (satisfiable != z3::sat) {
          _s.pop();
          std::cout << _s << "\n" << "solving...\n" << satisfiable << "\n";
          II = II + 1;
          cout << "II:" << II << endl;
          _s.push();
          _s.add(y - w + (int) II * 2 >= 1);
          _s.add(u - t + (int) II * 2 >= 1);
          satisfiable = _s.check();
          std::cout << _s << "\n" << "solving...\n" << satisfiable << "\n";
          if (II > 10) { break; }
      }

      z3::model m = _s.get_model();
      std::cout << "solution\n" << m;

      this->startTimes = {{&g.getVertexByName("vertex_0"), m.eval(z).get_numeral_int()},
                          {&g.getVertexByName("vertex_1"), m.eval(y).get_numeral_int()},
                          {&g.getVertexByName("vertex_2"), m.eval(x).get_numeral_int()},
                          {&g.getVertexByName("vertex_3"), m.eval(w).get_numeral_int()},
                          {&g.getVertexByName("vertex_4"), m.eval(v).get_numeral_int()},
                          {&g.getVertexByName("vertex_5"), m.eval(u).get_numeral_int()},
                          {&g.getVertexByName("vertex_6"), m.eval(t).get_numeral_int()},
                          {&g.getVertexByName("vertex_7"), m.eval(s).get_numeral_int()}};

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

  z3::expr_vector SMTModScheduler::creatingStartTimesVec() {
      if (!quiet) { cout << "Creating Dependency Constraints:" << endl; }

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
  }

  pair <z3::expr_vector,z3::expr_vector> SMTModScheduler::createDependencyConstraints(z3::expr_vector &expressions) {

      z3::expr_vector edgesWithoutWeight (c);
      z3::expr_vector edgesWithWeight (c);

      int i = 0;
      for(auto &it : g.Edges()){
          if (it->getDistance() == 0){
              z3::expr e = (  expressions[it->getVertexDst().getId()]
                            - expressions[it->getVertexSrc().getId()]
                            >= resourceModel.getVertexLatency(&it->getVertexSrc()));
              edgesWithoutWeight.push_back(e);
          }else{
              z3::expr e = (  expressions[it->getVertexDst().getId()]
                            - expressions[it->getVertexSrc().getId()]
                            + (int) II * it->getDistance()
                            >= resourceModel.getVertexLatency(&it->getVertexSrc()));
              edgesWithWeight.push_back(e);
              exprToEdgeMap.insert({i, it});
              i++;
          }
      }
      return {edgesWithoutWeight, edgesWithWeight};
  }
}

#endif

