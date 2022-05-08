//
// Created by bkessler on 5/7/22.
//

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
  }

  void SMTModScheduler::schedule() {

      auto es = EichenbergerDavidson97Scheduler(g, resourceModel, {"CPLEX", "Gurobi", "SCIP", "LPSolve"});
      es.schedule();
      this->startTimes = es.getSchedule();
      this->II = es.getII();

  }

  void SMTModScheduler::handmadeSchedule() {

      II = minII;
      II = 0;
      cout << "Starting with II = " << II << endl;

      z3::context c;
      z3::solver _s(c);

      z3::expr z = c.int_const("vertex_0");
      z3::expr y = c.int_const("vertex_1");
      z3::expr x = c.int_const("vertex_2");
      z3::expr w = c.int_const("vertex_3");
      z3::expr v = c.int_const("vertex_4");
      z3::expr u = c.int_const("vertex_5");
      z3::expr t = c.int_const("vertex_6");
      z3::expr s = c.int_const("vertex_7");

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
      _s.add(y - w + (int)II * 2 >= 1);
      _s.add(u - t + (int)II * 2 >= 1);
      auto satisfiable = _s.check(); //Scheint der Solver zu brauchen.

      while (satisfiable != z3::sat){
          _s.pop();
          std::cout << _s << "\n" << "solving...\n"<< satisfiable << "\n";
          II = II + 1;
          cout << "II:" << II << endl;
          _s.push();
          _s.add(y - w + (int)II * 2 >= 1);
          _s.add(u - t + (int)II * 2 >= 1);
          satisfiable = _s.check();
          std::cout << _s << "\n" << "solving...\n" << satisfiable << "\n";
          if (II > 10) {break;}
      }

      z3::model m = _s.get_model();
      std::cout << "solution\n" << m;

      this->startTimes = {{&g.getVertexByName("vertex_0"), m.eval(z).get_numeral_int()+2},
                          {&g.getVertexByName("vertex_1"), m.eval(y).get_numeral_int()+2},
                          {&g.getVertexByName("vertex_2"), m.eval(x).get_numeral_int()+2},
                          {&g.getVertexByName("vertex_3"), m.eval(w).get_numeral_int()+2},
                          {&g.getVertexByName("vertex_4"), m.eval(v).get_numeral_int()+2},
                          {&g.getVertexByName("vertex_5"), m.eval(u).get_numeral_int()+2},
                          {&g.getVertexByName("vertex_6"), m.eval(t).get_numeral_int()+2},
                          {&g.getVertexByName("vertex_7"), m.eval(s).get_numeral_int()+2}};

  }
}