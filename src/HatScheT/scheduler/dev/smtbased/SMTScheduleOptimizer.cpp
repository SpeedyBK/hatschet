//
// Created by bkessler on 8/25/22.
//

#include <cmath>

#include "SMTScheduleOptimizer.h"

#ifdef USE_Z3

namespace HatScheT {

  SMTScheduleOptimizer::SMTScheduleOptimizer(Graph &g, ResourceModel &resourceModel, map<Vertex*, int> &initialSchedule, double II) : SchedulerBase(g, resourceModel)
  {
      this->timeouts = 0;
      this->initialSchedule = initialSchedule;
      startTimes.clear();
      scheduleFound = true;
      minII = ceil(minII);
      this->II = II;
  }

  void SMTScheduleOptimizer::schedule() {

      z3::optimize opti(c);

      auto superSink = createSuperSink();

      createTimeVariables();
      createBooleanVariables();
      addDependencyConstraints(opti);

      opti.minimize(timeVariables.at(superSink));
      cout << opti.check() << endl;
      auto m = opti.get_model();

      for (auto &t : timeVariables){
          cout << t.first->getName() << ": " << m.eval(t.second).get_numeral_int() << endl;
      }
  }

  void SMTScheduleOptimizer::createTimeVariables() {
      for (auto &v : g.Vertices()){
          std::stringstream name;
          name << v->getName();
          z3::expr tVar(c.int_const(name.str().c_str()));
          timeVariables.insert({v, tVar});
      }
  }

  void SMTScheduleOptimizer::createBooleanVariables() {
      for (auto &v : g.Vertices()){
          for (int i = 0; i <= initialSchedule.at(v); i++){
              std::stringstream name;
              name << "B" << i << "_" << v->getName();
              z3::expr bVar(c.bool_const(name.str().c_str()));
              //cout << v->getName() <<": " << bVar << endl;
              booleanVariables.insert({std::make_pair(v, i), bVar});
          }
      }
  }

  void SMTScheduleOptimizer::addDependencyConstraints(z3::optimize &opt) {

      for (auto &t : timeVariables){
          opt.add(t.second >= 0);
      }

      for (auto &e : g.Edges()){
          auto dst = &e->getVertexDst();
          auto src = &e->getVertexSrc();
          int delay = e->getDelay();
          int distance = e->getDistance();
          opt.add(timeVariables.at(dst) - timeVariables.at(src) >= resourceModel.getResource(src)->getLatency() + delay - (distance * (int) II));
      }
  }

  Vertex* SMTScheduleOptimizer::createSuperSink() {

      auto superSink = &g.createVertex();
      superSink->setName("SuperSink");
      auto res = &resourceModel.makeResource("Dummy", UNLIMITED, 0, 0);
      resourceModel.registerVertex(superSink, res);

      int maxTime = 0;
      for (auto &vtpair : initialSchedule){
          if (vtpair.second + resourceModel.getVertexLatency(vtpair.first) > maxTime){
              maxTime = vtpair.second + resourceModel.getVertexLatency(vtpair.first);
          }
      }

      cout << "SMTScheduleOptimizer::createSuperSink(): setting initial time of Supersink to " << maxTime << endl;
      initialSchedule[superSink] = maxTime;

      for (auto &v : g.Vertices()){
          if (g.hasNoZeroDistanceOutgoingEdges(v)){
              auto e = &g.createEdge(*v, *superSink, 0);
              if (!quiet) { cout << *e << endl; }
          }
      }

      std::stringstream name;
      name << "SuperSink";
      z3::expr ex(c.int_const(name.str().c_str()));
      timeVariables.insert({superSink, ex});

      return superSink;
  }
}

#endif //USE_Z3