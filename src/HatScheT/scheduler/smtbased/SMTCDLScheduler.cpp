//
// Created by bkessler on 8/25/22.
//

#include <stack>
#include <HatScheT/utility/Utility.h>
#include <HatScheT/utility/Verifier.h>
#include <iomanip>

#include "SMTCDLScheduler.h"

#ifdef USE_Z3

namespace HatScheT {

  SMTCDLScheduler::SMTCDLScheduler(Graph &g, ResourceModel &resourceModel, double II) : IterativeModuloSchedulerLayer(g, resourceModel, II)
  {
      this->secondObjectiveOptimal = true;
  }

  void SMTCDLScheduler::scheduleInit()
  {
      if(!this->quiet)
      {
          cout << "Scheduling with " << this->getName() <<":" << endl;
          cout << "Rec-Min-II: " << recMinII << endl;
          cout << "Res-Min-II: " << resMinII << endl;
          cout << "Max. II: " << maxII << endl;
          cout << "Vertices of G: " << g.getNumberOfVertices() << endl;
          cout << "Edges of G: " << g.getNumberOfEdges() << endl;
          this->printZ3Params();
      }

      modifyResourceModel();
  }

  void SMTCDLScheduler::scheduleIteration() {
      if (!quiet) { cout << "Calculating ASAP and ALAP Starttimes: " << endl; }
      latEst = Utility::getLatencyEstimation(&g, &resourceModel, II, Utility::latencyBounds::both);
      if (!quiet) { cout << "Min. Latency: " << latEst.minLat << " Max. Latency: " << latEst.maxLat << endl; }
      for (int latency = latEst.minLat; latency <= latEst.maxLat; latency++) {
          z3Reset();
          int i = 0;
          calculateStartimes(latency);
          setInitialStartTimes();
          createBooleanVariables();
          //printbooleanVeriables();
          stack<Edge *> violatedEdges;
          addOneSlotConstraintToSolver();
          addResourceContraintsToSolver();
          do {
              i++;
              if (!quiet) { cout << "II: " << this->II << " | Latency: " << latency << " | Iteration: " << i; }
              if (!quiet) { cout << " || Time Used: " << this->getSolvingTimePerIteration() << " | Time Remaining: " << this->getTimeRemaining() << endl;}
              if (!quiet) { cout << p << endl;}
              addConflictClauseNextTry(violatedEdges);
              this->setZ3Timeout((uint32_t) getTimeRemaining()+1);
              startTimeTracking();
              z3Check();
              endTimeTracking();
              if (getZ3Result() == z3::sat) {
                  parseSMTModel();
                  violatedEdges = checkSchedule(g, resourceModel, startTimes, (int) II, quiet);
              }
              scheduleFound = (getZ3Result() == z3::sat);
              if (this->timeRemaining <= 0) {
                  scheduleFound = false;
                  firstObjectiveOptimal = false;
                  return;
              }
              if (getZ3Result() == z3::unknown)
              {
                  secondObjectiveOptimal = false;
              }
          } while (getZ3Result() != z3::unsat and !violatedEdges.empty());

          if (scheduleFound)
          {
              cout << "Schedule Found!" << endl;
              resetResourceModel();
              return;
          }
      }
  }

  int SMTCDLScheduler::getScheduleLatency(map<Vertex *, int> &sched) {
      int maxTime = -1;
      for (std::pair<Vertex *, int> vtPair : sched) {
          try {
              Vertex *v = vtPair.first;
              if ((vtPair.second + resourceModel.getVertexLatency(v) > maxTime)) {
                  maxTime = (vtPair.second + resourceModel.getVertexLatency(v));
                  //Debugging:
                  if (!quiet) {
                      //cout << vtPair.first->getName() << ": " << vtPair.second << " + "
                      //     << resourceModel.getVertexLatency(v) << " = " << maxTime << endl;
                  }
              }
          }catch(std::out_of_range&){
              cout << vtPair.first->getName() << ": " << vtPair.second << endl;
              throw (HatScheT::Exception("SMTUnaryScheduler::getScheduleLatency() OUT_OF_RANGE"));
          }
      }
      return maxTime;
  }

  void SMTCDLScheduler::createBooleanVariables() {
      for (auto &v : g.Vertices()){
          for (int i = latEst.asapStartTimes.at(v); i <= latestStartTimesWithOffset.at(v); i++){
              std::stringstream name;
              name << "B" << i << "_" << v->getName();
              z3::expr bVar(c.bool_const(name.str().c_str()));
              booleanVariables.insert({std::make_pair(v, i), bVar});
          }
      }
  }

  void SMTCDLScheduler::addOneSlotConstraintToSolver() {
      for (auto &v : g.Vertices()) {
          z3::expr_vector oneSlotVector(c);
          vector<int>oneSlotWeight;
          for (int i = latEst.asapStartTimes.at(v); i <= latestStartTimesWithOffset.at(v); i++){
              oneSlotVector.push_back(booleanVariables.at({v, i}));
              oneSlotWeight.push_back(1);
          }
          s.add(z3::pbeq(oneSlotVector, &oneSlotWeight.at(0), 1));
          oneSlotWeight.clear();
      }
  }

  void SMTCDLScheduler::addResourceContraintsToSolver() {
      for (auto &r : resourceModel.Resources()){
          if (r->isUnlimited()){
              continue;
          }
          for (int i = 0; i < (int)II; i++) {
              set<const Vertex *> vSet = resourceModel.getVerticesOfResource(r);
              z3::expr_vector b_expressions(c);
              for (auto &v : vSet) {
                  for (int j = latEst.asapStartTimes.at((Vertex*)v); j <= latestStartTimesWithOffset.at((Vertex*)v); j++) {
                      if ((j % (int)II) != i) {
                          continue;
                      }
                      b_expressions.push_back(booleanVariables.at({(Vertex *) v, j}));
                  }
              }
              if (!b_expressions.empty()) {
                  s.add(z3::atmost(b_expressions, r->getLimit()));
              }
          }
      }
  }

  stack<Edge*> SMTCDLScheduler::checkSchedule(Graph &g, ResourceModel &rm, std::map<Vertex *, int> &proposedDchedule, int II, bool quiet)
  {
      bool ok;
      stack<Edge*> violatedEdges;
      for (auto it = g.edgesBegin(), end = g.edgesEnd(); it != end; it++) {
          auto e = *it;
          auto i = &e->getVertexSrc();
          auto j = &e->getVertexDst();

          ok = proposedDchedule[i] + rm.getVertexLatency(i) + e->getDelay() <= proposedDchedule[j] + e->getDistance() * II;
          if (! ok) {
              //if (!quiet) cerr << *e << " violated: " << proposedDchedule[i] << " + " << rm.getVertexLatency(i) << " + " << e->getDelay() << " <= " << proposedDchedule[j] << " + " << e->getDistance() << "*" << II << endl;
              violatedEdges.push(e);
          }
      }
      return violatedEdges;
  }

  void SMTCDLScheduler::calculateStartimes(int candidateLatency) {
      latestStartTimesWithOffset.clear();
      if (candidateLatency == -1)
      {
          for (auto &it: latEst.alapStartTimes)
          {
              this->latestStartTimesWithOffset[it.first] = latEst.maxLat - (getScheduleLatency(latEst.alapStartTimes) - it.second);
          }
      }
      else
      {
          for (auto &it: latEst.alapStartTimes)
          {
              this->latestStartTimesWithOffset[it.first] = candidateLatency - (getScheduleLatency(latEst.alapStartTimes) - it.second);
          }
      }
  }

  void SMTCDLScheduler::modifyResourceModel() {
      for (auto &r : resourceModel.Resources()){
          resourceLimits[r]=r->getLimit();
          if (resourceModel.getNumVerticesRegisteredToResource(r) <= r->getLimit()){
              r->setLimit(UNLIMITED, false);
          }
      }
  }

  void SMTCDLScheduler::resetResourceModel() {
      for (auto &r : resourceLimits){
          r.first->setLimit(resourceLimits.at(r.first), false);
      }
  }

  void SMTCDLScheduler::parseSMTModel() {
      for (auto &v : g.Vertices()) {
          for (int i = 0; i < latEst.maxLat; i++) {
              try {
                  if (m.eval(booleanVariables.at({v, i})).is_true()) {
                      startTimes.at(v) = i;
                  }
              }catch (std::out_of_range&){
                  continue;
              }
          }
      }
  }

  void SMTCDLScheduler::setInitialStartTimes() {
      for (auto &v : g.Vertices()) {
          startTimes[v] = latEst.asapStartTimes.at(v);
      }
  }

  void SMTCDLScheduler::addConflictClauseNextTry(stack<Edge *> &eStack) {

      while(!eStack.empty()){
          auto e = eStack.top();
          eStack.pop();
          auto vSrc = &e->getVertexSrc();
          auto vDst = &e->getVertexDst();
          s.add(!booleanVariables.at({vSrc, startTimes.at(vSrc)}) || !booleanVariables.at({vDst, startTimes.at(vDst)}));
      }
  }

  void SMTCDLScheduler::setSolverTimeout(double seconds) {
      this->solverTimeout = seconds;
      this->setZ3Timeout((uint32_t)seconds);
  }

  void SMTCDLScheduler::endTimeTracking() {
      this->end_t = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_t - start_t).count();
      solvingTimePerIteration += (double) duration / 1000000;
      timeRemaining = solverTimeout - solvingTimePerIteration;
  }

}
#endif //USE_Z3