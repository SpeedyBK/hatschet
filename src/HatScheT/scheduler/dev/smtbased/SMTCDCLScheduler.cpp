//
// Created by bkessler on 8/25/22.
//

#include <cmath>
#include <stack>
#include <algorithm>
#include <HatScheT/utility/Utility.h>
#include <HatScheT/utility/Verifier.h>
#include <iomanip>

#include "SMTCDCLScheduler.h"

#ifdef USE_Z3

namespace HatScheT {

  SMTCDCLScheduler::SMTCDCLScheduler(Graph &g, ResourceModel &resourceModel) : SchedulerBase(g, resourceModel)
  {
      this->timeouts = 0;
      startTimes.clear();
      scheduleFound = false;
      computeMinII(&g, &resourceModel);
      minII = ceil(minII);
      computeMaxII(&g, &resourceModel);

      this->timeLimit = -1;
      this->timeBudget = INT32_MAX;
  }

  void SMTCDCLScheduler::schedule() {

      if (!quiet) { cout << "Starting SMTCDCLScheduler..." << endl; }
      modifyResourceModel();

      z3::solver s(c);

      z3::params p(c);
      if (timeLimit > 0) {
          timeBudget = timeLimit;
      }
      if (timeBudget > 0) { p.set("timeout", (uint32_t) timeBudget); }
      if (!quiet) { cout << p << endl; }
      s.set(p);

      II = minII;

      while (II <= maxII) {
          if (!quiet) { cout << "Calculating ASAP and ALAP Starttimes: " << endl; }
          s.reset();
          calculateStartimes(-1);
          setInitialStartTimes();
          createBooleanVariables();
          printbooleanVeriables();
          z3::check_result SAT = z3::unknown;
          stack<Edge *> violatedEdges;
          addOneSlotConstraintToSolver(s);
          addResourceContraintsToSolver(s);
          z3::model m(c);
          int i = 0;
          do {
              if (!quiet) {
                  cout << "Fixing " << violatedEdges.size() << " Dependency Constraints..." << endl << endl;
              }
              addConflictClauseNextTry(violatedEdges, s);
              if (!quiet) { cout << "******************************" << endl; }
              if (!quiet) { cout << "* Begin of Solving Iteration *" << endl; }
              if (!quiet) { cout << "******************************" << endl; }
              if (!quiet) { cout << "System of " << s.assertions().size() << " Assertions." << endl; }
              if (!quiet) { cout << "Check SMT: "; }
              auto oldM = m;
              SAT = s.check();
              if (!quiet) { cout << " >> " << SAT << " << " << endl; }
              if (SAT == z3::sat) {
                  m = s.get_model();
                  //if (!quiet) { compareModels(oldM, m); }
                  parseSMTModel(m);
                  if (!quiet) { cout << "Checking Dependency Constraints:" << endl; }
                  violatedEdges = checkSchedule(g, resourceModel, startTimes, (int) II, quiet);
                  if (!quiet) {
                      if (violatedEdges.empty()) {
                          cout << "Dependency Constraints:" << " >> sat << " << endl;
                      } else {
                          cout << "Dependency Constraints:" << " >> unsat << " << endl;
                      }
                  }
                  if (!quiet) { i++; }
                  if (!quiet) { cout << " -- " << i << " -- " << endl; }
              }
              scheduleFound = (SAT == z3::sat);
          } while (!(SAT == z3::unsat) and !violatedEdges.empty());

          if (scheduleFound) {
              break;
          }
          II++;
      }

      resetResourceModel();

      cout << endl;
  }

  int SMTCDCLScheduler::getScheduleLatency(map<Vertex *, int> &sched) {
      int maxTime = -1;
      for (std::pair<Vertex *, int> vtPair : sched) {
          try {
              Vertex *v = vtPair.first;
              if ((vtPair.second + resourceModel.getVertexLatency(v) > maxTime)) {
                  maxTime = (vtPair.second + resourceModel.getVertexLatency(v));
                  //Debugging:
                  if (!quiet) {
                      cout << vtPair.first->getName() << ": " << vtPair.second << " + "
                           << resourceModel.getVertexLatency(v) << " = " << maxTime << endl;
                  }
              }
          }catch(std::out_of_range&){
              cout << vtPair.first->getName() << ": " << vtPair.second << endl;
              throw (HatScheT::Exception("SMTBinaryScheduler::getScheduleLatency() OUT_OF_RANGE"));
          }
      }
      return maxTime;
  }

  void SMTCDCLScheduler::createBooleanVariables() {
      for (auto &v : g.Vertices()){
          for (int i = latEst.asapStartTimes.at(v); i <= latestStartTimesWithOffset.at(v); i++){
              std::stringstream name;
              name << "B" << i << "_" << v->getName();
              z3::expr bVar(c.bool_const(name.str().c_str()));
              booleanVariables.insert({std::make_pair(v, i), bVar});
          }
      }
  }

  void SMTCDCLScheduler::addOneSlotConstraintToSolver(z3::solver &s) {
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

  void SMTCDCLScheduler::addResourceContraintsToSolver(z3::solver &s) {
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

  stack<Edge*> SMTCDCLScheduler::checkSchedule(Graph &g, ResourceModel &rm, std::map<Vertex *, int> &proposedDchedule, int II, bool quiet)
  {
      bool ok;
      stack<Edge*> violatedEdges;
      for (auto it = g.edgesBegin(), end = g.edgesEnd(); it != end; it++) {
          auto e = *it;
          auto i = &e->getVertexSrc();
          auto j = &e->getVertexDst();

          ok = proposedDchedule[i] + rm.getVertexLatency(i) + e->getDelay() <= proposedDchedule[j] + e->getDistance() * II;
          if (! ok) {
              if (!quiet) cerr << *e << " violated: " << proposedDchedule[i] << " + " << rm.getVertexLatency(i) << " + " << e->getDelay() << " <= " << proposedDchedule[j] << " + " << e->getDistance() << "*" << II << endl;
              violatedEdges.push(e);
          }
      }
      return violatedEdges;
  }

  void SMTCDCLScheduler::printbooleanVeriables() {
      for(auto &v : g.Vertices()){
          cout << setw(10) << v->getName() << ": ";
          for (int i = 0; i < latEst.maxLat; i++){
              if (i % (int)II == 0) {
                  cout << " ";
              }
              try {
                  booleanVariables.at({v, i});
                  cout << "1";
              }catch (std::out_of_range&){
                  cout << "0";
              }
          }
          cout << endl;
      }
  }

  void SMTCDCLScheduler::calculateStartimes(int candidateLatency) {
      latEst = Utility::getLatencyEstimation(&g, &resourceModel, II, Utility::latencyBounds::both);
      if (candidateLatency == -1)
      {
          latestStartTimesWithOffset.clear();
          for (auto &it: latEst.alapStartTimes) {
              this->latestStartTimesWithOffset[it.first] = latEst.maxLat - (getScheduleLatency(latEst.alapStartTimes) - it.second);
          }
      }
      else
      {
          latestStartTimesWithOffset.clear();
          for (auto &it: latEst.alapStartTimes) {
              this->latestStartTimesWithOffset[it.first] = candidateLatency - (getScheduleLatency(latEst.alapStartTimes) - it.second);
          }
      }
  }

  void SMTCDCLScheduler::printSMTModel(z3::model &m) {
      for (auto &v : g.Vertices()) {
          cout << setw(10) << v->getName() << ": ";
          for (int i = 0; i < latEst.maxLat; i++) {
              if (i % (int) II == 0) {
                  cout << " ";
              }
              try {
                  cout << m.eval(booleanVariables.at({v, i})).is_true();
              } catch (std::out_of_range &) {
                  cout << "0" ;
              }
          }
          cout << endl;
      }
  }

  void SMTCDCLScheduler::modifyResourceModel() {
      for (auto &r : resourceModel.Resources()){
          resourceLimits[r]=r->getLimit();
          if (resourceModel.getNumVerticesRegisteredToResource(r) <= r->getLimit()){
              r->setLimit(UNLIMITED, false);
          }
      }
  }

  void SMTCDCLScheduler::resetResourceModel() {
      for (auto &r : resourceLimits){
          r.first->setLimit(resourceLimits.at(r.first), false);
      }
  }

  void SMTCDCLScheduler::parseSMTModel(z3::model &m) {
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

  void SMTCDCLScheduler::setInitialStartTimes() {
      for (auto &v : g.Vertices()) {
          startTimes[v] = latEst.asapStartTimes.at(v);
      }
  }

  void SMTCDCLScheduler::fixDependencyConstraints(stack<Edge*> &violatedEdges, z3::solver &s) {
      while(!violatedEdges.empty()){
          auto e = violatedEdges.top();
          addConflictClause(e, s);
          violatedEdges.pop();
      }
  }

  void SMTCDCLScheduler::addConflictClause(Edge *e, z3::solver &s) {
      //tj + distance * this->candidateII - ti - lSrc - delay >= 0
      int i = 0;
      auto *vSrc = &e->getVertexSrc();
      auto *vDst = &e->getVertexDst();
      auto lSrc = this->resourceModel.getVertexLatency(vSrc);
      auto distance = e->getDistance();
      auto delay = e->getDelay();
      for (int ti = latEst.asapStartTimes.at(vSrc); ti <= latestStartTimesWithOffset.at(vSrc); ti++) {
          for (int tj = latEst.asapStartTimes.at(vDst); tj <= latestStartTimesWithOffset.at(vDst); tj++) {
              if (tj + distance * (int) II - ti - lSrc - delay >= 0) {
                  //No Conflict... Do nothing
                  continue;
              }
              //Conflict... Not both Operations in this timeslot;
              z3::expr ex = !booleanVariables.at({vSrc, ti}) || !booleanVariables.at({vDst, tj});
              s.add(ex);
          }
      }
  }

  void SMTCDCLScheduler::compareModels(z3::model &m1, z3::model &m2) {
      for (auto &b : booleanVariables){
          if (m1.eval(b.second).is_true() != m2.eval(b.second).is_true()){
              cout << "Model Different" << endl;
          }
      }
  }

  void SMTCDCLScheduler::addConflictClauseNextTry(stack<Edge *> &eStack, z3::solver &s) {

      while(!eStack.empty()){
          auto e = eStack.top();
          eStack.pop();
          auto vSrc = &e->getVertexSrc();
          auto vDst = &e->getVertexDst();
          s.add(!booleanVariables.at({vSrc, startTimes.at(vSrc)}) || !booleanVariables.at({vDst, startTimes.at(vDst)}));
      }
  }

  void SMTCDCLScheduler::setSolverTimeout(unsigned int seconds) {
      this->timeLimit = seconds * 1000;
  }

  void SMTCDCLScheduler::reduceLatency(z3::solver &s) {
      for (auto it : startTimes){
          s.add(!booleanVariables.at({it.first, it.second}));
      }
  }

}
#endif //USE_Z3