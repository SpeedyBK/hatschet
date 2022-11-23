/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Julian Oppermann (oppermann@esa.tu-darmstadt.de)

    Copyright (C) 2018

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

    -- 20.10.2022 Benjamin Lagershausen-Keßler: Integrated "IterativeModuloSchedulerLayer" Class in ED97
*/

#include "HatScheT/scheduler/ilpbased/EichenbergerDavidson97Scheduler.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/Exception.h"

#include <iostream>
#include <map>
#include <limits>
#include <cmath>

namespace HatScheT
{

  EichenbergerDavidson97Scheduler::EichenbergerDavidson97Scheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist, int II)
      : IterativeModuloSchedulerLayer(g, resourceModel, II), ILPSchedulerBase(solverWishlist) {
  }

//  void EichenbergerDavidson97Scheduler::scheduleOLD()
//  {
//      if(this->quiet==false){
//          std::cout << "ED97: min/maxII = " << minII << " " << maxII << ", (minResII/minRecII " << this->resMinII << " / " << this->recMinII << ")" << std::endl;
//          std::cout << "ED97: solver timeout = " << this->solverTimeout << " (sec)" << endl;
//      }
//
//      //set maxRuns, e.g., maxII - minII, iff value if not -1
//      if(this->maxRuns > 0){
//          int runs = this->maxII - this->minII;
//          if(runs > this->maxRuns) this->maxII = this->minII + this->maxRuns - 1;
//          if(this->quiet==false) std::cout << "ED97: maxII changed due to maxRuns value set by user!" << endl;
//          if(this->quiet==false) std::cout << "ED97: min/maxII = " << minII << " " << maxII << std::endl;
//      }
//
//      if (minII > maxII)
//          throw HatScheT::Exception("Inconsistent II bounds");
//
//      // let's be optimistic and set them to false on timeout
//      this->firstObjectiveOptimal = true;
//      this->secondObjectiveOptimal = true;
//      for (int candII = minII; candII <= maxII; ++candII) {
//          bool proven = false;
//          scheduleAttempt(candII, feasible, proven);
//          scheduleFound |= feasible;
//          optimalResult &= proven;
//          secondObjectiveOptimal = proven;
//          if (feasible) {
//              II = candII;
//              auto solution = solver->getResult().values;
//              for (auto *i : g.Vertices())
//                  startTimes[i] = (int) std::lround(solution.find(time[i])->second);
//
//              if(this->quiet==false) {
//                  std::cout << "ED97: found " << (optimalResult ? "optimal" : "feasible") << " solution with II = " << II << std::endl;
//                  for(auto it : this->startTimes) {
//                      std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
//                  }
//              }
//              break;
//          }
//          if(!feasible and !this->quiet) cout << "  II " << candII << " : " << this->stat << endl;
//      }
//      if(scheduleFound == false) this->II = -1;
//      if(this->quiet==false) std::cout << "ED97: solving time was " << this->solvingTimePerIteration << " seconds" << std::endl;
//  }

  void EichenbergerDavidson97Scheduler::setUpSolverSettings()
  {
      this->solver->quiet   = this->solverQuiet;
      this->solver->timeout = this->solverTimeout;
      this->solver->threads = this->threads;
  }

  void EichenbergerDavidson97Scheduler::scheduleAttempt(int candII, bool &feasible, bool &proven)
  {
      solver->reset();

      constructDecisionVariables(candII);
      setObjective();
      constructConstraints(candII);
      setUpSolverSettings();

      if(this->quiet==false) {
          std::cout << "ED97: attempt II = " << candII << std::endl;
          std::cout << "ED97: solver timeout = " << this->solver->timeout << " (sec)" << endl;
      }

      //timestamp
      startTimeTracking();
      //solve
      this->stat = this->solver->solve();
      //timestamp
      endTimeTracking();

      if(this->quiet==false) {
          std::cout << "Attempt done... " << std::endl;
      }

      if(stat == ScaLP::status::TIMEOUT_INFEASIBLE) {
          this->firstObjectiveOptimal = false;
          this->timeouts++;
      }
      feasible = stat == ScaLP::status::OPTIMAL | stat == ScaLP::status::FEASIBLE   | stat == ScaLP::status::TIMEOUT_FEASIBLE;
      proven   = stat == ScaLP::status::OPTIMAL | stat == ScaLP::status::INFEASIBLE;
  }

  void EichenbergerDavidson97Scheduler::constructDecisionVariables(int candII)
  {
      time.clear();
      row.clear();
      a.clear(); a.resize(candII);
      k.clear();

      for (auto *i : g.Vertices()) {
          auto id = "_" + std::to_string(i->getId());
          // (1)
          for (int r = 0; r < candII; ++r) a[r][i] = ScaLP::newBinaryVariable("a_" + std::to_string(r) + id);

          // (2)
          k[i]    = ScaLP::newIntegerVariable("k" + id);
          row[i]  = ScaLP::newIntegerVariable("row" + id, 0, candII - 1);
          time[i] = ScaLP::newIntegerVariable("time" + id);

          solver->addConstraint(k[i] >= 0);
          solver->addConstraint(time[i] >= 0);
          if (maxLatencyConstraint >= 0) {
              solver->addConstraint(k[i]    <= maxLatencyConstraint / candII);
              solver->addConstraint(time[i] <= maxLatencyConstraint);
          }
      }
  }

  void EichenbergerDavidson97Scheduler::setObjective()
  {
      // currently only one objective: minimise the schedule length
      ScaLP::Variable ss = ScaLP::newIntegerVariable("supersink");
      solver->addConstraint(ss >= 0);
      if (maxLatencyConstraint >= 0)
          solver->addConstraint(ss <= maxLatencyConstraint);

      for (auto *i : g.Vertices()) {
          // nfiege: this is not enough!
          // sometimes (with this functionality) there is no connection to the supersink
          // -> imagine two vertices being connected in a loop with one of the two edges having a distance > 0
          // then, the source vertex of that edge must also be connected to the supersink!
          /*
          if (g.isSinkVertex(i))
            solver->addConstraint(ss - time[i] >= resourceModel.getVertexLatency(i));
            */

          // now, let's do it correctly
          if (g.hasNoZeroDistanceOutgoingEdges(i)) {
              cout << "Adding Constraint for: " << i->getName() << endl;
              solver->addConstraint(ss - time[i] >= resourceModel.getVertexLatency(i));
          }
      }
      if (!disableSecObj)
      {
          this->solver->setObjective(ScaLP::minimize(ss));
      }
      else
      {
          cout << "EichenbergerDavidson97Scheduler: Latencyminimize objective Disabled!" << endl;
      }
  }

  static inline int mod(int a, int b) {
      int m = a % b;
      return m >= 0 ? m : m + b;
  }

  void EichenbergerDavidson97Scheduler::constructConstraints(int candII)
  {
      for (auto *i : g.Vertices()) {
          // anchor source vertices, but only if they are not resource-limited
          if (g.isSourceVertex(i) && resourceModel.getResource(i)->getLimit() == UNLIMITED) {
              solver->addConstraint(k[i] == 0);
              solver->addConstraint(a[0][i] == 1);
          }

          // bind result variables (2)
          ScaLP::Term sumBind;
          for (int r = 0; r < candII; ++r) sumBind.add(a[r][i], r);
          solver->addConstraint(row[i] - sumBind == 0);
          solver->addConstraint(time[i] - (k[i] * candII) - row[i] == 0);

          // assignment constraints (1)
          ScaLP::Term sumAssign;
          for (int r = 0; r < candII; ++r) sumAssign.add(a[r][i], 1);
          solver->addConstraint(sumAssign == 1);
      }

      // resource constraints (5)
      // this could be extended to general reservation tables
      for (auto qIt = resourceModel.resourcesBegin(), qEnd = resourceModel.resourcesEnd(); qIt != qEnd; ++qIt) {
          auto *q = *qIt;
          if (q->getLimit() == UNLIMITED)
              continue;
          if (q->isReservationTable())
              throw HatScheT::Exception("ED97 formulation currently handles only simple resources");

          auto using_q = resourceModel.getVerticesOfResource(q);

          for (int r = 0; r < candII; ++r) {
              ScaLP::Term sumRes;
              for (auto *i : using_q)
                  for (int c = 0; c < q->getBlockingTime(); ++c)
                      sumRes.add(a[mod(r - c, candII)][i], 1);
              solver->addConstraint(sumRes <= q->getLimit());
          }
      }

      // 0-1-structured dependence constraints (20)
      for (auto *e : g.Edges()) {
          auto *i = &e->getVertexSrc();
          auto *j = &e->getVertexDst();
          auto l_ij     = resourceModel.getVertexLatency(i) + e->getDelay();
          auto omega_ij = e->getDistance();

          for (int r = 0; r < candII; ++r) {
              ScaLP::Term sumDep;

              for (int x = r; x < candII; ++x)
                  sumDep.add(a[x][i], 1);

              // XXX: has to be the %-operator (producing negative values!)
              for (int x = 0; x <= ((r + l_ij - 1) % candII); ++x)
                  sumDep.add(a[x][j], 1);

              sumDep.add(k[i], 1);
              sumDep.add(k[j], -1);

              solver->addConstraint(sumDep <= (omega_ij - ((r + l_ij - 1) / candII) + 1));
          }
      }
  }

  void EichenbergerDavidson97Scheduler::scheduleInit() {

      if (!this->quiet)
      {
          cout << "Scheduling with " << this->getName() <<"!" << endl;
      }

      if (!this->quiet) {
          std::cout << this->getName() << ": min/maxII = " << minII << " " << maxII << ", (minResII/minRecII " << this->resMinII << " / " << this->recMinII << ")" << std::endl;
          std::cout << this->getName() << ": solver timeout = " << this->solverTimeout << " (sec)" << endl;
      }

      if (minII > maxII)
      {
          throw HatScheT::Exception("Inconsistent II bounds");
      }

      this->firstObjectiveOptimal = true;
      this->secondObjectiveOptimal = true;

  }

  void EichenbergerDavidson97Scheduler::scheduleIteration() {
      bool proven = false;
      scheduleAttempt(this->II, feasible, proven);
      scheduleFound |= feasible;
      optimalResult &= proven;
      secondObjectiveOptimal = proven;
      if (feasible) {
          auto solution = solver->getResult().values;
          for (auto *i : g.Vertices())
              startTimes[i] = (int) std::lround(solution.find(time[i])->second);

          if(!this->quiet) {
              std::cout << "ED97: found " << (optimalResult ? "optimal" : "feasible") << " solution with II = " << II << std::endl;
//              for(auto it : this->startTimes) {
//                  std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
//              }
          }
          return;
      }
      if(!feasible and !this->quiet) cout << "  II " << this->II << " : " << this->stat << endl;
  }

  void EichenbergerDavidson97Scheduler::setSolverTimeout(double timeoutInSeconds) {
      this->solverTimeout = timeoutInSeconds;
      solver->timeout = (long)timeoutInSeconds;
      if (!this->quiet)
      {
          cout << "Solver Timeout set to " << this->solver->timeout << " seconds." << endl;
      }
  }

}