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
*/

#include "HatScheT/scheduler/ilpbased/SuchaHanzalek11Scheduler.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/Exception.h"

#include <iostream>
#include <map>
#include <cmath>

namespace HatScheT
{

SuchaHanzalek11Scheduler::SuchaHanzalek11Scheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist, int II)
	: IterativeModuloSchedulerLayer(g, resourceModel), ILPSchedulerBase(solverWishlist) {

    optimalResult = true;
    this->feasible = false;

}

void SuchaHanzalek11Scheduler::setUpSolverSettings()
{
  solver->quiet   = solverQuiet;
  solver->timeout = solverTimeout;
  solver->threads = threads;
}

void SuchaHanzalek11Scheduler::scheduleAttempt(int candII, bool &feasible, bool &proven)
{
  std::cout << "SH11: attempt II=" << candII << std::endl;
  solver->reset();

  constructDecisionVariables(candII);
  setObjective();
  constructConstraints(candII);

  //timestamp
  startTimeTracking();
  //solve
  stat = solver->solve();
  //timestamp
  endTimeTracking();

  feasible = stat == ScaLP::status::OPTIMAL | stat == ScaLP::status::FEASIBLE   | stat == ScaLP::status::TIMEOUT_FEASIBLE;
  proven   = stat == ScaLP::status::OPTIMAL | stat == ScaLP::status::INFEASIBLE;
}

void SuchaHanzalek11Scheduler::constructDecisionVariables(int candII)
{
  s.clear();
  s_hat.clear();
  q_hat.clear();
  x_hat.clear();
  y_hat.clear();

  for (auto *i : g.Vertices()) {
    auto id = "_" + std::to_string(i->getId());

    s[i]     = ScaLP::newIntegerVariable("s" + id);
    s_hat[i] = ScaLP::newIntegerVariable("s_hat" + id, 0, candII - 1);
    q_hat[i] = ScaLP::newIntegerVariable("q_hat" + id);

    solver->addConstraint(q_hat[i] >= 0);
    solver->addConstraint(s[i] >= 0);
    if (maxLatencyConstraint >= 0) {
      solver->addConstraint(q_hat[i] <= maxLatencyConstraint / candII);
      solver->addConstraint(s[i] <= maxLatencyConstraint);
    }

    for (auto *j : g.Vertices()) {
      if (i == j)
        continue;

      auto pairId = "_" + std::to_string(i->getId()) + "_" + std::to_string(j->getId());
      x_hat[i][j] = ScaLP::newBinaryVariable("x_hat" + pairId);
      y_hat[i][j] = ScaLP::newBinaryVariable("y_hat" + pairId);
    }
  }
}

void SuchaHanzalek11Scheduler::setObjective()
{
  // currently only one objective: minimise the schedule length
  ScaLP::Variable ss = ScaLP::newIntegerVariable("supersink");
  solver->addConstraint(ss >= 0);
  if (maxLatencyConstraint >= 0)
    solver->addConstraint(ss <= maxLatencyConstraint);

  for (auto *i : g.Vertices())
    if (g.isSinkVertex(i))
      solver->addConstraint(ss - s[i] >= resourceModel.getVertexLatency(i));

  solver->setObjective(ScaLP::minimize(ss));
}

void SuchaHanzalek11Scheduler::constructConstraints(int candII)
{
  constructGeneralConstraints(candII);
  constructDependenceConstraints(candII);
  constructResourceConstraints(candII);
}

void SuchaHanzalek11Scheduler::constructGeneralConstraints(int candII)
{
  const int w = candII;
  for (auto *i : g.Vertices()) {
    // anchor source vertices, but only if they are not resource-limited
    if (g.isSourceVertex(i) && resourceModel.getResource(i)->getLimit() == UNLIMITED) {
      solver->addConstraint(s_hat[i] == 0);
      solver->addConstraint(q_hat[i] == 0);
    }

    // bind result variable (3)
    solver->addConstraint(s[i] - q_hat[i] * w - s_hat[i] == 0);
  }
}

void SuchaHanzalek11Scheduler::constructDependenceConstraints(int candII)
{
  const int w = candII;
  for (auto *e : g.Edges()) {
    auto *i = &e->getVertexSrc();
    auto *j = &e->getVertexDst();
    auto l_ij = resourceModel.getVertexLatency(i) + e->getDelay();
    auto h_ij = e->getDistance();

    // (5)
    solver->addConstraint(s_hat[j] + q_hat[j] * w - s_hat[i] - q_hat[i] * w >= l_ij - h_ij * w);
  }
}

void SuchaHanzalek11Scheduler::constructResourceConstraints(int candII)
{
  const int w = candII;
  for (auto pIt = resourceModel.resourcesBegin(), pEnd = resourceModel.resourcesEnd(); pIt != pEnd; ++pIt) {
    auto *p = *pIt;
    if (p->getLimit() == UNLIMITED)
      continue;
    if (p->isReservationTable())
      throw HatScheT::Exception("SH11 formulation currently handles only simple resources");
    if (p->getBlockingTime() > 1)
      throw HatScheT::Exception("SH11: this is the unit-processing time variant");

    auto pvs = resourceModel.getVerticesOfResource(p);
    std::vector<const Vertex *> using_p(pvs.begin(), pvs.end());

    const int m_p = p->getLimit();
    const int n_p = using_p.size();

    // for all i, j in {1, ..., n_p}: i < j
    for (int _j = 0; _j < n_p; ++_j) {
      for (int _i = 0; _i < _j; ++_i) {
        auto *i = using_p[_i];
        auto *j = using_p[_j];

        // overlap 1 (6)
        solver->addConstraint(s_hat[i] - s_hat[j] + w * x_hat[i][j] + (1-w) * y_hat[i][j] >= 1);

        // overlap 2 (7)
        solver->addConstraint(s_hat[i] - s_hat[j] + w * x_hat[i][j] - y_hat[i][j] <= w - 1);

        // overlap 3 (8)
        solver->addConstraint(-1 * x_hat[i][j] + y_hat[i][j] <= 0);
      }
    }

    // limit resource conflicts (9)
    for (int _i = 0; _i < n_p - m_p; ++_i) {
      auto *i = using_p[_i];
      ScaLP::Term sum;
      for (int _j = _i+1; _j < n_p; ++_j) {
        auto *j = using_p[_j];
        sum.add(y_hat[i][j], 1);
      }
      solver->addConstraint(sum <= m_p - 1);
    }
  }
}

  void SuchaHanzalek11Scheduler::scheduleInit() {
      // reset previous solutions
      startTimes.clear();
      this->feasible = false;

      if (!this->quiet)
      {
          cout << "Scheduling with " << this->getName() <<"!" << endl;
          std::cout << "SH11: min/maxII = " << minII << " " << maxII << std::endl;
      }

      if (minII > maxII)
      {
          throw HatScheT::Exception("Inconsistent II bounds");
      }

      setUpSolverSettings();
  }

  void SuchaHanzalek11Scheduler::scheduleIteration() {
      bool proven = false;
      scheduleAttempt((int)II, this->feasible, proven);
      scheduleFound |= feasible;
      optimalResult &= proven;
      if (feasible)
      {
          auto solution = solver->getResult().values;
          for (auto *i : g.Vertices())
          {
              startTimes[i] = (int) std::lround(solution.find(s[i])->second);
          }
          if (!this->quiet)
          {
              std::cout << "SH11: found " << (optimalResult ? "optimal" : "feasible") << " solution with II=" << II
                        << std::endl;
          }
          return;
      }
  }

  void SuchaHanzalek11Scheduler::setSolverTimeout(double timeoutInSeconds) {
      this->solverTimeout = timeoutInSeconds;
      solver->timeout = (long)timeoutInSeconds;
      if (!this->quiet)
      {
          cout << "Solver Timeout set to " << this->solver->timeout << " seconds." << endl;
      }
  }
}
