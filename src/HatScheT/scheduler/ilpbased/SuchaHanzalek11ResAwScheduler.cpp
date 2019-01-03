/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Julian Oppermann (oppermann@esa.tu-darmstadt.de)

    Copyright (C) 2019

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

#include "HatScheT/scheduler/ilpbased/SuchaHanzalek11ResAwScheduler.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/Exception.h"

#include <iostream>
#include <map>
#include <cmath>

namespace HatScheT
{

SuchaHanzalek11ResAwScheduler::SuchaHanzalek11ResAwScheduler(Graph &g, ResourceModel &resourceModel, Target &target, std::list<std::string> solverWishlist)
  : SuchaHanzalek11Scheduler(g, resourceModel, solverWishlist), target(target) { }

void SuchaHanzalek11ResAwScheduler::schedule()
{
  // reset previous solutions
  II = -1;
  startTimes.clear();
  scheduleFound = false;
  optimalResult = true;
  computeMinII(&g, &resourceModel, &target);
  minII = ceil(minII);
  computeMaxII(&g, &resourceModel);

  std::cout << "SH11: min/maxII = " << minII << " " << maxII << std::endl;

  if (minII > maxII)
    throw HatScheT::Exception("Inconsistent II bounds");

  setUpSolverSettings();

  bool feasible = false;
  for (int candII = minII; candII <= maxII; ++candII) {
    bool proven = false;
    scheduleAttempt(candII, feasible, proven);
    scheduleFound |= feasible;
    optimalResult &= proven;
    if (feasible) {
      II = candII;
      auto solution = solver->getResult().values;
      for (auto *i : g.Vertices())
        startTimes[i] = (int) std::lround(solution.find(s[i])->second);

      std::cout << "SH11: found " << (optimalResult ? "optimal" : "feasible") << " solution with II=" << II << std::endl;
      break;
    }
  }
}

void SuchaHanzalek11ResAwScheduler::constructDecisionVariables(int candII)
{
  SuchaHanzalek11Scheduler::constructDecisionVariables(candII);
  m.clear();

  // for r in R : ...
}

void SuchaHanzalek11ResAwScheduler::setObjective()
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

void SuchaHanzalek11ResAwScheduler::constructResourceConstraints(int candII)
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

}
