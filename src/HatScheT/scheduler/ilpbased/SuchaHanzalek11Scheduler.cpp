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
#include <limits>
#include <cmath>

namespace HatScheT
{

SuchaHanzalek11Scheduler::SuchaHanzalek11Scheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist) : SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist) { }

void SuchaHanzalek11Scheduler::schedule()
{
  // reset previous solutions
  II = -1;
  startTimes.clear();
  scheduleFound = false;
  optimalResult = true;
  computeMinII(&g, &resourceModel);
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
        startTimes[i] = 0; // TODO

      std::cout << "SH11: found " << (optimalResult ? "optimal" : "feasible") << " solution with II=" << II << std::endl;
      break;
    }
  }
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

  stat     = solver->solve();
  feasible = stat == ScaLP::status::OPTIMAL | stat == ScaLP::status::FEASIBLE   | stat == ScaLP::status::TIMEOUT_FEASIBLE;
  proven   = stat == ScaLP::status::OPTIMAL | stat == ScaLP::status::INFEASIBLE;
}

void SuchaHanzalek11Scheduler::constructDecisionVariables(int candII)
{
}

void SuchaHanzalek11Scheduler::setObjective()
{
}

void SuchaHanzalek11Scheduler::constructConstraints(int candII)
{
}

}
