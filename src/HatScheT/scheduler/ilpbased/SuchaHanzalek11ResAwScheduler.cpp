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

namespace HatScheT {

  SuchaHanzalek11ResAwScheduler::SuchaHanzalek11ResAwScheduler(Graph &g, ResourceModel &resourceModel, Target &target,
                                                               std::list<std::string> solverWishlist)
      : SuchaHanzalek11Scheduler(g, resourceModel, solverWishlist), target(target) {

      alpha = 0.0;

  }

  void SuchaHanzalek11ResAwScheduler::constructDecisionVariables(int candII) {
      SuchaHanzalek11Scheduler::constructDecisionVariables(candII);
      m.clear();

      for (auto pIt = resourceModel.resourcesBegin(), pEnd = resourceModel.resourcesEnd(); pIt != pEnd; ++pIt) {
          Resource *p = *pIt;
          int n_p = resourceModel.getNumVerticesRegisteredToResource(p);
          if (p->isUnlimited()) {
              m[p] = ScaLP::newIntegerVariable("m_" + p->getName());
              solver->addConstraint(m[p] == n_p);
              continue;
          }
          // limited resource
          if (p->isReservationTable())
              throw HatScheT::Exception("SH11 formulation currently handles only simple resources");
          if (p->getBlockingTime() > 1)
              throw HatScheT::Exception("SH11: this is the unit-processing time variant");

          m[p] = ScaLP::newIntegerVariable("m_" + p->getName(), 0, min(m_max[p], n_p));
      }
  }

  void SuchaHanzalek11ResAwScheduler::setObjective() {
      // first objective: minimise (device) resource usage
      ScaLP::Term deviceUsage;
      for (auto pIt = resourceModel.resourcesBegin(), pEnd = resourceModel.resourcesEnd(); pIt != pEnd; ++pIt) {
          Resource *p = *pIt;

          // do not consider unlimited resource in the objective. They are only relevant for the device constraints.
          if (p->isUnlimited())
              continue;

          for (auto targetEntry : target.getElements()) {
              auto costId = targetEntry.first;
              auto deviceLimit = targetEntry.second;

              double usage = p->getHardwareCost(costId);
              if (usage == 0.0)
                  continue;

              ScaLP::Term relativeUsage = m[p] * (usage / deviceLimit);
              deviceUsage += relativeUsage;
          }
      }

      // second objective: minimise the schedule length
      ScaLP::Variable ss = ScaLP::newIntegerVariable("supersink");
      solver->addConstraint(ss >= 0);
      if (maxLatencyConstraint >= 0)
          solver->addConstraint(ss <= maxLatencyConstraint);

      for (auto *i : g.Vertices())
          if (g.isSinkVertex(i))
              solver->addConstraint(ss - s[i] >= resourceModel.getVertexLatency(i));

      ScaLP::Term objective;
      objective += (1 - alpha) * deviceUsage;
      objective += (alpha) * ss;
      solver->setObjective(ScaLP::minimize(objective));
  }

  void SuchaHanzalek11ResAwScheduler::constructResourceConstraints(int candII) {
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

          const int m_p_max = m_max[p];
          const int n_p = using_p.size();

          // for all i, j in {1, ..., n_p}: i < j
          for (int _j = 0; _j < n_p; ++_j) {
              for (int _i = 0; _i < _j; ++_i) {
                  auto *i = using_p[_i];
                  auto *j = using_p[_j];

                  // overlap 1 (6)
                  solver->addConstraint(s_hat[i] - s_hat[j] + w * x_hat[i][j] + (1 - w) * y_hat[i][j] >= 1);

                  // overlap 2 (7)
                  solver->addConstraint(s_hat[i] - s_hat[j] + w * x_hat[i][j] - y_hat[i][j] <= w - 1);

                  // overlap 3 (8)
                  solver->addConstraint(-1 * x_hat[i][j] + y_hat[i][j] <= 0);
              }
          }

          // limit resource conflicts: (9) BEFORE Lemma 1
          //
          // The 'm' on the right-hand side of the constraint can be the new decision variable, but
          // the constant 'm' did also influence the number of constraints. Shall we use m_max? (feels wrong).
          // To this end, we use the unoptimised variant here.
          for (int _i = 0; _i < n_p; ++_i) {
              auto *i = using_p[_i];
              ScaLP::Term sum;
              for (int _j = 0; _j < n_p; ++_j) {
                  if (_i == _j)
                      continue;
                  auto *j = using_p[_j];
                  sum += y_hat[i][j];
              }
              solver->addConstraint(sum - m[p] <= -1);
          }

          // limit usage of device's resources (à la (17), (18))
          for (auto targetEntry : target.getElements()) {
              auto costId = targetEntry.first;
              auto deviceLimit = targetEntry.second;

              ScaLP::Term usage;
              for (auto pIt = resourceModel.resourcesBegin(), pEnd = resourceModel.resourcesEnd(); pIt != pEnd; ++pIt) {
                  Resource *p = *pIt;
                  double pUsage = p->getHardwareCost(costId);
                  if (pUsage == 0.0)
                      continue;

                  usage += m[p] * pUsage;
              }

              solver->addConstraint(usage <= deviceLimit);
          }
      }
  }

// TODO: this is an almost verbatim copy of MoovacResAwScheduler::getAk().
//       We should move this functionality to a base class.
  void SuchaHanzalek11ResAwScheduler::compute_m_max() {
      for (auto it = this->resourceModel.resourcesBegin(); it != resourceModel.resourcesEnd(); ++it) {
          Resource *r = *it;
          if (r->isUnlimited() == true) continue;
          //skip when resource is in model which is never used
          if (this->resourceModel.getNumVerticesRegisteredToResource(r) == 0) continue;

          //calculate minimum costs of all other resources
          map<std::string, double> minCosts;

          for (auto it2 = this->resourceModel.resourcesBegin(); it2 != this->resourceModel.resourcesEnd(); ++it2) {
              Resource *r_it = *it2;
              //skip the resource itself
              //skip unlimited
              if (r == r_it || r_it->isUnlimited() == true) continue;

              //add resource costs of one time implementation
              for (auto it3 = r_it->getHardwareCosts().begin(); it3 != r_it->getHardwareCosts().end(); ++it3) {
                  std::string costName = it3->first;
                  double resCost = it3->second;

                  //check whether costs already in map or new
                  if (minCosts.find(costName) == minCosts.end()) {
                      minCosts.insert(*it3); //not found, insert new
                  } else {
                      minCosts[costName] += resCost; //found, add to existing
                  }
              }
          }

          //calculate remaining resources for this resource
          map<std::string, double> remainingSpace;

          //handle case when only limited resource is provided
          if (minCosts.size() == 0) {
              remainingSpace = this->target.getElements();
          }

          //handle case when more limited resource are provided
          for (auto it2 = minCosts.begin(); it2 != minCosts.end(); ++it2) {
              if (this->target.getElement(it2->first) - minCosts[it2->first] < 0)
                  throw Exception(
                      "SH11RA.compute_m_max: Error negative space detected for hardware element " + it2->first);
              //add difference to remaining space map
              remainingSpace.insert(make_pair(it2->first, this->target.getElement(it2->first) - minCosts[it2->first]));
          }

          //finally, calculate maximal possible number of hardware units for this resource
          int Ak = 0;
          for (auto it2 = r->getHardwareCosts().begin(); it2 != r->getHardwareCosts().end(); ++it2) {
              std::string costName = it2->first;
              double resCost = it2->second;
              //skip if no costs for this element
              if (resCost == 0.0f) continue;

              int unitsFit = remainingSpace[costName] / resCost;

              if (unitsFit < 1) {
                  cout << "SH11RA.compute_m_max: Error for " << costName << " of " << r->getName() << endl;
                  cout << "SH11RA.compute_m_max: Error " << remainingSpace[costName] << " / " << resCost << " < 1 "
                       << endl;
                  throw Exception(
                      "SH11RA.compute_m_max: Error no space left allocating one hardware unit of resource " +
                      r->getName());
              }
              if (Ak == 0) Ak = unitsFit;
              else if (unitsFit < Ak) Ak = unitsFit;
          }
          if (Ak == 0)
              throw Exception(
                  "SH11RA.compute_m_max: Error maximal resource allocation of 0 determined: " + r->getName());

          this->m_max.insert(make_pair(r, Ak));

          cout << "m_max: " << r->getName() << " = " << Ak << endl;
      }
  }

  void SuchaHanzalek11ResAwScheduler::scheduleInit() {

      compute_m_max();

      alpha = 0.0;

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

  void SuchaHanzalek11ResAwScheduler::scheduleIteration() {
      bool proven = false;
      scheduleAttempt((int) II, feasible, proven);
      scheduleFound |= feasible;
      optimalResult &= proven;
      if (feasible) {
          auto solution = solver->getResult().values;
          for (auto *i : g.Vertices())
              startTimes[i] = (int) std::lround(solution.find(s[i])->second);

          std::cout << "SH11: found " << (optimalResult ? "optimal" : "feasible") << " solution with II=" << II
                    << std::endl;

          std::cout << "      Determined resource allocation (at alpha=" << alpha << "):" << std::endl;
          for (auto entry : m) {
              auto p = entry.first;
              auto dvar = entry.second;

              int numAllocated = (int) std::lround(solution.find(dvar)->second);
              if (!p->isUnlimited())
                  resourceModel.getResource(p->getName())->setLimit(
                      numAllocated); // XXX: is this the right thing to do?

              std::cout << "      " << p->getName() << ":\t" << numAllocated << std::endl;
          }
          return;
      }
  }
}