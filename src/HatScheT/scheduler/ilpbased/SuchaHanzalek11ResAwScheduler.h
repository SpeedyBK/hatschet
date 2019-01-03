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

#pragma once

#include <HatScheT/scheduler/ilpbased/SuchaHanzalek11Scheduler.h>
#include <map>

namespace HatScheT
{
/*!
 * \brief Implementation of the resource-aware ILP formulation by Šůcha and Hanzálek.
 *
 * --> EXPERIMENTAL <--
 *
 * Reference:
 *   Přemysl Šůcha and Zdenĕk Hanzálek: A cyclic scheduling problem with an undetermined number of parallel identical
 *   processors. Comp. Opt. and Appl., Vol 48, 2011.
 */
class SuchaHanzalek11ResAwScheduler :  public SuchaHanzalek11Scheduler
{
public:
    SuchaHanzalek11ResAwScheduler(Graph& g, ResourceModel &resourceModel, Target &target, std::list<std::string> solverWishlist);
  /*!
   * \brief Attempts to schedule the given instances. The candidate II is incremented until a feasible schedule is found.
   */
  virtual void schedule();

protected:
  virtual void constructDecisionVariables(int candII);
  virtual void constructResourceConstraints(int candII);
  virtual void setObjective();

  void compute_m_max();

  // weighting factor used in the bi-criteria objective
  // 0 = only resource minimisation
  // 1 = only schedule length minimisation
  double alpha;

  // description of the target device
  Target &target;

  // precomputed maximum allocatable resource instances
  std::map<const Resource*, int> m_max;

  // additional decision variables
  std::map<const Resource*, ScaLP::Variable> m;
};
}
