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

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <vector>

namespace HatScheT
{
/*!
 * \brief Implementation of the ILP formulation by Eichenberger and Davidson.
 *
 * Reference:
 *   Alexandre E. Eichenberger and Edward S. Davidson: Efficient Formulation for Optimal Modulo Schedulers.
 *   Proceedings of the ACM SIGPLAN '97 Conference on Programming Language Design and Implementation (PLDI), 1997
 */
class EichenbergerDavidson97Scheduler :  public SchedulerBase, public ILPSchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase
{
public:
  EichenbergerDavidson97Scheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist, int II=-1);
  /*!
   * \brief Attempts to schedule the given instances. The candidate II is incremented until a feasible schedule is found.
   */
  virtual void schedule();

protected:
  virtual void setUpSolverSettings();

  virtual void scheduleAttempt(int candII, bool &feasible, bool &proven);
  virtual void constructDecisionVariables(int candII);
  virtual void constructConstraints(int candII);
  virtual void setObjective();

    /*!
     * not needed
     */
    virtual void resetContainer(){}

  virtual void constructProblem() {/* unused */}

  // decision variables
  std::vector<std::map<const Vertex*, ScaLP::Variable>> a;
  std::map<const Vertex*, ScaLP::Variable> k, row, time;

	int maxLatency = -1;
	int minLatency = -1;
  bool useLatencyEstimation = true;
};
}
