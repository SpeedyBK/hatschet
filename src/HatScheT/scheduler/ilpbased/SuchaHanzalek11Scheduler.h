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

    -- 20.10.2022 Benjamin Lagershausen-Keßler: Integrated "IterativeModuloSchedulerLayer" Class
*/

#pragma once

#include <HatScheT/layers/IterativeModuloSchedulerLayer.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <map>

namespace HatScheT
{
/*!
 * \brief Implementation of the ILP formulation by Šůcha and Hanzálek.
 *
 * Reference:
 *   Přemysl Šůcha and Zdenĕk Hanzálek: A cyclic scheduling problem with an undetermined number of parallel identical
 *   processors. Comp. Opt. and Appl., Vol 48, 2011.
 */
class SuchaHanzalek11Scheduler :  public IterativeModuloSchedulerLayer, public ILPSchedulerBase
{
public:
    SuchaHanzalek11Scheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist, int II=-1);
  /*!
   * \brief Attempts to schedule the given instances. The candidate II is incremented until a feasible schedule is found.
   */

  /*!
   * Mainly for debugging.
   * @return Name of the scheduler
   */
  string getName() override { return "SuchaHanzalek11"; }

  /*!
   * Function to set the solver Timeout
   * @param seconds
   */
  void setSolverTimeout(double timeoutInSeconds) override;

protected:
  virtual void setUpSolverSettings();

  virtual void scheduleAttempt(int candII, bool &feasible, bool &proven);
  virtual void constructDecisionVariables(int candII);
  virtual void constructConstraints(int candII);
  virtual void constructGeneralConstraints(int candII);
  virtual void constructDependenceConstraints(int candII);
  virtual void constructResourceConstraints(int candII);
  virtual void setObjective();

  virtual void constructProblem() {/* unused */}

  /*!
   * Initialize stuff before II-Search-Loop starts.
   */
  void scheduleInit() override;
  /*!
   * \brief Schedule Iteration for one II.
   */
  void scheduleIteration() override;

  bool feasible;

    /*!
     * not needed
     */
    virtual void resetContainer(){}

  // decision variables
  std::map<const Vertex*, ScaLP::Variable> s, s_hat, q_hat;
  std::map<const Vertex*, std::map<const Vertex*, ScaLP::Variable>> x_hat, y_hat;

};
}
