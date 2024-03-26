/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)

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

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <vector>

namespace HatScheT
{

/*!
 * Scheduler for ilp based asap scheduling
 * This formulation is time-index based and uses binary variables
 * As a result, an upper bound for the schedule is needed
 * use the setMaxLatencyConstraint method to do this before calling the schedule() method!!
 */
class ASAPILPScheduler:  public SchedulerBase, public ILPSchedulerBase
{
public:
  /*!
   * constructor
   * @param g graph to schedule
   * @param resourceModel info and resource constraints
   * @param solverWishlist scalp solver selection
   */
  ASAPILPScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
  /*!
   * use this methode to solve the scheduling problem
   */
  void schedule() override;

protected:
  /*!
   * \brief setGeneralConstraints e.g. depencies, recurrences, latencies
   */
  virtual void setGeneralConstraints();
  /*!
  * \brief constructProblem Using the graph, resource model, an II and solver settings, the problem is constructed
  */
  void constructProblem() override;
  /*!
  *  \brief setObjective asap
  */
  void setObjective() override;
  /*!
   * \brief generate and store the needed ilp variables
   */
  void setVectorVariables();
  /*!
   * \brief provide all the scheduling information to the interface iff a valid schedule was found
   */
  void fillSolutionStructure();
  /*!
   * \brief generate ilp to respect resource constraints
   */
  void setResourceConstraints();
  /*!
   * not needed
   */
  void resetContainer() override{}
  /*!
   * \brief container for time variables
   */
  vector<ScaLP::Variable> ti;
  /*!
   * \brief container to store relations between vertices and variables
   */
  map<const Vertex*, unsigned int> tIndices;
  /*!
   * \brief container to stare relations of binary variables
   */
  map<const Vertex*, vector<ScaLP::Variable> > binVarMap;
};

}



