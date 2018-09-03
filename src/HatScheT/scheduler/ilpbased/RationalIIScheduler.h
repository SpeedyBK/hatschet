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

#pragma once

#include <HatScheT/scheduler/ilpbased/MoovacScheduler.h>

namespace HatScheT
{
/*!
 * experimental: This scheduler determines a modulo schedule with uneven/rational initiation intervals
 * THIS CLASSES CONSTRUCTOR IS CURRENTLY DISABLED
 */
class RationalIIScheduler : public SchedulerBase, public ILPSchedulerBase, public ModuloSchedulerBase
{
public:
  /*!
   * provide a graph, resource and solver wishlist since this scheduler is ilp-based and uses ScaLP for
   * generating ILP and solving
   * @param g
   * @param resourceModel
   * @param solverWishlist
   */
  RationalIIScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
  virtual void schedule();

private:
  void setResourceConstraints();
  void setGeneralConstraints();
  void setModuloConstraints();
  virtual void constructProblem();
  virtual void setObjective();
  void fillTMaxtrix();
  void fillIIVector();
  //--------

  unsigned int moduloClasses;
  unsigned int consideredTimeSteps;
  unsigned int consideredModuloCycle;
  vector<int> foundIIs;
  ScaLP::Result r;
  vector<vector<ScaLP::Variable> > t_matrix;
  vector<ScaLP::Variable> II_vector;
  map<const Vertex*,int> tIndices;

};
}
