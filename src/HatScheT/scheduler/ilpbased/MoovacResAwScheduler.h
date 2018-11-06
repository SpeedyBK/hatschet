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
#include "HatScheT/scheduler/ilpbased/MoovacScheduler.h"
#include "HatScheT/TargetModel.h"

namespace HatScheT {
/*!
 * EXPERIMENTAL! DO NOT USE THIS CLASS
 * @param g
 * @param resourceModel
 * @param solverWishlist
 */
class MoovacResAwScheduler : public MoovacScheduler {
public:
  MoovacResAwScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist, Target& target);

  virtual void schedule();

private:
  virtual void constructProblem();
  virtual void setObjective();
  virtual void setGeneralConstraints();
  virtual void setModuloAndResourceConstraints();
  /*!
   * calculate and set the limit of every resource to the maximum possible
   * value when only instance of all the others resource is used
   *
   * This is done to generate the corner in the design space for resource allocation
   */
  void getAk();
  map<Resource*,int > A_k;

  Target& target;
};

}

