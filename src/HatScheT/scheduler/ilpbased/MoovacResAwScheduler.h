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
  /*!
   *
   * @param g
   * @param resourceModel
   * @param solverWishlist
   * @param target provide a hardware target and resource limitation
   * for resource aware modulo scheduling
   */
  MoovacResAwScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist, Target& target);
  /*!
   * schedule the provided problem
   */
  virtual void schedule();
  /*!
   * This is variable is used to balance the objective between resource and latency reduction
   * 0 <= lambda <= 1
   * 0 means only resources are minimzed
   * 1 means only the latency is minimized
   *
   * default is 0
   * @param l set lambda
   */
  void setLambda(double l){
    if(l < 0.0f or l > 1.0f) throw Exception("MoovacResAwScheduler.setLambda: Lambda value is restricted to be 0 <=  lambda <= 1! You request: " + to_string(l));
    this->lambda = l;
  }
  double getLambda(){return this->lambda;}
private:
  /*!
   * extension of the base class to use RAMS scheduling
   * see base class for more information
   */
  virtual void constructProblem();
  /*!
   * extension of the base class to use RAMS scheduling
   * see base class for more information
   */
  virtual void setObjective();
  /*!
   * extension of the base class to use RAMS scheduling
   * see base class for more information
   */
  virtual void setGeneralConstraints();
  /*!
   * extension of the base class to use RAMS scheduling
   * see base class for more information
   */
  virtual void setModuloAndResourceConstraints();
  /*!
   * the new allocation constraints force the allocation
   * to respect the hardware restriction
   */
  void setAllocationConstraints();
  /*!
   * the aks vector contains the allocation variables for each limited resource
   */
  void fillAksVectorAndSetConstaints();
  /*!
   * this vector contains the allocation variables for each limited resource
   */
  vector<ScaLP::Variable> aks;
  map<Resource*,int > aksIndices; //store info Resource -> ScaLP::Variable
  /*!
   * calculate and set the limit of every resource to the maximum possible
   * value when only instance of all the others resource is used
   *
   * This is done to generate the corner in the design space for resource allocation
   */
  void getAk();
  /*!
   * the corner case maximum resource allocations for every limited resource is stored here
   */
  map<Resource*,int > A_k;
  /*!
   * information about the hardware target is stored here
   */
  Target& target;
  /*!
   * This is variable is used to balance the objective between resource and latency reduction
   * 0 <= lambda <= 1
   * 0 means only resources are minimzed
   * 1 means only the latency is minimized
   *
   * default is 0
   */
  double lambda;
};

}

