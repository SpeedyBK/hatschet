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

    -- 20.10.2022 Benjamin Lagershausen-Keßler: Integrated "IterativeModuloSchedulerLayer" Class
*/
#pragma once
#include "HatScheT/scheduler/ilpbased/MoovacScheduler.h"
#include "HatScheT/TargetModel.h"

namespace HatScheT {
/*!
 * This scheduler determines an optimal resource allocation and a modulo schedule
 *
 * Developed by Patrick Sittel (affiliated with University of Kassel, Imperial College London)
 *
 * For the details see
 * Oppermann, Sittel, Kumm, Reuter-Oppermann, Koch, Sinnen: Design-space Exploration with Multi-objective Resource
 * Aware Modulo Scheduling,
 * Presented at the Proceedings of the 25th International Conference on Parallel and Distributed Computing, Goettingen 2019.
 *
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
   * get the hardware bindings of the determined II
   * @return
   */
  virtual std::map<const Vertex*,int> getBindings();
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
  /*!
   * this value needs to be set true iff a full design space exploration is desired
   * otherwise the rams scheduler will terminate after the first found II even if there might be other throughput/resource tradeoffs possible
   *
   * Default FALSE (no DSE)
   * @param b
   */
  void setFullDSE(bool b){
    this->fullDSE = b;
  }
  bool getFullDSE(){return  this->fullDSE;}
  /*!
   * when a design space exploration is done, this map stores valid schedules (starttimes) for each found II
   * @return
   */
  std::map<int , std::map<Vertex*,int> >& getDSEStartTimes(){
    return this->dseStartTimes;
  }
  /*!
   * when a design space exploration is done, this map stores valid resource allocations for each found II
   * @return
   */
  std::map<int , std::map<Resource*,int> >& getDSEAllocations(){
    return this->dseAllocations;
  }
    /*!
   * when a design space exploration is done, this map stores whether a solution is optimal for each found II
   * @return
   */
    std::map<int , bool >& getDSEResultOptimal(){
      return this->dseResultOptimal;
    }
  /*!
   * fill the standard containers with a II specific design space exploration result iff it exists
   * @param requII
   */
  void setDSEResult(int requII);
  /*!
   * get a vector of all found IIs during a design space exploration
   * the result is sorting begining with the smallest value
   * @return
   */
  vector<int > getFoundIIs();

  string getName() override { return "MoovacResAwScheduler"; }

protected:

  void scheduleIteration() override;

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
   * store determined schedule and allocation for this II when fullDSE == TRUE
   */
  void storeScheduleAndAllocation();
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
  /*!
   * this value needs to be set true iff a full design space exploration is desired
   * otherwise the rams scheduler will terminate after the first found II even if there might be other throughput/resource tradeoffs possible
   *
   * Default FALSE (no DSE)
   */
  bool fullDSE;
  /*!
   * this flag is used to stop the design space exploration when no more resource can be saved
   */
  bool DSEfinshed;
  /*!
   * when a design space exploration is done, this map stores valid schedules (starttimes) for each found II
   */
  std::map<int , std::map<Vertex*,int> > dseStartTimes;
  /*!
   * when a design space exploration is done, this map stores valid resource allocations for each found II
   */
  std::map<int , std::map<Resource*,int> > dseAllocations;
  /*!
   * when a design space exploration is done, this map stores valid hardware bindings for each found II
   */
  std::map<int , std::map<const Vertex*, int> > dseBindings;
  /*!
   * when a design space exploration is done, this map stores whether a solution is optimal for each found II
   */
  std::map<int , bool > dseResultOptimal;
};

}

