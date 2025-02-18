/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)

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

#pragma once

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <HatScheT/layers/RationalIISchedulerLayer.h>
#include <vector>

namespace HatScheT
{
/*!
 * This scheduler determines a modulo schedule with uneven/rational initiation intervals
 *
 * Developed by Patrick Sittel (affiliated with University of Kassel, Imperial College London)
 *
 * provides: schedule, initiation interval sequence, task latency sequence
 * the standard configuration returns a static schedule
 * the configuration that supports dynamic schedules is not supported and tested yet!
 *
 * For the details see
 * Sittel, Wickerson, Kumm, Zipf: Modulo Scheduling with Rational Initiation Intervals in Custom Hardware Design,
 * Proceedings of the 25th Asia and South Pacific Design Automation Conference (ASP-DAC), Beijing 2020
 *
 */
class RationalIIScheduler : public ILPSchedulerBase, public RationalIISchedulerLayer
{
public:
  /*!
   * the rational II modulo scheduler is able to provide a higher throughput than traditional integer II schedulers in many cases
   * the trade off is a more complex control flow structure
   * @param g
   * @param resourceModel
   * @param solverWishlist
   */
  RationalIIScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
  /*!
   * print the rational II modulo schedule
   */
  void printScheduleToConsole();
  /*!
   * @brief print the MRTs of all resources after rational II scheduling and binding
   */
  void printBindingToConsole();

  /*!
   * EXPERIMENTAL: DONT USE THIS CURRENTLY
   * @param b
   */
  void setUniformScheduleFlag(bool b){this->uniformSchedule=b;}
  bool getUniformScheduleFlag(){return this->uniformSchedule;}
  /*!
   * @brief the edgePortMapping can be used to optmize the binding in order
   * to minimize the effort for MUX hardware
   * @param epm
   */
  void setedgePortMapping(map<Edge*, pair<int, int> > epm){
    this->edgePortMapping = epm;
  }
  map<Edge*, pair<int, int> > getedgePortMapping(){
    return this->edgePortMapping;
  }

  /*!
   * Function to set the solver Timeout
   * @param seconds
   */
  void setSolverTimeout(double timeoutInSeconds) override;

protected:
  /*!
	 * each scheduler should overload this function for one schedule iteration
	 * M/S are already set automatically by RationalIISchedulerLayer::schedule
	 *
	 */
  void scheduleIteration() override;
  /*!
   * constructProblem Using the graph, resource model, an II and solver settings, the problem is constructed
   */
  virtual void constructProblem();
  /*!
   * setObjective currently asap
   */
  virtual void setObjective();
  /*!
   *
   */
  virtual void resetContainer();

private:
  /*!
   * set the resource constranints / often referred to as modulo reservation table (MRT)
   */
  void setResourceConstraints();
  /*!
   * set dependency contraints
   */
  void setGeneralConstraints();
  /*!
   * set modulo constraints
   */
  void setModuloConstraints();
  /*!
   * the ILP variables of the operations in the input graph
   */
  void fillTMaxtrix();
  /*!
   * the ILP variables for the data insertion intervals
   */
  void fillIIVector();
  /*!
   * fill interface to pass values to next step in the tool flow after solving
   */
  void fillSolutionStructure();
  /*!
   * this method is used to determine the distances in clock cycles
   * @param d
   * @param startIndex
   * @return
   */
  ScaLP::Term getSampleDistanceAsTerm(int d, int startIndex);
  int getSampleIndexFromDistance(int d, int startSample);
  /*!
   * EXPERIMETAL: DONT USE THIS
   */
  bool uniformSchedule;
  /*!
   * the considered time steps for solving the problem
   */
  unsigned int consideredTimeSteps;
  /*!
   * container for ILP variables
   */
  vector<vector<ScaLP::Variable> > t_matrix;
  /*!
   * container for ILP variables
   */
  vector<ScaLP::Variable> II_vector;
  /*!
   * container to find the correct ILP variables later on
   */
  map<const Vertex*,int> tIndices;
  /*!
   * the determined initIntervals of the rational II schedule after solving
   */
  //vector<int> latencySequence;
  /*!
   * buffer
   */
  double tpBuffer;
  /*!
   * @brief the edgePortMapping can be used to optmize the binding in order
   * to minimize the effort for MUX hardware
   */
  map<HatScheT::Edge*, pair<int,int> > edgePortMapping;
};
}
