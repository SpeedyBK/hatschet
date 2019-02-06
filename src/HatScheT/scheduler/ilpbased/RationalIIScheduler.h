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
 * experimental: This scheduler determines a modulo schedule with uneven/rational initiation intervals
 * THIS CLASSES IS UNDER DEVELOPMENT
 */
class RationalIIScheduler : public SchedulerBase, public ILPSchedulerBase, public RationalIISchedulerLayer, public IterativeSchedulerBase
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
   * the main function of the scheduler. The rational II scheduler tries to identify high throughput schedules on
   * the theoretical min II boundary. For this the variables s / m are used
   */
  virtual void schedule();
    /*!
 * \brief getLifeTimes using the determined rational II
 * lifetimes in rational II schedules are determined using the initiation intervall vector
 * this is crucial because samples are inserted in not evenly spaced intervalls
 * remark: overloaded function from the scheduler base class
 * \return
 */
  virtual std::map<Edge*,vector<int> > getRatIILifeTimes();
  /*!
   * dont use this function for rational II modulo schedules
   * this function will throw an exception
   * use getRatIILifeTimes()
   * @return
   */
  virtual std::map<Edge*,int> getLifeTimes();
  /*!
   * dont use this function fo rational II modulo schedules
   * @return
   */
  virtual std::map<const Vertex*,int> getBindings();
  /*!
   * generate a binding using the determined rational II schedule
   * TODO figure out the best binding method (ILP?)
   * @return
   */
   virtual vector<std::map<const Vertex*,int> > getRationalIIBindings();
  /*!
   *
   * @return the schedule length / sample latency of the determined rational II modulo schedule
   */
  virtual int getScheduleLength();
  /*!
   * the number of clock cycles after which the schedule repeats itself
   * @param m
   */
  void setModulo(int m) {
    this->modulo=m;
  }
  int getModulo(){
    return this->modulo;
  }
  /*!
   * the number of samples that are inserted every m clock cycles
   * @param s
   */
  void setSamples(int s){
    this->samples=s;
  }
  int getSamples(){
    return this->samples;
  }
  /*!
   * print the rational II modulo schedule
   */
  void printScheduleToConsole();
  /*!
   * @brief print the MRTs of all resources after rational II scheduling and binding
   */
  void printBindingToConsole();
  /*!
   * print the uneven spaced initiation times of data samples
   * those repeat every m cycles
   * @return
   */
  vector<std::map<Vertex*,int> >& getStartTimeVector(){return this->startTimesVector;}

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
private:
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
   * this function sets the s and m values in a way that not needed values are skipped
   * and the rational II becomes as small as possible
   */
  void autoSetMAndS();
  /*!
   * dito
   */
  void autoSetNextMAndS();
  /*!
   * this method is used to determine the distances in clock cycles
   * @param d
   * @param startIndex
   * @return
   */
  ScaLP::Term getSampleDistance(int d, int startIndex);
  int getDeterminedSampleDistance(int d, int startIndex);
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
   * the final rational II schedule
   */
  vector<std::map<Vertex*,int> > startTimesVector;
  /*!
   * the determined initIntervals of the rational II schedule after solving
   */
  //vector<int> initIntervals;
  /*!
   * the minimum interger II that is possible
   */
  int integerMinII;
  /*!
   * buffer
   */
  double tpBuffer;
  /*!
   * flag
   */
  bool minRatIIFound;
  /*!
   * @brief the edgePortMapping can be used to optmize the binding in order
   * to minimize the effort for MUX hardware
   */
  map<HatScheT::Edge*, pair<int,int> > edgePortMapping;
};
}
