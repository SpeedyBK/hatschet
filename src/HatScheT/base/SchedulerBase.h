/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Martin Kumm, Patrick Sittel ({kumm, sittel}@uni-kassel.de)

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

#include <HatScheT/Graph.h>
#include <HatScheT/Vertex.h>
#include <HatScheT/Edge.h>
#include <map>
#include <HatScheT/ResourceModel.h>

namespace HatScheT
{
/*!
 * \brief ILPSchedulerBase is the abstract base class of all schedulers.
 *
 * Derived classes should implement schedule() which has to fill the startTimes data structure.
 */
class SchedulerBase
{
public:

  SchedulerBase(Graph& g,ResourceModel &resourceModel);

  virtual ~SchedulerBase();

  /*!
   * \brief schedule main method for all schedulers, not implemented in base class
   */
  virtual void schedule() = 0;
  /*!
   * \brief Returns the start times of all nodes
   * \return The start times of all nodes
   */
  std::map<Vertex*,int>& getSchedule();

  /*!
   * \brief Gets the start time of vertex v
   * \param v The vertex for which the start time is requested
   * \return The start time of v or -1 if vertex does not exist
   */
  int getStartTime(Vertex &v);

  /*!
   * \brief Returns the length of the schedule (i.e., the maximum start time)
   * \return The length of the schedule (i.e., the maximum start time)
   */
  int getScheduleLength();
  /*!
   * \brief setMaxLatencyConstraint manage the allowed maximum latency of the schedule
   * \param l
   */
  void setMaxLatencyConstraint(int l){this->maxLatencyConstrain =l;}
  int getMaxLatencyConstraint(){return this->maxLatencyConstrain;}
  /*!
   * \brief getBindings calculate a naive binding in base class
   * should be overloaded by scheduler that determine specific bindings
   * \return
   */
  virtual std::map<const Vertex*,int> getBindings();
  /*!
   * \brief getLifeTimes II = scheduleLength
   * \return
   */
  virtual std::map<Edge*,int> getLifeTimes();
  /*!
   * \brief printStartTimes
   */
  void printStartTimes();
  /*!
   * \brief Generates an HTML file which contains the schedule chart
	 * \param filename is the file name where to store the chart
   */
  void writeScheduleChart(string filename);
  /*!
   * \brief getII getter for the II. The II has to be set by each scheduler when schedule is done (also non modulo schedulers!)
   * \return initiation interval (II)
   */
  virtual int getII() { return this->II;}

  /*!
   *
   * @return the used graph description of the scheduling problem
   */
  Graph* getGraph(){return &this->g;}
  /*!
   *
   * @return the used resource Model
   */
  ResourceModel* getResourceModel(){return &this->resourceModel;}
protected:
  /*!
   * \brief resourceModel
   */
  ResourceModel &resourceModel;
  /*!
   * \brief maxLatencyConStraint default is -1 (unlimited)
   */
  int maxLatencyConstrain = -1;
  /*!
   * \brief Container for the start times
   */
  std::map<Vertex*,int> startTimes;
  /*!
   * \brief A reference to the data dependency graph
   */
  Graph& g;
  /*!
   * \brief II the initiation interval of a schedule
   */
  unsigned int II = 1;
};
}
