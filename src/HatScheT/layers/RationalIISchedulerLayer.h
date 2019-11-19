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

#include <vector>
#include "HatScheT/base/SchedulerBase.h"
#include "HatScheT/base/ModuloSchedulerBase.h"

namespace HatScheT
{
/*!
 * \brief The RationalIISchedulerLayer class providing information that is needed implement rational II modulo schedules
 */
class RationalIISchedulerLayer : public SchedulerBase, public ModuloSchedulerBase
{
public:
  RationalIISchedulerLayer(Graph &g, ResourceModel &resourceModel);
  /*!
   * @brief
   * @return
   */
  virtual int getLatency() final;
  /*!
   * \brief getInitIntervalls specific timeslots for initiation of samples into the schedule
   * \return
   */
  std::vector<int> getLatencySequence(){return this->latencySequence;}
  /*!
   * \brief getModulo the modulo number determines the ratio in which the initiation of samples is repeated
   * \return
   */
  int getModulo() const {return this->modulo;}
  /*!
   * \brief getSamples get the number of samples that can inserted every 'this->modulo' clock cycles
   * @return
   */
  int getSamples() const {return this->samples;}
  /*!
   * \brief setModulo manually set modulo
   * \return
   */
  void setModulo(int m){this->modulo = m;}
  /*!
   * \brief setSamples manually set samples
   * @return
   */
  void setSamples(int s){this->samples = s;}
  /*!
   * base function for rational II schedule bindings
   * @return
   */
  virtual vector<std::map<const Vertex*,int> > getRationalIIBindings() = 0;
  /*!
   * print the uneven spaced initiation times of data samples
   * those repeat every m cycles
   * @return
   */
  vector<std::map<Vertex*,int> >& getStartTimeVector(){return this->startTimesVector;}
  /*!
   * this algorithm creates a sorted queue with M/S pairs in the interval [minII_Q, minII_N)
   * list is sorted by the values of M/S
   * first list element is always mMinII/sMinII which corresponds to II = minII_Q
   * the list contains only non-reducable fractions! So if e.g. M/S = 3/2 is in the list, 6/4, 9/6, ... will NOT be!
   * @param sMinII samples for minII_Q
   * @param mMinII modulo for minII_Q
   * @param integerII minimum integer II (minII_N)
   * @param sMax maximum number of samples => all M/S pairs have S leq sMax -> -1: sMax = sMinII
   * @param maxListSize only return the best maxListSize M/S pairs -> -1: return all found M/S pairs
   * @return first pair element: M, second pair element: S
   */
  static std::list<pair<int, int>> getRationalIIQueue(int sMinII, int mMinII, int integerII, int sMax=-1, int maxListSize=-1);
protected:
  /*!
   * the bindings of rational II schedule
   */
  vector<std::map<const Vertex *, int> > ratIIbindings;
  /*!
   * \brief initIntervals
   */
  std::vector<int> latencySequence;
  /*!
   * \brief modulo
   */
  int modulo;
  /*!
   * \brief samples
   */
  int samples;
  /*!
   * the final rational II schedule
   */
  vector<std::map<Vertex*,int> > startTimesVector;


};

}
