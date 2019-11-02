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
#include "HatScheT/base/ModuloSchedulerBase.h"

namespace HatScheT
{
/*!
 * \brief The RationalIISchedulerLayer class providing information that is needed implement rational II modulo schedules
 */
class RationalIISchedulerLayer : public ModuloSchedulerBase
{
public:
  RationalIISchedulerLayer();
  /*!
   * \brief getInitIntervalls specific timeslots for initiation of samples into the schedule
   * \return
   */
  std::vector<int> getLatencySequence(){return this->latencySequence;}
  /*!
   * \brief getModulo the modulo number determines the ratio in which the initiation of samples is repeated
   * \return
   */
  int getModulo(){return this->modulo;}
  /*!
   * \brief getSamples get the number of samples that can inserted every 'this->modulo' clock cycles
   * @return
   */
  int getSamples(){return this->samples;}
  /*!
   * base function for rational II schedule bindings
   * @return
   */
  virtual vector<std::map<const Vertex*,int> > getRationalIIBindings() = 0;
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


};

}
