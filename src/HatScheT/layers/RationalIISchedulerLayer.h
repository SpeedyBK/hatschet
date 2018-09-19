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
   * \brief getRationalII the rationalII is number of inserted samples over the used modulo class
   * \return
   */
  float getRationalII(){return this->rationalII;}
  /*!
   * \brief getInitIntervalls specific timeslots for initiation of samples into the schedule
   * \return
   */
  std::vector<int> getInitIntervalls(){return this->initInvervals;}
  /*!
   * \brief getModulo the modulo number determines the ratio in which the initiation of samples is repeated
   * \return
   */
  int getModulo(){return this->modulo;}
protected:
  /*!
   * \brief rationalII
   */
  float rationalII;
  /*!
   * \brief initInvervals
   */
  std::vector<int> initInvervals;
  /*!
   * \brief modulo
   */
  int modulo;

};

}
