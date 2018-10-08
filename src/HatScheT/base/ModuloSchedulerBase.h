/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Martin Kumm, Patrick Sittel ({kumm, sittel}@uni-kassel.de)
    Author: Julian Oppermann (oppermann@esa.tu-darmstadt.de)

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
#include <HatScheT/ResourceModel.h>
#include <HatScheT/TargetModel.h>
#include <map>

namespace HatScheT
{
/*!
 * \brief ModuloSchedulerBase is the base class of all schedulers that solve the modulo scheduling problem.
 *
 * It contains helper functions to determine bounds on the II
 */
class ModuloSchedulerBase
{
public:
   ModuloSchedulerBase();
  /*!
   * \brief getMinII
   * \return
   */
  int getMinII(){return this->minII;}
  /*!
   * \brief getMaxII
   * \return
   */
  int getMaxII(){return this->maxII;}
  /*!
   * get the rational minimum recurrence II contraint
   * this value is stored after the computeMinII method is called by the respective scheduler
   * @return
   */
  double getRecMinII(){return this->recMinII;}
  /*!
   * get the rational minimum recurrence II contraint
   * this value is stored after the computeMinII method is called by the respective scheduler
   * @return
   */
  double getResMinII(){return this->resMinII;}
protected:
  /*!
   * \brief minII lower bound for II
   */
  double minII=-1.0f;
  /*!
   * \brief maxII upper bound for II
   */
  int maxII=-1;
  /*!
   * the rational minimum recurrence II contraint
   * this value is stored after the computeMinII method is called by the respective scheduler
   */
  double recMinII=-1.0f;
  /*!
   * the rational minimum resource II contraint
   * this value is stored after the computeMinII method is called by the respective scheduler
   */
  double resMinII=-1.0f;
  /*!
   * \brief computeMinII / computeMaxII with or without hardware target
   * \param g
   * \param rm
   */
  void computeMinII(Graph* g, ResourceModel* rm, Target* t = nullptr);
  void computeMaxII(Graph* g, ResourceModel* rm);
};

}
