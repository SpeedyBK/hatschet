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
protected:
  /*!
   * \brief computeMinII
   * \param g
   * \param rm
   */
  int computeMinII(Graph* g, ResourceModel* rm);
  /*!
   * \brief minII lower bound for II
   */
  unsigned int minII;
  /*!
   * \brief maxII upper bound for II
   */
  unsigned int maxII;


};

}
