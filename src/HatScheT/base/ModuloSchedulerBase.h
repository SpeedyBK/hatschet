/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Martin Kumm, Patrick Sittel (kumm, sittel@uni-kassel.de)
  All rights reserved.
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
