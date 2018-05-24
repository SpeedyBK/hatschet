/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Martin Kumm, Patrick Sittel (kumm, sittel@uni-kassel.de)
  All rights reserved.
*/
#pragma once

#include <vector>

namespace HatScheT
{
/*!
 * \brief The RationalIISchedulerBase class providing information that is needed implement rational II modulo schedules
 */
class RationalIISchedulerBase
{
public:
  RationalIISchedulerBase();
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
