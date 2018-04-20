/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Patrick Sittel (sittel@uni-kassel.de)
  All rights reserved.
*/
#pragma once

#include <HatScheT/base/SchedulerBase.h>

namespace HatScheT
{
/*!
 * \brief The ULScheduler class
 */
class ULScheduler : public SchedulerBase
{
public:
  /*!
   * \brief ASAPScheduler
   * \param g
   * \param resourceModel
   */
  ULScheduler(Graph& g,ResourceModel &resourceModel);
  /*!
   * \brief schedule
   */
  virtual void schedule();
  /*!
   * \brief getII
   * \return
   */
  virtual int getII() { return this->getScheduleLength();}

protected:
private:

};
}
