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
 * \brief The ASAPScheduler class
 */
class ASAPScheduler : public SchedulerBase
{
public:
  /*!
   * \brief ASAPScheduler
   * \param g
   * \param resourceModel
   */
  ASAPScheduler(Graph& g,ResourceModel &resourceModel);
  /*!
   * \brief schedule generate an ASAP (HC) schedule
   */
  virtual void schedule();

protected:
private:

};
}
