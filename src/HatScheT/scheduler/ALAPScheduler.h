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
 * \brief The ALAPScheduler class
 */
class ALAPScheduler : public SchedulerBase
{
public:
  /*!
   * \brief ALAPScheduler
   * \param g
   * \param resourceModel
   */
  ALAPScheduler(Graph& g,ResourceModel &resourceModel);
  /*!
   * \brief schedule generate an ALAP (HC) schedule
   */
  virtual void schedule();

protected:
private:
  /*!
   * \brief resourceAvailable
   * \param r the resource that is looked for
   * \param checkV avoid self counting
   * \param timeStep the time step that is checked
   * \return
   */
  bool resourceAvailable(const Resource *r, Vertex *checkV, int timeStep);

};
}
