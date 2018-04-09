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
  bool resourceAvailable(Resource* r, int timeStep);

};
}
