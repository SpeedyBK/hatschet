#pragma once

#include <HatScheT/SchedulerBase.h>

namespace HatScheT
{

/*!
 * \brief Implementation of a simple ASAP scheduler
 */
class ASAPScheduler : SchedulerBase
{
public:

  using SchedulerBase::SchedulerBase;

  virtual void schedule();
};
}
