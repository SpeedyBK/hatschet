#pragma once

#include <HatScheT/SchedulerBase.h>

namespace HatScheT
{

  class ASAPScheduler : SchedulerBase
  {
    public:
    
	    using SchedulerBase::SchedulerBase;
	
			virtual void schedule();
	};
}