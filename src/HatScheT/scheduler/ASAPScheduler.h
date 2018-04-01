#pragma once

#include <HatScheT/base/SchedulerBase.h>

namespace HatScheT
{



class ASAPScheduler : public SchedulerBase
{
public:
  ASAPScheduler(Graph& g,ResourceModel &resourceModel);

protected:
private:

};
}
