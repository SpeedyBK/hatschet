#include <HatScheT/SchedulerBase.h>

namespace HatScheT
{
  SchedulerBase::SchedulerBase(Graph& g, ResourceModel &resourceModel) : g(g), resourceModel(resourceModel){
    this->maxLatencyConStraint = -1;
  }

  int SchedulerBase::getScheduleLength()
  {
    int maxTime=0;
    for(std::pair<Vertex*,int> vtPair : startTimes)
    {
      if(vtPair.second > maxTime) maxTime = vtPair.second;
    }
    return maxTime;
  }

  int SchedulerBase::getStartTime(Vertex &v)
  {
    std::map<Vertex*,int>::iterator it = startTimes.find(&v);
    if(it != startTimes.end())
      return it->second;
    else
      return -1;
  }

}
