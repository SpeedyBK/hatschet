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

std::map<const Vertex *, int> SchedulerBase::getBindings()
{
  std::map<const Vertex*,int> bindings;
  std::map<const Resource*, int> naiveBindingCounter;

  for(auto it:this->startTimes){
    Vertex* v = it.first;
    const Resource* r = this->resourceModel.getResource(v);
    if(r->getLimit() == -1) throw new Exception("SchedulerBase.getBindings: resource not unlimited " + r->getName() + "! SchedulerBase does not support this behavior! Use a ResourceConstraint Scheduler!");

    if(naiveBindingCounter.find(r) == naiveBindingCounter.end()) naiveBindingCounter.insert(make_pair(r,0));
    //make naive binding
    bindings.insert(make_pair(v,naiveBindingCounter[r]));
    //increment naive binding counter for next possible binding
    naiveBindingCounter[r]++;
  }

  return bindings;
}

}
