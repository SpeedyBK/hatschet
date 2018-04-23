#include <HatScheT/scheduler/ULScheduler.h>
#include <stack>
#include <map>

namespace HatScheT
{

ULScheduler::ULScheduler(Graph &g, ResourceModel &resourceModel) : SchedulerBase(g, resourceModel)
{
  this->sort_by=MOBILITY;
}

void ULScheduler::schedule()
{
  ASAPScheduler s= ASAPScheduler(this->g,this->resourceModel);
  s.schedule();
  ALAPScheduler l = ALAPScheduler(this->g,this->resourceModel);
  l.schedule();

  auto asap = s.getStartTimes();
  auto alap = l.getStartTimes();
  std::map<Vertex*,int> *sort_criterion=nullptr;

}

std::map<Vertex*,int> *ULScheduler::mobility(std::map<Vertex*,int> *asap, std::map<Vertex*,int> *alap)
{
  // mobility = alap - asap
  return ULScheduler::zipWith(std::minus<int>(),*alap,*asap);
}

std::map<Vertex*,int> *ULScheduler::zipWith(std::function<int(int,int)> f, std::map<Vertex*,int> m1, std::map<Vertex*,int> m2)
{
  std::map<Vertex*,int> *r = new std::map<Vertex*,int>;
  auto i1 = m1.begin();
  auto i2 = m2.begin();
  while(i1!=m1.end()) // both maps have the same keys, no check for m2 needed.
  {
    Vertex *key = (*i1).first;
    int v1 = (*i1).second;
    int v2 = (*i2).second;
    r->insert({key,f(v1,v2)});
    i1++;
    i2++;
  }

  return r;
}

}
