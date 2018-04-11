#include "HatScheT/utility/Utility.h"


namespace HatScheT {

bool Utility::examplUtilityFunction(ResourceModel *rm, Graph *g)
{
  return true;
}

int Utility::getNoOfInputs(Graph *g, const Vertex *v)
{
  int no=0;

  for(auto it=g->edgesBegin(); it!=g->edgesEnd(); ++it)
  {
    Edge* e = *it;
    Vertex* dstV = &e->getVertexDst();

    if(dstV==v) no++;
  }

  return no;
}

int Utility::getNoOfOutputs(Graph *g, const Vertex *v)
{
  int outputs = 0;

  for(auto it=g->edgesBegin(); it!=g->edgesEnd();++it)
  {
    Edge* e = *it;
    Vertex* vSrc = &e->getVertexSrc();

    if(vSrc==v) outputs++;
  }

  return outputs;
}

int Utility::calcMinII(ResourceModel *rm, Graph *g)
{
  int resMII = Utility::calcResMII(rm,g);
  int recMII = Utility::calcRecMII(g);

  if(resMII>recMII) return resMII;

  return recMII;
}

int Utility::calcResMII(ResourceModel *rm, Graph *g)
{
  int resMII = 0;

  for(auto it=rm->resourcesBegin(); it!=rm->resourcesEnd(); ++it){
    Resource* r = *it;
    //skip unlimited resources
    if(r->getLimit()==-1)continue;
    int opsUsingR = rm->getNoOfVerticesRegisteredToResource(r);
    int avSlots = r->getLimit();

    if(avSlots<=0) throw new Exception("Utility.calcResMII: avSlots <= 0 : " + to_string(avSlots));
    int tempMax = opsUsingR/avSlots + (opsUsingR % avSlots != 0);

    if(tempMax>resMII) resMII=tempMax;
  }

  return resMII;
}

int Utility::calcMaxII(SchedulerBase *sb)
{
  sb->schedule();;
  return sb->getScheduleLength();
}

int Utility::calcRecMII(Graph *g)
{
  int recMII=0;

  for(auto it=g->edgesBegin(); it!=g->edgesEnd(); it++){
    Edge* e = *it;
    if(e->getDistance() > recMII) recMII = e->getDistance();
  }

  return recMII;
}

int Utility::sumOfStarttimes(std::map<Vertex *, int> &startTimes)
{
  int sum = 0;

  for(auto it=startTimes.begin();it!=startTimes.end();++it){
    sum+=it->second;
  }

  return sum;
}

bool Utility::resourceAvailable(std::map<Vertex *, int> &startTimes, ResourceModel* rm, const Resource *r, Vertex *checkV, int timeStep)
{
  //unlimited
  if(r->getLimit()==-1) return true;

  int instancesUsed = 0;

  for(auto it=startTimes.begin(); it!=startTimes.end(); ++it){
    if(it->second == timeStep){
      Vertex* v = it->first;
      if(checkV != v && rm->getResource(v) == r) instancesUsed++;
    }
  }

  if(instancesUsed < r->getLimit()) return true;
  return false;
}

}
