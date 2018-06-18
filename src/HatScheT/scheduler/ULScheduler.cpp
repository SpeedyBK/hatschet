#include <HatScheT/scheduler/ULScheduler.h>
#include <HatScheT/utility/Utility.h>
#include <stack>
#include <map>

namespace HatScheT
{

ULScheduler::ULScheduler(Graph &g, ResourceModel &resourceModel) : SchedulerBase(g, resourceModel)
{
  this->sort_by=MOBILITY;
}

bool ULScheduler::inputs_not_in(Vertex *v, list<Vertex *> vList)
{
  set<Vertex*> inputs = this->g.getPreceedingVertices(v);

  for(auto it:inputs){
    //register inputs are considered to be input for the graph
    Edge& e = this->g.getEdge(it,v);
    if(e.getDistance()>0) continue;
    for(auto it2:vList){
      if(it==it2) return false;
    }
  }
  return true;
}

bool ULScheduler::vertexRdyForScheduling(Vertex *v, int timeSlot)
{
  if(Utility::isInput(&this->g,v)==true) return true;
  set<Vertex*> inputs = this->g.getPreceedingVertices(v);

  for(auto it:inputs){
    //skip inputs over edges with distance as they represent next samples
    Edge& e= this->g.getEdge(it,v);
    if(e.getDistance()>0) continue;
    //input not scheduled yet
    if(this->startTimes[it]==-1) return false;

    //this input demands later execution time
    if(this->startTimes[it]+this->resourceModel.getVertexLatency(it)+e.getDelay()>timeSlot){
      return false;
    }
  }
  return true;
}

void ULScheduler::schedule()
{
  bool sched_operation;
  int c=0; //the number of clock cycles
  ASAPScheduler s= ASAPScheduler(this->g,this->resourceModel);
  s.schedule();
  ALAPScheduler l = ALAPScheduler(this->g,this->resourceModel);
  l.schedule();

  auto asap = s.getSchedule();
  auto alap = l.getSchedule();
  std::map<Vertex*,int> *sort_criterion=nullptr;

  // create the sort_criterion for the vertices.
  switch(this->sort_by){
  //other cases not implemented yet
  case MOBILITY:
        sort_criterion = this->mobility(&asap,&alap);
        break;
  }

  std::list<Vertex*> nodes;
  std::list<Vertex*> scheduled;

  //init the vertices
  for(auto it=this->g.verticesBegin(); it!=this->g.verticesEnd(); ++it){
    Vertex* v = *it;
    this->startTimes.emplace(v,-1);
    nodes.push_front(v);
  }

  if(nodes.empty()==true){
    cout << "ULScheduler.ULScheduler: Error stack not initialized! No schedule found!" << endl;
    throw new Exception("ULScheduler.ULScheduler: Error stack not initialized! No schedule found!");
  }

  while(!nodes.empty()){ // nodes left?
    // sort-object for the set to sort while inserting.
    struct sort_struct{
      std::map<Vertex*,int> *criterion;
      bool operator()(Vertex *a,Vertex *b){
        return criterion->at(a) <= criterion->at(b);
      }
    } sort_obj;
    sort_obj.criterion=sort_criterion;

    // nodes that are ready for scheduling.
    std::set<Vertex*,sort_struct> ready(sort_obj);

    for(Vertex *n:nodes){
      if(this->inputs_not_in(n,nodes)==true){
        ready.insert(n);
      }
    }
    // is anything sheduled in this iteration?
    // nothing jet.
    sched_operation=false;
    //schedule nodes if possible
    for(Vertex* node:ready){
      //check whether vertex is rdy for scheduling
      if(this->vertexRdyForScheduling(node,c)==true){
        //check whether hardware for node is available in this timestep
        const Resource* r = this->resourceModel.getResource(node);
        if(Utility::resourceAvailable(this->startTimes,&this->resourceModel,r,node,c) == true){
          this->startTimes[node]=c;
          sched_operation=true;
          scheduled.push_front(node);
        }
      }
    }
    // removes the scheduled nodes from the leftover.
    for(Vertex *node:scheduled){
      nodes.remove(node);
    }

    if(sched_operation==false){
      scheduled.clear();
      c++;
    }
  }

  //set II
  this->II = this->getScheduleLength();
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
