#include <HatScheT/scheduler/ASAPScheduler.h>
#include <stack>
#include <map>

namespace HatScheT
{

ASAPScheduler::ASAPScheduler(Graph &g, ResourceModel &resourceModel) : SchedulerBase(g, resourceModel)
{

}

void ASAPScheduler::schedule()
{
  std::stack<Vertex*> stack;
  std::map<Vertex*,int> input_counts;

  //initialize vertices
  for(auto it=this->g.verticesBegin(); it!=this->g.verticesEnd(); ++it){
    Vertex* v = *it;
    this->startTimes.emplace(v,-1);

    int inputCount = this->g.getNoOfInputs(v);
    input_counts.emplace(v,inputCount);

    // put nodes without inport to the stack, start ASAP schedule with them
    if(inputCount==0)
    {
      //check for limited resources with no inputs
      int time = 0;
      const Resource* r = this->resourceModel.getResource(v);
      while (this->resourceAvailable(r,v,time) == false) {
        time++;
      }

      this->startTimes[v] = time;
      stack.push(v);
    }
  }

  //schedule
  while(stack.empty()==false){
    Vertex* v = stack.top();
    const Resource* r = this->resourceModel.getResource(v);
    stack.pop();

    set<Vertex*> subVertices = this->g.getSubsequentVertices(v);

    for(auto it=subVertices.begin(); it!=subVertices.end(); ++it){
      Vertex* subV = *it;

      Edge* e = &this->g.getEdge(v,subV);
      if(e->getDistance() > 0){
        throw new Exception("ASAPScheduler::schedule: register are not supported yet!");
      }

      //set start time
      int time = std::max(this->startTimes[v] + r->getLatency(), this->startTimes[subV]);
      const Resource* rSub = this->resourceModel.getResource(subV);

      while (this->resourceAvailable(rSub,subV,time) == false) {
        time++;
      }

      this->startTimes[subV] = time;

      input_counts[subV]--;
      // if there are no more inputs left, add n to the stack
      if(input_counts[subV]==0){
        stack.push(subV);
      }
    }
  }
}

bool ASAPScheduler::resourceAvailable(const Resource *r, Vertex* checkV, int timeStep)
{
  //unlimited
  if(r->getLimit()==-1) return true;

  int instancesUsed = 0;

  for(auto it=this->startTimes.begin(); it!=this->startTimes.end(); ++it){
    if(it->second == timeStep){
      Vertex* v = it->first;
      if(checkV != v && this->resourceModel.getResource(v) == r) instancesUsed++;
    }
  }

  if(instancesUsed < r->getLimit()) return true;
  return false;
}

}
