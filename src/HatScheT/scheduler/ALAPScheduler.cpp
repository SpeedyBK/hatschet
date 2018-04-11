#include <HatScheT/scheduler/ALAPScheduler.h>
#include <HatScheT/utility/Utility.h>
#include <stack>
#include <map>

namespace HatScheT
{

ALAPScheduler::ALAPScheduler(Graph &g, ResourceModel &resourceModel) : SchedulerBase(g, resourceModel)
{

}

void ALAPScheduler::schedule()
{
  std::stack<Vertex*> stack;
  std::map<Vertex*,int> ouput_counts;

  //initialize vertices
  for(auto it=this->g.verticesBegin(); it!=this->g.verticesEnd(); ++it){
    Vertex* v = *it;
    this->startTimes.emplace(v,1);

    int outputCount = this->g.getNoOfOutputs(v);
    ouput_counts.emplace(v,outputCount);

    // put nodes without inport to the stack, start ASAP schedule with them
    if(outputCount==0)
    {
      //check for limited resources with no inputs
      int time = 0;
      const Resource* r = this->resourceModel.getResource(v);
      while (Utility::resourceAvailable(this->startTimes,&this->resourceModel,r,v,time)  == false) {
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

    set<Vertex*> procVertices = this->g.getProceedingVertices(v);

    for(auto it=procVertices.begin(); it!=procVertices.end(); ++it){
      Vertex* procV = *it;

      //algorithmic delays are considered as inputs to the circuit
      Edge* e = &this->g.getEdge(procV,v);
      if(e->getDistance() > 0){
        this->startTimes[procV] = std::min(this->startTimes[procV], 0);
        ouput_counts[procV]--;
        if(ouput_counts[procV]==0){
          stack.push(procV);
        }
        continue;
      }

      //set start time
      int time = std::min(this->startTimes[v] - r->getLatency(), this->startTimes[procV]);
      const Resource* rSub = this->resourceModel.getResource(procV);

      while (Utility::resourceAvailable(this->startTimes,&this->resourceModel,rSub,v,time)  == false) {
        time--;
      }

      this->startTimes[procV] = time;

      ouput_counts[procV]--;
      // if there are no more inputs left, add n to the stack
      if(ouput_counts[procV]==0){
        stack.push(procV);
      }
    }
  }

  int offset=0;

  for(const auto &p:this->startTimes) offset = std::min(offset,p.second);
  for(auto &p:this->startTimes) p.second -=offset;
}

}
