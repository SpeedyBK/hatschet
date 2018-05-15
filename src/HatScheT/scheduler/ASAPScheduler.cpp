#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/utility/Utility.h>
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

    int inputCount = Utility::getNoOfInputsWithoutRegs(&this->g,v);
    input_counts.emplace(v,inputCount);

    // put nodes without inport to the stack, start ASAP schedule with them
    if(inputCount==0)
    {
      //check for limited resources with no inputs
      int time = 0;
      const Resource* r = this->resourceModel.getResource(v);
      while (Utility::resourceAvailable(this->startTimes,&this->resourceModel,r,v,time) == false) {
        time++;
      }

      this->startTimes[v] = time;
      stack.push(v);
    }
  }

  if(stack.size()==0){
    for(auto it=this->g.verticesBegin(); it!=this->g.verticesEnd(); ++it){
      Vertex* v = *it;

      // put nodes with only register input edges to init stack
      if(Utility::allInputsAreRegisters(&this->g,v)==true)
      {
        //check for limited resources with no inputs
        int time = 0;
        const Resource* r = this->resourceModel.getResource(v);
        while (Utility::resourceAvailable(this->startTimes,&this->resourceModel,r,v,time) == false) {
          time++;
        }

        this->startTimes[v] = time;
        stack.push(v);cout << "pushed " << v->getName() << endl;
      }
    }
  }

  if(stack.size()==0){
    cout << "ASAPScheduler.schedule: Error stack not initialized! No schedule found!" << endl;
    throw new Exception("ASAPScheduler.schedule: Error stack not initialized! No schedule found!");
  }

  //schedule
  while(stack.empty()==false){
    Vertex* v = stack.top();
    const Resource* r = this->resourceModel.getResource(v);
    stack.pop();

    set<Vertex*> subVertices = this->g.getSubsequentVertices(v);

    for(auto it=subVertices.begin(); it!=subVertices.end(); ++it){
      Vertex* subV = *it;

      //algorithmic delays are considered as inputs to the circuit
      Edge* e = &this->g.getEdge(v,subV);
      if(e->getDistance() > 0){
        this->startTimes[subV] = std::max(this->startTimes[subV], 0);
        input_counts[subV]--;
        if(input_counts[subV]==0){
        }
        continue;
      }

      //set start time
      //there might be a delay that is not a distance
      int time = std::max(this->startTimes[v] + r->getLatency() + e->getDelay(), this->startTimes[subV]);
      const Resource* rSub = this->resourceModel.getResource(subV);

      while (Utility::resourceAvailable(this->startTimes,&this->resourceModel,rSub,v,time) == false) {
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

  this->II = this->getScheduleLength()+1;
}

}
