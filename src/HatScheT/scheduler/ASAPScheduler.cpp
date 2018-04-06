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

  //init resources
  for(auto it=this->resourceModel.resourcesBegin(); it!=this->resourceModel.resourcesEnd(); it++){
    Resource* r = *it;
  }

  //initialize vertices
  for(auto it=this->g.verticesBegin(); it!=this->g.verticesEnd(); ++it){
    Vertex* v = *it;
    this->startTimes.emplace(v,0);

    int inputCount = this->g.getNoOfInputs(v);
    input_counts.emplace(v,inputCount);

    // put nodes without inport to the stack, start ASAP schedule with them
    if(inputCount==0)
    {
      //ToDo : Consider Resource limited ops without inputs
      stack.push(v);
    }
  }

  //schedule
  while(stack.empty()==false){
    Vertex* v = stack.top();
    const Resource* r = this->resourceModel.getResource(v);
    stack.pop();

    //check for hardware constraints, hardware available
    if(r->getLimit() != -1){

    }
  }
}

bool ASAPScheduler::resourceAvailable(Resource *r, int timeStep)
{
  //unlimited
  if(r->getLimit()==-1) return true;
}

}
