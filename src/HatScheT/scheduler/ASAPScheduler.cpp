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
  /*for(auto it=this->resourceModel.resourcesBegin(); it!=this->resourceModel.resourcesEnd(); it++){
    Resource* r = *it;
  }*/

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

    set<Vertex*> subVertices = this->g.getSubsequentVertices(v);

    for(auto it=subVertices.begin(); it!=subVertices.end(); ++it){
      Vertex* subV = *it;

      Edge* e = &this->g.getEdge(v,subV);
      if(e->getDistance() > 0){
        throw new Exception("ASAPScheduler::schedule: register are not supported yet!");
      }

      for(auto it=this->startTimes.begin();it!=this->startTimes.end();++it){
        if(subV==it->first && ((this->startTimes[v]+r->getLatency())>it->second))
          it->second = this->startTimes[v] + r->getLatency();
      }

      input_counts[subV]--;
      // if there are no more inputs left, add n to the stack
      if(input_counts[subV]==0){
        stack.push(subV);
      }
    }
  }
}

bool ASAPScheduler::resourceAvailable(Resource *r, int timeStep)
{
  //unlimited
  if(r->getLimit()==-1) return true;
}

}
