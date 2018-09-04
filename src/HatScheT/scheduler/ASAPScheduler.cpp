/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)

    Copyright (C) 2018

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

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
    throw HatScheT::Exception("ASAPScheduler.schedule: Error stack not initialized! No schedule found!");
  }

  //schedule
  while(stack.empty()==false){
    Vertex* v = stack.top();
    const Resource* r = this->resourceModel.getResource(v);
    stack.pop();

    set<Vertex*> subVertices = this->g.getSuccessors(v);

    for(auto it=subVertices.begin(); it!=subVertices.end(); ++it){
      Vertex* subV = *it;

      //algorithmic delays are considered as inputs to the circuit
      Edge* e = &this->g.getEdge(v,subV);
      if(e->getDistance() > 0){
        continue;
      }

      //set start time
      //there might be a delay that is not a distance
      int time = std::max(this->startTimes[v] + r->getLatency() + e->getDelay(), this->startTimes[subV]);
      const Resource* rSub = this->resourceModel.getResource(subV);

      while (Utility::resourceAvailable(this->startTimes,&this->resourceModel,rSub,subV,time) == false) {
        ++time;
      }

      this->startTimes[subV] = time;

      input_counts[subV]--;
      // if there are no more inputs left, add n to the stack
      if(input_counts[subV]==0){
        stack.push(subV);
      }
    }
  }

  this->II = this->getScheduleLength();
}

}
