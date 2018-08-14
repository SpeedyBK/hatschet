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

    int outputCount = Utility::getNoOfOutputsWithoutDistance(&this->g,v);
    ouput_counts.emplace(v,outputCount);

    // put nodes without outport to the stack, start ALAP schedule with them
    if(outputCount==0)
    {
      //check for limited resources with no inputs
      int time = 0;
      const Resource* r = this->resourceModel.getResource(v);
      while (Utility::resourceAvailable(this->startTimes,&this->resourceModel,r,v,time)  == false) {
        time--;
      }

      this->startTimes[v] = time;
      stack.push(v);
    }
  }

  //schedule
  while(stack.empty()==false){
    Vertex* v = stack.top();
    stack.pop();

    set<Vertex*> procVertices = this->g.getPreceedingVertices(v);

    for(auto it=procVertices.begin(); it!=procVertices.end(); ++it){
      Vertex* procV = *it;
      const Resource* r = this->resourceModel.getResource(procV);
      //algorithmic delays are considered as inputs to the circuit
      Edge* e = &this->g.getEdge(procV,v);
      if(e->getDistance() > 0){
        continue;
      }

      //set start time
      int time = std::min(this->startTimes[v] - r->getLatency() - e->getDelay(), this->startTimes[procV]);

      while (Utility::resourceAvailable(this->startTimes,&this->resourceModel,r,v,time)  == false) {
        time--;
      }

      this->startTimes[procV] = time;
      ouput_counts[procV]--;
      // if there are no more outputs left, add to the stack
      if(ouput_counts[procV]==0){
        stack.push(procV);
      }
    }
  }

  int offset=0;

  for(const auto &p:this->startTimes) offset = std::min(offset,p.second);
  for(auto &p:this->startTimes) p.second -=offset;

  this->II = this->getScheduleLength()+1;
}

}
