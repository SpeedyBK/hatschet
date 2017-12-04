#pragma once

#include <HatScheT/Graph.h>
#include <map>

namespace HatScheT
{

  class Scheduler
  {
    public:

		Scheduler(Graph g);
		
		virtual void schedule() = 0;
		
		std::map<Vertex,int>& getStartTimes(){ return startTimes; }
		
		int getScheduleLenght();
		
		protected:
		
		std::map<Vertex,int> startTimes;
	};
}