#pragma once

#include <HatScheT/Graph.h>
#include <map>

namespace HatScheT
{

  class ModuloSchedulerBase
  {
    public:

			int getII() { return II; } ;
			
			int computeMinII();
			int computeMaxII();
			
		protected:
		
			int II;
	};
}