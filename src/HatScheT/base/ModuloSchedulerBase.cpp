#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/utility/Utility.h>

namespace HatScheT
{

  int ModuloSchedulerBase::computeMinII(Graph *g, ResourceModel *rm)
	{
    return Utility::calcMinII(rm,g);
  }
	
}
