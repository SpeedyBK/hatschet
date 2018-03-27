#include <HatScheT/ModuloSchedulerBase.h>
#include <HatScheT/utility/Utility.h>

namespace HatScheT
{

  int ModuloSchedulerBase::computeMinII(Graph *g, ResourceModel *rm)
	{
    return Utility::calcMinII(rm,g);
  }
	
  int ModuloSchedulerBase::computeMaxSL()
	{
    //ToDo: implement
    return -1;
  }
	
}
