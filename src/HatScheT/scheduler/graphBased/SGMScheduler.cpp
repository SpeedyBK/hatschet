#include <HatScheT/scheduler/graphBased/SGMScheduler.h>
#include <HatScheT/utility/Utility.h>
#include <HatScheT/scheduler/ASAPScheduler.h>

namespace HatScheT
{

SGMScheduler::SGMScheduler(Graph &g, ResourceModel &resourceModel, std::list<string> solverWishlist) : MoovacScheduler(g, resourceModel, solverWishlist)
{
  this->minII = this->computeMinII(&g,&resourceModel);
  HatScheT::ASAPScheduler asap(g,resourceModel);
  this->maxII = Utility::calcMaxII(&asap);
  this->SLMax = 0;
}

void SGMScheduler::schedule()
{

}

}
