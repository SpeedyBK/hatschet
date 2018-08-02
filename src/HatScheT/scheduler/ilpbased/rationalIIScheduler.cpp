#include <HatScheT/scheduler/ilpbased/rationalIIScheduler.h>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/utility/Utility.h>

namespace HatScheT
{
    rationalIIScheduler::rationalIIScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist)
    : MoovacScheduler(g, resourceModel, solverWishlist)
{
  this->minII = this->computeMinII(&g,&resourceModel);
  HatScheT::ASAPScheduler asap(g,resourceModel);
  this->maxII = Utility::calcMaxII(&asap);
  this->SLMax = 0;
}

}
