#include <HatScheT/scheduler/ilpbased/RationalIISchedulerFimmel.h>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/utility/Utility.h>

namespace HatScheT
{
    RationalIISchedulerFimmel::RationalIISchedulerFimmel(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist)
    : MoovacScheduler(g, resourceModel, solverWishlist)
    {
      throw new Exception("rationalIIScheduler::rationalIIScheduler: This constructor is currently under construction and disabled!");
  /*this->minII = this->computeMinII(&g,&resourceModel);
  HatScheT::ASAPScheduler asap(g,resourceModel);
  this->maxII = Utility::calcMaxII(&asap);
  this->SLMax = 0;*/
}

}
