#include <HatScheT/scheduler/graphBased/SGMScheduler.h>
#include <HatScheT/utility/Utility.h>
#include <HatScheT/scheduler/ASAPScheduler.h>

namespace HatScheT
{

SGMScheduler::SGMScheduler(Graph &g, ResourceModel &resourceModel, std::list<string> solverWishlist, OccurrenceSetCombination *occSC) : MoovacScheduler(g, resourceModel, solverWishlist)
{
  this->minII = this->computeMinII(&g,&resourceModel);
  HatScheT::ASAPScheduler asap(g,resourceModel);
  this->maxII = Utility::calcMaxII(&asap);
  this->SLMax = 0;
  this->occSC = occSC;
}

void SGMScheduler::setGeneralConstraints()
{
  for(std::set<Edge*>::iterator it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it)
  {
    Edge* e = *it;
    Vertex* src = &(e->getVertexSrc());
    unsigned int srcTVecIndex = this->t_vectorIndices[src];
    Vertex* dst = &(e->getVertexDst());
    unsigned int dstTVecIndex = this->t_vectorIndices[dst];

    this->solver->addConstraint(this->II*(e->getDistance()) - t_vector[srcTVecIndex] + t_vector[dstTVecIndex] - this->resourceModel.getVertexLatency(src) >= 0);
  }

  //add subgraph connections
}

}
