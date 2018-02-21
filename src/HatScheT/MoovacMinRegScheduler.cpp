#include <HatScheT/MoovacMinRegScheduler.h>

namespace HatScheT
{

MoovacMinRegScheduler::MoovacMinRegScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist, unsigned int minII, unsigned int maxII)
    : MoovacScheduler(g, resourceModel, solverWishlist, minII, maxII)
{
  this->minII = minII;
  this->maxII = maxII;
  this->SLMax = 0;
}

void MoovacMinRegScheduler::setGeneralConstraints()
{
  //5
  for(std::set<Edge*>::iterator it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it)
  {
    Edge* e = *it;
    Vertex* src = &(e->getVertexSrc());
    unsigned int srcTVecIndex = this->t_vectorIndices[src];
    Vertex* dst = &(e->getVertexDst());
    unsigned int dstTVecIndex = this->t_vectorIndices[dst];

    unsigned regVecIndex = this->reg_vectorIndices[e];

    this->solver->addConstraint(this->II*(e->getDelay()) - t_vector[srcTVecIndex] + t_vector[dstTVecIndex] - this->resourceModel.getVertexLatency(src) >= 0);
    this->solver->addConstraint((this->II)*(e->getDelay()) - t_vector[srcTVecIndex] + t_vector[dstTVecIndex]
                    - this->resourceModel.getVertexLatency(src) - this->regVector[regVecIndex] == 0);
  }
}

void MoovacMinRegScheduler::fillRegVector()
{
  for(std::set<Edge*>::iterator it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it)
  {
    Edge* e = *it;

    Vertex& srcV = e->getVertexSrc();
    Vertex& dstV = e->getVertexDst();

    //infinity is limited might cause overflow during solving process
    this->regVector.push_back(ScaLP::newIntegerVariable("n_" + std::to_string(srcV.getId()) + "_" + std::to_string(dstV.getId()) +"_e" + to_string(e->getID()),0,1000));
    this->reg_vectorIndices.insert(make_pair(e, this->regVector.size() - 1));
  }
}

void MoovacMinRegScheduler::setObjective()
{
    ScaLP::Term sum;

    for(ScaLP::Variable &r:regVector)
    {
      sum = sum + r;
    }

    this->solver->setObjective(ScaLP::minimize(sum));
}

}
