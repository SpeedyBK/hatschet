//
// Created by Patrick Sittel on 10.09.18.
//

#include "ASAPILPScheduler.h"

namespace HatScheT
{
ASAPILPScheduler::ASAPILPScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string> solverWishlist)
        : SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist) {

}

void ASAPILPScheduler::setObjective() {
  //supersink latency objective
  ScaLP::Variable supersink = ScaLP::newIntegerVariable("supersink",0,ScaLP::INF());
  for(ScaLP::Variable &v:this->ti){
    this->solver->addConstraint(supersink - v >= 0);
  }
  this->solver->setObjective(ScaLP::minimize(supersink));
}

void ASAPILPScheduler::setGeneralConstraints() {
  //5
  for(std::set<Edge*>::iterator it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it) {
    Edge* e = *it;
    Vertex* src = &(e->getVertexSrc());
    unsigned int srcTVecIndex = this->tIndices[src];
    Vertex* dst = &(e->getVertexDst());
    unsigned int dstTVecIndex = this->tIndices[dst];

    this->solver->addConstraint(ti[srcTVecIndex] - ti[dstTVecIndex] + this->resourceModel.getVertexLatency(src) + e->getDelay() - this->II*(e->getDistance()) <= 0);
  }
}

void ASAPILPScheduler::setVectorVariables() {
  for(std::set<Vertex*>::iterator it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it) {
    Vertex* v = *it;
    int id = v->getId();

    //17
    this->ti.push_back(ScaLP::newIntegerVariable("t_" + std::to_string(id),0,ScaLP::INF()));
    //store tIndex
    this->tIndices.insert(make_pair(v,ti.size()-1));

  }

  for(std::list<Resource*>::iterator it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); ++it) {
    Resource* r = *it;
    if(this->resourceModel.getNumVerticesRegisteredToResource(r)==0) continue;

    int ak = r->getLimit();
    if(ak==-1) continue;
    set<const Vertex*> verOfRes = this->resourceModel.getVerticesOfResource(r);
    for(set<const Vertex*>::iterator it2 = verOfRes.begin(); it2 != verOfRes.end(); it2++)
    {
      const Vertex* v1 = (*it2);
      //18
      this->ri.push_back(ScaLP::newIntegerVariable("r_" + std::to_string(v1->getId()),0,ak-1));
      this->rIndices.insert(make_pair(v1, this->ri.size() -1));
    }
  }
}

}