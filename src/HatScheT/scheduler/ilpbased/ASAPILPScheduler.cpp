/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)

    Copyright (C) 2018

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "ASAPILPScheduler.h"

namespace HatScheT
{
ASAPILPScheduler::ASAPILPScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string> solverWishlist)
        : SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist) {

}

void ASAPILPScheduler::schedule() {
  this->constructProblem();

  stat = this->solver->solve();

  if(stat == ScaLP::status::OPTIMAL || stat == ScaLP::status::FEASIBLE || stat == ScaLP::status::TIMEOUT_FEASIBLE) this->scheduleFound = true;
  if(stat == ScaLP::status::OPTIMAL) this->optimalResult = true;

  if(this->scheduleFound == true)
  {
    this->r = this->solver->getResult();
    this->fillSolutionStructure();
    this->II = this->getScheduleLength();
  }

  else{
    cout << "No ASAP schedule found!" << endl;
    this->II = -1;
  }
}

void ASAPILPScheduler::constructProblem() {
  this->setVectorVariables();
  this->setGeneralConstraints();
  this->setObjective();
}

void ASAPILPScheduler::setObjective() {
  //supersink latency objective
  ScaLP::Variable supersink = ScaLP::newIntegerVariable("supersink",0,ScaLP::INF());
  for(ScaLP::Variable &v:this->ti){
    this->solver->addConstraint(supersink - v >= 0);
  }
  this->solver->setObjective(ScaLP::minimize(supersink));
}

void ASAPILPScheduler::fillSolutionStructure() {
  for(std::set<Vertex*>::iterator it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it) {
    Vertex* v = *it;
    unsigned int index =this->tIndices.at(v);
    ScaLP::Variable svTemp = this->ti[index];

    int startTime = this->r.values[svTemp];
    this->startTimes.insert(make_pair(v, startTime));
  }
}

void ASAPILPScheduler::setGeneralConstraints() {
  //5
  for(std::set<Edge*>::iterator it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it) {
    Edge* e = *it;
    //distances for asap scheduling
    if(e->getDistance() > 0) continue;

    Vertex* src = &(e->getVertexSrc());
    unsigned int srcTVecIndex = this->tIndices[src];
    Vertex* dst = &(e->getVertexDst());
    unsigned int dstTVecIndex = this->tIndices[dst];

    this->solver->addConstraint(ti[srcTVecIndex] - ti[dstTVecIndex] + this->resourceModel.getVertexLatency(src) + e->getDelay() <= 0);
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