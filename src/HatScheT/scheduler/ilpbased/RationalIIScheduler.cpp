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

#include <HatScheT/scheduler/ilpbased/RationalIIScheduler.h>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/utility/Utility.h>

namespace HatScheT
{
    RationalIIScheduler::RationalIIScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist)
    : MoovacScheduler(g, resourceModel, solverWishlist)
    {
      throw HatScheT::Exception("rationalIIScheduler::rationalIIScheduler: This constructor is currently under construction and disabled!");
  /*this->minII = this->computeMinII(&g,&resourceModel);
  HatScheT::ASAPScheduler asap(g,resourceModel);
  this->maxII = Utility::calcMaxII(&asap);
  this->SLMax = 0;*/
}

void RationalIIScheduler::fillIIVector()
{
  this->II_vector.clear();

  for(unsigned int i = 0; i < this->moduloClasses; i++) {
    if(i==0) II_vector.push_back(ScaLP::newIntegerVariable("II_" + std::to_string(i+1),0,0));
    else II_vector.push_back(ScaLP::newIntegerVariable("II_" + std::to_string(i+1),i,this->consideredModuloCycle));
  }
}

void RationalIIScheduler::setObjective()
{

}

void RationalIIScheduler::constructProblem()
{

}

void RationalIIScheduler::setGeneralConstraints()
{
  //general constraints, data path dependencies
  for(std::set<Edge*>::iterator it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it)
  {
    Edge* e = *it;
    Vertex* src = &(e->getVertexSrc());
    unsigned int srcTVecIndex = this->tIndices[src];
    Vertex* dst = &(e->getVertexDst());
    unsigned int dstTVecIndex = this->tIndices[dst];

    for(unsigned int j = 0; j < this->moduloClasses; j++)
    {
      for(unsigned int k = 0; k < II_vector.size()-1; k++)
      {
        this->solver->addConstraint(t_matrix[j][dstTVecIndex] - t_matrix[j][srcTVecIndex] + e->getDistance()*(II_vector[k+1]-II_vector[k]) - e->getDelay() >= 0);
      }
    }
  }

  //limit max latency
  for(auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it){
    Vertex* v = *it;

    if(this->g.isSinkVertex(v)==true)
    {
      this->solver->addConstraint(t_matrix[0][this->tIndices.at(v)] <= this->maxLatencyConstrain);
    }
  }

  //distinguish IIs
  for(unsigned int i = 0; i < II_vector.size()-1; i++)
  {
    this->solver->addConstraint(II_vector[i] - II_vector[i+1]  + 1 <= 0);
  }
}

void RationalIIScheduler::fillTMaxtrix()
{
  this->t_matrix.clear();

  //i modulo classes considered
  for(unsigned int i = 0; i < this->moduloClasses; i++) {
    vector<ScaLP::Variable> t_vector;

    //j vertices
    for(auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it){
      Vertex* v = *it;
      int id = v->getId();

      t_vector.push_back(ScaLP::newIntegerVariable("t'" + std::to_string(i) + "_" + std::to_string(id),i,this->maxLatencyConstrain*(i+1) + i*1));

      if(i == 0) this->tIndices.insert(make_pair(v,t_vector.size()-1));
    }

    t_matrix.push_back(t_vector);
  }
}


}
