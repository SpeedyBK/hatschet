/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)

    Copyright (C) 2019

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

#include <HatScheT/scheduler/graphBased/SGMScheduler.h>
#include <HatScheT/utility/Utility.h>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <algorithm>
#include <math.h>

namespace HatScheT
{

SGMScheduler::SGMScheduler(Graph &g, ResourceModel &resourceModel, std::list<string> solverWishlist, OccurrenceSetCombination *occSC) : MoovacScheduler(g, resourceModel, solverWishlist)
{
  this->computeMinII(&g,&resourceModel);
  this->minII = ceil(this->minII);
  this->computeMaxII(&g,&resourceModel);
  if (this->minII >= this->maxII) this->maxII = this->minII+1;
  this->SLMax = 0;
  this->occSC = occSC;
  cout << "SGMScheduler.SGMScheduler: Start minII/maxII " << this->minII << " / " << this->maxII << endl;

  //ToDo: dedicated subgraph based minII calculation
}

void SGMScheduler::setSubgraphConstraints()
{
  //manage subgraph connections in the ilp formulation
  set<OccurrenceSet*> occSets = this->occSC->getOccurrenceSets();

  //iterate of occ Sets
  for(auto it:occSets){
    //fetch corresponding vertices and edges
    vector<vector<Vertex*> > vertsContainer;
    vector<vector<const Edge*> > edgesContainer;
    OccurrenceSet* occS = it;
    set<Occurrence*> occurrences = occS->getOccurrences();

    for(auto it2=occurrences.begin();it2!=occurrences.end();++it2){
      Occurrence* occ = *it2;

      vertsContainer.push_back(occ->getVertices());
      edgesContainer.push_back(occ->getEdges());
    }

  //add constraints (lifetimes on corressponding edges have to be the same)
  for(int i=0; i<edgesContainer.size()-1;i++){
    for(int j=0;j<edgesContainer[i].size();j++){
      const Edge* e1 = edgesContainer[i][j];
      Vertex* src1 = &(e1->getVertexSrc());
      unsigned int srcTVecIndex1 = this->tIndices[src1];
      Vertex* dst1 = &(e1->getVertexDst());
      unsigned int dstTVecIndex1 = this->tIndices[dst1];

      const Edge* e2 = edgesContainer[i+1][j];
      Vertex* src2 = &(e2->getVertexSrc());
      unsigned int srcTVecIndex2 = this->tIndices[src2];
      Vertex* dst2 = &(e2->getVertexDst());
      unsigned int dstTVecIndex2 = this->tIndices[dst2];

      this->solver->addConstraint(this->ti[dstTVecIndex1] - this->ti[srcTVecIndex1]
       - this->ti[dstTVecIndex2] + this->ti[srcTVecIndex2]  == 0);
    }
  }

  int binCounter = 0;
  //add constraints for subgraph binding
  for(int i=0;i<vertsContainer.size()-1;i++){
    for(int j=0;j<vertsContainer[i].size();j++){
      Vertex* v1= vertsContainer[i][j];
      int rvecIndex1 = this->rIndices.at(v1);
      Vertex* v2= vertsContainer[i+1][j];
      int rvecIndex2 = this->rIndices.at(v2);

      //same position on subgraph means the operator has to be bound to the same unit
      this->solver->addConstraint(this->ri[rvecIndex1] - this->ri[rvecIndex2] == 0);
    }
  }

  for(int i=0;i<vertsContainer.size();i++){
    for(int j=0;j<vertsContainer[i].size();j++){
      const Vertex* vconst = vertsContainer[i][j];
      int rvecIndex1 = this->rIndices.at(vconst);
      //no other operation of this resource is allowed to be bound to the same units that are used for binding of this occurrenceSet
      const Resource* r = this->resourceModel.getResource(vconst);
      set<const Vertex*> vSet = this->resourceModel.getVerticesOfResource(r);

      //other vertices that are not in the occurrenceSet are not allowed to be bound on the respective units
      for(auto it:vSet){
        const Vertex* vIter = it;
        int rvecIndex3 = -1;
        bool found = false;

        for(int k=0;k<vertsContainer.size();k++){
            if (std::find(vertsContainer[k].begin(), vertsContainer[k].end(), vIter) != vertsContainer[k].end()){
                found = true;
            }
        }

        if(found == false){
          for(auto it2:this->rIndices){
            if(it2.first==vIter){
              rvecIndex3=it2.second;
            }
          }
        }

        if(rvecIndex3!=-1){
            ScaLP::Variable boolVar = ScaLP::newBinaryVariable("subgrBin_" + to_string(binCounter));
            binCounter++;

            //unequal constraint using bigM currently
            this->solver->addConstraint(this->ri[rvecIndex1]-this->ri[rvecIndex3] + 5000*boolVar >= 1);
            this->solver->addConstraint(this->ri[rvecIndex1]-this->ri[rvecIndex3] + 5000*boolVar <= 5000 - 1);
          }
        }
      }
    }
  }
}


void SGMScheduler::setGeneralConstraints()
{
  for(std::set<Edge*>::iterator it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it)
  {
    Edge* e = *it;
    Vertex* src = &(e->getVertexSrc());
    unsigned int srcTVecIndex = this->tIndices[src];
    Vertex* dst = &(e->getVertexDst());
    unsigned int dstTVecIndex = this->tIndices[dst];

    this->solver->addConstraint(this->II*(e->getDistance()) - ti[srcTVecIndex] + ti[dstTVecIndex] - this->resourceModel.getVertexLatency(src) >= 0);
  }

  this->setSubgraphConstraints();

}

}
