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

  //manage subgraph connections in the ilp formulation
  set<OccurrenceSet*> occSets = this->occSC->getOccurrenceSets();

  //fetch corresponding vertices and edges
  vector<vector<Vertex*> > vertsContainer;
  vector<vector<Edge*> > edgesContainer;

  for(auto it=occSets.begin(); it!=occSets.end();++it){
    OccurrenceSet* occS = *it;
    set<Occurrence*> occurrences = occS->getOccurrences();

    for(auto it2=occurrences.begin();it2!=occurrences.end();++it2){
      Occurrence* occ = *it2;

      vertsContainer.push_back(occ->getVertices());
      edgesContainer.push_back(occ->getEdges());
    }
  }

  //add constraints (lifetimes on corressponding edges have to be the same)
  for(int i=0; i<edgesContainer.size()-1;i++){
    for(int j=0;j<edgesContainer[i].size();j++){
      Edge* e1 = edgesContainer[i][j];
      Vertex* src1 = &(e1->getVertexSrc());
      unsigned int srcTVecIndex1 = this->t_vectorIndices[src1];
      Vertex* dst1 = &(e1->getVertexDst());
      unsigned int dstTVecIndex1 = this->t_vectorIndices[dst1];

      Edge* e2 = edgesContainer[i+1][j];
      Vertex* src2 = &(e2->getVertexSrc());
      unsigned int srcTVecIndex2 = this->t_vectorIndices[src2];
      Vertex* dst2 = &(e2->getVertexDst());
      unsigned int dstTVecIndex2 = this->t_vectorIndices[dst2];

      this->solver->addConstraint(this->II*(e1->getDistance()) - t_vector[srcTVecIndex1] + t_vector[dstTVecIndex1] - this->resourceModel.getVertexLatency(src1)
                                 - this->II*(e2->getDistance()) + t_vector[srcTVecIndex2] - t_vector[dstTVecIndex2] + this->resourceModel.getVertexLatency(src2) == 0);
    }
  }

  //add constraints for subgraph binding
  for(int i=0;i<vertsContainer.size()-1;i++){
    for(int j=0;j<vertsContainer[i].size();j++){
      Vertex* v1= vertsContainer[i][j];
      int rvecIndex1 = this->r_vectorIndices.at(v1);
      Vertex* v2= vertsContainer[i+1][j];
      int rvecIndex2 = this->r_vectorIndices.at(v2);

      //same position on subgraph means the operator has to be bound to the same unit
      this->solver->addConstraint(this->r_vector[rvecIndex1] - this->r_vector[rvecIndex2] == 0);

      //no other operation of this resource is allowed to be bound to the same units that are used for binding of this occurrenceSet
      if(i==0){
        const Vertex* vconst = vertsContainer[i][j];
        const Resource* r = this->resourceModel.getResource(vconst);
        set<const Vertex*> vSet = this->resourceModel.getVerticesOfResource(r);

        for(auto it:vSet){
          const Vertex* vIter = it;


        }
      }
    }
  }

}

}
