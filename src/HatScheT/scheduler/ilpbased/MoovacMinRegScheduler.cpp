#include <HatScheT/scheduler/ilpbased/MoovacMinRegScheduler.h>
#include <HatScheT/utility/Utility.h>
#include <HatScheT/scheduler/ASAPScheduler.h>

namespace HatScheT
{

MoovacMinRegScheduler::MoovacMinRegScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist)
    : MoovacScheduler(g, resourceModel, solverWishlist)
{
  this->minII = this->computeMinII(&g,&resourceModel);
  HatScheT::ASAPScheduler asap(g,resourceModel);
  this->maxII = Utility::calcMaxII(&asap);
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

    this->solver->addConstraint((this->II)*(e->getDistance()) - t_vector[srcTVecIndex] + t_vector[dstTVecIndex]
                    - this->resourceModel.getVertexLatency(src) - this->regVector[regVecIndex] - e->getDelay() == 0);
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
  vector<vector<ScaLP::Variable> > minMaxRegVariablesContainer;

  for(std::list<Resource*>::iterator it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); ++it)
  {
    Resource* r = *it;
    set<const Vertex*> verticesOfR = this->resourceModel.getVerticesOfResource(r);

    vector<ScaLP::Variable> minMaxRegVariablesVector;

    //unlimited resource are assummed to be implemented in paralell
    if(r->getLimit() == -1)
    {
      //iterate of every implementation unit of resource r
      for(auto it2:verticesOfR)
      {
        const Vertex* v = it2;
        set<Vertex*> followingVertices = this->g.getSubsequentVertices(v);
        if(followingVertices.size()==0) continue;

        minMaxRegVariablesVector.push_back(ScaLP::newIntegerVariable("D_" + v->getName(),0,10000));

        for(auto it3:followingVertices)
        {
          const Vertex* followV = it3;
          Edge* e = &this->g.getEdge(v,followV);
          int eRegContainerIndex = this->reg_vectorIndices[e];

          this->solver->addConstraint(minMaxRegVariablesVector.back() - this->regVector[eRegContainerIndex] >= 0);
        }
      }

      minMaxRegVariablesContainer.push_back(minMaxRegVariablesVector);
    }

    //limited resource
    else
    {
      int ak = r->getLimit();
      vector<Edge*> outgoingEdgesOfRes = this->getOutGoingEdgesOfResource(r);

      //iterate over possible bindings
      for(int i = 0; i < ak; i++){
        minMaxRegVariablesVector.push_back(ScaLP::newIntegerVariable("D_" + r->getName()  + "_ak_" + to_string(i),0,1000));
      }
      //iterate over all edges where source vertex is of resource r
      for(unsigned int i = 0; i < outgoingEdgesOfRes.size(); i++){
        Edge* e = outgoingEdgesOfRes[i];
        int eRegContainerIndex = this->reg_vectorIndices[e];
        Vertex* vSrc = &e->getVertexSrc();
        Vertex* vDst = &e->getVertexDst();
        int vSrcRVectorIndex = this->r_vectorIndices[vSrc];

        vector<ScaLP::Variable> allocBinVars;

        for(unsigned int j = 0; j < (unsigned int)ak; j++){
            allocBinVars.push_back(ScaLP::newBinaryVariable("r_" + r->getName()  + "_ak_" + to_string(j)
                                                           + "_d" + to_string(vSrc->getId()) + "-" + to_string(vDst->getId()) + "e" + to_string(e->getID()),0,1));
        }

        //only one r_ik is used
        ScaLP::Term binSum;
        for(ScaLP::Variable &r_bin:allocBinVars){
            binSum = binSum + r_bin;
        }
        this->solver->addConstraint(binSum == 1);

        //binary varibales constraint to connect with r_i
        ScaLP::Term binWeightedSum;
        int weight = 0;
        for(ScaLP::Variable &r_bin:allocBinVars){
            binWeightedSum = binWeightedSum + weight * r_bin;
            weight++;
        }
        this->solver->addConstraint(this->r_vector[vSrcRVectorIndex] - binWeightedSum == 0);

        for(unsigned int j = 0; j < (unsigned int)ak; j++){
          this->solver->addConstraint(minMaxRegVariablesVector[j] - this->regVector[eRegContainerIndex] + (1- allocBinVars[j]) * 10000 >= 0);
        }
      }

      if(outgoingEdgesOfRes.size() > 0) minMaxRegVariablesContainer.push_back(minMaxRegVariablesVector);
    }
  }

  ScaLP::Term mmrSum;
  for(unsigned int i = 0; i < minMaxRegVariablesContainer.size(); i++)
  {
    for(ScaLP::Variable &mmr:minMaxRegVariablesContainer[i])
    {
        mmrSum = mmrSum + mmr;
    }
  }

  this->solver->setObjective(ScaLP::minimize(mmrSum));
}

vector<Edge*> MoovacMinRegScheduler::getOutGoingEdgesOfResource(Resource *r)
{
  vector<Edge*> edges;

  for(auto it=this->g.edgesBegin(); it!=this->g.edgesEnd(); ++it)
  {
    Edge* e = *it;
    const Vertex* vSrc = &e->getVertexSrc();

    if(this->resourceModel.getResource(vSrc)==r) edges.push_back(e);
  }

  return edges;
}

}
