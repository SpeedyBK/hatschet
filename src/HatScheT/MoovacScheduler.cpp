#include <HatScheT/MoovacScheduler.h>
#include "utility/Utility.h"
#include <HatScheT/scheduler/ASAPScheduler.h>

namespace HatScheT
{

MoovacScheduler::MoovacScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist) : SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist)
{
  this->minII = this->computeMinII(&g,&resourceModel);
  HatScheT::ASAPScheduler asap(g,resourceModel);
  this->maxII = Utility::calcMaxII(&asap);
  if (minII > maxII) maxII = minII;
  this->SLMax = 0;
}

void MoovacScheduler::resetContainer()
{
  this->solver->reset();
  this->m_container.clear();
  this->regVector.clear();
  this->t_vectorIndices.clear();
  this->reg_vectorIndices.clear();
  this->t_vector.clear();
  this->r_vectorIndices.clear();
  this->r_vector.resize(0);
}

void MoovacScheduler::setUpSolverSettings()
{
  // disable solver output
  if(this->solverQuiet) solver->quiet = true;
  else solver->quiet = false;

  //set solver timeout
  this->solver->timeout = this->solverTimeout;
  this->solver->threads = this->threads;
}

void MoovacScheduler::constructProblem()
{
  //case unlimited
  if(this->maxLatencyConStraint == -1){
    this->SLMax = this->g.getNumberOfVertices() * ( this->resourceModel.getMaxLatency() + 1);
  }
  else if(this->maxLatencyConStraint > 0)
  {
    this->SLMax = this->maxLatencyConStraint;
  }
  else
  {
     throw new Exception("MoovacScheduler::constructProblem: irregular maxLatencyConstraint " + to_string(this->maxLatencyConStraint));
  }

  this->setVectorVariables();
  this->fillRegVector();

  //add constraints
  this->setSourceVerticesToZero();
  this->setGeneralConstraints();
  this->setModuloAndResourceConstraints();

  //set Objective
  this->setObjective();
}

void MoovacScheduler::setObjective()
{
    ScaLP::Term sum;
    for(ScaLP::Variable &v:t_vector){
      sum = sum + v;
    }
    this->solver->setObjective(ScaLP::minimize(sum));
}

void MoovacScheduler::schedule()
{
  this->timeoutCounter = 0;
  this->totalTime = 0;
  this->II = this->minII;

  while(this->II <= this->maxII)
  {
    this->resetContainer();
    this->setUpSolverSettings();
    this->constructProblem();

    ScaLP::status stat = this->solver->solve();

    if(stat == ScaLP::status::OPTIMAL || stat == ScaLP::status::FEASIBLE) this->scheduleFound = true;

    if(this->writeLPFile == true) this->solver->writeLP(to_string(this->II));

    if(this->scheduleFound == false){

      (this->II)++;
    }
    else if(this->scheduleFound == true) break;
  }

  if(this->scheduleFound == true)
  {
    this->r = this->solver->getResult();

    this->fillSolutionStructure();   
  }

  else this->II = -1;
}

void MoovacScheduler::fillSolutionStructure()
{
  for(std::set<Vertex*>::iterator it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it)
  {
    Vertex* v = *it;
    unsigned int index =this->t_vectorIndices.at(v);
    ScaLP::Variable svTemp = this->t_vector[index];

    int startTime = this->r.values[svTemp];
    this->startTimes.insert(make_pair(v, startTime));
  }
}

void MoovacScheduler::setVectorVariables()
{
  for(std::set<Vertex*>::iterator it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it)
  {
    Vertex* v = *it;
    int id = v->getId();

    //17
     t_vector.push_back(ScaLP::newIntegerVariable("t_" + std::to_string(id),0,this->SLMax - this->resourceModel.getVertexLatency(v)));
    //store tIndex
    this->t_vectorIndices.insert(make_pair(v,t_vector.size()-1));

    //16
    this->solver->addConstraint(t_vector.back() + this->resourceModel.getVertexLatency(v) <= this->SLMax);
  }

  for(std::list<Resource*>::iterator it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); ++it)
  {
    Resource* r = *it;
    if(this->resourceModel.getNoOfVerticesRegisteredToResource(r)==0) continue;

    int ak = r->getLimit();
    if(ak==-1) continue;
    set<const Vertex*> verOfRes = this->resourceModel.getVerticesOfResource(r);
    for(set<const Vertex*>::iterator it2 = verOfRes.begin(); it2 != verOfRes.end(); it2++)
    {
      const Vertex* v1 = (*it2);
      //18
      this->r_vector.push_back(ScaLP::newIntegerVariable("r_" + std::to_string(v1->getId()),0,ak-1));
      this->r_vectorIndices.insert(make_pair(v1, this->r_vector.size() -1));
    }
  }
}

void MoovacScheduler::setSourceVerticesToZero()
{
  for(std::set<Vertex*>::iterator it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it)
  {
    Vertex* v = *it;

    unsigned int index =this->t_vectorIndices.at(v);
    ScaLP::Variable  temp = this->t_vector[index];
    //source vertix of unlimited(!) resource
    if(this->g.isSourceVertex(v) && this->resourceModel.getResource(v)->getLimit()==-1) this->solver->addConstraint(temp  == 0);
  }
}

void MoovacScheduler::setGeneralConstraints()
{
  //5
  for(std::set<Edge*>::iterator it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it)
  {
    Edge* e = *it;
    Vertex* src = &(e->getVertexSrc());
    unsigned int srcTVecIndex = this->t_vectorIndices[src];
    Vertex* dst = &(e->getVertexDst());
    unsigned int dstTVecIndex = this->t_vectorIndices[dst];

    this->solver->addConstraint(this->II*(e->getDistance()) - t_vector[srcTVecIndex] + t_vector[dstTVecIndex] - this->resourceModel.getVertexLatency(src) - e->getDelay() >= 0);
  }
}

std::map<Edge*,int> MoovacScheduler::getLifeTimes()
{
  if(this->startTimes.size()==0) throw new Exception("SchedulerBase.getLifeTimes: cant return lifetimes! no startTimes determined!");

  std::map<Edge*,int> lifetimes;

  for(auto it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it){
    Edge* e = *it;
    Vertex* vSrc = &e->getVertexSrc();
    Vertex* vDst = &e->getVertexDst();
    int lifetime = this->startTimes[vDst] - this->startTimes[vSrc] - this->resourceModel.getVertexLatency(vSrc) + e->getDistance()*this->II;

    if(lifetime < 0) throw new Exception("SchedulerBase.getLifeTimes: negative lifetime detected!");
    else lifetimes.insert(make_pair(e, lifetime));
  }
  return lifetimes;
}

std::map<const Vertex *, int> MoovacScheduler::getBindings()
{
  if (this->scheduleFound==false || this->startTimes.size()==0) throw new Exception("MoovacScheduler.getBindings: schedule was found!");
  std::map<const Vertex*,int> bindings;

  for(auto it:this->startTimes){
    const Vertex* v = it.first;
    ScaLP::Variable bv = this->r_vector[this->r_vectorIndices[v]];
    ScaLP::Result r = this->solver->getResult();
    bindings.insert(make_pair(v,r.values[bv]));
  }

  return bindings;
}

int MoovacScheduler::getNoOfMuxInputs()
{
  if (this->scheduleFound==false) return -1;

  int noOfMuxInputs = 0;

  for(std::list<Resource*>::iterator it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); ++it)
  {
    Resource* r = *it;
    //unlimited in paralell without mux
    if(r->getLimit()==-1) continue;
    else{
      set<const Vertex*> verticesOfR = this->resourceModel.getVerticesOfResource(r);
      int bindings = r->getLimit();

      //iterate over possible bindings
      for(int i = 0; i < bindings; i++)
      {
        set<const Vertex*> verticesOfSameResourceBinding;

        for(auto it3:verticesOfR)
        {
          const Vertex* candidateV = it3;
          int candidateVRIndex = this->r_vectorIndices[candidateV];
          ScaLP::Result r = this->solver->getResult();

          if(r.values[this->r_vector[candidateVRIndex]]==i) verticesOfSameResourceBinding.insert(candidateV);
        }

        const Vertex* v = *verticesOfSameResourceBinding.begin();
        int vInputs=Utility::getNoOfInputs(&(this->g), v);

        noOfMuxInputs += vInputs*verticesOfSameResourceBinding.size();
      }

    }
  }

  return noOfMuxInputs;
}

int MoovacScheduler::getNoOfImplementedRegisters()
{
  if (this->scheduleFound==false) return -1;

  int noOfRegs = 0;

  for(std::list<Resource*>::iterator it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); ++it)
  {
    Resource* r = *it;
    set<const Vertex*> verticesOfR = this->resourceModel.getVerticesOfResource(r);

    //unlimited resource are assummed to be implemented in paralell
    if(r->getLimit() == -1)
    {
      //iterate of every implementation unit of resource r
      for(auto it2:verticesOfR)
      {
        int maxLifetime = 0;

        const Vertex* v = it2;
        int vTIndex = this->t_vectorIndices[v];
        set<Vertex*> followingVertices = this->g.getSubsequentVertices(v);

        for(auto it3:followingVertices)
        {
          const Vertex* followV = it3;
          Edge* e = &this->g.getEdge(v,followV);
          int followVTIndex = this->t_vectorIndices[followV];
          ScaLP::Result r = this->solver->getResult();

          int currLifetime = r.values[this->t_vector[followVTIndex]] - r.values[this->t_vector[vTIndex]] - this->resourceModel.getVertexLatency(v) + this->II*e->getDistance();
          if(currLifetime > maxLifetime) maxLifetime = currLifetime;
        }

        //add the max lifetime to overall registercount, because they are implemented using register sharing
        noOfRegs += maxLifetime;
      }     
    }

    //limited resource
    else
    {
      int bindings = r->getLimit();

      //iterate over possible bindings
      for(int i = 0; i < bindings; i++)
      {
        set<const Vertex*> verticesOfSameResourceBinding;

        for(auto it3:verticesOfR)
        {
          const Vertex* candidateV = it3;
          int candidateVRIndex = this->r_vectorIndices[candidateV];
          ScaLP::Result r = this->solver->getResult();

          if(r.values[this->r_vector[candidateVRIndex]]==i) verticesOfSameResourceBinding.insert(candidateV);
        }

        int maxLifetime = 0;

        for(auto it3:verticesOfSameResourceBinding)
        {
          const Vertex* v = it3;
          int vTIndex = this->t_vectorIndices[v];
          set<Vertex*> followingVertices = this->g.getSubsequentVertices(v);

          for(auto it4:followingVertices)
          {
            const Vertex* followV = it4;
            Edge* e = &this->g.getEdge(v,followV);
            int followVTIndex = this->t_vectorIndices[followV];
            ScaLP::Result res = this->solver->getResult();

            int currLifetime = res.values[this->t_vector[followVTIndex]] - res.values[this->t_vector[vTIndex]] - this->resourceModel.getVertexLatency(v) + this->II*e->getDistance();

            if(currLifetime > maxLifetime) maxLifetime = currLifetime;
          }
        }

        //add the max lifetime to overall registercount, because they are implemented using register sharing
        noOfRegs += maxLifetime;
      }
    }
  }

  return noOfRegs;
}

void MoovacScheduler::setModuloAndResourceConstraints()
{
    this->m_container.resize(0);

    for(std::list<Resource*>::iterator it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); ++it)
    {
      Resource* r = *it;
      if(this->resourceModel.getNoOfVerticesRegisteredToResource(r)==0) continue;

      int ak = r->getLimit();
      if(ak==-1) continue;
      set<const Vertex*> verOfRes = this->resourceModel.getVerticesOfResource(r);

      //declare y-vector
      vector<ScaLP::Variable> y_vector;
      //declare m-vector
      vector<ScaLP::Variable> m_vector;
      //declare eps-matrix
      vector<vector<ScaLP::Variable> > eps_matrix;
      //declare mu-matrix
      vector<vector<ScaLP::Variable> > mu_matrix;
      //store corresponding pointer
      vector<vector<pair<const Vertex*, const Vertex*> > > corrVerticesMatrix;

      for(set<const Vertex*>::iterator it2 = verOfRes.begin(); it2 != verOfRes.end(); it2++)
      {        
        const Vertex* v1 = (*it2);

        int tIndex = this->t_vectorIndices.at(v1);
        //18
        int rvecIndex = this->r_vectorIndices.at(v1);
        //19
        m_vector.push_back(ScaLP::newIntegerVariable("m_" + std::to_string(v1->getId()),0,10000));
        //20
        y_vector.push_back(ScaLP::newIntegerVariable("y_" + std::to_string(v1->getId()),0,10000));

        //13
        this->solver->addConstraint(this->t_vector[tIndex] - y_vector.back()*this->II - m_vector.back() == 0);
        //14
        this->solver->addConstraint(this->r_vector[rvecIndex] <= ak - 1);
        //15
        this->solver->addConstraint(m_vector.back() <= this->II - 1);

        //declare eps-vector
        vector<ScaLP::Variable> eps_vector;
        //declare mu-vector
        vector<ScaLP::Variable> mu_vector;
        vector<pair<const Vertex*, const Vertex*> > corrVerticesVec;

        for(set<const Vertex*>::iterator it3 = verOfRes.begin(); it3 != verOfRes.end(); it3++)
        {
          const Vertex* v2 = (*it3);
          corrVerticesVec.push_back(make_pair(v1,v2));

          if(v1 != v2)
          {
            eps_vector.push_back(ScaLP::newBinaryVariable("eps_" + std::to_string(v1->getId()) + "_" + std::to_string(v2->getId()),0,1));
            mu_vector.push_back(ScaLP::newBinaryVariable("mu_" + std::to_string(v1->getId()) + "_" + std::to_string(v2->getId()),0,1));
          }

          else if(v1==v2)
          {
            eps_vector.push_back(ScaLP::newBinaryVariable("eps_" + std::to_string(v1->getId())  + "_" + std::to_string(v1->getId()),0,1));
            mu_vector.push_back(ScaLP::newBinaryVariable("mu_" + std::to_string(v1->getId())  + "_" + std::to_string(v1->getId()),0,1));
          }
        }

        eps_matrix.push_back(eps_vector);
        mu_matrix.push_back(mu_vector);
        corrVerticesMatrix.push_back(corrVerticesVec);
      }

      this->m_container.push_back(m_vector);
      this->y_container.push_back(y_vector);
      this->eps_container.push_back(eps_matrix);
      this->mu_container.push_back(mu_matrix);

      if(eps_matrix.size() > 1)
      {
        for(unsigned int j = 0; j < eps_matrix.size(); j++)
        {
          for(unsigned int k = 0; k < eps_matrix.size(); k++)
          {
            if(k!=j && j<k)
            {
              //6
              this->solver->addConstraint(eps_matrix[j][k] + eps_matrix[k][j] <= 1);
              //12
              this->solver->addConstraint(eps_matrix[j][k] + eps_matrix[k][j] + mu_matrix[j][k] + mu_matrix[k][j] >= 1);
            }

            if(k!=j)
            {
              pair<const Vertex*, const Vertex*> vPair = corrVerticesMatrix[j][k];
              ScaLP::Variable vj =  this->r_vector[this->r_vectorIndices[vPair.first]];
              ScaLP::Variable vk =  this->r_vector[this->r_vectorIndices[vPair.second]];
              //7
              this->solver->addConstraint(vj - vk - (ak*eps_matrix[j][k]) + ak >= 1);
              //8
              this->solver->addConstraint(vj - vk - (ak*eps_matrix[j][k]) <= 0);
              //10
              this->solver->addConstraint(m_vector[j]-m_vector[k] - (this->II*mu_matrix[j][k]) + this->II >= 1);
              //11
              this->solver->addConstraint(m_vector[j]-m_vector[k] - (this->II*mu_matrix[j][k]) <= 0);
            }
          }
        }
      }
  }
}

}
