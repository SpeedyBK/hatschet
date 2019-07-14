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

#include <HatScheT/scheduler/ilpbased/MoovacScheduler.h>
#include "HatScheT/utility/Utility.h"
#include "math.h"

namespace HatScheT
{

MoovacScheduler::MoovacScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist) : SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist)
{
  this->computeMinII(&g,&resourceModel);
  this->minII = ceil(this->minII);
  this->computeMaxII(&g,&resourceModel);
  if (this->minII >= this->maxII) this->maxII = this->minII+1;
  this->SLMax = 0;
}

void MoovacScheduler::resetContainer()
{
  this->solver->reset();
  this->mi.clear();
  this->registers.clear();
  this->tIndices.clear();
  this->registerIndices.clear();
  this->ti.clear();
  this->rIndices.clear();
  this->ri.resize(0);

  this->scheduleFound = false;
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

void MoovacScheduler::setMaxLatency() {
  //case unlimited
  if(this->maxLatencyConstraint == -1){
    this->SLMax = this->g.getNumberOfVertices() * ( this->resourceModel.getMaxLatency() + 1);
  } else if(this->maxLatencyConstraint > 0) {
    this->SLMax = this->maxLatencyConstraint;
  } else {
    throw HatScheT::Exception("MoovacScheduler::constructProblem: irregular maxLatencyConstraint " + to_string(this->maxLatencyConstraint));
  }
}

void MoovacScheduler::constructProblem()
{
  this->setMaxLatency();

  this->setVectorVariables();
  this->fillRegVector();

  //set up constraints
  this->setSourceVerticesToZero();
  this->setGeneralConstraints();
  this->setModuloAndResourceConstraints();

  //set Objective
  this->setObjective();
}

void MoovacScheduler::setObjective()
{
  //supersink latency objective
  ScaLP::Variable supersink = ScaLP::newIntegerVariable("supersink",0,this->SLMax);
  for(ScaLP::Variable &v:this->ti){
    this->solver->addConstraint(supersink - v >= 0);
  }
  this->solver->setObjective(ScaLP::minimize(supersink));
}

void MoovacScheduler::schedule()
{
  this->timeouts = 0;
  this->totalTime = 0;
  this->II = this->minII;

  //set maxRuns, e.g., maxII - minII, iff value if not -1
  if(this->maxRuns > 0){
    int runs = this->maxII - this->minII;
    if(runs >= this->maxRuns) this->maxII = this->minII + this->maxRuns - 1;
    std::cout << "Moovac: maxII changed due to maxRuns value set by user to " << this->maxRuns << endl;
    std::cout << "Moovac: min/maxII = " << minII << " " << maxII << std::endl;
  }

  bool timeoutOccured=false;

  cout << "Starting Moovac ILP-based modulo scheduling! minII is " << this->minII << ", maxII is " << this->maxII << endl;
  if(this->maxLatencyConstraint!=-1) cout << "MaxLatency is " << this->maxLatencyConstraint << endl;
  else cout << "Unlimited MaxLatency" << endl;
  cout << "Timeout: " << this->solverTimeout << " (sec) using " << this->threads << " threads." << endl;

  while(this->II <= this->maxII) {
    cout << "Starting Moovac ILP-based modulo scheduling with II " << this->II << endl;
    this->resetContainer();
    this->setUpSolverSettings();
    this->constructProblem();

    if(this->writeLPFile == true) this->solver->writeLP(to_string(this->II));

    //timestamp
    this->begin = clock();
    //solve
    stat = this->solver->solve();
    //timestamp
    this->end = clock();

    //log time
    if(this->solvingTime == -1.0) this->solvingTime = 0.0;
    this->solvingTime += (double)(this->end - this->end) / CLOCKS_PER_SEC;

    if(stat == ScaLP::status::OPTIMAL || stat == ScaLP::status::FEASIBLE || stat == ScaLP::status::TIMEOUT_FEASIBLE) this->scheduleFound = true;
    if(stat == ScaLP::status::TIMEOUT_INFEASIBLE) {
      this->timeouts++;
      timeoutOccured = true;
    }
    if(stat == ScaLP::status::OPTIMAL && timeoutOccured == false) this->optimalResult = true;

    if(scheduleFound == false) (this->II)++;
    else break;
  }

  if(this->scheduleFound == true) {
    this->r = this->solver->getResult();
    this->fillSolutionStructure();

    if(this->optimalResult == true) cout << "Found optimal solution for II: " << this->II << endl;
    else cout << "Found feasible solution for II: " << this->II << endl;
  }
  else{
    cout << "Moovac: Passed maxII boundary! No modulo schedule identified!" << endl;
    this->II = -1;
  }
}

void MoovacScheduler::fillSolutionStructure()
{
  for(std::set<Vertex*>::iterator it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it) {
    Vertex* v = *it;
    unsigned int index =this->tIndices.at(v);
    ScaLP::Variable svTemp = this->ti[index];

    int startTime = this->r.values[svTemp];
    this->startTimes.insert(make_pair(v, startTime));
  }
}

void MoovacScheduler::setVectorVariables()
{
  for(std::set<Vertex*>::iterator it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it) {
    Vertex* v = *it;
    int id = v->getId();

    //17
    this->ti.push_back(ScaLP::newIntegerVariable("t_" + std::to_string(id),0,this->SLMax - this->resourceModel.getVertexLatency(v)));
    //store tIndex
    this->tIndices.insert(make_pair(v,ti.size()-1));
    //16
    this->solver->addConstraint(ti.back() + this->resourceModel.getVertexLatency(v) <= this->SLMax);
  }

  for(std::list<Resource*>::iterator it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); ++it) {
    Resource* r = *it;
    if(this->resourceModel.getNumVerticesRegisteredToResource(r)==0) continue;

    int ak = r->getLimit();
    if(ak==-1) continue;
    set<const Vertex*> verOfRes = this->resourceModel.getVerticesOfResource(r);
    for(set<const Vertex*>::iterator it2 = verOfRes.begin(); it2 != verOfRes.end(); it2++) {
      const Vertex* v1 = (*it2);
      //18
      this->ri.push_back(ScaLP::newIntegerVariable("r_" + std::to_string(v1->getId()),0,ScaLP::INF()));
      this->rIndices.insert(make_pair(v1, this->ri.size() -1));
    }
  }
}

void MoovacScheduler::setSourceVerticesToZero()
{
  for(std::set<Vertex*>::iterator it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it) {
    Vertex* v = *it;

    unsigned int index =this->tIndices.at(v);
    ScaLP::Variable  temp = this->ti[index];
    //source vertix of unlimited(!) resource
    if(this->g.isSourceVertex(v) && this->resourceModel.getResource(v)->getLimit()==-1) this->solver->addConstraint(temp == 0);
  }
}

void MoovacScheduler::setGeneralConstraints()
{
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

std::map<Edge*,int> MoovacScheduler::getLifeTimes()
{
  if(this->startTimes.size()==0) throw HatScheT::Exception("SchedulerBase.getLifeTimes: cant return lifetimes! no startTimes determined!");

  std::map<Edge*,int> lifetimes;

  for(auto it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it){
    Edge* e = *it;
    Vertex* vSrc = &e->getVertexSrc();
    Vertex* vDst = &e->getVertexDst();
    int lifetime = this->startTimes[vDst] - this->startTimes[vSrc] - this->resourceModel.getVertexLatency(vSrc) + e->getDistance()*this->II;

    if(lifetime < 0) throw HatScheT::Exception("SchedulerBase.getLifeTimes: negative lifetime detected!");
    else lifetimes.insert(make_pair(e, lifetime));
  }
  return lifetimes;
}

std::map<const Vertex *, int> MoovacScheduler::getBindings()
{
  if (this->scheduleFound==false || this->startTimes.size()==0) throw HatScheT::Exception("MoovacScheduler.getBindings: schedule was found!");
  std::map<const Vertex*,int> bindings;

  //iterate over resources
  for(auto it=this->resourceModel.resourcesBegin();it!=this->resourceModel.resourcesEnd();++it){
    const Resource* r = *it;
    set<const Vertex*> vs = this->resourceModel.getVerticesOfResource(r);

    //unlimited resource, all in parallel
    if(r->getLimit()<0){
      int binding=0;
      for(auto v:vs){
        bindings.emplace(v,binding);
        binding++;
      }
    }
    //limited resource
    else{
      for(auto v:vs){
        ScaLP::Variable bv = this->ri[this->rIndices[v]];
        ScaLP::Result r = this->solver->getResult();
        bindings.insert(make_pair(v,r.values[bv]));
      }
    }
  }

  return bindings;
}

int MoovacScheduler::getNoOfMuxInputs()
{
  if (this->scheduleFound==false) return -1;

  int noOfMuxInputs = 0;

  for(std::list<Resource*>::iterator it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); ++it) {
    Resource* r = *it;
    //unlimited in paralell without mux
    if(r->getLimit()==-1) continue;
    else{
      set<const Vertex*> verticesOfR = this->resourceModel.getVerticesOfResource(r);
      int bindings = r->getLimit();

      //iterate over possible bindings
      for(int i = 0; i < bindings; i++) {
        set<const Vertex*> verticesOfSameResourceBinding;

        for(auto it3:verticesOfR) {
          const Vertex* candidateV = it3;
          int candidateVRIndex = this->rIndices[candidateV];
          ScaLP::Result r = this->solver->getResult();

          if(r.values[this->ri[candidateVRIndex]]==i) verticesOfSameResourceBinding.insert(candidateV);
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

  for(std::list<Resource*>::iterator it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); ++it) {
    Resource* r = *it;
    set<const Vertex*> verticesOfR = this->resourceModel.getVerticesOfResource(r);

    //unlimited resource are assummed to be implemented in paralell
    if(r->getLimit() == -1) {
      //iterate of every implementation unit of resource r
      for(auto it2:verticesOfR) {
        int maxLifetime = 0;

        const Vertex* v = it2;
        int vTIndex = this->tIndices[v];
        set<Vertex*> followingVertices = this->g.getSuccessors(v);

        for(auto it3:followingVertices) {
          const Vertex* followV = it3;
          std::list<const Edge *> edges = this->g.getEdges(v, followV);

          for (auto it = edges.begin(); it != edges.end(); ++it) {
            const Edge *e = *it;
            int followVTIndex = this->tIndices[followV];
            ScaLP::Result r = this->solver->getResult();

            int currLifetime = r.values[this->ti[followVTIndex]] - r.values[this->ti[vTIndex]] -
                               this->resourceModel.getVertexLatency(v) + this->II * e->getDistance();
            if (currLifetime > maxLifetime) maxLifetime = currLifetime;
          }
        }

        //add the max lifetime to overall registercount, because they are implemented using register sharing
        noOfRegs += maxLifetime;
      }     
    }

    //limited resource
    else {
      int bindings = r->getLimit();

      //iterate over possible bindings
      for(int i = 0; i < bindings; i++) {
        set<const Vertex*> verticesOfSameResourceBinding;

        for(auto it3:verticesOfR) {
          const Vertex* candidateV = it3;
          int candidateVRIndex = this->rIndices[candidateV];
          ScaLP::Result r = this->solver->getResult();

          if(r.values[this->ri[candidateVRIndex]]==i) verticesOfSameResourceBinding.insert(candidateV);
        }

        int maxLifetime = 0;

        for(auto it3:verticesOfSameResourceBinding) {
          const Vertex* v = it3;
          int vTIndex = this->tIndices[v];
          set<Vertex*> followingVertices = this->g.getSuccessors(v);

          for(auto it4:followingVertices) {
            const Vertex* followV = it4;
            std::list<const Edge *> edges = this->g.getEdges(v, followV);

            for (auto it = edges.begin(); it != edges.end(); ++it) {
              const Edge *e = *it;
              int followVTIndex = this->tIndices[followV];
              ScaLP::Result res = this->solver->getResult();

              int currLifetime = res.values[this->ti[followVTIndex]] - res.values[this->ti[vTIndex]] -
                                 this->resourceModel.getVertexLatency(v) + this->II * e->getDistance();

              if (currLifetime > maxLifetime) maxLifetime = currLifetime;
            }
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
  this->mi.resize(0);

  for(std::list<Resource*>::iterator it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); ++it) {
    Resource* r = *it;

    int ak = r->getLimit();
    if(ak==-1) continue;
    set<const Vertex*> verOfRes = this->resourceModel.getVerticesOfResource(r);
    if(verOfRes.size()==0) continue;

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

    for(set<const Vertex*>::iterator it2 = verOfRes.begin(); it2 != verOfRes.end(); it2++) {
      const Vertex* v1 = (*it2);

      int tIndex = this->tIndices.at(v1);
      //18
      int rvecIndex = this->rIndices.at(v1);
      //19
      m_vector.push_back(ScaLP::newIntegerVariable("m_" + std::to_string(v1->getId()),0,10000));
      //20
      y_vector.push_back(ScaLP::newIntegerVariable("y_" + std::to_string(v1->getId()),0,10000));

      //13
      this->solver->addConstraint(this->ti[tIndex] - y_vector.back()*((int)this->II) - m_vector.back() == 0);
      //14
      this->solver->addConstraint(this->ri[rvecIndex] <= ak - 1);
      //15
      this->solver->addConstraint(m_vector.back() <= this->II - 1);

      //declare eps-vector
      vector<ScaLP::Variable> eps_vector;
      //declare mu-vector
      vector<ScaLP::Variable> mu_vector;
      vector<pair<const Vertex*, const Vertex*> > corrVerticesVec;

      for(set<const Vertex*>::iterator it3 = verOfRes.begin(); it3 != verOfRes.end(); it3++) {
        const Vertex* v2 = (*it3);
        corrVerticesVec.push_back(make_pair(v1,v2));

        if(v1 != v2) {
          eps_vector.push_back(ScaLP::newBinaryVariable("eps_" + std::to_string(v1->getId()) + "_" + std::to_string(v2->getId()),0,1));
          mu_vector.push_back(ScaLP::newBinaryVariable("mu_" + std::to_string(v1->getId()) + "_" + std::to_string(v2->getId()),0,1));
        }

        else if(v1==v2) {
          eps_vector.push_back(ScaLP::newBinaryVariable("eps_" + std::to_string(v1->getId())  + "_" + std::to_string(v1->getId()),0,1));
          mu_vector.push_back(ScaLP::newBinaryVariable("mu_" + std::to_string(v1->getId())  + "_" + std::to_string(v1->getId()),0,1));
        }
      }

      eps_matrix.push_back(eps_vector);
      mu_matrix.push_back(mu_vector);
      corrVerticesMatrix.push_back(corrVerticesVec);
    }

    this->mi.push_back(m_vector);
    this->yi.push_back(y_vector);
    this->epsij.push_back(eps_matrix);
    this->muij.push_back(mu_matrix);

    if(eps_matrix.size() > 1) {
      for(unsigned int j = 0; j < eps_matrix.size(); j++) {
        for(unsigned int k = 0; k < eps_matrix.size(); k++) {
          if(k!=j && j<k) {
            //6
            this->solver->addConstraint(eps_matrix[j][k] + eps_matrix[k][j] <= 1);
            //12
            this->solver->addConstraint(eps_matrix[j][k] + eps_matrix[k][j] + mu_matrix[j][k] + mu_matrix[k][j] >= 1);
          }

          if(k!=j) {
            pair<const Vertex*, const Vertex*> vPair = corrVerticesMatrix[j][k];
            //7
            this->solver->addConstraint(this->ri[this->rIndices[vPair.first]] - this->ri[this->rIndices[vPair.second]]
                - (ak*eps_matrix[j][k]) + ak >= 1);
            //8
            this->solver->addConstraint(this->ri[this->rIndices[vPair.first]] - this->ri[this->rIndices[vPair.second]]
                - (ak*eps_matrix[j][k]) <= 0);
            //9
            this->solver->addConstraint(mu_matrix[j][k] + mu_matrix[k][j]<= 1);
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
