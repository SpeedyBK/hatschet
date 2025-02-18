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

    -- 20.10.2022 Benjamin Lagershausen-Keßler: Integrated "IterativeModuloSchedulerLayer" Class
*/

#include "MoovacResAwScheduler.h"
#include <HatScheT/utility/Utility.h>
#include <math.h>
#include <algorithm>

namespace HatScheT
{

MoovacResAwScheduler::MoovacResAwScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist, Target& target)
        : MoovacScheduler(g, resourceModel, solverWishlist), target(target) {
  if(Utility::resourceModelAndTargetValid(resourceModel,target) == false){
    throw HatScheT::Exception("MoovacResAwScheduler.MoovacResAwScheduler: ERROR Resource Model and Hardware Target are not corresponding!");
  }

  this->computeMinII(&g,&resourceModel,&target);
  this->minII = ceil(this->minII);
  this->computeMaxII(&g,&resourceModel);
  if (this->minII >= this->maxII) this->maxII = this->minII+1;
  this->SLMax = 0;
  this->lambda = 0.0f;
  this->fullDSE = false;
  this->DSEfinshed = false;
}

std::map<const Vertex*,int> MoovacResAwScheduler::getBindings() {
  if(this->fullDSE == true){
    if(this->dseBindings.size() == 0) throw Exception("MoovacResAwScheduler.getBindings: ERROR no bindings found yet!");
    return this->dseBindings.at(this->II);
  } else {
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
}

void MoovacResAwScheduler::storeScheduleAndAllocation(){
  std::map<Vertex*,int> schedule;
  std::map<Resource*,int> allocationMap;
  std::map<const Vertex*, int> bindings;
  bool finished = true;

  //store schedule
  for(std::set<Vertex*>::iterator it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it) {
    Vertex* v = *it;
    unsigned int index =this->tIndices.at(v);
    ScaLP::Variable svTemp = this->ti[index];

    int startTime = this->r.values[svTemp];
    schedule.insert(make_pair(v, startTime));
  }
  this->dseStartTimes.insert(make_pair(this->II, schedule));

  //store resource allocation
  for(auto it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); ++it){
    Resource* r = *it;
    if(r->isUnlimited()== true) continue;
    int allocation = this->r.values[this->aks[this->aksIndices[r]]];

    //check whether the design space exploration is finished
    if(allocation > 1) finished = false;

    allocationMap.insert(make_pair(r,allocation));
  }
  this->dseAllocations.insert(make_pair(this->II,allocationMap));

  //store hardware binding
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
  this->dseBindings.insert(make_pair(this->II, bindings));

  if(finished == true) this->DSEfinshed = true;
}



void MoovacResAwScheduler::constructProblem()
{
  //set up the problem
  this->setMaxLatency();
  this->setVectorVariables();
  this->fillRegVector();
  this->setSourceVerticesToZero();

  //set up new values that are needed for RAMS scheduling
  this->getAk();
  //set up new aks vector that is needed for resource allocation
  this->fillAksVectorAndSetConstaints();

  //set up constraints
  this->setGeneralConstraints();
  this->setModuloAndResourceConstraints();
  //new constraints
  this->setAllocationConstraints();

  //set Objective
  this->setObjective();
}

void MoovacResAwScheduler::setAllocationConstraints() {
  //iterate over hardware cost types
  for(auto it = this->target.getElements().begin(); it != this->target.getElements().end(); ++it){
    std::string element = it->first;
    double constraint = it->second;

    ScaLP::Term ScaLPSum;

    //iterate over resouces
    for(auto it2 = this->resourceModel.resourcesBegin(); it2 != this->resourceModel.resourcesEnd(); ++it2){
      Resource* r = *it2;
      //skip unlimited
      if(r->isUnlimited()== true) continue;

      double costs = r->getHardwareCost(element);
      //skip iff costs are 0
      if(costs == 0.0f) continue;
      else {
        ScaLP::Term term = costs*this->aks[this->aksIndices[r]];
        ScaLPSum += term;
      }
    }

    //add constraints
    //16-18 in new formulation sheet
    this->solver->addConstraint(ScaLPSum <= constraint);
  }
}

vector<int> MoovacResAwScheduler::getFoundIIs() {
  if(this->dseStartTimes.size() == 0) throw Exception("MoovacResAwScheduler.getFoundIIs: ERROR no IIs found yet! startTimes map empty!");

  vector<int> IIs;

  for(auto it = this->dseStartTimes.begin(); it != this->dseStartTimes.end(); ++it){
    IIs.push_back(it->first);
  }

  std::sort(IIs.begin(), IIs.end());

  return IIs;
}

void MoovacResAwScheduler::setDSEResult(int requII) {
  //set start times
  if ( this->dseStartTimes.find(requII) == this->dseStartTimes.end() ) {
    throw Exception("MoovacResAwScheduler.setDSEResult: No startTimes entry found for requII " + to_string(requII));
  } else {
    this->startTimes = this->dseStartTimes.at(requII);
  }

  //set allocations
  if ( this->dseAllocations.find(requII) == this->dseAllocations.end() ) {
    throw Exception("MoovacResAwScheduler.setDSEResult: No allocation entry found for requII " + to_string(requII));
  } else {
    for(auto it = this->dseAllocations.at(requII).begin(); it != this->dseAllocations.at(requII).end(); ++it){
      Resource* r = it->first;
      r->setLimit(it->second);
    }
  }

  //set requII to this value
  this->II = requII;
}

void MoovacResAwScheduler::fillAksVectorAndSetConstaints() {
  for(auto it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); ++it){
    Resource* r = *it;
    int count = this->resourceModel.getNumVerticesRegisteredToResource(r);
    if(r->isUnlimited()==true){
      //14 in new formulation sheet
      //does this work for ak == | Ok | ? Or is another contraint (==) needed?
      this->aks.push_back(ScaLP::newIntegerVariable("ak_" + r->getName(),count,count));
    }
    else{
      //13 in new formulation sheet
      int Ak_tmp = this->A_k[r];

      if(Ak_tmp < count ) this->aks.push_back(ScaLP::newIntegerVariable("ak_" + r->getName(),0,Ak_tmp));
      else this->aks.push_back(ScaLP::newIntegerVariable("ak_" + r->getName(),0,count));

      //store information
      this->aksIndices.insert(make_pair(r,this->aks.size()-1));
    }
  }
}

void MoovacResAwScheduler::setObjective()
{
  ScaLP::Term objective;
  //iterate over resouces to set resource part of the objective
  for(auto it2 = this->resourceModel.resourcesBegin(); it2 != this->resourceModel.resourcesEnd(); ++it2){
    Resource* r = *it2;
    //skip unlimited
    if(r->isUnlimited()== true) continue;

    ScaLP::Term ScaLPSum;

    //iterate over hardware cost types
    for(auto it = this->target.getElements().begin(); it != this->target.getElements().end(); ++it){
      std::string element = it->first;
      double constraint = it->second;

      double costs = r->getHardwareCost(element);
      //skip iff costs are 0
      if(costs == 0.0f) continue;
      else {
        ScaLP::Term term = this->aks[this->aksIndices[r]]*(costs/constraint);
        ScaLPSum += term;
      }
    }

    objective += ScaLPSum;
  }

  //set resource part of the objective
  //lambda is used for weighting between latency and resource minimization
  objective = (1-lambda)*objective;

  //use supersink node for latency minimization
  ScaLP::Variable supersink = ScaLP::newIntegerVariable("supersink",0,ScaLP::INF());
  for(auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it){
    Vertex* v = *it;

    this->solver->addConstraint(supersink - this->ti[this->tIndices[v]] - this->resourceModel.getVertexLatency(v) >= 0);
  }

  //set latency part of the objective
  //lambda is used for weighting between latency and resource minimization
  objective += lambda*supersink;

  //set overall objective using minimize
  this->solver->setObjective(ScaLP::minimize(objective));
}

void MoovacResAwScheduler::setGeneralConstraints()
{
  //5 in moovac paper
  for(std::set<Edge*>::iterator it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it) {
    Edge* e = *it;
    Vertex* src = &(e->getVertexSrc());
    unsigned int srcTVecIndex = this->tIndices[src];
    Vertex* dst = &(e->getVertexDst());
    unsigned int dstTVecIndex = this->tIndices[dst];

    this->solver->addConstraint(ti[srcTVecIndex] - ti[dstTVecIndex] + this->resourceModel.getVertexLatency(src) + e->getDelay() - this->II*(e->getDistance()) <= 0);
  }
}

void MoovacResAwScheduler::setModuloAndResourceConstraints()
{
  this->mi.resize(0);

  for(std::list<Resource*>::iterator it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); ++it) {
    Resource* r = *it;
    if(r->isUnlimited()==true) continue;

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
      //18 in moovac paper
      int rvecIndex = this->rIndices.at(v1);
      //19 in moovac paper
      m_vector.push_back(ScaLP::newIntegerVariable("m_" + std::to_string(v1->getId()),0,10000));
      //20 in moovac paper
      y_vector.push_back(ScaLP::newIntegerVariable("y_" + std::to_string(v1->getId()),0,10000));

      //13 in moovac paper
      this->solver->addConstraint(this->ti[tIndex] - y_vector.back()*((int)this->II) - m_vector.back() == 0);
      //11 in new paper sheet
      this->solver->addConstraint(this->ri[rvecIndex] + 1 - this->aks[this->aksIndices[r]] <= 0);
      //15 in moovac paper
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
            //6 in moovac paper
            this->solver->addConstraint(eps_matrix[j][k] + eps_matrix[k][j] <= 1);
            //12 in moovac paper
            this->solver->addConstraint(eps_matrix[j][k] + eps_matrix[k][j] + mu_matrix[j][k] + mu_matrix[k][j] >= 1);
          }

          if(k!=j) {
            int Ak_temp;
            if(verOfRes.size() < this->A_k[r]) Ak_temp = verOfRes.size();
            else Ak_temp = this->A_k[r];
            pair<const Vertex*, const Vertex*> vPair = corrVerticesMatrix[j][k];
            //4 in new sheet
            this->solver->addConstraint(this->ri[this->rIndices[vPair.first]] - this->ri[this->rIndices[vPair.second]]
                                        - (Ak_temp*eps_matrix[j][k]) + Ak_temp >= 1);
            //5 in new sheet
            this->solver->addConstraint(this->ri[this->rIndices[vPair.first]] - this->ri[this->rIndices[vPair.second]]
                                        - (Ak_temp*eps_matrix[j][k]) <= 0);
            //9 in moovac paper
            this->solver->addConstraint(mu_matrix[j][k] + mu_matrix[k][j]<= 1);
            //10 in moovac paper
            this->solver->addConstraint(m_vector[j]-m_vector[k] - (this->II*mu_matrix[j][k]) + this->II >= 1);
            //11 in moovac paper
            this->solver->addConstraint(m_vector[j]-m_vector[k] - (this->II*mu_matrix[j][k]) <= 0);
          }
        }
      }
    }
  }
}

void MoovacResAwScheduler::getAk() {
  for(auto it = this->resourceModel.resourcesBegin(); it != resourceModel.resourcesEnd(); ++it){
    Resource* r = *it;
    if(r->isUnlimited() == true) continue;
    //skip when resource is in model which is never used
    if(this->resourceModel.getNumVerticesRegisteredToResource(r) == 0 ) continue;

    //calculate minimum costs of all other resources
    map<std::string, double> minCosts;

    for(auto it2 = this->resourceModel.resourcesBegin(); it2 != this->resourceModel.resourcesEnd(); ++it2){
      Resource* r_it = *it2;
      //skip the resource itself
      //skip unlimited
      if(r == r_it || r_it->isUnlimited() == true) continue;

      //add resource costs of one time implementation
      for(auto it3 = r_it->getHardwareCosts().begin(); it3 != r_it->getHardwareCosts().end(); ++it3){
        std::string costName = it3->first;
        double resCost = it3->second;

        //check whether costs already in map or new
        if(minCosts.find(costName) == minCosts.end()){
          minCosts.insert(*it3); //not found, insert new
        } else {
          minCosts[costName] += resCost; //found, add to existing
        }
      }
    }

    //calculate remaining resources for this resource
    map<std::string, double> remainingSpace;

    //handle case when only limited resource is provided
    if(minCosts.size() == 0){
      remainingSpace = this->target.getElements();
    }

    //handle case when more limited resource are provided
    for(auto it2 = minCosts.begin(); it2 != minCosts.end(); ++it2){
      if(this->target.getElement(it2->first) - minCosts[it2->first] < 0)
        throw Exception("MoovacResAwScheduler.getAk: Error negative space detected for hardware element " + it2->first);
      //add difference to remaining space map
      remainingSpace.insert(make_pair(it2->first, this->target.getElement(it2->first) - minCosts[it2->first]));
    }

    //finally, calculate maximal possible number of hardware units for this resource
    int Ak = 0;
    for(auto it2 = r->getHardwareCosts().begin(); it2 != r->getHardwareCosts().end(); ++it2){
      std::string costName = it2->first;
      double resCost = it2->second;
      //skip if no costs for this element
      if(resCost == 0.0f) continue;

      int unitsFit = remainingSpace[costName] / resCost;

      if(unitsFit < 1){
        cout << "MoovacResAwScheduler.getAk: Error for " << costName << " of " << r->getName() << endl;
        cout << "MoovacResAwScheduler.getAk: Error " << remainingSpace[costName] << " / " << resCost << " < 1 " << endl;
        throw Exception("MoovacResAwScheduler.getAk: Error no space left allocating one hardware unit of resource " + r->getName());
      }
      if(Ak == 0) Ak = unitsFit;
      else if(unitsFit < Ak) Ak = unitsFit;
    }
    if(Ak == 0) throw Exception("MoovacResAwScheduler.getAk: Error maximal resource allocation of 0 determined: " + r->getName());

    this->A_k.insert(make_pair(r,Ak));
  }
}

  void MoovacResAwScheduler::scheduleIteration() {
    cout << "Starting RAMS ILP-based modulo scheduling with II " << this->II << endl;
    this->optimalResult = false;
    this->resetContainer();
    this->setUpSolverSettings();
    this->constructProblem();
    this->setSolverTimeout(solverTimeout);

    if(this->writeLPFile == true)
    {
      this->solver->writeLP(to_string(this->II));
    }

    //timestamp
    startTimeTracking();
    //solve
    stat = this->solver->solve();
    //timestamp
    endTimeTracking();

    if(stat == ScaLP::status::OPTIMAL || stat == ScaLP::status::FEASIBLE || stat == ScaLP::status::TIMEOUT_FEASIBLE)
    {
      this->scheduleFound = true;
    }
    if(stat == ScaLP::status::TIMEOUT_INFEASIBLE)
    {
      this->timeouts++;
    }
    if(stat == ScaLP::status::OPTIMAL && timeouts == 0)
    {
      this->optimalResult = true;
    }

    if(this->fullDSE == true)
    {
      cout << "Finished RAMS ILP-based modulo scheduling with II " << this->II << " status " << stat << endl;
    }

    if(scheduleFound == false)
    {
      (this->II)++;
    }

    if(scheduleFound == true and this->fullDSE == false)
    {
      return;
    }
    if(scheduleFound == true and this->fullDSE == true) {
      //store info about the result quality when DSE is active
      if (this->fullDSE == true) this->dseResultOptimal.insert(make_pair(this->II, this->optimalResult));

      this->r = this->solver->getResult();

      //display resource allocation here during developement
      for (auto it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); ++it) {
        Resource *r = *it;
        if (r->isUnlimited() == true) continue;
        int allocation = this->r.values[this->aks[this->aksIndices[r]]];
        cout << "Allocated units for resource: " << r->getName() << ": " << allocation << endl;
        r->setLimit(allocation);
      }

      //store determined schedule and allocation for this II
      //continue design space exploration in resource aware modulo scheduling afterwards
      this->storeScheduleAndAllocation();

      //continue with next II iff there are still resource to be saved
      // (at least one resource got more than 1 unit allocated in hardware)
      if (this->DSEfinshed == true) {
        return;
      }
    }
  }
}