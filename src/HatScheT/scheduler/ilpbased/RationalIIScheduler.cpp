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
#include <iomanip>
#include <math.h>
#include <HatScheT/scheduler/ilpbased/RationalIIScheduler.h>
#include <HatScheT/scheduler/ilpbased/ASAPILPScheduler.h>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/utility/Utility.h>
#include <HatScheT/utility/Verifier.h>

namespace HatScheT
{
RationalIIScheduler::RationalIIScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist)
: RationalIISchedulerLayer(g, resourceModel), ILPSchedulerBase(solverWishlist)
{
  this->consideredTimeSteps = 0;
  this->uniformSchedule = true;
  this->tpBuffer = 0.0f;
}

void RationalIIScheduler::resetContainer() {
  cout << "RationalIIScheduler::resetContainer: start " << endl;
  this->t_matrix.clear();
  this->t_matrix.resize(0);

  this->II_vector.clear();
  this->II_vector.resize(0);

  this->tIndices.clear();

  cout << "RationalIIScheduler::resetContainer: finished " << endl;
}

void RationalIIScheduler::fillIIVector()
{
  for(unsigned int i = 0; i < this->samples; i++) {
    if(i==0) II_vector.push_back(ScaLP::newIntegerVariable("II_" + std::to_string(i),0,0));
    else II_vector.push_back(ScaLP::newIntegerVariable("II_" + std::to_string(i),i,this->modulo-1));
  }
}

void RationalIIScheduler::setObjective()
{
  //supersink latency objective
  ScaLP::Variable supersink = ScaLP::newIntegerVariable("supersink",0,this->maxLatencyConstraint);

  for(auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it) {
    Vertex *v = *it;
    this->solver->addConstraint(supersink - t_matrix[0][this->tIndices.at(v)] + this->resourceModel.getVertexLatency(v) >= 0);
  }

  this->solver->setObjective(ScaLP::minimize(supersink));
}

void RationalIIScheduler::constructProblem()
{
  //case unlimited
  if(this->maxLatencyConstraint == -1){
    this->maxLatencyConstraint = this->g.getNumberOfVertices() * ( this->resourceModel.getMaxLatency() + 1);
    if(this->quiet==false) cout << "maxLatencyConstraint automatically set to " << this->maxLatencyConstraint << endl;
  }
  //correct limit
  else if(this->maxLatencyConstraint > 0) {
  }
  else {
    throw HatScheT::Exception("RationalIIScheduler::constructProblem: irregular maxLatencyConstraint " + to_string(this->maxLatencyConstraint));
  }

  this->setGeneralConstraints();
  this->setResourceConstraints();
  this->setModuloConstraints();
}

void RationalIIScheduler::setGeneralConstraints()
{
  //general constraints, data path dependencies
  for(std::set<Edge*>::iterator it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it) {
    Edge* e = *it;
    Vertex* src = &(e->getVertexSrc());
    unsigned int srcTVecIndex = this->tIndices[src];
    Vertex* dst = &(e->getVertexDst());
    unsigned int dstTVecIndex = this->tIndices[dst];

    for(unsigned int j = 0; j < this->samples; j++) {
      if(e->getDistance()==0){
        this->solver->addConstraint(t_matrix[j][srcTVecIndex] + this->resourceModel.getVertexLatency(src) - t_matrix[j][dstTVecIndex]
                                    - e->getDelay() <= 0);
      } else {
        if(this->uniformSchedule == true) {
          ScaLP::Term distanceIIs = this->getSampleDistanceAsTerm(e->getDistance(), j);
          this->solver->addConstraint(
            t_matrix[j][srcTVecIndex] + this->resourceModel.getVertexLatency(src) - t_matrix[j][dstTVecIndex] +
            distanceIIs - e->getDelay() <= 0);
        }
        else {
          int sampleIndex = this->getSampleIndexFromDistance(e->getDistance(), j);

          int buffer = 0;

          // !hack! solve better
          if(e->getDistance() >= this->samples) buffer = this->modulo;
          if(j < e->getDistance()) buffer = this->modulo;
          //--------

          //cout << "distance " << e->getDistance() << ", start sample " << j << " found index " << sampleIndex << ", buffer " << buffer << endl;

          this->solver->addConstraint(
            t_matrix[sampleIndex][srcTVecIndex] + this->resourceModel.getVertexLatency(src) - t_matrix[j][dstTVecIndex]
             - e->getDelay() - buffer <= 0);
        }
      }
    }
  }

  //limit max latency
  for(auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it){
    Vertex* v = *it;
      this->solver->addConstraint(t_matrix[0][this->tIndices.at(v)] + this->resourceModel.getVertexLatency(v) <=
                                  this->maxLatencyConstraint);
  }

  //distinguish IIs
  for(unsigned int i = 0; i < II_vector.size()-1; i++) {
    if(this->uniformSchedule==true) this->solver->addConstraint(II_vector[i] - II_vector[i+1]  + 1 <= 0);
  }
}

void RationalIIScheduler::printBindingToConsole() {
  Utility::printRationalIIMRT(this->startTimes, this->ratIIbindings, &this->resourceModel, this->modulo, this->latencySequence);
}

void RationalIIScheduler::printScheduleToConsole()
{
  cout << "----" << "Samples: " << this->samples << " mod: "
       << this->modulo << " maxLat: " << this->maxLatencyConstraint <<  " timeSteps: " << this->consideredTimeSteps << endl;

  cout << "----" << "Found IIs: ";
  for(unsigned int i = 0; i < II_vector.size(); i++)
  {
    cout << (unsigned int)(lround(r.values[II_vector[i]])) << "(" << II_vector[i] << ")  ";
  }
  cout << endl;
  cout << "----" << "Resulting Insertion latency: ";
  unsigned int lat;
  vector<unsigned int> latVector;
  for(unsigned int i = 0; i < II_vector.size()-1; i++)
  {
    lat = (unsigned int)(lround(r.values[II_vector[i+1]])) - (unsigned int)(lround(r.values[II_vector[i]]));
    latVector.push_back(lat);
    cout << lat << "  ";
  }
  lat = this->modulo - (unsigned int)(lround(r.values[II_vector[II_vector.size()-1]]));
  latVector.push_back(lat);
  cout << lat << "  ";

  cout << endl;

  if ( std::equal(latVector.begin() + 1, latVector.end(), latVector.begin()) )
  {
    cout << "----" << "Fixed Latency Modulo Schedule Found! Modulo: " << latVector[0];
    cout << endl;
  }

  std::setprecision(6);
  cout << "----" << "Throughput: " << ((double)II_vector.size())/((double)this->modulo) << endl;

  cout << "Printing absolut start times" << endl;

  for(unsigned int i = 0; i < t_matrix.size(); i++)
  {
    for(unsigned int j = 0; j < t_matrix[i].size(); j++)
    {
      if(j !=0) cout << ",";
      cout << "(" << t_matrix[i][j] << "," << (unsigned int)(lround(r.values[t_matrix[i][j]]))  << ")";
    }

    cout << endl << "-" << endl;
  }

  cout << "-------" << endl;
  cout << "Printing modulo " << this->modulo << " start times" << endl;

  for(unsigned int i = 0; i < t_matrix.size(); i++)
  {
    for(unsigned int j = 0; j < t_matrix[i].size(); j++)
    {
      if(j !=0) cout << ",";
      cout << "(" << t_matrix[i][j] << "," << (unsigned int)(lround(r.values[t_matrix[i][j]])) % (this->modulo) << ")";
    }

    cout << endl << "-" << endl;
  }

  cout << "-------" << endl;
}

void RationalIIScheduler::setModuloConstraints() {
  if(this->uniformSchedule==true) {
    //edges in different IIs have the same "length in time" respectively
    for (unsigned int i = 0; i < t_matrix.size() - 1; i++) {
      for (unsigned int j = 0; j < t_matrix[i].size(); j++) {
        this->solver->addConstraint(t_matrix[i + 1][j] - t_matrix[i][j] - II_vector[i + 1] + II_vector[i] == 0);
      }
    }
  }
}

int RationalIIScheduler::getSampleIndexFromDistance(int d, int startSample) {
  //immediately return startSample when requested distance was 0
  if (d == 0) return startSample;

  int sampleIndex = startSample;

  while(d>0){
    if(sampleIndex>0) {
      sampleIndex-=1;
    }
    else sampleIndex = this->samples - 1;

    d--;
  }

  return sampleIndex;
}

ScaLP::Term RationalIIScheduler::getSampleDistanceAsTerm(int d, int startIndex) {
  if(startIndex > this->II_vector.size()-1) throw Exception("RationalIIScheduler.getSampleDistanceAsTerm: out of range II_vector entry requested: " + to_string(startIndex));

  ScaLP::Term w;
  while(d>0){
    if(startIndex == 0) w = w - this->modulo + this->II_vector.back();
    else w = w - this->II_vector[startIndex] + this->II_vector[startIndex-1];

    if(startIndex>0) {
      startIndex-=1;
    }
    else if(startIndex==0) startIndex=this->II_vector.size()-1;

    d--;
  }

  return w;
}

void RationalIIScheduler::fillSolutionStructure() {
  //reset possible old values
  this->startTimesVector.resize(0);
  this->latencySequence.resize(0);

  //store schedule using standard interface if uniform schedule is true (TODO how to calc latency for non uniform)
  //if(this->uniformSchedule == true){
    for (std::set<Vertex *>::iterator it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it) {
      Vertex* v = *it;
      unsigned int index =this->tIndices.at(v);
      ScaLP::Variable svTemp = this->t_matrix[0][index];

      int startTime = this->r.values[svTemp];
      this->startTimes.insert(make_pair(v,startTime));
    }
  //}

  //store start times of the scheduled samples
  for(int i = 0; i < this->t_matrix.size(); i++) {
    std::map<Vertex*,int> tempMap;

    for (std::set<Vertex *>::iterator it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it) {
      Vertex* v = *it;
      unsigned int index =this->tIndices.at(v);
      ScaLP::Variable svTemp = this->t_matrix[i][index];

      int startTime = this->r.values[svTemp];
      tempMap.insert(make_pair(v, startTime));
    }
    this->startTimesVector.push_back(tempMap);
  }

  //store differences (in clock cycles) between the scheduled samples
  for(int i = 0; i < this->II_vector.size(); i++){
    int IITimeDiff;

    if(i==II_vector.size()-1) IITimeDiff = this->modulo - (unsigned int)(lround(r.values[II_vector[i]]));
    else IITimeDiff = (unsigned int)(lround(r.values[II_vector[i+1]])) - (unsigned int)(lround(r.values[II_vector[i]]));

    this->latencySequence.push_back(IITimeDiff);
  }
}

void RationalIIScheduler::setResourceConstraints() {
  for(auto it = this->resourceModel.resourcesBegin(); it != this->resourceModel.resourcesEnd(); ++it) {
    Resource* r = *it;
    set<const Vertex*> vSet = this->resourceModel.getVerticesOfResource(r);
    vector<vector< vector<ScaLP::Variable> > > resourceVarContainer;

    int ak = r->getLimit();
    if (ak==-1) continue;

    for (auto it = vSet.begin(); it != vSet.end(); ++it) {
      const Vertex* v = *it;
      unsigned int tIndex = this->tIndices[v];

      //declare tia-matrix
      vector< vector<ScaLP::Variable> > tia_matrix;

      for(unsigned int j = 0; j < this->samples; j++) {
        //declare tia-vector
        vector<ScaLP::Variable> tia_vector;

        ScaLP::Term weightedtSum;
        ScaLP::Term tSum;

        for(unsigned int k = 0; k < this->consideredTimeSteps+1; k++) {
          tia_vector.push_back(ScaLP::newIntegerVariable("tia'" + std::to_string(j) + "_" + std::to_string(v->getId()) + "," + std::to_string(k) ,0,1));
          weightedtSum = weightedtSum + k*tia_vector[k];
          tSum = tSum + tia_vector[k];
        }

        //restrict the time step assigned to a t
        this->solver->addConstraint(weightedtSum - t_matrix[j][tIndex] == 0);
        //each t is performed exactly one time
        this->solver->addConstraint(tSum == 1);

        tia_matrix.push_back(tia_vector);
      }

      resourceVarContainer.push_back(tia_matrix);
    }

    //iterate over time steps
    for(unsigned int j = 0; j < this->consideredTimeSteps+1; j++) {
      ScaLP::Term tiaSum;
      bool b = 0;

      //iterate over nodes of constraint resource (1.dim of container)
      for(unsigned int k = 0; k < resourceVarContainer.size(); k++) {
        //iterate over considered modulo classes (2.dim of container)
        for(unsigned int l = 0; l < resourceVarContainer[k].size(); l++) {
          for(unsigned int m = 0; m < this->consideredTimeSteps+1; m++) {
            if(m % (this->modulo) == j) {
              tiaSum = tiaSum + resourceVarContainer[k][l][m];
              b =1;
            }
          }
        }
      }

      //restrict resources used in every time step
      if(b) this->solver->addConstraint(tiaSum - ak <= 0);
    }
  }
}

void RationalIIScheduler::fillTMaxtrix()
{
  //i modulo classes considered
  for(unsigned int i = 0; i < this->samples; i++) {
    vector<ScaLP::Variable> t_vector;

    //j vertices
    for(auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it){
      Vertex* v = *it;

      if(i==0) t_vector.push_back(ScaLP::newIntegerVariable("t'" + std::to_string(i) + "_" + v->getName(),i,this->maxLatencyConstraint /*-  this->resourceModel.getVertexLatency(v)*/));
      else t_vector.push_back(ScaLP::newIntegerVariable("t'" + std::to_string(i) + "_" + v->getName(),i,this->consideredTimeSteps*(i+1) + i*1));

      this->tIndices.insert(make_pair(v,t_vector.size()-1));
    }

    t_matrix.push_back(t_vector);
  }
}

  void RationalIIScheduler::scheduleIteration() {

    if(this->g.getNumberOfVertices() > 150 and this->modulo  > 75) {
      cout << "RationalIIScheduler.schedule: WARNING skipped this iteration cause M=" << this->modulo << " ( > 75)" << endl;
      cout << "RationalIIScheduler.schedule: WARNING this scheduling formulation is not suited for large M in large graphs!" << endl;
      this->II = -1;
      this->scheduleFound=false;
      return;
    }

    if(this->maxLatencyConstraint > this->modulo) this->consideredTimeSteps = 2*this->maxLatencyConstraint + 2;
    else this->consideredTimeSteps = 2*this->modulo + 2;

    if(this->consideredTimeSteps <= 0) {
      throw HatScheT::Exception("RationalIIScheduler.schedule : consideredTimeSteps <= 0! Scheduling not possible!");
    }

    if(this->maxLatencyConstraint <= 0) {
      if(this->quiet==false) cout << "RationalIIScheduler.schedule: maxLatency not set, start identification of L"  << endl;
      //experimental
      this->maxLatencyConstraint = Utility::getCyclesOfLongestPath(&this->g,&this->resourceModel, ((double)this->modulo)/((double)this->samples)) + 1;

      this->consideredTimeSteps = 2*this->maxLatencyConstraint + 2;
      if(this->quiet==false) cout << "RationalIIScheduler.schedule: maxLatency not set, finished identification of L"  << endl;
    }

    if(this->quiet==false) cout << "RationalIIScheduler.schedule: maxLatency Constraint is  " << this->maxLatencyConstraint  << endl;
    if(this->quiet==false) cout << "RationalIIScheduler.schedule: Considered time steps " << this->consideredTimeSteps  << endl;

    //clear up and reset
    this->solver->reset();
    this->resetContainer();

    //set up new variables and constraints
    this->fillTMaxtrix();
    this->fillIIVector();
    this->constructProblem();

    //set up objective, currently asap using supersink
    this->setObjective();

    if(this->quiet==false) cout << "RationalIIScheduler.schedule: try to solve for s / m : " << this->samples << " / " << this->modulo << endl;
    //solve the current problem
    if(this->writeLPFile == true) this->solver->writeLP(to_string(this->samples) + to_string(this->modulo) + ".lp");

//    //timestamp
//    this->begin = clock();
//    //solve
//    stat = this->solver->solve();
//    //timestamp
//    this->end = clock();
//
//    //log time
//    if(this->solvingTime == -1.0) this->solvingTime = 0.0;
//    this->solvingTime += (double)(this->end - this->begin) / CLOCKS_PER_SEC;

    //timestamp
    startTimeTracking();
    //solve
    stat = this->solver->solve();
    //timestamp
    endTimeTracking();

    if(this->quiet==false) cout << "Finished solving: " << stat << endl;

    //check result and act accordingly
    if(stat==ScaLP::status::FEASIBLE || stat==ScaLP::status::OPTIMAL || stat==ScaLP::status::TIMEOUT_FEASIBLE) {
      this->r = this->solver->getResult();
      this->tpBuffer = (double)(this->samples) / (double)(this->modulo);

      this->scheduleFound = true;
      this->fillSolutionStructure();

      //determine whether rational minimum II was identified
      if(((double)this->modulo / (double)this->samples) == this->getMinII()) this->minRatIIFound = true;

      this->s_found = this->samples;
      this->m_found = this->modulo;
      this->II = (double)(this->modulo) / (double)(this->samples);
      if(this->uniformSchedule==true) this->getRationalIIBindings();

      if(this->quiet==false) {
        cout << "------------------------" << endl;
        cout << "RationalIIScheduler.schedule: Found result is " << stat << endl;
        cout << "RationalIIScheduler.schedule: this solution is s / m : " << this->samples << " / " << this->modulo
             << endl;
        cout << "RationalIIScheduler.schedule: II: " << (double) (this->modulo) / (double) (this->samples)
             << " (integer minII " << ceil((double) (this->modulo) / (double) (this->samples)) << ")" << endl;
        cout << "RationalIIScheduler.schedule: throughput: " << this->tpBuffer << endl;
        cout << "------------------------" << endl;
        this->printScheduleToConsole();
      }
    }

    else{
      if(this->quiet==false) cout << "RationalIIScheduler.schedule: no schedule found for s / m : " << this->samples << " / " << this->modulo << " ( " << stat << " )" << endl;
      this->scheduleFound = false;
      this->tpBuffer = (double)this->modulo / (double)this->samples;
    }
  }

  void RationalIIScheduler::setSolverTimeout(double timeoutInSeconds) {
      this->solverTimeout = timeoutInSeconds;
      solver->timeout = (long)timeoutInSeconds;
      if (!this->quiet)
      {
          cout << "Solver Timeout set to " << this->solver->timeout << " seconds." << endl;
      }
  }

}
