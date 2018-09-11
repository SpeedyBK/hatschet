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
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/utility/Utility.h>

namespace HatScheT
{
RationalIIScheduler::RationalIIScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist)
: SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist)
{
  this->consideredTimeSteps = 0;
  this->moduloClasses = 0;
  this->consideredModuloCycle = 0;
  this->uniformSchedule = false;
}

void RationalIIScheduler::fillIIVector()
{
  this->II_vector.clear();

  for(unsigned int i = 0; i < this->moduloClasses; i++) {
    if(i==0) II_vector.push_back(ScaLP::newIntegerVariable("II_" + std::to_string(i+1),0,0));
    else II_vector.push_back(ScaLP::newIntegerVariable("II_" + std::to_string(i+1),i,this->consideredModuloCycle-1));
  }
}

void RationalIIScheduler::setObjective()
{

}

void RationalIIScheduler::constructProblem()
{
  //case unlimited
  if(this->maxLatencyConstraint == -1){
    this->maxLatencyConstraint = this->g.getNumberOfVertices() * ( this->resourceModel.getMaxLatency() + 1);
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


    for(unsigned int j = 0; j < this->moduloClasses; j++) {
      if(j==0) {
        this->solver->addConstraint(t_matrix[j][srcTVecIndex] + this->resourceModel.getVertexLatency(src) - t_matrix[j][dstTVecIndex] -
                                    e->getDistance() * (this->consideredModuloCycle - II_vector.back())
                                    + e->getDelay() <= 0);
      }
      else {
        this->solver->addConstraint(t_matrix[j][srcTVecIndex] + this->resourceModel.getVertexLatency(src) - t_matrix[j][dstTVecIndex] -
                                    e->getDistance() * (II_vector[j] - II_vector[j-1])
                                    + e->getDelay() <= 0);
      }
    }
  }

  //limit max latency
  for(auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it){
    Vertex* v = *it;

    if(this->uniformSchedule==true) {
      this->solver->addConstraint(t_matrix[0][this->tIndices.at(v)] + this->resourceModel.getVertexLatency(v) <=
                                  this->maxLatencyConstraint);
    }
    else{
      for(int i = 0; i < t_matrix.size(); i++){
        this->solver->addConstraint(t_matrix[i][this->tIndices.at(v)] + this->resourceModel.getVertexLatency(v) - II_vector[i] <=
                                    this->maxLatencyConstraint);
      }
    }
  }

  //distinguish IIs
  for(unsigned int i = 0; i < II_vector.size()-1; i++) {
    this->solver->addConstraint(II_vector[i] - II_vector[i+1]  + 1 <= 0);
  }
}

void RationalIIScheduler::printScheduleToConsole()
{
  cout << "----" << "modClasses: " << this->moduloClasses << " modCycle: "
       << this->consideredModuloCycle << " maxLat: " << this->maxLatencyConstraint <<  " timeSteps: " << this->consideredTimeSteps << endl;

  cout << "----" << "Found IIs: ";
  for(unsigned int i = 0; i < II_vector.size(); i++)
  {
    this->foundIIs.push_back((unsigned int)(lround(r.values[II_vector[i]])));
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
  lat = this->consideredModuloCycle - (unsigned int)(lround(r.values[II_vector[II_vector.size()-1]]));
  latVector.push_back(lat);
  cout << lat << "  ";

  cout << endl;

  if ( std::equal(latVector.begin() + 1, latVector.end(), latVector.begin()) )
  {
    cout << "----" << "Fixed Latency Modulo Schedule Found! Modulo: " << latVector[0];
    cout << endl;
  }

  std::setprecision(6);
  cout << "----" << "Throughput: " << ((double)II_vector.size())/((double)this->consideredModuloCycle) << endl;

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
  cout << "Printing modulo " << this->consideredModuloCycle << " start times" << endl;

  for(unsigned int i = 0; i < t_matrix.size(); i++)
  {
    for(unsigned int j = 0; j < t_matrix[i].size(); j++)
    {
      if(j !=0) cout << ",";
      cout << "(" << t_matrix[i][j] << "," << (unsigned int)(lround(r.values[t_matrix[i][j]])) % (this->consideredModuloCycle) << ")";
    }

    cout << endl << "-" << endl;
  }

  cout << "-------" << endl;
}

void RationalIIScheduler::schedule()
{
  this->scheduleFound = false;
  this->solver->reset();
  //experimental
  if(this->maxLatencyConstraint > this->consideredModuloCycle) this->consideredTimeSteps = 2*this->maxLatencyConstraint + 2;
  else this->consideredTimeSteps = 2*this->consideredModuloCycle + 2;

  if(this->consideredTimeSteps <= 0) {
    throw HatScheT::Exception("RationalIIScheduler.schedule : consideredTimeSteps == 0! Scheduling not possible!");
  }

  if(this->moduloClasses <= 0) {
    throw HatScheT::Exception("RationalIIScheduler.schedule : moduloClasses == 0! Scheduling not possible!");
  }

  if(this->consideredModuloCycle <= 0) {
    throw HatScheT::Exception("RationalIIScheduler.schedule : consideredModuloCycle == 0! Scheduling not possible!");
  }

  this->fillTMaxtrix();
  this->fillIIVector();

  this->constructProblem();
  this->setObjective();

  cout << "Start Rational II scheduler: " << this->moduloClasses << " / " << this->consideredModuloCycle << endl;
  if(this->consideredModuloCycle==6 && this->moduloClasses==3) this->solver->writeLP("3_6.lp");

  stat = this->solver->solve();
  cout << "Solution ScaLP status: " << stat << endl;
  if(stat==ScaLP::status::FEASIBLE || stat==ScaLP::status::OPTIMAL || stat==ScaLP::status::TIMEOUT_FEASIBLE) {
    r = this->solver->getResult();
    //this->printScheduleToConsole();
    this->scheduleFound = true;
    this->fillSolutionStructure();
  }
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

void RationalIIScheduler::fillSolutionStructure() {
  //reset possible old values
  this->startTimeVector.resize(0);
  this->IIs.resize(0);

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

    this->startTimeVector.push_back(tempMap);
  }

  //store differences (in clock cycles) between the scheduled samples
  for(int i = 0; i < this->II_vector.size()-1; i++){
    ScaLP::Variable svTemp1 = this->II_vector[i];
    ScaLP::Variable svTemp2 = this->II_vector[i+1];
    int IITimeDiff = this->r.values[svTemp1] - this->r.values[svTemp2];
    this->IIs.push_back(IITimeDiff);
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

      for(unsigned int j = 0; j < this->moduloClasses; j++) {
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

    //iterate overe time steps
    for(unsigned int j = 0; j < this->consideredTimeSteps+1; j++) {
      ScaLP::Term tiaSum;
      bool b = 0;

      //iterate over nodes of constraint resource (1.dim of container)
      for(unsigned int k = 0; k < resourceVarContainer.size(); k++) {
        //iterate over considered modulo classes (2.dim of container)
        for(unsigned int l = 0; l < resourceVarContainer[k].size(); l++) {
          for(unsigned int m = 0; m < this->consideredTimeSteps+1; m++) {
            if(m % (this->consideredModuloCycle) == j) {
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
  this->t_matrix.clear();

  //i modulo classes considered
  for(unsigned int i = 0; i < this->moduloClasses; i++) {
    vector<ScaLP::Variable> t_vector;

    //j vertices
    for(auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it){
      Vertex* v = *it;
      int id = v->getId();

      if(i==0) t_vector.push_back(ScaLP::newIntegerVariable("t'" + std::to_string(i) + "_" + std::to_string(id),i,this->maxLatencyConstraint -  this->resourceModel.getVertexLatency(v)));
      else t_vector.push_back(ScaLP::newIntegerVariable("t'" + std::to_string(i) + "_" + std::to_string(id),i,this->consideredTimeSteps*(i+1) + i*1));

      this->tIndices.insert(make_pair(v,t_vector.size()-1));
    }

    t_matrix.push_back(t_vector);
  }
}

}
