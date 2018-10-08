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

#include "MoovacResAwScheduler.h"
#include <HatScheT/utility/Utility.h>

namespace HatScheT
{

MoovacResAwScheduler::MoovacResAwScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist, Target& target)
        : MoovacScheduler(g, resourceModel, solverWishlist), target(target) {
  if(Utility::resourceModelAndTargetValid(resourceModel,target) == false){
    throw HatScheT::Exception("MoovacResAwScheduler.MoovacResAwScheduler: ERROR Resource Model and Hardware Target are not corresponding!");
  }

  this->minII = this->computeMinII(&g,&resourceModel);
  this->maxII = this->computeMaxII(&g,&resourceModel);
  if (this->minII >= this->maxII) this->maxII = this->minII+1;
  this->SLMax = 0;
}

void MoovacResAwScheduler::schedule()
{
  this->totalTime = 0;
  this->II = this->minII;

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

    stat = this->solver->solve();

    if(stat == ScaLP::status::OPTIMAL || stat == ScaLP::status::FEASIBLE || stat == ScaLP::status::TIMEOUT_FEASIBLE) this->scheduleFound = true;
    if(stat == ScaLP::status::TIMEOUT_INFEASIBLE) timeoutOccured = true;
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
    cout << "Passed maxII boundary! No modulo schedule identified by Moovac!" << endl;
    this->II = -1;
  }
}

void MoovacResAwScheduler::constructProblem()
{
  this->setMaxLatency();
}

void MoovacResAwScheduler::setObjective()
{

}

void MoovacResAwScheduler::setGeneralConstraints()
{

}

}