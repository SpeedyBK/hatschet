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

#pragma once

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/layers/RationalIISchedulerLayer.h>
#include <vector>

namespace HatScheT
{
/*!
 * experimental: This scheduler determines a modulo schedule with uneven/rational initiation intervals
 * THIS CLASSES IS UNDER DEVELOPMENT
 */
class RationalIIScheduler : public SchedulerBase, public ILPSchedulerBase, public RationalIISchedulerLayer
{
public:
  RationalIIScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
  virtual void schedule();
  virtual int getScheduleLength();
  void setModuloCycles(int m) {this->consideredModuloCycle=m;}
  void setModuloClasses(int m ){this->moduloClasses=m;}
  void printScheduleToConsole();


  vector<std::map<Vertex*,int> >& getStartTimeVector(){return this->startTimeVector;}
  vector<int>& getIIs(){return this->IIs;}

  void setUniformScheduleFlag(bool b){this->uniformSchedule=b;}
  bool getUniformScheduleFlag(){return this->uniformSchedule;}
private:
  bool uniformSchedule;
  virtual void constructProblem();
  virtual void setObjective();
  //--------
  void setResourceConstraints();
  void setGeneralConstraints();
  void setModuloConstraints();
  void fillTMaxtrix();
  void fillIIVector();
  void fillSolutionStructure();
  //--------
  //TODO: use rational II scheduler layer attributes for this
  unsigned int moduloClasses;
  unsigned int consideredModuloCycle;
  //---------------
  unsigned int consideredTimeSteps;
  vector<int> foundIIs;

  vector<vector<ScaLP::Variable> > t_matrix;
  vector<ScaLP::Variable> II_vector;
  vector<int > IIs;
  map<const Vertex*,int> tIndices;

  vector<std::map<Vertex*,int> > startTimeVector;
};
}
