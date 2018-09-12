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

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <vector>

namespace HatScheT
{

/*!
 * currently under development and an experimental version!
 * Scheduler for ilp based asap scheduling
 */
class ASAPILPScheduler:  public SchedulerBase, public ILPSchedulerBase
{
public:
    ASAPILPScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
    virtual void schedule();

protected:
    virtual void setGeneralConstraints();
    virtual void constructProblem();
    virtual void setObjective();
    void setVectorVariables();
    void fillSolutionStructure();
    void setResourceConstraints();

    vector<ScaLP::Variable> ti;
    map<const Vertex*, unsigned int> tIndices;
    ScaLP::Result r;

    map<const Vertex*, vector<ScaLP::Variable> > binVarMap;
};

}



