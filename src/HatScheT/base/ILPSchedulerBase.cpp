/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Martin Kumm, Patrick Sittel ({kumm, sittel}@uni-kassel.de)
    Author: Julian Oppermann (oppermann@esa.tu-darmstadt.de)

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

#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/utility/Exception.h>

namespace HatScheT
{

ILPSchedulerBase::ILPSchedulerBase(std::list<std::string> solverWishlist) : solver(new ScaLP::Solver(solverWishlist))
{
  this->threads = 1;
  this->solver->threads = 1;
  this->solverQuiet = true;
  this->optimalResult = false;
  this->solverTimeout = 300;
  this->solvingTime = -1.0;
  this->totalTime = 0;
  this->writeLPFile = false;
  this->stat = ScaLP::status::UNKNOWN;
  //default false to avoid errors using SCIP and LPSolve
  this->solver->presolve=false;
}

ILPSchedulerBase::~ILPSchedulerBase()
{
  solver->reset();
  delete solver;
}

std::string ILPSchedulerBase::getSolverName()
{
  return solver->getBackendName();
}

void ILPSchedulerBase::setSolverTimeout(long timeoutInSeconds)
{
  this->solverTimeout = timeoutInSeconds;
  solver->timeout = timeoutInSeconds;
}

}
