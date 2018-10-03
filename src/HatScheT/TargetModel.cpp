/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Patrick Sittel (sittel@uni-kassel.de)

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

#include "TargetModel.h"
#include "HatScheT/utility/Exception.h"

namespace HatScheT
{

Target::Target()
{
  this->name = "";
}

void Target::addConstraint(string constraint, double limit)
{
  this->constraints.insert(make_pair(constraint,limit));
}

bool Target::constraintExists(string c)
{
  if ( this->constraints.find(c) == this->constraints.end() ) {
    return false;
  }
  return true;
}

double Target::getConstraint(string c)
{
  if(this->constraintExists(c) == false){
    throw HatScheT::Exception("HardwareTargetBase::getConstraint: No " + c + " Constraints found! Plz use the setConstraint method first!");
  }
  return this->constraints[c];
}

}