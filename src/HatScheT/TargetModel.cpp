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

void Target::addElement(string element, double limit)
{
  this->elements.insert(make_pair(element,limit));
}

bool Target::elementExists(string e)
{
  if ( this->elements.find(e) == this->elements.end() ) {
    return false;
  }
  return true;
}

double Target::getElement(string e)
{
  if(this->elementExists(e) == false){
    throw HatScheT::Exception("HardwareTargetBase::getElement: No " + e + " element found! Plz use the setElement method first!");
  }
  return this->elements[e];
}

}