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

ostream& operator<<(ostream& os, const Target& t){
  os << "------------------------------------------------------------------------------------" << endl;
  os << "------------------------------ Hardware Target Model -------------------------------" << endl;
  os << "------------------------------------------------------------------------------------" << endl;

  os << "Vendor: " << t.vendor << endl;
  os << "Familiy: " << t.family << endl;
  os << "Name: " << t.name << endl;
  os << "------------------------------------------------------------------------------------" << endl;

  for(auto it=t.elements.begin();it!=t.elements.end(); ++it){
    os << it->first << ": " << to_string(it->second) << endl;
  }

  os << "------------------------------------------------------------------------------------" << endl;

  return os;
}

bool Target::isEmpty() {
  if(this->elements.size() == 0) return true;
  return false;
}

void Target::addElement(string element, double limit) {
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