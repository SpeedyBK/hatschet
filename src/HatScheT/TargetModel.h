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
#pragma once

#include <string>
#include <map>

using namespace std;

namespace HatScheT {
/*!
 * This class is a placeholder for future attributes and functionality of hardware targets
 */
class Target {
public:
  /*!
   * constructor
   * @param name
   */
  Target();
  /*!
   * get the hardware targets name
   * @return
   */
  std::string getName(){return this->name;}
  void setName(string n){this->name = n;}
  /*!
 * get the product family
 * @return
 */
  string getFamily(){return this->family;}
  void setFamily(string f){this->family = f;}
  /*!
   * manage the vendor variable of the target
   * @return
   */
  string getVendor(){return  this->vendor;}
  void setVendor(string v){this->vendor = v;}
  /*!
   * add a generic hardware element to this target
   * e.g. "(c1,54),(cm^2, 47.34)"
   * @param constraint
   * @param limit
   */
  void addElement(string element, double limit);
  map<std::string, double>& getElements() {return this->elements;}
  double getElement(string e);
  /*!
   * check whether an element exists in the constraints map
   * @param c
   * @return
   */
  bool elementExists(string e);
protected:
  /*!
   * this container manages the hardware constraints of the target
   */
  map<std::string, double> elements;
  /*!
   * the hardware targets name
   */
  std::string name;
    /*!
   * the product family
   */
   string family;
   /*!
    * the vendor of the hardware target
    */
   string vendor;
};

}

