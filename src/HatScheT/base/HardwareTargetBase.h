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

using namespace std;

namespace HatScheT {
/*!
 * This class is a placeholder for future attributes and functionality of hardware targets
 */
class HardwareTargetBase {
public:
  /*!
   * constructor
   * @param name
   */
  HardwareTargetBase();
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
protected:
  /*!
   * the hardware targets name
   */
  std::string name;
    /*!
   * the product family
   */
    string family;
};

/*!
 * The FPGA class, is a hardware target
 */
class FPGA : public HardwareTargetBase {
public:
  /*!
   * constructor
   */
  FPGA();
  ~FPGA(){};
  /*!
   * get total No of LUTs on the FPGA
   * @return
   */
  int getLUTs(){return this->LUTs;}
  void setLUTs(int l){this->LUTs = l;}
  /*!
   * get total No of Slices on the FPGA
   * @return
   */
  int getSlices(){return this->Slices;}
  void setSlices(int s){this->Slices = s;}
  /*!
   * get total No of DSPs on the FPGA
   * @return
   */
  int getDSPs(){return this->DSPs;}
  void setDSPs(int d){this->DSPs = d;}
  /*!
   * get total No of BRAMs on the FPGA
   */
  int getBRAMs(){return this->BRAMs;}
  void setBRAMs(int b){this->BRAMs = b;}

protected:
  /*!
   * total No of LUTs on the FPGA
   */
  int LUTs;
  /*!
   * total No of Slices on the FPGA
   */
  int Slices;
  /*!
   * total No of DSPs on the FPGA
   */
  int DSPs;
  /*!
   * total No of BRAMs on the FPGA
   */
  int BRAMs;
};

}

