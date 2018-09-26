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

#include "HatScheT/base/HardwareTargetBase.h"

namespace HatScheT {

enum FPGAVendor {XILINX};
/*!
 * The FPGA layer class defines a layer between the HardwareTargetBase and FPGA classes for different vendors
 */
class FPGALayer : public HardwareTargetBase {

public:
    /*!
     * constructor, a vendor for an fpga is required
     * Dont use this class. Use the specific vendor classes, e.g. XilinxFPGA
     * @param vendor
     */
    FPGALayer(FPGAVendor vendor);
    ~FPGALayer(){};
    /*!
     * get the name of the vendor
     * @return
     */
    FPGAVendor getVendor() const {return this->vendor;}
protected:
    /*!
     * each fpga has to have a vendor
     */
    FPGAVendor vendor;
};

/*!
 * The XilinxFPGA is a hardware target class
 */
class XilinxFPGA : public FPGALayer {
public:
  /*!
   * constructor
   */
  XilinxFPGA(FPGAVendor vendor);
  ~XilinxFPGA(){};
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