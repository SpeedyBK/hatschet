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
  /*!
  * manange DSP constraint of the FPGA
  * @param d
  */
  void setDSPConstraint(int d){this->addConstraint("DSPs",d);}
  int getDSPConstraint();
  /*!
  * get total No of DSPs on the FPGA
  * @return
  */
  int getTotalDSPs(){return this->totalDSPs;}
  void setTotalDSPs(int d){this->totalDSPs = d;}
protected:
  /*!
   * each fpga has to have a vendor
   */
  FPGAVendor vendor;
  /*!
  * total No of DSPs on the FPGA
  */
  int totalDSPs;
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
   * manange LUT constraint of the xilinx FPGA
   * @param l
   */
  void setLUTConstraint(int l){this->addConstraint("LUTs",l);}
  int getLUTConstraint();
  /*!
   * manange Slice constraint of the xilinx FPGA
   * @param s
   */
  void setSliceConstraint(int s){this->addConstraint("Slices",s);}
  int getSliceConstraint();

  /*!
   * manange BRAM constraint of the xilinx FPGA
   * @param b
   */
  void setBRAMConstraint(int b){this->addConstraint("BRAMSs",b);}
  int getBRAMConstraint();
  /*!
   * get total No of LUTs on the FPGA
   * @return
   */
  int getTotalLUTs(){return this->totalLUTs;}
  void setTotalLUTs(int l){this->totalLUTs = l;}
  /*!
   * get total No of Slices on the FPGA
   * @return
   */
  int getTotalSlices(){return this->totalSlices;}
  void setTotalSlices(int s){this->totalSlices = s;}
  /*!
   * get total No of BRAMs on the FPGA
   */
  int getTotalBRAMs(){return this->totalBRAMs;}
  void setTotalBRAMs(int b){this->totalBRAMs = b;}

protected:
  /*!
   * total No of LUTs on the FPGA
   */
  int totalLUTs;
  /*!
   * total No of Slices on the FPGA
   */
  int totalSlices;
  /*!
   * total No of BRAMs on the FPGA
   */
  int totalBRAMs;
};

}