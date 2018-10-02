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

#include "FPGALayer.h"
#include "HatScheT/utility/Exception.h"

namespace HatScheT {

FPGALayer::FPGALayer(FPGAVendor vendor) {
  this->vendor = vendor;
}

XilinxFPGA::XilinxFPGA(FPGAVendor vendor) : FPGALayer(vendor) {
  this->family = "";
  this->totalLUTs = -1;
  this->totalSlices = -1;
  this->totalDSPs = -1;
  this->totalBRAMs = -1;
}

int XilinxFPGA::getLUTConstraint() {
  if ( this->constraints.find("LUTs") == this->constraints.end() ) {
    throw HatScheT::Exception("XilinxFPGA::getLUTConstraint: No LUT Constraints found! Plz use the setLUTConstraint method first!");
  } else {
    return this->constraints["LUTs"];
  }
}

int XilinxFPGA::getSliceConstraint() {
  if ( this->constraints.find("Slices") == this->constraints.end() ) {
    throw HatScheT::Exception("XilinxFPGA::getSliceConstraint: No Slice Constraints found! Plz use the setSliceConstraint method first!");
  } else {
    return this->constraints["Slices"];
  }
}


int FPGALayer::getDSPConstraint() {
  if ( this->constraints.find("DSPs") == this->constraints.end() ) {
    throw HatScheT::Exception("XilinxFPGA::getDSPConstraint: No DSP Constraints found! Plz use the setDSPConstraint method first!");
  } else {
    return this->constraints["DSPs"];
  }
}

int XilinxFPGA::getBRAMConstraint() {
  if ( this->constraints.find("BRAMs") == this->constraints.end() ) {
    throw HatScheT::Exception("XilinxFPGA::getBRAMConstraint: No BRAM Constraints found! Plz use the setBRAMConstraint method first!");
  } else {
    return this->constraints["BRAMs"];
  }
}

}