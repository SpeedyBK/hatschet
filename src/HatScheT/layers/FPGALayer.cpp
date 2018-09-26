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

namespace HatScheT {

FPGALayer::FPGALayer(FPGAVendor vendor) {
  this->vendor = vendor;
}

XilinxFPGA::XilinxFPGA(FPGAVendor vendor) : FPGALayer(vendor) {
  this->family = "";
  this->LUTs = -1;
  this->Slices = -1;
  this->DSPs = -1;
  this->BRAMs = -1;
}

}