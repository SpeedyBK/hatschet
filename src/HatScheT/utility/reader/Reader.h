/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)

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

#include "HatScheT/Graph.h"
#include "HatScheT/ResourceModel.h"
#include "HatScheT/base/HardwareTargetBase.h"
#include "HatScheT//utility/Exception.h"

#ifdef USE_XERCESC
#include "xercesc/sax2/DefaultHandler.hpp"
#endif //USE_XERCESC

namespace HatScheT
{
/*!
 * \brief The Reader class use this class as base for reading into HatScheT
 */
class Reader
{
public:
  /*!
     * \brief graphReader
     */
  Reader();
  ~Reader();
  /*!
     * \brief readGraph this function will generate a new graph object instance
     * afterwards the respective parseGraph function of a derived class is called
     * for parsing/filling the graph object
     * \return a reference to the read Graph is returned
     */
  virtual Graph& readGraph(const char* path)=0;
  /*!
   * \brief readResourceModel this function will generate a new resourcemodel object instance
   * \param path
   * \param g
   * \return
   */
  virtual ResourceModel& readResourceModel(const char* path)=0;
  /*!
   * this function will generate a new FPGA object instance
   * @param path
   * @return
   */
  virtual XilinxFPGA& readFPGA(const char* path)=0;
protected:


};

}

