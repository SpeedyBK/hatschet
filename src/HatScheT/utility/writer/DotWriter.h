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
#include "Writer.h"
#include "../../ResourceModel.h"
#include "../../Graph.h"

namespace HatScheT
{
/*!
 * \brief The DotWriter class use this class to write your graphs to the .dot format
 */
class DotWriter : public Writer
{
public:
  /*!
   * \brief DotWriter
   * \param path provide a filename (without .dot!)
   * \param g
   * \param rm
   */
  DotWriter(std::string path, Graph* g, ResourceModel* rm);
  ~DotWriter();
  /*!
   * \brief setDisplayNames default is false
   * \param b
   */
  void setDisplayNames(bool b){this->displayNames = b;}
  /*!
   * \brief write writes to .dot
   * supported 7 different resources/colours
   */
  virtual void write();

protected:
  bool displayNames;
  /*!
   * \brief g save pointer to graph to write
   */
  Graph* g;
  /*!
   * \brief rm save pointer to used resource model
   */
  ResourceModel* rm;
};

}

