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

#ifdef USE_XERCESC
#pragma once

#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/framework/StdOutFormatTarget.hpp>
#include <xercesc/framework/LocalFileFormatTarget.hpp>

#include "Writer.h"
#include "../../ResourceModel.h"
#include "../../Graph.h"

using namespace xercesc;

namespace HatScheT {


class XMLResourceWriter : public Writer {
public:
  XMLResourceWriter(std::string path, ResourceModel* rm);
  ~XMLResourceWriter();

  virtual void write();

private:
  /*!
   * save pointer to used resource model
   */
  ResourceModel* rm;
  /*!
   * write the xml file with the generated info
   */
  void DoOutput2File(DOMDocument*, XMLCh*);
};

}

#endif