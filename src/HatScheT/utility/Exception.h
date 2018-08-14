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

#include <exception>
#include <ostream>

namespace HatScheT
{

/*!
 * \brief The Exception class implements exceptions thrown by HatScheT
 */
class Exception : public std::exception
{
public:
  std::string msg;

  Exception(std::string s) :msg(s)
  {
  }

  virtual const char* what() const noexcept override;
};

std::ostream& operator<<( std::ostream& oss, HatScheT::Exception &e);


}

