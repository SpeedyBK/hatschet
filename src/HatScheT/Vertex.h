/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Martin Kumm, Patrick Sittel ({kumm, sittel}@uni-kassel.de)
    Author: Julian Oppermann (oppermann@esa.tu-darmstadt.de)

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
#include <iostream>
#include <string>
using namespace std;

namespace HatScheT
{

/*!
 * \brief Implementation of a vertex used in a Graph
 *
 * Vertices represent the entities (such as operation, instructions, ...) to be scheduled, but do not store their
 * latencies/delay. Instead, a vertex must be registered to use a certain Resource, which then provides the latency
 * value.
 *
 * \sa Graph
 *     Resource
 *     ResourceModel
 */
class Vertex
{
public:
  Vertex(int id);
  /*!
   * copy constructor is forbidden for this class
   */
  Vertex(const Vertex&) = delete;
  /*!
   * \return the vertex ID
   */
  int getId() const { return id; }
  /*!
   * \param s the new name
   */
  void setName(string s){this->name = s;}
  /*!
   * @return the (optional) vertex name
   */
  const string& getName() const {return this->name;}
  /*!
   * \brief < operator used for map container
   */
  bool operator<(const Vertex& vref) const
  {
    return id < vref.id;
  }

protected:
  /*!
   * \brief an optional vertex name
   */
  string name;
  /*!
   * \brief the mandatory numerical vertex ID
   */
  const int id;
};
ostream& operator<<(ostream& os, const Vertex& v);
}
