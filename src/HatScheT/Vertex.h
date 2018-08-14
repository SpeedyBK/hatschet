/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Martin Kumm, Patrick Sittel (kumm, sittel@uni-kassel.de)
  All rights reserved.
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
   * \brief getId
   * \return
   */
  int getId() const { return id; }
  /*!
   * \brief setName optional
   * \param s
   */
  void setName(string s){this->name = s;}
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
   * \brief name optional
   */
  string name;
  /*!
   * \brief id mandatory
   */
  const int id;
};
ostream& operator<<(ostream& os, const Vertex& v);
}
