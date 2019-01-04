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

#include <HatScheT/Graph.h>
#include <vector>

namespace HatScheT
{
/*!
 * \brief The Occurrence class This class can be used for describing occurrences of subgraphs in the Graph g
 */
class Occurrence
{
public:
  /*!
   * \brief Occurrence
   * \param g
   */
  Occurrence(Graph *g);
  /*!
   * \brief addEdge try to add an edge to the occurrence. It has to be a new and connected Edge.
   * \param e
   * \return
   */
  bool addEdge(const Edge* e);
  /*!
   * \brief getEdges
   * \return
   */
  std::vector<const Edge*> getEdges() const {return this->edges;}
  /*!
   * \brief getVertices
   * \return
   */
  std::vector<Vertex*> getVertices() const {return this->vertices;}
  /*!
   * \brief getGraph return the graph the occurrence is in
   * \return
   */
  Graph* getGraph() const {return this->g;}
  /*!
   * \brief operator << overloaded stream operator for cout
   * \param os
   * \param o
   * \return
   */
  friend ostream& operator<<(ostream& os, const Occurrence& o);
  /*!
   * \brief edgeIsNew
   * \param e
   * \return
   */
  bool edgeIsNew(const Edge* e);
  /*!
   * \brief vertexIsNew
   * \param v
   * \return
   */
  bool vertexIsNew(Vertex* v);
protected:

private:
  /*!
   * \brief isConnected determine whether the edge is connected to the already existing edges in g
   * \param e
   * \return
   */
  bool isConnected(const Edge* e);
  /*!
   * \brief g this Occurrence occurs in the graph g
   */
  Graph* g;
  /*!
   * The container for edges
   */
  std::vector<const Edge*> edges;
  /*!
   * The container for vertices
   */
  std::vector<Vertex*> vertices;
};

}

