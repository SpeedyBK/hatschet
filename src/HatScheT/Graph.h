/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Martin Kumm, Patrick Sittel ({kumm, sittel}@uni-kassel.de)

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

#include "Vertex.h"
#include "Edge.h"
#include <map>
#include <set>

namespace HatScheT
{

typedef std::map<unsigned,Vertex*> vertex_t;
typedef std::map<unsigned,Edge*> edge_t;

/*!
 * \brief The Graph class represents all the data dependencies using a graph of vertices and edges.
 *
 * The Graph class is a very simple and lightweight implementation of a graph. Each vertex and edge is assigned to a unique int value.
 * Although the id's are assigned in a sequence, there is no guarantee that all id's exist.
 *
 * Note that neither the Graph nor the vertices contain latency information - therefore a Graph must always be accompanied
 * by a ResourceModel when scheduling it.
 *
 * \sa Vertex
 *     Edge
 *     ResourceModel
 */
class Graph
{
public:
  Graph();
  ~Graph();
  /*!
   * copy constructor is forbidden for this class
   */
   Graph(const Graph&) = delete;
  /*!
   * Creates a vertex, assigns a non-existing id and inserts it in the graph
   *G
   * \return The vertex
   */
  Vertex& createVertex();
  /*!
   * Creates a vertex, inserts it in the graph
   *
   * \param The id of the vertex
   * \return The vertex
   */
  Vertex& createVertex(int id);
  /*!
   * @brief creates an edge in the graph
   * @param Vsrc The source vertex
   * @param Vdst The destination vertex
   * @param distance The distance (=no of registers) on an edge
   * @param dependencyType Enum to distinguish dependencies and data edges
   * @return A reference to the created edge
   */
  Edge &createEdge(Vertex &Vsrc, Vertex &Vdst, int distance=0, Edge::DependencyType dependencyType=Edge::Data);
  /*!
   * \brief operator <<
   * \param os
   * \param g
   * \return
   */
  friend ostream& operator<<(ostream& os, const Graph& g);
  /*!
   * \brief getNumberOfVertices
   * \return
   */
  unsigned int getNumberOfVertices()
  {
    return this->vertices.size();
  }
  /*!
   * \brief getNumberOfEdges
   * \return
   */
  unsigned int getNumberOfEdges()
  {
    return this->edges.size();
  }
  /*!
   * \brief isEmpty test whether this graph has vertices
   * \return
   */
  bool isEmpty();
  /*!
   * \brief isSourceVertex
   * \param v
   * \return
   */
  bool isSourceVertex(Vertex* v);
  /*!
   * \brief getVertexById
   * \param id
   * \return
   */
  Vertex& getVertexById(int id) const;
  /*!
   * \brief getEdge
   * \param srcV
   * \param dstV
   * \return
   */
  Edge& getEdge(const Vertex* srcV, const Vertex* dstV) const;
  /*!
   * \brief getSubsequentVertices get the subsequent vertices of v
   * \param v
   * \return
   */
  set<Vertex*> getSubsequentVertices(const Vertex* v) const;
  /*!
   * \brief getProceedingVertices get the proceeding vertices of v
   * \param v
   * \return
   */
  set<Vertex*> getPreceedingVertices(const Vertex* v) const;
  /*!
   * \brief setName
   * \param s
   */
  void setName(string s){this->name = s;}
  const string& getName() const {return this->name;}
  /*!
   * \brief verticesBegin iterate over vertices
   * \return
   */
  const std::set<Vertex*>::iterator verticesBegin()
  {
      return vertices.begin();
  }
  /*!
   * \brief verticesEnd iterate over vertices
   * \return
   */
  const std::set<Vertex*>::iterator verticesEnd()
  {
      return vertices.end();
  }
  /*!
   * \brief edgesBegin iterate over edges
   * \return
   */
  const std::set<Edge*>::iterator edgesBegin()
  {
      return edges.begin();
  }
  /*!
   * \brief edgesEnd iterate over edges
   * \return
   */
  const std::set<Edge*>::iterator edgesEnd()
  {
      return edges.end();
  }

  // The following classes and functions are experimental and for testing this concept.
  class Edges
  {
    public:
    Edges(const std::set<Edge*>::iterator& begin, const std::set<Edge*>::iterator& end)
      : b(begin), e(end)
    {
    }
    std::set<Edge*>::iterator begin() { return b; }
    std::set<Edge*>::iterator end() { return e; }
    std::set<Edge*>::iterator b;
    std::set<Edge*>::iterator e;
  };

  Edges Edges()
  {
    return {edges.begin(),edges.end()};
  }

  class Vertices
  {
    public:
    Vertices(const std::set<Vertex*>::iterator& begin, const std::set<Vertex*>::iterator& end)
      : b(begin), e(end)
    {
    }
    std::set<Vertex*>::iterator begin() { return b; }
    std::set<Vertex*>::iterator end() { return e; }
    std::set<Vertex*>::iterator b;
    std::set<Vertex*>::iterator e;
  };

  Vertices Vertices()
  {
    return {vertices.begin(),vertices.end()};
  }

protected:
  /*!
   * The container for vertices
   */
  std::set<Vertex*> vertices;

  /*!
   * The container for edges
   */
  std::set<Edge*> edges;

  /*!
   * This int is used to track the next valid vertex id
   */
  int maxVertexId;

  /*!
   * This int is used to track the next valid edge id
   */
  int maxEdgeId;
  /*!
   * \brief name
   */
  string name;
};

}
