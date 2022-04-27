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

#include "Vertex.h"
#include "Edge.h"
#include <list>
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
   * \return the number of vertices in this graph
   */
  unsigned int getNumberOfVertices()
  {
    return this->vertices.size();
  }
  /*!
   * \return the number of edges in this graph
   */
  unsigned int getNumberOfEdges()
  {
    return this->edges.size();
  }
  /*!
   * \return whether this graph has vertices
   */
  bool isEmpty();
  /*!
   * \brief determines whether the given vertex is a source concerning the graph, i.e. has no incoming edges
   * \param v the vertex to inspect
   * \return true if v has no incoming edges, false otherwise
   */
  bool isSourceVertex(Vertex* v);
  /*!
   * \brief determines whether the given vertex is a sink concerning the graph, i.e. has no outgoing edges
   * \param v the vertex to inspect
   * \return true if v has no outgoing edges, false otherwise
   */
  bool isSinkVertex(Vertex* v);
  /*!
   * \brief looks up the vertex with the given ID
   * \param id the ID to find
   * \return the vertex matching the given ID
   * \throws Exception if no vertex in the graph has the given ID
   */
  Vertex& getVertexById(int id) const;
  /*!
   * \brief looks up the vertex with the given name
   * \param n the name to find
   * \return the vertex matching the given name
   * \throws Exception if no vertex in the graph has the given name
   */
  Vertex& getVertexByName(std::string n) const;
  /*!
   * \brief looks up the edge between the given vertices
   * IMPORTANT NOTE: This does not support multiple edges between vertices
   * REMOVED / UNSUPPORTED CODE
   * \param srcV the source vertex
   * \param dstV the destination vertex
   * \return the edge from `srcV` to `dstV`
   * \throws Exception if no matching edge was found
   */
  //Edge& getEdge(const Vertex* srcV, const Vertex* dstV) const;
  /*!
   * \brief looks up the edges between the given vertices
   * IMPORTANT NOTE: This supports multiple edges between vertices
   * @param srcV
   * @param dstV
   * @return
   */
  std::list<const Edge* > getEdges(const Vertex* srcV, const Vertex* dstV) const;
  /*!
   * looks up the edges for a specific id
   * @param id unique edge id
   * @return the requested edge
   * \throws Exception if no matching edge was found
   */
  Edge& getEdge(int id) const;
  /*!
   * check whether there is already an edge between these vertices
   * double edges are not supported yet,
   * because it is currently not possible to call a getter function for those kinds of edges in a graph
   * @param srcV
   * @param dstV
   * @return
   */
  bool edgeExists(const Vertex* srcV, const Vertex* dstV);
  /*!
   * \brief determines `v`'s successor vertices, i.e. the destination vertices of `v`'s outgoing edges
   * \param v the reference vertex
   * \return a set containing `v`'s successors
   */
  set<Vertex*> getSuccessors(const Vertex *v) const;
  /*!
   * \brief determines `v`'s predecessor vertices, i.e. the source vertices of `v`'s incoming edges
   * \param v the reference vertex
   * \return a set containing `v`'s predecessors
   */
  set<Vertex*> getPredecessors(const Vertex *v) const;
  /*!
   * \brief sets the graph name
   * \param s the new name
   */
  void setName(string s){this->name = s;}
  /*! \return the graph name */
  const string& getName() const {return this->name;}
  /*!
   * \return iterator to beginning of vertices
   */
  const std::set<Vertex*>::iterator verticesBegin()
  {
      return vertices.begin();
  }
  /*!
   * \return iterator to end of vertices
   */
  const std::set<Vertex*>::iterator verticesEnd()
  {
      return vertices.end();
  }
  /*!
   * \return iterator to beginning of vertices
   */
  const std::set<Edge*>::iterator edgesBegin()
  {
      return edges.begin();
  }
  /*!
   * \return iterator to end of vertices
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
    Vertices(const std::set<Vertex*>::iterator& begin, const std::set<Vertex*>::iterator& end) : b(begin), e(end) {}

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
   * determine the largest of all vertices
   * @return
   */
  int getMaxVertexId();
  /*!
   * \brief The container for vertices
   */
  std::set<Vertex*> vertices;

  /*!
   * \brief The container for edges
   */
  std::set<Edge*> edges;

  /*!
   * \brief This int is used to track the next valid vertex ID
   */
  int maxVertexId;
  /*!
   * \brief The graph's name
   */
  string name;
};

}
