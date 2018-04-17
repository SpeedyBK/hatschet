/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Patrick Sittel (sittel@uni-kassel.de)
  All rights reserved.
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
  bool addEdge(Edge* e);
  /*!
   * \brief getEdges
   * \return
   */
  std::vector<Edge*> getEdges() const {return this->edges;}
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
  bool edgeIsNew(Edge* e);
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
  bool isConnected(Edge* e);
  /*!
   * \brief g this Occurrence occurs in the graph g
   */
  Graph* g;
  /*!
   * The container for edges
   */
  std::vector<Edge*> edges;
  /*!
   * The container for vertices
   */
  std::vector<Vertex*> vertices;
};

}

