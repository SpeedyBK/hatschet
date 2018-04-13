/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Patrick Sittel (sittel@uni-kassel.de)
  All rights reserved.
*/
#pragma once

#include <HatScheT/Graph.h>

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
  std::set<Edge*> getEdges() const {return this->edges;}
  /*!
   * \brief getVertices
   * \return
   */
  std::set<Vertex*> getVertices() const {return this->vertices;}
  /*!
   * \brief getGraph return the graph the occurrence is in
   * \return
   */
  Graph* getGraph() const {return this->g;}
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
  std::set<Edge*> edges;
  /*!
   * The container for vertices
   */
  std::set<Vertex*> vertices;
};
ostream& operator<<(ostream& os, const Occurrence& o);
}

