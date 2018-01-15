#pragma once

#include "Vertex.h"
#include "Edge.h"
#include <map>

namespace HatScheT
{

typedef std::map<unsigned,Vertex*> vertex_t;
typedef std::map<unsigned,Edge*> edge_t;

/*!
 * \brief The Graph class represents all the data dependencies using a graph of vertices (@see Vertex) and edges (@see Edge).
 *
 * The Graph class is a very simple and lightweight implementation of a graph. Each vertex and edge is assigned to a unique int value.
 * Although the id's are assigned in a sequence, there is no guarantee that all id's exist.
 */
class Graph
{
public:
  Graph();

  /*!
   * Adds a vertex to the graph, assigns a non-existing id and returns it
   *
   * \param v The vertex to be added
   * \return The id of the vertex
   */
  int addVertex(Vertex &v);

  /*!
   * Adds a vertex to the graph with id
   *
   * \param v The vertex to be added
   * \param id The id, the vertex should be assigned to
   * \return The id of the vertex or -1 if id already exists
   */
  int addVertex(Vertex &v, unsigned id);

  /*!
   * Returns the reference to vertex with id 'id', an Exception is thrown if id does not exist
   *
   * \param id The if of the vertex
   */
  Vertex& getVertex(int id);

  /*!
   * Adds an edge to the graph
   *
   * \param e The edge to be added
   * \return The id of the edge
   */
  int addEdge(Edge &e);

  /*!
   * Adds a edge to the graph with id
   *
   * \param v The edge to be added
   * \param id The id, the edge should be assigned to
   * \return The id of the edge or -1 if id already exists
   */
  int addEdge(Edge &e, unsigned id);


  /*!
   * Returns the reference to edge with id 'id', an Exception is thrown if id does not exist
   *
   * \param id The if of the edge
   */
  Edge& getEdge(int id);


protected:
  /*!
   * The container for vertices
   */
  vertex_t vertices;

  /*!
   * The container for edges
   */
  edge_t edges;

  /*!
   * This int is used to track the next valid vertex id
   */
  int maxVertexId;

  /*!
   * This int is used to track the next valid edge id
   */
  int maxEdgeId;
};

}
