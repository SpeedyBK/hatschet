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
 * \brief The Graph class represents all the data dependencies using a graph of vertices (@see Vertex) and edges (@see Edge).
 *
 * The Graph class is a very simple and lightweight implementation of a graph. Each vertex and edge is assigned to a unique int value.
 * Although the id's are assigned in a sequence, there is no guarantee that all id's exist.
 */
class Graph
{
public:
  Graph();
  ~Graph();


  /*!
   * Creates a vertex, assigns a non-existing id and inserts it in the graph
   *
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
   * @param delay The delay (=no of registers) on an edge
   * @param backward Flag that is true if edge is a backward edge
   * @param dependencyType Enum to distinguish dependencies and data edges
   * @return A reference to the created edge
   */
  Edge &createEdge(Vertex &Vsrc, Vertex &Vdst, int delay=0, bool backward=false, Edge::DependencyType dependencyType=Edge::Data);

  friend ostream& operator<<(ostream& os, const Graph& g);

  /*!
   * \brief getNumberOfVertices
   * \return
   */
  unsigned int getNumberOfVertices()
  {
//    return this->vertices.count();
    return this->verticesOld.size();
  }

  /*!
   * \brief setName
   * \param s
   */
  void setName(string s){this->name = s;}

  const string getName() const {return this->name;}
  /*!
   * \brief verticesBegin
   * \return
   */


  /*! The following methods are DEPRECATED !*/



  /*!
   * Adds a vertex to the graph, assigns a non-existing id and returns it
   *
   * \param v The vertex to be added
   * \return The id of the vertex
   */
//  int addVertex(Vertex &v);

  /*! DEPRECATED!! Remove me when not further in use !
   *
   * Adds a vertex to the graph with id
   *
   * \param v The vertex to be added
   * \param id The id, the vertex should be assigned to
   * \return The id of the vertex or -1 if id already exists
   */
  int addVertex(Vertex &v, unsigned id);

  /*! DEPRECATED!! Remove me when not further in use !
   *
   * Returns the reference to vertex with id 'id', an Exception is thrown if id does not exist
   *
   * \param id The if of the vertex
   */
  Vertex& getVertex(int id);

  /*! DEPRECATED!! Remove me when not further in use !
   *
   * Adds an edge to the graph
   *
   * \param e The edge to be added
   * \return The id of the edge
   */
  int addEdge(Edge &e);

  /*! DEPRECATED!! Remove me when not further in use !
   *
   * Adds a edge to the graph with id
   *
   * \param v The edge to be added
   * \param id The id, the edge should be assigned to
   * \return The id of the edge or -1 if id already exists
   */
  int addEdge(Edge &e, unsigned id);

  /*! DEPRECATED!! Remove me when not further in use !
   *
   * Returns the reference to edge with id 'id', an Exception is thrown if id does not exist
   *
   * \param id The if of the edge
   */
  Edge& getEdge(int id);
  /*!
   * \brief operator <<
   * \param os
   * \param g
   * \return
   */

  /*! DEPRECATED!! Remove me when not further in use ! */
  const std::map<unsigned,Vertex*>::iterator verticesBegin()
  {
      return verticesOld.begin();
  }

  /*! DEPRECATED!! Remove me when not further in use ! */
  const std::map<unsigned,Vertex*>::iterator verticesEnd()
  {
      return verticesOld.end();
  }

  /*! DEPRECATED!! Remove me when not further in use ! */
  const std::map<unsigned,Edge*>::iterator edgesBegin()
  {
      return edgesOld.begin();
  }

  /*! DEPRECATED!! Remove me when not further in use ! */
  const std::map<unsigned,Edge*>::iterator edgesEnd()
  {
      return edgesOld.end();
  }

protected:
  /*!
   * The container for vertices
   */
  vertex_t verticesOld;  /*! DEPRECATED!! Remove me when not further in use ! */
  std::set<Vertex*> vertices;

  /*!
   * The container for edges
   */
  edge_t edgesOld;  /*! DEPRECATED!! Remove me when not further in use ! */
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
