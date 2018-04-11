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
   * \brief getNoOfInputs
   * \param v
   * \return
   */
  int getNoOfInputs(const Vertex* v) const;
  /*!
   * \brief setName
   * \param s
   */
  void setName(string s){this->name = s;}
  const string getName() const {return this->name;}
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
