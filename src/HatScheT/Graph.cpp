#include <HatScheT/Graph.h>
#include <HatScheT/Exception.h>
#include <string>

namespace HatScheT {

Graph::Graph()
{
  this->maxVertexId=0;
  this->maxEdgeId=0;
  this->name = "";
}

Graph::~Graph()
{
  for(Vertex *v : vertices)
    delete v;
}

Vertex& Graph::createVertex()
{
  return createVertex(++maxVertexId);
}

Vertex& Graph::createVertex(int id)
{
  Vertex *v = new Vertex(id);
  vertices.insert(v);
  return *v;
}

Edge& Graph::createEdge(Vertex &Vsrc, Vertex &Vdst, int delay, bool backward, Edge::DependencyType dependencyType)
{
  Edge *e = new Edge(Vsrc,Vdst,delay,backward,dependencyType);
  edges.insert(e);
  return *e;
}


/* ToDo: Remove me
int Graph::addVertex(Vertex &v)
{
  verticesOld[++maxVertexId] = &v;
  return maxVertexId;
}
*/

int Graph::addVertex(Vertex &v, unsigned id)
{
  if(verticesOld.find(id) == verticesOld.end())
  {
    verticesOld[id] = &v;
    return id;
  }
  else
  {
    return -1;
  }
}

Vertex& Graph::getVertex(int id)
{
  vertex_t::iterator it = verticesOld.find(id);
  if(it != verticesOld.end())
    return *(it->second);
  else
    throw Exception("Failure in accessing non-existing vertex id: " + std::to_string(id));
}

int Graph::addEdge(Edge &e)
{
  edgesOld[++maxEdgeId] = &e;
  e.setID(maxEdgeId);
  return maxEdgeId;
}

int Graph::addEdge(Edge &e, unsigned id)
{
  if(edgesOld.find(id) == edgesOld.end())
  {
    edgesOld[id] = &e;
    return id;
  }
  else
  {
    return -1;
  }
}

Edge& Graph::getEdge(int id)
{
  edge_t::iterator it = edgesOld.find(id);
  if(it != edgesOld.end())
    return *(it->second);
  else
    throw Exception("Failure in accessing non-existing edge id: " + std::to_string(id));
}

ostream& operator<<(ostream& os, const Graph& g)
{
  os << "Printing Graph Info:" << endl;

  for(auto a:g.verticesOld)
  {
    Vertex* v = a.second;
    os << *v << endl;
  }

  for(auto a:g.edgesOld)
  {
    Edge* e = a.second;
    os << *e << endl;
  }

  return os;
}

}
