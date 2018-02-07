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

bool Graph::isSourceVertex(Vertex *v)
{
  for(auto it:this->edges)
  {
    Edge* e = it;
    Vertex* dstV = &e->getVertexDst();

    if(dstV == v) return false;
  }

  return true;
}

bool Graph::isEmpty()
{
  if(this->vertices.size() == 0) return true;
  return false;
}

Vertex& Graph::getVertexById(int id) const
{
  for(auto a:this->vertices)
  {
    Vertex* v = a;
    if(v->getId() == id) return *v;
  }
}

ostream& operator<<(ostream& os, const Graph& g)
{
  os << "Printing Graph Info:" << endl;

  for(auto a:g.vertices)
  {
    Vertex* v = a;
    os << *v << endl;
  }

  for(auto a:g.edges)
  {
    Edge* e = a;
    os << *e << endl;
  }

  return os;
}

}
