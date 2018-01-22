#include <HatScheT/Graph.h>
#include <HatScheT/Exception.h>
#include <string>

namespace HatScheT {

Graph::Graph()
{
  maxVertexId=0;
  maxEdgeId=0;
}

int Graph::addVertex(Vertex &v)
{
  vertices[++maxVertexId] = &v;
  return maxVertexId;
}

int Graph::addVertex(Vertex &v, unsigned id)
{
  if(vertices.find(id) == vertices.end())
  {
    vertices[id] = &v;
    return id;
  }
  else
  {
    return -1;
  }
}

Vertex& Graph::getVertex(int id)
{
  vertex_t::iterator it = vertices.find(id);
  if(it != vertices.end())
    return *(it->second);
  else
    throw Exception("Failure in accessing non-existing vertex id: " + std::to_string(id));
}

int Graph::addEdge(Edge &e)
{
  edges[++maxEdgeId] = &e;
  e.setID(maxEdgeId);
  return maxEdgeId;
}

int Graph::addEdge(Edge &e, unsigned id)
{
  if(edges.find(id) == edges.end())
  {
    edges[id] = &e;
    return id;
  }
  else
  {
    return -1;
  }
}

unsigned int Graph::getMaxLatency()
{
    unsigned int maxLatency = 0;

    for(std::map<unsigned,Vertex*>::iterator it = this->vertices.begin(); it!=this->vertices.end(); ++it)
    {
        Vertex* v = it->second;

        if(v->getLatency() > maxLatency) maxLatency = v->getLatency();
    }

    for(std::map<unsigned,Edge*>::iterator it = this->edges.begin(); it!=this->edges.end(); ++it)
    {
        Edge* e = it->second;

        if(e->getDelay() > maxLatency) maxLatency = e->getDelay();
    }

    return maxLatency;
}

Edge& Graph::getEdge(int id)
{
  edge_t::iterator it = edges.find(id);
  if(it != edges.end())
    return *(it->second);
  else
    throw Exception("Failure in accessing non-existing edge id: " + std::to_string(id));
}

ostream& operator<<(ostream& os, const Graph& g)
{
  os << "Printing Graph Info:" << endl;

  for(auto a:g.vertices)
  {
    Vertex* v = a.second;
    os << *v << endl;
  }

  for(auto a:g.edges)
  {
    Edge* e = a.second;
    os << *e << endl;
  }

  return os;
}

}
