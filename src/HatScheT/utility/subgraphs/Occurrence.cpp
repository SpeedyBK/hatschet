#include <HatScheT/utility/subgraphs/Occurrence.h>
#include <HatScheT/utility/Utility.h>

namespace HatScheT
{
Occurrence::Occurrence(Graph *g)
{
  this->g =g;
}

ostream& operator<<(ostream& os, const Occurrence& o)
{
  os << "---Printing Occurrence---" << endl;
  os << "---Printing Vertices---" << endl;
  for(auto it:o.vertices){
    Vertex* v = it;
    os << *v << endl;
  }
  os << "---Printing Edges---" << endl;
  for(auto it:o.edges){
    Edge* e = it;
    os << *e << endl;
  }

  os << "---Finished Printing Occurrence---" << endl;
  return os;
}

bool Occurrence::addEdge(Edge *e)
{
  if(this->edges.size()==0){
    this->edges.insert(e);
    this->vertices.insert(&e->getVertexDst());
    this->vertices.insert(&e->getVertexSrc());
    return true;
  }

  if(Utility::edgeIsInGraph(this->g, e) == false){
    cout << "Occurrence.addEdge: WARNING tried to add an edge that is not in g!" << endl;
    return false;
  }

  const bool is_in = this->edges.find(e) != this->edges.end();

  if(is_in==true){
    cout << "Occurrence.addEdge: WARNING tried to add an existing edge" << endl;
    return false;
  }

  if(this->isConnected(e)==true){
    this->edges.insert(e);
    this->vertices.insert(&e->getVertexDst());
    this->vertices.insert(&e->getVertexSrc());
    return true;
  }
  else{
    cout << "Occurrence.addEdge: WARNING tried to add a non connected edge" << endl;
    return false;
  }
}

bool Occurrence::isConnected(Edge *e)
{
  for(auto it=this->edges.begin();it!=this->edges.end();++it){
    Edge* iterE = *it;
    if(&e->getVertexDst()==&iterE->getVertexDst()) return true;
    if(&e->getVertexSrc()==&iterE->getVertexDst()) return true;
    if(&e->getVertexDst()==&iterE->getVertexSrc()) return true;
    if(&e->getVertexSrc()==&iterE->getVertexSrc()) return true;
  }
  return false;
}

}
