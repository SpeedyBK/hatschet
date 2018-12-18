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

#include <HatScheT/Graph.h>
#include <HatScheT/utility/Exception.h>
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
  for(Edge *e : edges)
    delete e;
}

Vertex& Graph::createVertex()
{
  return createVertex(++maxVertexId);
}

Vertex& Graph::createVertex(int id)
{
  for(auto it=this->vertices.begin(); it!=this->vertices.end(); ++it){
    Vertex* v = *it;
    if(v->getId()==id) throw HatScheT::Exception("Graph.createVertex: Error! This id is already occupied: " + to_string(id) + "( " + v->getName() +" )");
  }

  Vertex *v = new Vertex(id);
  vertices.insert(v);
  return *v;
}

Edge& Graph::createEdge(Vertex &Vsrc, Vertex &Vdst, int distance, Edge::DependencyType dependencyType)
{
  if(this->edgeExists(&Vsrc,&Vdst) == true) {
    throw HatScheT::Exception("Graph.createEdge: Error! You tried add an already existing edge : " + Vsrc.getName() + " -> " + Vdst.getName());
  }

  Edge *e = new Edge(Vsrc,Vdst,distance,dependencyType);
  e->setID(edges.size()+1);
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

bool Graph::isSinkVertex(Vertex *v)
{
  for(auto it:this->edges)
  {
    Edge* e = it;
    Vertex* srcV = &e->getVertexSrc();

    if(srcV == v) return false;
  }

  return true;
}

bool Graph::isEmpty()
{
  if(this->vertices.size() == 0) return true;
  return false;
}

set<Vertex *> Graph::getPredecessors(const Vertex *v) const
{
  set<Vertex*> vset;

  for(auto it:this->edges)
  {
    Edge* e = it;
    Vertex* dstV = &e->getVertexDst();

    if(dstV == v) vset.insert(&e->getVertexSrc());
  }

  return vset;
}

set<Vertex *> Graph::getSuccessors(const Vertex *v) const
{
  set<Vertex*> vset;

  for(auto it:this->edges)
  {
    Edge* e = it;
    Vertex* srcV = &e->getVertexSrc();

    if(srcV == v) vset.insert(&e->getVertexDst());
  }

  return vset;
}

Edge& Graph::getEdge(const Vertex *srcV, const Vertex *dstV) const
{
  for(auto e:this->edges)
  {
    if(&e->getVertexSrc()==srcV && &e->getVertexDst()==dstV) return *e;
  }

  throw HatScheT::Exception("Graph::getEdge: Edge not found!");
}

bool Graph::edgeExists(const HatScheT::Vertex *srcV, const HatScheT::Vertex *dstV) {
  for(auto e:this->edges)
  {
    if(&e->getVertexSrc()==srcV && &e->getVertexDst()==dstV) return true;
  }
  return false;
}

Vertex& Graph::getVertexById(int id) const
{
  for(auto a:this->vertices)
  {
    Vertex* v = a;
    if(v->getId() == id) return *v;
  }
  throw HatScheT::Exception("Graph::getVertexById: Vertex not found!");
}

ostream& operator<<(ostream& os, const Graph& g)
{
  os << "------------------------------------------------------------------------------------" << endl;
  os << "---------------------------------- Graph Model -------------------------------------" << endl;
  os << "------------------------------------------------------------------------------------" << endl;

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
