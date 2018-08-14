/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)

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

bool Occurrence::edgeIsNew(Edge *e)
{
  for(auto it:this->edges){
    Edge* eIter = it;

    if(eIter==e) return false;
  }
  return true;
}

bool Occurrence::vertexIsNew(Vertex *v)
{
  for(auto it:this->vertices){
    Vertex* vIter = it;

    if(vIter==v) return false;
  }

  return true;
}

bool Occurrence::addEdge(Edge *e)
{
  if(this->edges.size()==0){
    this->edges.push_back(e);
    if(this->vertexIsNew(&e->getVertexDst())) this->vertices.push_back(&e->getVertexDst());
    if(this->vertexIsNew(&e->getVertexSrc())) this->vertices.push_back(&e->getVertexSrc());
    return true;
  }

  if(Utility::edgeIsInGraph(this->g, e) == false){
    cout << "Occurrence.addEdge: WARNING tried to add an edge that is not in g!" << endl;
    return false;
  }

  if(this->edgeIsNew(e)==false){
    cout << "Occurrence.addEdge: WARNING tried to add an existing edge" << endl;
    return false;
  }

  if(this->isConnected(e)==true){
    this->edges.push_back(e);
    if(this->vertexIsNew(&e->getVertexDst())) this->vertices.push_back(&e->getVertexDst());
    if(this->vertexIsNew(&e->getVertexSrc())) this->vertices.push_back(&e->getVertexSrc());
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
