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

Graph::Graph(){
  this->maxVertexId=0;
  this->name = "";
}

Graph::~Graph(){
  for(Vertex *v : vertices)
    delete v;
  for(Edge *e : edges)
    delete e;
}

int Graph::getMaxVertexId() {
  int maxId=0;
  for(auto it=this->vertices.begin(); it!=this->vertices.end(); ++it){
    Vertex* v = *it;
    if(maxId < v->getId()) maxId=v->getId();
  }
  return maxId;
}

Vertex& Graph::createVertex(){
  return createVertex(++maxVertexId);
}

Vertex& Graph::createVertex(int id){
  for(auto it=this->vertices.begin(); it!=this->vertices.end(); ++it){
    Vertex* v = *it;
    if(v->getId()==id) throw HatScheT::Exception("Graph.createVertex: Error! This id is already occupied: " + to_string(id) + "( " + v->getName() +" )");
  }

  Vertex *v = new Vertex(id);
  vertices.insert(v);

  // init trackers for incoming and outgoing edges
	this->incomingEdges[v] = {};
	this->outgoingEdges[v] = {};

  //keep maxVertexId consistent
  if(this->maxVertexId < v->getId()) this->maxVertexId = this->getMaxVertexId()+1;

  return *v;
}

Edge& Graph::getEdge(int id) const {
  for(auto e:this->edges){
    if(e->getId()==id) return *e;
  }
  throw HatScheT::Exception("Graph::getEdge: Edge not found! Request Id was: " + to_string(id));
}

Edge& Graph::createEdge(Vertex &Vsrc, Vertex &Vdst, int distance, Edge::DependencyType dependencyType){
  Edge *e = new Edge(Vsrc,Vdst,distance,dependencyType,edges.size()+1);

  for(auto it=this->edges.begin(); it!=this->edges.end(); ++it){
    Edge* eIt = *it;

    if(e->getId()==eIt->getId()) throw HatScheT::Exception("Graph.createEdge: Error! This edge id is already occupied: " + to_string(e->getId()));
  }

  edges.insert(e);
  outgoingEdges[&Vsrc].insert(e);
  incomingEdges[&Vdst].insert(e);
  return *e;
}

bool Graph::isSourceVertex(Vertex *v){
	/*
  for(auto it:this->edges){
    Edge* e = it;
    Vertex* dstV = &e->getVertexDst();

    if(dstV == v) return false;
  }

  return true;
	 */
	return this->getNumIncomingEdges(v) == 0;
}

bool Graph::isSinkVertex(Vertex *v){
	/*
  for(auto it:this->edges){
    Edge* e = it;
    Vertex* srcV = &e->getVertexSrc();

    if(srcV == v) return false;
  }

  return true;
	 */
	return this->getNumOutgoingEdges(v) == 0;
}

bool Graph::hasNoNonZeroDistanceOutgoingEdges(Vertex *v) {
	try {
		for (auto &e : this->outgoingEdges.at(v)) {
			if (e->getDistance() != 0) return false;
		}
		return true;
	}
	catch (std::out_of_range&) {
		// vertex has no outgoing edges
		return true;
	}
}

bool Graph::hasNoNonZeroDistanceIncomingEdges(Vertex *v) {
	try {
		for (auto &e : this->incomingEdges.at(v)) {
			if (e->getDistance() != 0) return false;
		}
		return true;
	}
	catch (std::out_of_range&) {
		// vertex has no outgoing edges
		return true;
	}
}

bool Graph::isEmpty(){
  if(this->vertices.size() == 0) return true;
  return false;
}

set<Vertex *> Graph::getPredecessors(const Vertex *v) const{
  set<Vertex*> vset;

  for(auto it:this->edges){
    Edge* e = it;
    Vertex* dstV = &e->getVertexDst();

    if(dstV == v) vset.insert(&e->getVertexSrc());
  }

  return vset;
}

set<Vertex *> Graph::getSuccessors(const Vertex *v) const{
  set<Vertex*> vset;

  for(auto it:this->edges){
    Edge* e = it;
    Vertex* srcV = &e->getVertexSrc();

    if(srcV == v) vset.insert(&e->getVertexDst());
  }

  return vset;
}

std::list<const Edge* > Graph::getEdges(const HatScheT::Vertex *srcV, const HatScheT::Vertex *dstV) const {
  std::list<const Edge* > edges;
  for(auto e:this->edges){
    if(&e->getVertexSrc()==srcV && &e->getVertexDst()==dstV) edges.push_back(e);
  }

  if(edges.size() == 0){
    cout << *this << endl;
    throw HatScheT::Exception("Graph::getEdges: No Edge could be found: " + srcV->getName() + " -> " + dstV->getName());
  }

  return edges;
}

/*Edge& Graph::getEdge(const Vertex *srcV, const Vertex *dstV) const{
  for(auto e:this->edges){
    if(&e->getVertexSrc()==srcV && &e->getVertexDst()==dstV) return *e;
  }

  throw HatScheT::Exception("Graph::getEdge: Edge not found!");
}*/

bool Graph::edgeExists(const HatScheT::Vertex *srcV, const HatScheT::Vertex *dstV) {
  for(auto e:this->edges){
    if(&e->getVertexSrc()==srcV && &e->getVertexDst()==dstV) return true;
  }
  return false;
}

Vertex& Graph::getVertexById(int id) const{
  for(auto a:this->vertices){
    Vertex* v = a;
    if(v->getId() == id) return *v;
  }
  throw HatScheT::Exception("Graph::getVertexById: Vertex not found!");
}



	Vertex &Graph::getVertexByName(std::string n) const {
		for(auto &v : this->vertices){
			if(v->getName() == n) return *v;
		}
		throw HatScheT::Exception("Graph::getVertexByName: Vertex '"+n+"' not found!");
	}

ostream& operator<<(ostream& os, const Graph& g){
  os << "------------------------------------------------------------------------------------" << endl;
  os << "---------------------------------- Graph Model -------------------------------------" << endl;
  os << "------------------------------------------------------------------------------------" << endl;

  for(auto a:g.vertices){
    Vertex* v = a;
    os << *v << endl;
  }

  for(auto a:g.edges){
    Edge* e = a;
    os << *e << endl;
  }

  return os;
}

  std::set<Edge *> &Graph::getOutgoingEdges(const Vertex *v) {
      try {
          return this->outgoingEdges.at(v);
      }
      catch (std::out_of_range &) {
          throw Exception("Graph::getOutgoingEdges: requested invalid vertex");
      }
  }

  std::set<Edge *> &Graph::getIncomingEdges(const Vertex *v) {
      try {
          return this->incomingEdges.at(v);
      }
      catch (std::out_of_range &) {
          throw Exception("Graph::getIncomingEdges: requested invalid vertex");
      }
  }

  size_t Graph::getNumOutgoingEdges(const Vertex *v) const {
      try {
          return this->outgoingEdges.at(v).size();
      }
      catch (std::out_of_range &) {
          throw Exception("Graph::getNumOutgoingEdges: requested invalid vertex");
      }
  }

  size_t Graph::getNumIncomingEdges(const Vertex *v) const {
      try {
          return this->incomingEdges.at(v).size();
      }
      catch (std::out_of_range &) {
          throw Exception("Graph::getNumIncomingEdges: requested invalid vertex");
      }
  }

	void Graph::reset() {
		this->maxVertexId = 0;
		for (auto &v : this->vertices) {
			delete v;
		}
		this->vertices.clear();
		for (auto &e : this->edges) {
			delete e;
		}
		this->edges.clear();
		this->incomingEdges.clear();
		this->outgoingEdges.clear();
	}

}
