//
// Created by bkessler on 21/08/19.
//

#include "SCC.h"

HatScheT::SCC::SCC() {
  this->_type = unknown;
  this->maxVertexId = 0;
  this->name = "";
  this->id = 0;
}

//Getter Functions:

HatScheT::scctype HatScheT::SCC::getSccType() { return _type; }

int HatScheT::SCC::getId() { return this->id; }

vector<int> HatScheT::SCC::getConnections() { return connections; }

map<HatScheT::Vertex *, HatScheT::Vertex *> HatScheT::SCC::getVertexMap() { return vertexMap; }

map<HatScheT::Vertex *, HatScheT::Vertex *> HatScheT::SCC::getVertexMapReverse() { return vertexMapReverse; }




//Setter Functions:

void HatScheT::SCC::setId(int newid) { this->id = newid; }

void HatScheT::SCC::setSCCType(scctype sT) { this -> _type = sT ;}

void HatScheT::SCC::setConnections(int conID) { this -> connections.push_back(conID); }




void HatScheT::SCC::createVertexMap(HatScheT::Vertex *V) {

  this -> vertexMap.insert(std::make_pair(&getVertexById(vertices.size()), V));
  this -> vertexMapReverse.insert(std::make_pair(V, &getVertexById(vertices.size())));

}


void HatScheT::SCC::printVertexMap() {
  for (auto i : vertices){
    cout << i->getName() << "; ";
  }

  cout << endl << endl;

  for(auto it : this->vertexMap) {
    cout << it.first->getName() << " - " << it.second->getName() << endl;
  }

  cout << endl;
}








