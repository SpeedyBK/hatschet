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

map<HatScheT::Vertex *, HatScheT::Vertex *> HatScheT::SCC::getVertexMap() { return vertexMap; }




//Setter Functions:

void HatScheT::SCC::setId(int newid) { this->id = newid; }

void HatScheT::SCC::setSCCType(scctype sT) { this -> _type = sT ;}




void HatScheT::SCC::createVertexMap(HatScheT::Vertex *V) {

  this -> vertexMap.insert(std::make_pair(&getVertexById(vertices.size()), V));

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




