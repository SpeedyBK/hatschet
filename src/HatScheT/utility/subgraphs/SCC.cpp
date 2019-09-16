//
// Created by bkessler on 21/08/19.
//

#include "SCC.h"

HatScheT::SCC::SCC(Graph &g) {
  this->g = &g;
  this->typeOfSCC = unknown;
  this->name = "";
  this->id = 0;

  for (auto &it : g.Vertices()){
    vertexInSCC[it] = false;
  }
}

//Getter Functions:

HatScheT::scctype HatScheT::SCC::getSccType() { return typeOfSCC; }

int HatScheT::SCC::getId() { return this->id; }

vector<int> HatScheT::SCC::getConnections() { return connections; }

list<HatScheT::Vertex *> HatScheT::SCC::getVerticiesOfSCC() { return verticiesOfSCC; }



//Setter Functions:

void HatScheT::SCC::setId(int newid) { this->id = newid; }

void HatScheT::SCC::setSCCType(scctype sT) { this -> typeOfSCC = sT ;}

void HatScheT::SCC::setConnections(int conID) { this -> connections.push_back(conID); }


void HatScheT::SCC::setVertexAsPartOfSCC(HatScheT::Vertex *V) {

  for (auto &it : g->Vertices()){
    if (it->getId() == V->getId()){
      vertexInSCC[it] = true;
      verticiesOfSCC.push_back(it);
    }
  }
}

int HatScheT::SCC::getNumberOfVertices() {

  int i = 0;

  for (auto &it : vertexInSCC){
    if (it.second){
      i++;
    }
  }
  return i;
}

void HatScheT::SCC::printVertexStatus() {

  for (auto &it:vertexInSCC){
    cout << it.first->getName() << " -- " << it.second << endl;
  }
}










