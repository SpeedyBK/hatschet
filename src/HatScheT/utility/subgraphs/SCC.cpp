//
// Created by bkessler on 21/08/19.
//

#include "SCC.h"

/*!
 * Contructor, does a basic initialisation of an empty SCC.
 * @param g is the original graph.
 */
HatScheT::SCC::SCC(Graph &g) {
  this->g = &g;
  this->typeOfSCC = unknown;
  this->name = "";
  this->id = 0;

  for (auto &it : g.Vertices()) {
    vertexInSCC[it] = false;
  }
}

/*!
 * @return Type of the SCC
 */
HatScheT::scctype HatScheT::SCC::getSccType() { return typeOfSCC; }

/*!
 * @return ID of the SCC.
 */
int HatScheT::SCC::getId() { return this->id; }

/*!
 * @return the list of Vertices in the SCC.
 */
list<HatScheT::Vertex *> HatScheT::SCC::getVerticesOfSCC() { return VerticesOfSCC; }

/*!
 * @return a set of Vertices which are connected to the SCC
 */
set<HatScheT::Vertex *> HatScheT::SCC::getConnectedVertices() { return set<HatScheT::Vertex *>(); }

/*!
 * @return The number of Vertices which belong to the SCC.
 */
int HatScheT::SCC::getNumberOfVertices() { return vertexInSCC.size(); }

/*!
 * Sets the ID of a SCC.
 * @param newid
 */
void HatScheT::SCC::setId(int newid) { this->id = newid; }

/*!
 * Sets the type of an SCC to unknown, basic, trivial or complex
 * @param sT
 */
void HatScheT::SCC::setSCCType(scctype sT) { this -> typeOfSCC = sT ;}

/*!
 * If a vertex should be part of the SCC it can be passed to this function, which sets the value of vertexInSCC
 * for this vertex and pushes the to a list
 * @param V
 */
void HatScheT::SCC::setVertexAsPartOfSCC(HatScheT::Vertex *V) {

  for (auto &it : g->Vertices()){
    if (it->getId() == V->getId()){
      vertexInSCC[it] = true;
      VerticesOfSCC.push_back(it);
    }
  }
}

/*!
 * Used for debugging.
 */
void HatScheT::SCC::printVertexStatus() {

  for (auto &it:vertexInSCC){
    cout << it.first->getName() << " -- " << it.second << endl;
  }
}

map<HatScheT::Vertex *, bool> HatScheT::SCC::getVertexInSccMap() { return vertexInSCC; }
