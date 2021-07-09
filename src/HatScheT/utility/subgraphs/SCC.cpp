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
HatScheT::scctype HatScheT::SCC::getSccType(ResourceModel* rm) {
  if (this->getNumberOfVertices() == 1) {
    return trivial;
  }
  list<Vertex*> VerticesOfSCC = this->getVerticesOfSCC();
  for (auto &it:VerticesOfSCC) {
    if (rm->getResource(it)->getLimit() != -1) {
      return complex;
    }
  }
  return basic;
}

/*!
 * @return ID of the SCC.
 */
int HatScheT::SCC::getId() { return this->id; }

/*!
 * @return the list of Vertices in the SCC.
 */
list<HatScheT::Vertex *> HatScheT::SCC::getVerticesOfSCC() { return verticesOfSCC; }

/*!
 * @return The number of Vertices which belong to the SCC.
 */
int HatScheT::SCC::getNumberOfVertices() { return verticesOfSCC.size(); }

/*!
 * Sets the ID of a SCC.
 * @param newid
 */
void HatScheT::SCC::setId(int newid) { this->id = newid; }

/*!
 * If a vertex should be part of the SCC it can be passed to this function, which sets the value of vertexInSCC
 * for this vertex and pushes the to a list
 * @param V
 */
void HatScheT::SCC::setVertexAsPartOfSCC(HatScheT::Vertex *V) {

  for (auto &it : g->Vertices()){
    if (it->getId() == V->getId()){
      vertexInSCC[it] = true;
      verticesOfSCC.push_back(it);
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

void HatScheT::SCC::setConnectedSCCs(list<HatScheT::SCC *> conSCCs) { this -> connectedSCCs = conSCCs;}


list<HatScheT::SCC *> HatScheT::SCC::getConnectedSCCs() { return connectedSCCs;}


list<HatScheT::Edge *> HatScheT::SCC::getSCCEdges() {
	return this->sccEdges;
}

void HatScheT::SCC::findSCCEdges() {
	for (auto &eIt : g->Edges()){
		bool srcVertexInSCC = false;
		bool dstVertexInSCC = false;
		for (auto &vIt : verticesOfSCC){
			if(&eIt->getVertexSrc() == vIt){
				srcVertexInSCC = true;
			}
			if((&eIt->getVertexDst() == vIt)){
				dstVertexInSCC = true;
			}
		}
		if (srcVertexInSCC && dstVertexInSCC){
			this->sccEdges.emplace_back(eIt);
		}
	}
}
