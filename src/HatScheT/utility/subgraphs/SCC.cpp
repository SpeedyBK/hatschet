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
  this->numOfLimitedVerticesInSCC = 0;

  for (auto &it : g.Vertices()) {
    vertexInSCC[it] = false;
  }
}

/*!
 * @return Type of the SCC
 */
HatScheT::scctype HatScheT::SCC::getSccType(ResourceModel* rm) {
	if (this->typeOfSCC != unknown) return this->typeOfSCC;
	if (this->getNumberOfVertices() == 1) {
    return this->typeOfSCC = trivial;
  }
  list<Vertex*> VerticesOfSCC = this->getVerticesOfSCC();
  for (auto &it : VerticesOfSCC) {
    if (rm->getResource(it)->getLimit() != UNLIMITED) {
      numOfLimitedVerticesInSCC++;
      return this->typeOfSCC = complex;
    }
  }
  return this->typeOfSCC = basic;
}

/*!
 * @return ID of the SCC.
 */
int HatScheT::SCC::getId() const { return this->id; }

/*!
 * @return the list of Vertices in the SCC.
 */
list<HatScheT::Vertex *> HatScheT::SCC::getVerticesOfSCC() { return verticesOfSCC; }

/*!
 * @return The number of Vertices which belong to the SCC.
 */
int HatScheT::SCC::getNumberOfVertices() const { return verticesOfSCC.size(); }

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


list<HatScheT::Edge *> HatScheT::SCC::getSCCEdges() const {
	return this->sccEdges;
}

void HatScheT::SCC::findSCCEdges() {
	this->sccEdges.clear();
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

void HatScheT::SCC::printInfo() {
	std::cout << "Printing info about " << this->getName() << " (ID " << this->getId() << ")" << std::endl;
	std::cout << "  SCC vertices:" << std::endl;
	for (auto &v : this->verticesOfSCC) {
		std::cout << "    " << v->getName() << std::endl;
	}
	std::cout << "  SCC edges:" << std::endl;
	for (auto &e : this->sccEdges) {
		std::cout << "    '" << e->getVertexSrcName() << "' -> '" << e->getVertexDstName() << "'" << std::endl;
	}
	std::cout << "  connected SCCs:" << std::endl;
	for (auto &scc : this->connectedSCCs) {
		std::cout << "    " << scc->getName() << " (ID " << scc->getId() << ")" << std::endl;
	}
}

bool HatScheT::SCC::operator<(const HatScheT::SCC &a) const {
    if (this->getNumberofLimitedVertices() < a.getNumberofLimitedVertices()) {
        return true;
    }else if (this->getNumberofLimitedVertices() == a.getNumberofLimitedVertices()) {
        if (this->getNumberOfVertices() < a.getNumberOfVertices()){
            return true;
        }else if (this->getNumberOfVertices() == a.getNumberOfVertices()) {
            if (this->getNumOfEdges() < a.getNumOfEdges()) {
                return true;
            } else if (this->getNumOfEdges() == a.getNumOfEdges()) {
                return id < a.id;
            }
        }
    }
    return false;
}
