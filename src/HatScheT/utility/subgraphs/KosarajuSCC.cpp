/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)
            Benjamin Lagershausen-Kessler (benjaminkessler@student.uni-kassel.de)

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

#include "HatScheT/utility/subgraphs/KosarajuSCC.h"
#include "HatScheT/utility/Utility.h"

namespace HatScheT {

  KosarajuSCC::KosarajuSCC(HatScheT::Graph &g) {

    this->g = &g;
    this->quiet = true;

  }

  void KosarajuSCC::fillStack(Vertex *V) {
    //Mark vertex as visited:
    visited[V] = true;

    //This gets all the edges with source-vertex V. Now we have to check if the destination-vertex of the edge is
    //already visited.
    for (auto e:this->g->Edges()){
      if (&e->getVertexSrc() == V){

        //Now we have to check if the destination-vertex of the edge is already visited. If it is unvisited, we have to
        //perform a DFS on this destination-vertex.
        if(visited[&e->getVertexDst()]){
          // do nothing
        } else {
          fillStack(&e->getVertexDst());
        }
      }
    }
    //Filling the Stack with the finished vertecies.
    if(!this->quiet) cout << endl << "DFS on " << V->getName() << " is done !!";
    Stack.push(V);
  }




  void KosarajuSCC::dfs(Vertex *V) {

    //Mark vertex as visited:
    visited[V] = true;

    //This gets all the edges with source-vertex V. Now we have to check if the destination-vertex of the edge is
    //already visited.
    for (auto e:this->gT.Edges()) {
      if (&e->getVertexSrc() == V) {

        //Now we have to check if the destination-vertex of the edge is already visited. If it is unvisited, we have to
        //perform a DFS on this destination-vertex.
        if (visited[&e->getVertexDst()]) {
          // do nothing
        } else {
          this->dfs(&e->getVertexDst());
        }
      }
    }
    if(!this->quiet) cout << "DFS of Vertex " << V->getId() << " is finished" << endl;
    sccVector[sccVector.size()-1]->setVertexAsPartOfSCC(V);

  }

  vector <SCC*> KosarajuSCC::getSCCs () {
    //Generating a map which holds the information if a vertex is visited and mark all Vertices as unvisited.
    for (auto v:this->g->Vertices()) {
      visited.insert(std::make_pair(v, false));
    }

    //This performs Deep First Searches on each unvisited vertex of g. It will fill the stack with Vertices depending
    //on the finish time of the DFS. First finished vertex will be at the bottom of the vertex stack.
    for (auto v:this->g->Vertices()) {
      if (!visited[v]) {
        fillStack(v);
      }
    }

    if(!this->quiet) cout << endl << "-------------------------------------------------------------------------------------" << endl;

    //Getting the transposed graph of g. And a map, which maps the Vertices of g to the Vertices of gT.
		this->VertexMap = Utility::transposeGraph(this->g, &this->gT);

    //Marking all Vertices of the transposed graph as unvisited
    visited.clear();
    for (auto v:this->gT.Vertices()) {
      visited.insert(std::make_pair(v, false));
    }

    //Doing a DFS in the transposed graph. The order of the Vertices, where the DFS starts is determinated by
    //the order of the Vertices in the Stack.

    int i = 0;

    while (!Stack.empty()) {

      if (!visited[VertexMap[Stack.top()]]) {
        auto *scc = new SCC(*g);
        sccVector.emplace_back(scc);
        scc->setId(i);
        scc->setName("SCC_");
        dfs(VertexMap[Stack.top()]);
        scc->findSCCEdges(); // let the scc find its edges inside the original graph
        //cout << "Size of SCCVector is " << sccVector.size() << endl;
        if(!this->quiet) {
          cout << scc->getName() << scc->getId();
          cout << " Has " << scc->getNumberOfVertices() << " Vertices and " << scc->getSCCEdges().size() << " Edges:" << endl;
          for (auto &v : scc->getVerticesOfSCC()) {
            cout << "  " << v->getName() << endl;
          }
          for (auto &e : scc->getSCCEdges()) {
            cout << "  " << e->getVertexSrcName() << " -> " << e->getVertexDstName() << endl;
          }
        }
        i++;
      }

      Stack.pop();

    }

    return sccVector;
  }

  Vertex *KosarajuSCC::getOriginalVertex(Vertex *V) {

    for (auto itr : VertexMap){
      if (itr.second == V){
        return itr.first;
      }
    }
    throw HatScheT::Exception("A vertex of the transposed graph does not exist in the original graph.");
  }
}

