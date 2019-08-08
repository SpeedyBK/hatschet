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

namespace HatScheT {

  KosarajuSCC::KosarajuSCC(HatScheT::Graph &g) {

    this->g = &g;

  }

  void KosarajuSCC::DebugPrint() {
    static bool Bums = false;

    cout << "Graph G has " << g->getNumberOfVertices() << " Vertecies" << endl;

    cout << '\t' << "Vertex:" << '\t' << "Visited:" << endl;
    for (it = visited.begin(); it != visited.end(); ++it) {
      cout << '\t' << it->first->getName() << '\t' << it->second << endl;
    }

    cout << endl;
    cout << "Graph G has " << g->getNumberOfEdges() << " Edges" << endl;
    cout << "Edge ID" << '\t' << '\t' << "Src Vertex" << '\t' << "Dst Vertex" << endl;

    for (auto e:this->g->Edges()){
      cout << '\t' << e->getId() << '\t' << e->getVertexSrcName() << '\t' << e->getVertexDstName() << endl;
    }
    cout << endl;

    if (Bums) {
      cout << "Stack contains:" << endl;
      while (!Stack.empty()) {
        cout << Stack.top()->getId() << " ";
        Stack.pop();
      }
      cout << endl;
    }

    Bums = true;
  }


  void KosarajuSCC::FillStack(Vertex *V) {
    //Mark vertex as visited:
    visited[V] = true;
    cout << endl << V->getName() << " Visited." << endl;

    //This gets all the edges with source-vertex V. Now we have to check if the destination-vertex of the edge is
    //already visited.
    cout << V->getName() << " has the following neighbors:";
    for (auto e:this->g->Edges()){
      if (&e->getVertexSrc() == V){
        cout << " " << e->getVertexDst().getName() << endl;

        //Now we have to check if the destination-vertex of the edge is already visited. If it is unvisited, we have to
        //perform a DFS on this destination-vertex.
        if(visited[&e->getVertexDst()]){
          cout << e->getVertexDstName() << " is already visited... Moving on..." << endl;
        }else {
          cout << e->getVertexDstName() << " is not visited... Perfroming DFS on " << e->getVertexDstName() << endl;
          FillStack(&e->getVertexDst());
        }
      }
    }
    //Filling the Stack with the finished vertecies.
    cout << endl << "DFS on " << V->getName() << " is done !!" << endl;
    Stack.push(V);
  }

  void KosarajuSCC::printSCCs (){
    //Generating a map which holds the information if a vertex is visited and mark all verticies as unvisited.
    for (auto v:this->g->Vertices()) {
      visited.insert(std::make_pair(v, false));
    }

    //Calling Debug Output
    DebugPrint();

    //This performs Deep First Searches on each unvisited vertex of g. It will fill the stack with verticies depending
    //on the finish time of the DFS. First finished vertex will be at the bottom of the vertex stack.
    for (auto v:g->Vertices()) {
      if (!visited[v]) {
        FillStack(v);
      }
    }
  }
}

