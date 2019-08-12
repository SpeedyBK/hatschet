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
    this->gT = nullptr;

    getSCCs();

  }

  void KosarajuSCC::DebugPrint(bool Bums, Graph* gr) {


    cout << "Graph has " << gr->getNumberOfVertices() << " Vertecies" << endl;

    cout << '\t' << "Vertex:" << '\t' << "Visited:" << endl;
    for (it = visited.begin(); it != visited.end(); ++it) {
      cout << '\t' << it->first->getName() << '\t' << it->second << endl;
    }

    cout << endl;
    cout << "Graph has " << gr->getNumberOfEdges() << " Edges" << endl;
    cout << "Edge ID" << '\t' << '\t' << "Src Vertex" << '\t' << "Dst Vertex" << endl;

    for (auto e:gr->Edges()){
      cout << '\t' << e->getId() << '\t' << e->getVertexSrcName() << '\t' << e->getVertexDstName() << endl;
    }
    cout << endl;

    auto StackCopy = this -> Stack;
    if (Bums) {
      cout << "Stack contains:" << endl;
      while (!StackCopy.empty()) {
        cout << StackCopy.top()->getId() << " ";
        StackCopy.pop();
      }
      cout << endl << endl;
    }
  }


  void KosarajuSCC::fillStack(Vertex *V) {
    //Mark vertex as visited:
    visited[V] = true;
    //cout << endl << V->getName() << " Visited." << endl;

    //This gets all the edges with source-vertex V. Now we have to check if the destination-vertex of the edge is
    //already visited.
    //cout << V->getName() << " has the following neighbors:";
    for (auto e:this->g->Edges()){
      if (&e->getVertexSrc() == V){
        //cout << " " << e->getVertexDst().getName() << endl;

        //Now we have to check if the destination-vertex of the edge is already visited. If it is unvisited, we have to
        //perform a DFS on this destination-vertex.
        if(visited[&e->getVertexDst()]){
          //cout << e->getVertexDstName() << " is already visited... Moving on..." << endl;
        }else {
          //cout << e->getVertexDstName() << " is not visited... Perfroming DFS on " << e->getVertexDstName() << endl;
          fillStack(&e->getVertexDst());
        }
      }
    }
    //Filling the Stack with the finished vertecies.
    cout << endl << "DFS on " << V->getName() << " is done !!";
    Stack.push(V);
  }


  void KosarajuSCC::dfs(Vertex *V) {

    //Mark vertex as visited:
    visited[V] = true;
    //cout << endl << V->getName() << " Visited." << endl;

    //This gets all the edges with source-vertex V. Now we have to check if the destination-vertex of the edge is
    //already visited.
    //cout << V->getName() << " has the following neighbors:";
    for (auto e:this->gT->Edges()) {
      if (&e->getVertexSrc() == V) {

        //Now we have to check if the destination-vertex of the edge is already visited. If it is unvisited, we have to
        //perform a DFS on this destination-vertex.
        if (visited[&e->getVertexDst()]) {
          //cout << e->getVertexDstName() << " is already visited... Moving on..." << endl;
        } else {
          //cout << e->getVertexDstName() << " is not visited... Perfroming DFS on " << e->getVertexDstName() << endl;
          this->dfs(&e->getVertexDst());
        }
      }
    }
    cout << "DFS of Vertex " << V->getId() << " is finished" << endl;
    scc.push_back(V);
  }

  void KosarajuSCC::getSCCs (){
    //Generating a map which holds the information if a vertex is visited and mark all verticies as unvisited.
    for (auto v:this->g->Vertices()) {
      visited.insert(std::make_pair(v, false));
    }

    //Calling Debug Output

    //cout << endl << "-------------------------------------------------------------------------------------"  << endl;
    //DebugPrint(false, g);

    //This performs Deep First Searches on each unvisited vertex of g. It will fill the stack with verticies depending
    //on the finish time of the DFS. First finished vertex will be at the bottom of the vertex stack.
    for (auto v:this->g->Vertices()) {
      if (!visited[v]) {
        fillStack(v);
      }
    }

    cout << endl << "-------------------------------------------------------------------------------------"  << endl;
    //DebugPrint(true, g);

    //Getting the transposed graph of g. And a map, which maps the verticies of g to the verticies of gT.
    auto GraphMap = Utility::transposeGraph(g);
    this -> gT = GraphMap.first;
    auto VertexMap = GraphMap.second;


    //Marking all verticies of the transposed graph as unvisited
    visited.clear();
    for (auto v:this->gT->Vertices()){
      visited.insert(std::make_pair(v, false));
    }

    //cout << endl << "-------------------------------------------------------------------------------------"  << endl;
    //DebugPrint(true, this -> gT);

    //Doing a DFS in the transposed graph. The order of the verticies, where the DFS starts is determinated by
    //the order of the verticies in the Stack.
    while (!Stack.empty()){
      if (!visited[VertexMap[Stack.top()]]) {
        dfs(VertexMap[Stack.top()]);
        sccs.push_back(scc);
        scc.clear();
      }
      Stack.pop();
    }

    //Basic output for debugging.. Will be removed:
    //cout << endl << "-------------------------------------------------------------------------------------"  << endl;
    int i = 0;
    for (auto stronglyConectedComponent:sccs){
      i++;
      cout << endl << "SCC " << i << ": ";
      for (auto Vertex:stronglyConectedComponent){
        cout << Vertex->getName() << "  ";
      }
      cout << endl;
    }
  }
}

