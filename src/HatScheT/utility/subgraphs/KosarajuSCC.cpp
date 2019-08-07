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

    // Debug:
    cout << "KosarajuSCC Object built." << endl;

    this->g = &g;

    //Probaly Useless:
    NumOfVerticies = g.getNumberOfVertices();

    //Generating a Map and mark all Verticies as unvisited.
    for (auto v:this->g->Vertices()) {
      visited.insert(std::make_pair(v, false));
    }
  }

  void KosarajuSCC::DebugPrint() {
    cout << "Graph G has " << NumOfVerticies << " Vertecies" << endl;

    cout << '\t' << "Vertex:" << '\t' << "Visited:" << endl;
    for (it = visited.begin(); it != visited.end(); ++it) {
      cout << '\t' << it->first->getName() << '\t' << it->second << endl;
    }
  }


  void KosarajuSCC::Zeugs(){
    for (auto v:g->Vertices()) {
      DFS(v);
    }
  }

  void KosarajuSCC::DFS(Vertex *V) {
    //Mark vertex as visited:
    visited[V] = true;



    //Performing Deep First Search;

  }

}

/*  void KosarajuSCC::DFSUtil(int v, bool visited[]){
    // Mark the current node as visited and print it
    visited[v] = true;
    std::cout << v << " ";

    // Recur for all the vertices adjacent to this vertex
    std::list <int>::iterator i;
    for (i = adj[v].begin(); i != adj[v].end(); ++i)
      if (!visited[*i])
        DFSUtil(*i, visited);
  }

  KosarajuSCC KosarajuSCC::getTranspose(){
    KosarajuSCC g(Verticies);
    for (int v = 0; v < Verticies; v++)
    {
      // Recur for all the vertices adjacent to this vertex
      std::list<int>::iterator i;
      for(i = adj[v].begin(); i != adj[v].end(); ++i)
      {
        g.adj[*i].push_back(v);
      }
    }
    return g;
  }

  void KosarajuSCC::fillOrder(int v, bool visited[], std::stack<int> &Stack){
    // Mark the current node as visited and print it
    visited[v] = true;

    // Recur for all the vertices adjacent to this vertex
    std::list<int>::iterator i;
    for(i = adj[v].begin(); i != adj[v].end(); ++i)
      if(!visited[*i])
        fillOrder(*i, visited, Stack);

    // All vertices reachable from v are processed by now, push v
    Stack.push(v);
  }

// The main function that finds and prints all strongly connected
// components
  void KosarajuSCC::printSCCs(){
    std::stack<int> Stack;

    // Mark all the vertices as not visited (For first DFS)
    bool *visited = new bool[Verticies];
    for(int i = 0; i < Verticies; i++)
      visited[i] = false;

    // Fill vertices in stack according to their finishing times
    for(int i = 0; i < Verticies; i++)
      if(visited[i] == false)
        fillOrder(i, visited, Stack);

    // Create a reversed graph
    KosarajuSCC gr = getTranspose();

    // Mark all the vertices as not visited (For second DFS)
    for(int i = 0; i < Verticies; i++)
      visited[i] = false;

    // Now process all vertices in order defined by Stack
    while (Stack.empty() == false)
    {
      // Pop a vertex from stack
      int v = Stack.top();
      Stack.pop();

      // Print Strongly connected component of the popped vertex
      if (visited[v] == false)
      {
        gr.DFSUtil(v, visited);
        std::cout << std::endl;
      }
    }
  }
}*/