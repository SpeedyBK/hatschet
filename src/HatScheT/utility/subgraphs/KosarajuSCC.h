/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)
            Benjamin Lagershausen-Kessler (benjaminkessler@student.uni-kassel.de)

    Copyright (C) 2019

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
#ifndef HATSCHET_KOSARAJUSCC_H
#define HATSCHET_KOSARAJUSCC_H

#include <iostream>
#include <list>
#include <stack>
#include <vector>

#include "HatScheT/Graph.h"

namespace HatScheT {

  /*
   * M. Sharir; "A strong-connectivity algorithm and its applications in data flow analysis";
   * in Computers & Mathematics with Applications; 1981
   *
   * this algorithm was later credited to Kosaraju
   *
   */
  class KosarajuSCC {
  public:

    /*!
     * \brief Constructor for this class, it needs a graph, and is used to find the SCCs in this graph.
     * \param Parameter g is the graph, in which we want to find Strongly Connected Components.
     */
    explicit KosarajuSCC(Graph& g);

    /*!
    * \brief This Method performs a Deep First Search on the Vertex V
    * \param Parameter g is the graph, in which we want to find Strongly Connected Components.
    */
    void Zeugs();

    void DFS(Vertex* V);

    //void DFSUtil(int v, bool visited[]);

    //KosarajuSCC getTranspose();

    //void fillOrder(int v, bool visited[], std::stack<int> &Stack);

    void printSCCs(){}; //Muss weg... Später!

    /*!
    * \brief Is Used for Debugging, has to be removed...
    * \return nothing
    */
    void DebugPrint();

  private:
    Graph* g;
    int NumOfVerticies;
    std::map <Vertex*, bool> visited;
    std::map <Vertex*, bool> :: iterator it;

  };

}
#endif //HATSCHET_KOSARAJUSCC_H
