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
    * \brief Printfunction, which prints out the names of verticies for each SSC.
    */
    void printSSC();

    /*!
    * \brief This method performs a recursive deep first seach on the verticies of graph g and fills a stack with the
    * with the verticies for with the deep first search is finished in reversed order of the finish times.
    * \param Parameter V is a vertex-pointer which points to the vertex where the DFS starts.
    */
    void fillStack(Vertex* V);

    /*!
    * \brief The dfs function performes a DFS on vertex V. The Function is almost similar to the fillStack function. But
    * this time, we do not fill a stack with the verticies.
    * \param Parameter V is a vertex-pointer which points to the vertex where the DFS starts.
    */
    void dfs(Vertex* V);

    /*!
    * \brief GetSCCs() is the main function of this class. It finds stongly connected component of graph g. At this
    * moment it will store the verticies which are stronly connected in a vector called scc. The vector scc itself is
    * will be a component of the vector sccs which contains all the strongly connected components of graph g.
    */
    vector <vector<Vertex*>> getSCCs();

    Vertex* getOriginalVertex(Vertex* V);

    /*!
    * \brief Is used for debugging, has to be removed...
    */
    //void DebugPrint(bool energydrink, Graph* gr);

  private:

    Graph* g;
    Graph* gT; //Transposed Graph

    map <Vertex*, Vertex*> VertexMap;
    map <Vertex*, Vertex*> :: iterator itr;

    map <Vertex*, bool> visited;
    map <Vertex*, bool> :: iterator it;

    stack <Vertex*> Stack;

    vector <Vertex*> scc;
    vector <vector<Vertex*>> sccs;

  };

}
#endif //HATSCHET_KOSARAJUSCC_H
