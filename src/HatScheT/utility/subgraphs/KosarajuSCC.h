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
    KosarajuSCC(Graph& g);
    void printSCCs();

    void DFSUtil();
    /*!
        *\brief Will hopefully do something useful. (Under Development)
        *\return Don't know yet
    */


    void TranposeGraph();
    /*!
        *\brief Will hopefully do something useful. (Under Development)
        *\return Don't know yet
    */

    void AddEdge();
    /*!
        *\brief Will hopefully do something useful. (Under Development)
        *\return Don't know yet
    */

    void FillOrder();
    /*!
        *\brief Will hopefully do something useful. (Under Development)
        *\return Don't know yet
    */

  private:
    Graph* g;
  };

}
#endif //HATSCHET_KOSARAJUSCC_H
