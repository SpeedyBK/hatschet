/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Benjamin Lagershausen-Kessler (benjaminkessler@student.uni-kassel.de)

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

#ifndef HATSCHET_SCC_H
#define HATSCHET_SCC_H

#pragma once

#include <HatScheT/Graph.h>
#include <vector>


namespace HatScheT {

  enum scctype {unknown, basic, complex, trivial};

  class SCC : public Graph {
  public:
    SCC();

    //Getter Functions:
    scctype getSccType();

    int getId ();
    vector <int> getConnections();

    map <Vertex*, Vertex*> getVertexMap();
    map <Vertex*, Vertex*> getVertexMapReverse();



    //Setter Functions:
    void setId (int id);
    void setSCCType(scctype sT);
    void setConnections(int conID);

    //Methods
    void createVertexMap(Vertex* V);

    //Debugging:
    void printVertexMap();

  private:

    /*!
     * ID of the SCC.
     */
    int id;

    /*!
     * connections contains the ID of each component which is connected to the actual SCC (inbound and outbound connections).
     */
    vector <int> connections;

    /*!
    * Type of the SCC.
    * - unknown: Set by constructor, just for initialisation.
    * - trivial: SCCs which contain just 1 vertex.
    * - basic: SCCs which contain multiple verticies, but none of them has a ressource constraint.
    * - complex: SCCs which contain multiple verticies, and at least one vertex has a ressource constraint.
    */
    scctype _type;

    /*!
     * vertexMap contains Vertex* of the SCCs verticies as key, and Vertex* of the original graphs verticies as value
     */
    map <Vertex*, Vertex*> vertexMap;

    /*!
     * vertexMapReverse contains Vertex* of the original graphs verticies as key, and Vertex* of the SCCs verticies as value
     */
    map <Vertex*, Vertex*> vertexMapReverse;


  };

}
#endif //HATSCHET_SCC_H
