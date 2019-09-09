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

  /*!
  * Datatyp to classify the SCCs, more information at line 110..
  */
  enum scctype {unknown, basic, complex, trivial};

  class SCC : public Graph {
  public:
    SCC();

    //Getter Functions:
    /*!
    * @return The type of a SCC.
    */
    scctype getSccType();

    /*!
    * @return The ID of a SCC.
    */
    int getId ();

    /*!
    * @return The vector with the IDs of all SCCs the actual SCC is connected to.
    */
    vector <int> getConnections();

    /*!
    * @return The map in which the verticies of a SCC are mapped to the verticies of the original graph.
    */
    map <Vertex*, Vertex*> getVertexMap();

    /*!
    * @return The map in which the verticies of the origianl graph are mapped to the verticies of the SCC.
    */
    map <Vertex*, Vertex*> getVertexMapReverse();



    //Setter Functions:
    /*!
    * Can be used to set the ID of an SCC.
    */
    void setId (int id);

    /*!
    * Sets the type of a SCC.
    */
    void setSCCType(scctype sT);

    /*!
    * Can be used to set the connection vector, which contains the information to which SCCs this SCC is connected to.
    * Does not contain any directional Information.
    */
    void setConnections(int conID);

    /*!
    * createVertexMap creates two maps. The first one maps the vertex in the SCC to the corresponding vertex in the
    * original graph and the second one maps a vertex in the original graph to its corresponding vertex in the SCC.
    */
    void createVertexMap(Vertex* V);

    /*!
    * printVertexMap() prints the verticies of a SCC and the verticies in the original graph which it is mapped to.
    * used for debugging in the first place.
    */
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
