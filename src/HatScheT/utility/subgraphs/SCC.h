/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Benjamin Lagershausen-Kessler (benjaminkessler@student.uni-kassel.de)
            Patrick Sittel (sittel@uni-kassel.de)

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
#include <HatScheT/ResourceModel.h>
#include <vector>


namespace HatScheT {

  /*!
  * Datatyp to classify the SCCs, more information at line 140ff..
  */
  enum scctype {unknown, basic, complex, trivial};

  class SCC {
  public:
    SCC(Graph &g);

    //Getter Functions:
    /*!
     * @return Name of the SCC.
     */
    const string& getName() const {return this->name;}

    /*!
     * @return The ID of a SCC.
     */
    int getId () const;

    /*!
     * @return Number of Vertices which belong to the SCC.
     */
    int getNumberOfVertices() const;

    /*!
     * @return Number of Vertices with resource limits in SCC.
     */
    int getNumberofLimitedVertices() const { return numOfLimitedVerticesInSCC; }

    /*!
     * @return The type of a SCC.
     */
    scctype getSccType(ResourceModel* rm);

    /*!
     * @return A list of the Vertices which belong to an SCC.
     */
    list <Vertex*> getVerticesOfSCC();

    /*!
     * @return A map with the Vertices as key and a bool which tells if they belong to the SCC.
     */
    map <Vertex*, bool> getVertexInSccMap();

    /*!
     * this function must be called once to set this->sccEdges
     */
    void findSCCEdges();
    /*!
     * @return A list of edges which has Source and Destination in the SCC.
     */
    list<Edge*> getSCCEdges() const;

    int getNumOfEdges() const { return this->sccEdges.size(); }

    /*!
     * @return A list of SCC*s which are connected to the actual SCC.
     */
    list <SCC*> getConnectedSCCs();


    //Setter Functions:
    /*!
     * Sets the name of a SCC
     */
    void setName(string s){this->name = s;}

    /*!
     * Can be used to set the ID of an SCC.
     */
    void setId (int id);

    /*!
     * This function sets the Vertex V of Graph g as a part of the SCC.
     * @param V is the vertex which should be set as a part of the SCC
     */
    void setVertexAsPartOfSCC(Vertex* V);

    /*!
     * Sets a list, with the Neighbors of the SCC.
     */
    void setConnectedSCCs(list <SCC*> conSCCs);


    //Member Methods
    /*!
     * Used for debugging
     */
    void printVertexStatus();

    /*!
     * prints info about vertices and edges of this SCC -> used for debugging
     */
    void printInfo();

    /*!
     * Needed for sorting.
     * @param a
     * @return
     */
    bool operator<(const SCC &a) const;

  private:

  	/*!
  	 * a list of all edges of the original graph with source AND destination vertex inside this scc
  	 */
		list<Edge*> sccEdges;
    /*!
     * Pointer to the original graph.
     */
    Graph* g;

    /*!
     * ID of the SCC.
     */
    int id;

    /*!
     * Name of the SCC
     */
    string name;

    /*!
     * connectedSCCs contains the ID of each component which is connected to the actual SCC (inbound and outbound connections).
     */
    list <SCC*> connectedSCCs;

    /*!
     * Type of the SCC.
     * - unknown: Set by constructor, just for initialisation.
     * - trivial: SCCs which contain just 1 vertex.
     * - basic: SCCs which contain multiple Vertices, but none of them has a resource constraint.
     * - complex: SCCs which contain multiple Vertices, and at least one vertex has a resource constraint.
     */
    scctype typeOfSCC;

    /*!
     * vertexInSCC indicates, if a vertex of the Graph g belongs to the SCC.
     */
    map <Vertex*, bool> vertexInSCC;

    /*!
     * List of Vertices which belong to the SCC.
     */
    list <Vertex*> verticesOfSCC;

    int numOfLimitedVerticesInSCC;

  };

}
#endif //HATSCHET_SCC_H
