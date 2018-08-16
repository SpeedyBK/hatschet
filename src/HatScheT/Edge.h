/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Martin Kumm, Patrick Sittel ({kumm, sittel}@uni-kassel.de)
    Author: Julian Oppermann (oppermann@esa.tu-darmstadt.de)

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

#pragma once

#include "Vertex.h"

namespace HatScheT
{

/*!
 * \brief Implementation of a edge used in a Graph
 *
 * An edge expresses a precedence relation between the operations represented by the source and destination vertices.
 * Each edge carries two attributes:
 *
 * The **delay** is interpreted as an additional number of time steps that need to pass before `dst` can start after `src` finishes.
 * While the main use for this facility is to support (or more precisely, limit) operator chaining, it could be used to model
 * special propagation delays as well.
 *
 * The **distance** is only relevant for modulo scheduling problems. A non-zero distance indicates that a dependence
 * does not hold for the current iteration (or input sample), but `distance`-many iterations later.
 *
 * In the most generic case, an edge represents the following linear constraint between the start times \f$t_{src}\f$ and \f$t_{dst}\f$:
 *
 * \f$ t_{src} + ResModel.getVertexLatency(src) + delay <= t_{dst} + distance \cdot II \f$
 *
 * \sa Graph
 *     ResourceModel
 */
class Edge
{
public:
  enum DependencyType { Data, Precedence };
   /**
   * @brief Edge Constructor
   * @param Vsrc The source vertex
   * @param Vdst The destination vertex
   * @param distance The distance (=no of registers) on an edge
   * @param dependencyType Enum to distinguish dependencies and data edges
   */
  Edge(Vertex &src, Vertex &dst, int distance=0, DependencyType dependencyType=Data);
  /*!
   * copy constructor is forbidden for this class
   */
   Edge(const Edge&) = delete;

  Vertex& getVertexSrc(){ return Vsrc; } /*!< \return the source vertex */
  Vertex& getVertexDst(){ return Vdst; } /*!< \return the destination vertex */

  const string getVertexSrcName() const {return this->Vsrc.getName();} /*!< \return the source vertex's name */
  const string getVertexDstName() const {return this->Vdst.getName();} /*!< \return the destination vertex's name */

  /*! \return the edge delay */
  int getDelay() const { return this->delay; } /*!< \return the edge delay */
  /*! \brief sets the edge delay
   * @param delay the new delay
   */
  void setDelay(int delay) { this->delay = delay; }

  /*! \return the edge distance */
  int getDistance() const { return distance; }
  /*! \brief sets the edge distance
   * @param distance the new distance
   */
  void setDistance(int distance){ this->distance = distance; }

  /*! \return the edge ID */
  int getID() const {return this->id;}
  /*! \brief sets the edge ID
   * @param id the new ID
   */
  void setID(int id){this->id = id;}

  /*!
   * \brief sets the dependency type for this edge
   * @param dt the new dependency type
   */
  void setDependencyType(DependencyType dt){this->dependencyType = dt;}
  /*! \return the dependency type */
  DependencyType getDependencyType() const {return this->dependencyType;}

  /*!
   * @return whether this edge has the `Data` dependency type
   */
  bool isDataEdge();
protected:
  /*! \brief the edge ID */
  int id = -1;
  /*! \brief the edge's dependency type (i.e. whether the edge represents data flow, or only models a precedence relation) */
  DependencyType dependencyType;

  /**
   * @brief defines the integer edge delay, i.e. an additional delay between src and dst vertices.
   *
   * The delay is intended to be inferred internally by HatScheT, and thus is not a constructor argument.
   */
  int delay = 0;

  /**
   * @brief defines the integer distance in the schedule.
   *
   * Used to model algorithmic/functional delays (like appear in digital filters), or to indicate a dependence has to hold after a certain number of iterations (in a C-based HLS context).
   */
  int distance;

  /*!
   * \brief the source vertex
   */
  Vertex &Vsrc;

  /*!
   * \brief the destination vertex
   */
  Vertex &Vdst;

};
ostream& operator<<(ostream& os, const Edge& e);
}
