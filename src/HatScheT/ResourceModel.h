/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)

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

#include <HatScheT/Graph.h>
#include <HatScheT/Vertex.h>
#include <HatScheT/Edge.h>
#include <map>
#include <set>
#include <list>
#include <functional>
#include <HatScheT/utility/Exception.h>

namespace HatScheT
{

const int UNLIMITED = -1;

/*!
 * \brief The Resource class define a limited or unlimited resource, e.g. singlePrecAdd or doublePrecMult
 * use -1 as limit for an unlimited resource
 */
class Resource
{
public:
  /*!
   * \brief Resource
   * \param name
   * \param limit use -1 as limit for an unlimited resource
   * \param latency the number of clock cycles the resource need to perform the operation on one data sample
   * \param blockingTime the number of clock cycles the resource is blocked until the next data sample can be inserted
   */
  Resource(std::string name, int limit, int latency, int blockingTime) : name(name), limit(limit), latency(latency), blockingTime(blockingTime) {}
  /*!
   * copy constructor is forbidden for this class
   */
  Resource(const Resource&) = delete;
  /*!
   * \brief isReservationTable
   * \return
   */
  virtual bool isReservationTable(){
    return false;}
  /*!
   * \brief getLimit
   * \return
   */
  virtual int getLimit() const {
    return this->limit;}
  /*!
   * \brief getLatency
   * \return
   */
  virtual int getLatency() const {
    return this->latency;}
  /*!
   * \brief getBlockingTime
   * \return
   */
  virtual int getBlockingTime() const {
    return this->blockingTime;}
  /*!
   * \brief getName
   * \return
   */
  const std::string& getName() const {
    return this->name;}
protected:
  /*!
   * \brief name
   */
  const std::string name;
  /*!
   * \brief limit
   */
  const int limit;
  /*!
   * \brief latency
   */
  const int latency;
  /*!
   * \brief blockingTime
   */
  const int blockingTime;
};

/*!
 * \brief The ReservationBlock class describes one of possible many resource occupied by a reservation table at a specific start time
 */
class ReservationBlock
{
public:
  /*!
   * \brief ReservationBlock
   * \param resource provide the occupied resource
   * \param startTime provide the start time
   */
  ReservationBlock(const Resource* resource, int startTime) : resource(resource), startTime(startTime) {
    throw new Exception("ReservationBlock.ReservationBlock: ERROR from constructor! Currently disabled/not supported!");
  }
    /*!
   * copy constructor is forbidden for this class
   */
    ReservationBlock(const ReservationBlock&) = delete;
  /*!
   * \brief operator <<
   * \param os
   * \param rb
   * \return
   */
  friend ostream& operator<<(ostream& os, const ReservationBlock& rb);
  /*!
   * \brief getResource
   * \return
   */
  const Resource* getResource() const {return this->resource;}
  /*!
   * \brief getStartTime
   * \return
   */
  int getStartTime() const {return this->startTime;}
private:
  /*!
   * \brief resource
   */
  const Resource* resource;
  /*!
   * \brief startTime
   */
  const int startTime;
};

/*!
 * \brief The ReservationTable class describe a complex operation that is composite of many resources/operations
 * one of each operations is called a reservation block
 */
class ReservationTable : public Resource
{
public:
  /*!
   * \brief ReservationTable latency and limit are calculated based on the used reservation block resources
   * \param name provide a name
   */
  ReservationTable(std::string name) : Resource(name, -1, -1, -1) {
    throw new Exception("ReservationTable.ReservationTable: ERROR from constructor! Currently disabled/not supported!");
  }
  /*!
 * copy constructor is forbidden for this class
 */
  ReservationTable(const ReservationTable&) = delete;
  /*!
   * \brief operator <<
   * \param os
   * \param rt
   * \return
   */
  friend ostream& operator<<(ostream& os, const ReservationTable& rt);
  /*!
   * \brief makeReservationBlock factory to generate and add a reservation block to this reservation table
   * \param r
   * \param startTime
   * \return
   */
  ReservationBlock &makeReservationBlock(Resource* r, int startTime);
  /*!
   * \brief isReservationTable
   * \return
   */
  virtual bool isReservationTable(){
    return true;}
  /*!
   * \brief getLimit limit is lowest limit of all resources used
   * \return
   */
  virtual int getLimit() const;
  /*!
   * \brief getLatency latency is the last finished block
   * \return
   */
  virtual int getLatency() const;
  /*!
   * \brief getBlockingTime
   * \return
   */
  virtual int getBlockingTime() const {
    throw new Exception("ReservationTable.getBlockingTime: blockingTime not supported for RT!" );}
private:
  /*!
   * \brief blocks
   */
  std::list<ReservationBlock*> blocks;
};

/*!
 * \brief The ResourceModel manages the resource types known to the schedulers, and the association of vertices to a particular resource type.
 *
 * The schedulers implemented in HatScheT currently support only simple resource models where each operation requires exactly one fully-pipelined resource in its start time step.
 * In the future, support for blocking resource usage, as well as complex reservation tables, will be added.
 * 
 * \sa Resource
 *     ReservationTable
 */
class ResourceModel
{
public:
  /*!
   * \brief ResourceModel
   */
  ResourceModel();
  /*!
   * \brief ResourceModel
   */
  ~ResourceModel();
  /*!
   * copy constructor is forbidden for this class
   */
  ResourceModel(const ResourceModel&) = delete;
  /*!
   * \brief operator <<
   * \param os
   * \param rm
   * \return
   */
  friend ostream& operator<<(ostream& os, const ResourceModel& rm);
  /*!
   * \brief makeResource generate a resource in your resource model
   * \param name
   * \param limit
   * \param latency
   * \param blockingTime
   * \return
   */
  Resource &makeResource(std::string name, int limit, int latency, int blockingTime=0);
  /*!
   * \brief makeReservationTable generate a more complex operation
   * \param name
   * \return
   */
  ReservationTable& makeReservationTable(string name);
  /*!
   * \brief makeReservationBlock generate a reservation block of resource r in reservation table rt at start time
   * \param rt
   * \param r
   * \param startTime
   * \return
   */
  ReservationBlock& makeReservationBlock(ReservationTable* rt, Resource* r, int startTime);
  /*!
   * \brief registerVertex register a vertex to resource
   * \param v
   * \param r
   */
  void registerVertex(const Vertex* v, const Resource* r);
  /*!
   * \brief getResource
   * \param v
   * \return
   */
  const Resource *getResource(const Vertex* v) const;
  /*!
   * \brief getResource
   * \param name
   * \return
   */
  Resource *getResource(std::string name) const;
  /*!
   * \brief isEmpty
   * \return
   */
  bool isEmpty();
  /*!
   * \brief resourcesBegin iterate over resources
   * \return
   */
  const std::list<Resource*>::iterator resourcesBegin()
  {
      return resources.begin();
  }
  /*!
   * \brief resourcesEnd iterate over resources
   * \return
   */
  const std::list<Resource*>::iterator resourcesEnd()
  {
      return resources.end();
  }
  /*!
   * \brief getNoOfResource
   * \return
   */
  int getNoOfResources() const {return this->resources.size();}
  /*!
   * \brief getNoOfReservationTables
   * \return
   */
  int getNoOfReservationTables() const {return this->tables.size();}
  /*!
   * \brief getVertexLatency determine the latency of a registered vertex
   * \param v
   * \return
   */
  int getVertexLatency(Vertex* v) const;
  int getVertexLatency(const Vertex* v) const;
  /*!
   * \brief getMaxLatency determine the maximal latency of all registered latencies
   * TODO: HANDLE LATENCY/DELAY OF EDGES
   * \return
   */
  int getMaxLatency() const;
  /*!
   * \brief getNoOfVerticesRegisteredToResource
   * \param r
   * \return
   */
  int getNoOfVerticesRegisteredToResource(Resource* r) const;
  /*!
   * \brief getVerticesOfResources
   * \param r
   * \return
   */
  set<const Vertex*> getVerticesOfResource(const Resource *r) const;
private:
  /*!
   * \brief registrations
   */
  map<const Vertex*, const Resource*> registrations;
  /*!
   * \brief resources
   */
  std::list<Resource*> resources;
  /*!
   * \brief tables
   */
  std::list<ReservationTable*> tables;
};

}
