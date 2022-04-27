/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)
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

//class Utility; // forward declaration for cyclic include
const int UNLIMITED = -1; // code for unlimited resource type

/*!
 * \brief Instances of this class represent a simple resource
 */
class Resource
{
public:
  /*!
   * \brief Constructor
   * \param name the resource's name
   * \param limit the number of available instances. Use HatScheT::UNLIMITED (= -1) to denote an unlimited resource
   * \param latency the number of time steps the resource needs to complete its function
   * \param blockingTime the number of time steps a resource instance is blocked by an individual operation
   */
  Resource(std::string name, int limit, int latency, int blockingTime);
  /*!
   * copy constructor is forbidden for this class
   */
  Resource(const Resource&) = delete;
  /*!
   * \return whether this is a (complex) reservation table
   */
  virtual bool isReservationTable(){return false;}
  /*!
   * a resource is unlimited iff their limit is -1
   * @return
   */
  virtual bool isUnlimited() const {
    return (this->limit == -1);
  }
  /*!
   * \return the limit
   */
  virtual int getLimit() const {return this->limit;}
  /*!
   * set resource limit
   * @param l new limit
   */
  void setLimit(int l);
  /*!
   * this is used for resources that have different limits in different congruence classes
   * @param congruenceClass
   * @return the limit in that congruence class
   */
	virtual int getNonRectLimit(int congruenceClass) const;
	/*!
	 * this returns the total number of mrt slots.
	 * for a rectangular mrt this would be limit * II
	 * but for a non-rect one we have to count the mrt heights in each modulo slot
	 */
	virtual int getTotalNonRectSlots();
	/*!
	 * this can be used to specify the limit in a specific congruence class
	 * @param congruenceClass
	 * @param l new limit in that congruence class
	 */
  void setNonRectLimit(int congruenceClass, int l);
  /*!
   * \return the latency
   */
  virtual int getLatency() const {return this->latency;}
  /*!
   * \return the blocking time
   */
  virtual int getBlockingTime() const {return this->blockingTime;}
  /*!
   * \return the name
   */
  const std::string& getName() const {return this->name;}
  /*!
   * \brief the hardware costs of this resource are modeled using this map, e.g. LUT -> 127, DSP -> 1
   * @return
   */
  map<string,double>& getHardwareCosts(){return this->hardwareCost;}
  void addHardwareCost(string n, double c);
  double getHardwareCost(string n);
  /*!
   * the physical delay of this resource in hardware
   * @param d
   */
  void setPhysicalDelay(double d){
    if(d < 0.0) throw Exception("Resource.setPhysicalDelay: ERROR a physical dealy has to be >= 0 : " + this->name);
    this->phyDelay = d;}
    double getPhysicalDelay(){return this->phyDelay;}
protected:
  /*!
   * \brief the resource's name
   */
  const std::string name;
  /*!
   * \brief the number of available instances. Use HatScheT::UNLIMITED (= -1) to denote an unlimited resource
   */
  int limit;
  /*!
   * \brief this map is used for resources with non-rectangular MRTs, i.e. the limit depends on the congruence class
   */
  std::map<int, int> nonRectLimit;
  /*!
   * \brief the number of time steps the resource needs to complete its function
   */
  const int latency;
  /*!
   * \brief the number of time steps a resource instance is blocked by an individual operation
   */
  int blockingTime;
  /*!
   * the physical delay of this resource in hardware
   */
  double phyDelay;
  /*!
   * \brief the hardware costs of this resource are modeled using this map, e.g. LUT -> 127, DSP -> 1
   */
  map<string, double> hardwareCost;
};

/*!
 * \brief The ReservationBlock class describes one of possible many resource occupied by a reservation table at a specific start time
 */
class ReservationBlock
{
public:
  /*!
   * \brief ReservationBlock
   * \param resource the occupied resource
   * \param startTime the start time
   */
  ReservationBlock(const Resource* resource, int startTime) : resource(resource), startTime(startTime) {
    throw HatScheT::Exception("ReservationBlock.ReservationBlock: ERROR from constructor! Currently disabled/not supported!");
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
   * \return the resource
   */
  const Resource* getResource() const {return this->resource;}
  /*!
   * \return this block's start time
   */
  int getStartTime() const {return this->startTime;}
private:
  /*!
   * \brief the resource
   */
  const Resource* resource;
  /*!
   * \brief the start time (relative to time 0 of the surrounding reservation table, and the vertex's start time)
   */
  const int startTime;
};

/*!
 * \brief The ReservationTable class describes a complex resource comprised of several reservation blocks.
 *
 * Reservation tables are currently unsupported by HatScheT's schedulers.
 *
 * \sa ReservationBlock
 */
class ReservationTable : public Resource
{
public:
  /*!
   * \brief Constructor.
   *
   * Latency and limit are calculated based on the used reservation block resources.
   * \param name the table's name
   */
  ReservationTable(std::string name) : Resource(name, -1, -1, -1) {
    throw HatScheT::Exception("ReservationTable.ReservationTable: ERROR from constructor! Currently disabled/not supported!");
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
   * \brief factory to create and add a reservation block to this reservation table
   * \param r the block's resource
   * \param startTime the block's start time
   * \return the new reservation block
   */
  ReservationBlock &makeReservationBlock(Resource* r, int startTime);
  /*!
   * \return true
   */
  virtual bool isReservationTable(){return true;}
  /*!
   * \return lowest limit of all resources used
   */
  virtual int getLimit() const;
  /*!
   * \return the end time step of the latest finishing reservation block
   */
  virtual int getLatency() const;
  /*!
   * \brief not applicable
   * \return Nothing
   */
  virtual int getBlockingTime() const {throw HatScheT::Exception("ReservationTable.getBlockingTime: blockingTime not supported for RT!" );}
private:
  /*!
   * \brief all created reservation blocks
   */
  std::list<ReservationBlock*> blocks;
};

/*!
 * \brief The ResourceModel manages the resource types known to the schedulers, and the association of vertices to a particular resource type.
 *
 * Use the `makeResource` method to create new resources, and `registerVertex` to associate a vertex with a resource.
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
   * \brief Constructor.
   */
  ResourceModel();
  /*!
   * \brief Destructor.
   *
   * Frees all resources and reservation tables.
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
   * \brief creates a new (simple) resource
   * \param name the resource's name
   * \param limit the number of available instances
   * \param latency the number of time steps the resource needs to complete its function
   * \param blockingTime the number of time steps a resource instance is blocked by an individual operation
   * \return the new resource
   * \sa Resource
   */
  Resource &makeResource(std::string name, int limit, int latency, int blockingTime=1);
  /*!
   * \brief creates a complex, reservation table-based resource
   *
   * Currently not supported!
   *
   * \param name the reservation table's name
   * \return nothing
   */
  ReservationTable& makeReservationTable(string name);
  /*!
   * \brief associates a vertex with a resource
   * \param v the vertex
   * \param r the resource (or reservation table)
   */
  void registerVertex(const Vertex* v, const Resource* r);
  /*!
   * check whether a resource of name already exists
   * @param name
   * @return
   */
  bool resourceExists(std::string name);
  /*!
   * \param v the vertex
   * \return the resource associated with the given vertex
   */
  const Resource *getResource(const Vertex* v) const;
  /*!
   * \param name the resource name to look up
   * \return the resource with the given name
   * \throws Exception if no matching resource was foudnd
   */
  Resource *getResource(std::string name) const;
  /*!
   * \return whether this resource model is empty, i.e. has no resources
   */
  bool isEmpty();
  /*!
   * \return iterator to beginning of resources
   */
  const std::list<Resource*>::iterator resourcesBegin()
  {
      return resources.begin();
  }
  /*!
   * \return iterator to end of resources
   */
  const std::list<Resource*>::iterator resourcesEnd()
  {
      return resources.end();
  }
  /*!
   * \return number of resources
   */
  int getNumResources() const {return this->resources.size();}
  /*!
   * \return number of reservation tables
   */
  int getNumReservationTables() const {return this->tables.size();}
  /*!
   * \brief Convenience method to get the given vertex's latency
   * \param v the vertex
   * \return `v`'s latency
   * \throws Exception if vertex is not registered
   */
  int getVertexLatency(Vertex* v) const;
  /*!
   * \brief Convenience method to get the given vertex's latency
   * \param v the vertex
   * \return `v`'s latency
   * \throws Exception if vertex is not registered
   */
  int getVertexLatency(const Vertex* v) const;
  /*!
   * \brief determines the maximum latency of all registered latencies
   *
   * TODO: HANDLE LATENCY/DELAY OF EDGES
   * \return the maximum latency
   */
  int getMaxLatency() const;
  /*!
   * \brief determines how many vertices are registered to the given resource
   * \param r the resource
   * \return the number of vertices associated with `r`
   */
  int getNumVerticesRegisteredToResource(Resource *r) const;
  int getNumVerticesRegisteredToResource(const Resource *r) const;
  /*!
   * \brief collects all vertices that use the given resource
   * \param r the resource
   * \return a set of vertices associated with `r`
   */
  set<const Vertex*> getVerticesOfResource(const Resource *r) const;
  /*!
   * @brief Resource iterator
   */
  class Resources
  {
  public:
    Resources(const std::list<Resource*>::iterator& begin, const std::list<Resource*>::iterator& end) : b(begin), e(end) {}

    std::list<Resource*>::iterator begin() { return b; }
    std::list<Resource*>::iterator end() { return e; }
    std::list<Resource*>::iterator b;
    std::list<Resource*>::iterator e;
  };

  Resources Resources()
  {
    return {resources.begin(),resources.end()};
  }
private:
	/*!
	 * \brief the mapping between vertices and resources
	 */
	map<const Vertex*, const Resource*> registrations;
	/*!
	 * \brief the mapping between resources and vertices
	 */
	map<const Resource*, std::set<const Vertex*>> reverseRegistrations;
  /*!
   * \brief all created resources
   */
  std::list<Resource*> resources;
  /*!
   * \brief all created tables
   */
  std::list<ReservationTable*> tables;
};

}
