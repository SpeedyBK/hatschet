#pragma once

#include <HatScheT/Graph.h>
#include <HatScheT/Vertex.h>
#include <HatScheT/Edge.h>
#include <map>
#include <set>
#include <list>
#include <functional>
#include <HatScheT/Exception.h>

namespace HatScheT
{
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
  std::string getName() const {
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
 * \brief The ReservationBlock class descripes one of possible many resource occupied by a reservation table at a specific start time
 */
class ReservationBlock
{
public:
  /*!
   * \brief ReservationBlock
   * \param resource provide the occupied resource
   * \param startTime provide the start time
   */
  ReservationBlock(const Resource* resource, int startTime) : resource(resource), startTime(startTime) {}
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
  const int getStartTime() const {return this->startTime;}
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
 * \brief The ReservationTable class descripe a complex operation that is composite of many resources/operations
 * one of each operations is called a reservation block
 */
class ReservationTable : public Resource
{
public:
  /*!
   * \brief ReservationTable latency and limit are calculated based on the used reservation block resources
   * \param name provide a name
   */
  ReservationTable(std::string name) : Resource(name, -1, -1, -1) {}
  /*!
   * \brief operator <<
   * \param os
   * \param rt
   * \return
   */
  friend ostream& operator<<(ostream& os, const ReservationTable& rt);
  /*!
   * \brief makeReservationBlock factory to generate and add a reservation block to this reservation tbale
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
 * \brief The ResourceModel class use this class to manage the resources of your implementation to schedule your data flow graph
 */
class ResourceModel
{
public:
  /*!
   * \brief ResourceModel
   */
  ResourceModel();
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
  Resource &makeResource(std::string name, int limit, int latency, int blockingTime);
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
