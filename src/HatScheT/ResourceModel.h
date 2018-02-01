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

class Resource
{
public:
  Resource(std::string name, int limit, int latency, int blockingTime) : name(name), limit(limit), latency(latency), blockingTime(blockingTime) {}

  //convenience
  virtual bool isReservationTable(){return false;}
  virtual int getLimit() const {return this->limit;}
  virtual int getLatency() const {return this->latency;}
  virtual int getBlockingTime() const {return this->blockingTime;}
  std::string getName() const {return this->name;}
protected:
  const std::string name;
  const int limit;
  const int latency;
  const int blockingTime;

private:

};

class ReservationBlock
{
public:
  ReservationBlock(Resource* resource, int startTime) : resource(resource), startTime(startTime) {}

  const Resource* getResource() const {return this->resource;}
  const int getStartTime() const {return this->startTime;}
protected:
  const Resource* resource;
  const int startTime;
private:
};

class ReservationTable : public Resource
{
public:
  ReservationTable(std::string name) : Resource(name, -1, -1, -1) {}
  ReservationBlock& makeReservationBlock(Resource* r, int startTime);

  //convenience
  virtual bool isReservationTable(){return true;}
  virtual int getLimit() const;
  virtual int getLatency() const;
  virtual int getBlockingTime() const {throw new Exception("ReservationTable.getBlockingTime: blockingTime not supported for RT!" );}
protected:


private:
  std::list<ReservationBlock> blocks;
};

class ResourceModel
{
public:
  ResourceModel();
  friend ostream& operator<<(ostream& os, const ResourceModel& rm);

  //factories
  Resource& makeResource(std::string name, int limit, int latency, int blockingTime);
  ReservationTable& makeReservationTable(string name);

  //registration
  void registerVertex(const Vertex* v, const Resource* r);

  //convenience
  const Resource* getResource(const Vertex* v) const;
protected:

private:
  map<const Vertex*, const Resource*> registrations;
  std::list<Resource> resources;
  std::list<ReservationTable> tables;
};

/*class Resource
{
public:
  const int id;
  const std::string name;
  const int availableUnits;
  //here needed?! (PS)
  //const int latency;
  //const int blockingTime;

  Resource(int id, std::string name, int availableUnits) : id(id), name(name), availableUnits(availableUnits) { }
};

struct BlockReservation
{
  const int start, duration;

  BlockReservation(int start, int duration) : start(start), duration(duration) { }
};

class ReservationTable
{
public:
  const int id;
  const int latency;

  ReservationTable(int id, int latency) : id(id), latency(latency) { }

  // builder interface
  void addReservation(Resource* resource);
  void addReservation(Resource* resource, int start);
  void addReservation(Resource* resource, int start, int duration);

  // query interface
  //  1) "raw" access to the table
  std::set<Resource*>         getResources() const;
  std::list<BlockReservation> getReservations(Resource* resource) const;
  //  2) query a specific timestep
  bool                        usesResourceAt(Resource* resource, int timestep) const;
  int                         getResourceDemandAt(Resource* resource, int timestep) const;
  //  3) summary view
  int                         getMaxResourceDemand(Resource* resource) const;
  //  4) convenience access for simple and block tables. Return nullptr or {-1,-1} if the query cannot be answered uniquely.
  Resource*                   getSingleResource() const;
  BlockReservation            getSingleReservation() const;

  bool isEmpty() const;
  bool isSimple() const;
  bool isBlock() const;
  bool isComplex() const;

  void dump() const;

private:
  std::map<Resource*, std::list<BlockReservation>> reservationTable;
};*/

/*!
 * \brief ResourceModel
 */
/*
class ResourceModel
{
public:
  ResourceModel();
  friend ostream& operator<<(ostream& os, const ResourceModel& rm);

  // factories for Resources
  // Patrick ToDo(?): check for uniqueness
  Resource* makeResource(std::string name);
  Resource* makeResource(std::string name, int available);

  // factories for ReservationTables. Any of these can be extended to be a complex table.
  ReservationTable* makeEmptyReservationTable(int latency);
  ReservationTable* makeSimpleReservationTable(int latency, Resource* resource);
  ReservationTable* makeBlockReservationTablee(int latency, Resource* resource, int duration);

  // association of vertices/reservation tables
  void registerVertex(const Vertex* v, const ReservationTable* reservationTable);
  const ReservationTable* getReservationTable(const Vertex* v) const;

  // access to all known reservation tables
  std::list<const ReservationTable*> getReservationTables() const;
  const ReservationTable* getEmptyReservationTableByLatency(int l) const;

  // convenience
  bool isUnlimited(const Vertex* v) const;
  bool isResourceConstrained(const Vertex* v) const;
  int getLatency(const Vertex* v) const;
  std::set<const Vertex*> getUnlimitedVertices() const;
  std::set<const Vertex*> getResourceConstrainedVertices(const ReservationTable* reservationTable) const;
  Resource *getResource(std::string name) const;
  Resource* getResource(Vertex* v) const;
  bool resourceExists(std::string name) const;
  ReservationTable* getRelatedRtByResName(std::string name) const;

protected:


private:
  std::map<const Vertex*, const ReservationTable*> vertexMap;
  std::list<Resource> resources;
  std::list<ReservationTable> reservationTables;
  std::map<Resource*, ReservationTable*> resRtRelations;

  ReservationTable* makeReservationTable(int latency);
  std::set<const Vertex*> getFilteredVertices(std::function<bool(const ReservationTable*)> predicate) const;
};*/
}
