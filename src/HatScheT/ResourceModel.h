#pragma once

#include <HatScheT/Graph.h>
#include <HatScheT/Vertex.h>
#include <HatScheT/Edge.h>
#include <map>
#include <set>
#include <list>
#include <functional>

namespace HatScheT
{

class Resource
{
public:
  const int id;
  const std::string name;
  const int availableUnits;

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
};

/*!
 * \brief ResourceModel
 */
class ResourceModel
{
public:
  // TODO: graph is unused. Remove?
  ResourceModel(Graph &g) : g(g) { }

  // factories for Resources
  Resource* makeResource(std::string name);
  Resource* makeResource(std::string name, int available);

  // factories for ReservationTables. Any of these can be extended to be a complex table.
  ReservationTable* getEmptyReservationTable(int latency);
  ReservationTable* getSimpleReservationTable(int latency, Resource* resource);
  ReservationTable* getBlockReservationTable(int latency, Resource* resource, int duration);

  // association of vertices/reservation tables
  void registerVertex(const Vertex* v, const ReservationTable* reservationTable);
  const ReservationTable* getReservationTable(const Vertex* v) const;

  // access to all known reservation tables
  std::list<const ReservationTable*> getReservationTables() const;

  // convenience
  bool isUnlimited(const Vertex* v) const;
  bool isResourceConstrained(const Vertex* v) const;
  std::set<const Vertex*> getUnlimitedVertices() const;
  std::set<const Vertex*> getResourceConstrainedVertices(const ReservationTable* reservationTable) const;

protected:
  Graph &g;

private:
  std::map<const Vertex*, const ReservationTable*> vertexMap;
  std::list<Resource> resources;
  std::list<ReservationTable> reservationTables;

  ReservationTable* makeReservationTable(int latency);
  std::set<const Vertex*> getFilteredVertices(std::function<bool(const ReservationTable*)> predicate) const;
};
}
