#pragma once

#include <HatScheT/Graph.h>
#include <HatScheT/Vertex.h>
#include <HatScheT/Edge.h>
#include <map>
#include <set>
#include <list>
#include <memory>

namespace HatScheT
{

struct Resource
{
  const int id;
  const std::string name;
  const int availableUnits;

  Resource(int id, std::string name, int availableUnits);
};

typedef std::shared_ptr<Resource> ResourceRef;

struct BlockReservation
{
  const int start, duration;
};

class ReservationTable
{
public:
  // builder interface
  void addReservation(ResourceRef resource);
  void addReservation(ResourceRef resource, int start);
  void addReservation(ResourceRef resource, int start, int duration);

  // query interface
  //  1) "raw" access to the table
  std::set<ResourceRef>       getResources() const;
  std::list<BlockReservation> getReservations(ResourceRef resource) const;
  //  2) query a specific timestep
  bool                        usesResourceAt(ResourceRef resource, int timestep) const;
  int                         getResourceDemandAt(ResourceRef resource, int timestep) const;
  //     ... do we need a "give me all resources (and their demand) at t" method?
  //  3) summary view
  int                         getMaxResourceDemand(ResourceRef resource) const;


  bool isSimple() const;
  bool isBlock() const;
  bool isComplex() const;

  void dump() const;

private:
  std::map<ResourceRef, std::list<BlockReservation>> table;
};

typedef std::shared_ptr<ReservationTable> ReservationTableRef;

/*!
 * \brief ResourceModel
 */
class ResourceModel
{
public:
  ResourceModel(Graph &g);

  // factories for Resources
  ResourceRef makeResource(std::string name);
  ResourceRef makeResource(std::string name, int available);

  // factories for ReservationTables
  ReservationTableRef getEmptyReservationTable();
  ReservationTableRef getSimpleReservationTable(ResourceRef resource);
  ReservationTableRef getBlockReservationTable(ResourceRef resource, int duration);

  // association of vertices/reservation tables
  void registerVertex(Vertex v, ReservationTableRef reservationTable);
  ReservationTableRef getReservationTable(Vertex v);

protected:
  Graph &g;

private:
  // TODO: we want to encapsulate the resource handling from the vertices, right?
  // TODO: by value?
  std::map<Vertex, ReservationTableRef> resourceMap;
};
}
