#include <HatScheT/ResourceModel.h>

#include <algorithm>
#include <iostream>
#include <iomanip>

#include <HatScheT/Exception.h>

namespace HatScheT
{

void ReservationTable::addReservation(Resource* resource)
{
  addReservation(resource, 0);
}

void ReservationTable::addReservation(Resource* resource, int start)
{
  addReservation(resource, start, 1);
}

void ReservationTable::addReservation(Resource* resource, int start, int duration)
{
  if (start + duration > latency)
    throw new Exception("Resource reservation exceeds latency"); // TODO: discuss error handling
  reservationTable[resource].emplace_back(start, duration);
}

std::set<Resource*> ReservationTable::getResources() const
{
  std::set<Resource*> res;
  for (const auto& p : reservationTable)
    res.insert(p.first);
  return res;
}

std::list<BlockReservation> ReservationTable::getReservations(Resource* resource) const
{
  const auto it = reservationTable.find(resource);
  return it != reservationTable.end() ? it->second : std::list<BlockReservation>();
}

bool ReservationTable::usesResourceAt(Resource* resource, int timestep) const
{
  return getResourceDemandAt(resource, timestep) > 0;
}

int ReservationTable::getResourceDemandAt(Resource* resource, int timestep) const
{
  const auto it = reservationTable.find(resource);
  if (it == reservationTable.end())
    return false;

  const auto& blocks = it->second;
  return (int) std::count_if(blocks.begin(), blocks.end(),
               [timestep] (BlockReservation B) -> bool {
                 return B.start <= timestep && timestep < B.start + B.duration;
               });
}

int ReservationTable::getMaxResourceDemand(Resource* resource) const
{
  int maxDemand = 0;
  for (int t = 0; t < latency; ++t)
    maxDemand = std::max(maxDemand, getResourceDemandAt(resource, t));
  return maxDemand;
}

Resource* ReservationTable::getSingleResource() const
{
  if (reservationTable.size() != 1)
    return nullptr;
  return reservationTable.begin()->first;
}

BlockReservation ReservationTable::getSingleReservation() const
{
  if (reservationTable.size() != 1)
    return BlockReservation(-1, -1);
  const auto& blocks = reservationTable.begin()->second;
  if (blocks.size() != 1)
    return BlockReservation(-1, -1);
  return *blocks.begin();
}

bool ReservationTable::isEmpty() const
{
  return reservationTable.empty();
}

bool ReservationTable::isSimple() const
{
  auto block = getSingleReservation();
  return block.start == 0 && block.duration == 1;
}

bool ReservationTable::isBlock() const
{
  auto block = getSingleReservation();
  return block.start == 0 && block.duration > 1;
}

bool ReservationTable::isComplex() const
{
  return !(isEmpty() || isSimple() || isBlock());
}

void ReservationTable::dump() const
{
  using namespace std;

  const int nameColumnWidth = 16;
  string hline = string(nameColumnWidth + 2 + 5*latency, '-');
  string header = " id=" + to_string(id) + " latency=" + to_string(latency) + " ";
  string headerLine = hline;
  headerLine.replace(3, header.size(), header);

  cout << headerLine << endl;
  cout << setw(nameColumnWidth) << "Resource name" << " |";
  for (int t = 0; t < latency; ++t)
    cout << setw(3) << t << " |";
  cout << endl << hline << endl;

  for (const auto& p : reservationTable) {
    auto resource = p.first;
    cout << setw(nameColumnWidth) << (resource->name + "(" + to_string(resource->id) + ")").substr(0, 20) << " |";
    for (int t = 0; t < latency; ++t)
      cout << setw(3) << getResourceDemandAt(resource, t) << " |";
    cout << endl;
  }
  cout << hline << endl;
}

Resource* ResourceModel::makeResource(std::string name)
{
  return makeResource(name, 1);
}

Resource* ResourceModel::makeResource(std::string name, int available)
{
  return &*(resources.emplace(resources.end(), resources.size(), name, available));
}

ReservationTable* ResourceModel::makeReservationTable(int latency)
{
  return &*(reservationTables.emplace(reservationTables.end(), reservationTables.size(), latency));
}

ReservationTable* ResourceModel::getEmptyReservationTable(int latency)
{
  return makeReservationTable(latency);
}

ReservationTable* ResourceModel::getSimpleReservationTable(int latency, Resource* resource)
{
  auto rt = makeReservationTable(latency);
  rt->addReservation(resource);
  return rt;
}

ReservationTable* ResourceModel::getBlockReservationTable(int latency, Resource* resource, int duration)
{
  auto rt = makeReservationTable(latency);
  rt->addReservation(resource, 0, duration);
  return rt;
}

void ResourceModel::registerVertex(const Vertex *v, const ReservationTable *reservationTable)
{
  vertexMap[v] = reservationTable;
}

const ReservationTable* ResourceModel::getReservationTable(const Vertex *v) const
{
  const auto it = vertexMap.find(v);
  return it != vertexMap.end() ? it->second : nullptr;
}

std::list<const ReservationTable*> ResourceModel::getReservationTables() const
{
  std::list<const ReservationTable*> resTabs;
  for (const auto& rt : reservationTables)
    resTabs.push_back(&rt);
  return resTabs;
}

bool ResourceModel::isUnlimited(const Vertex *v) const
{
  const auto it = vertexMap.find(v);
  if (it == vertexMap.end())
    throw new Exception("Unregistered vertex");
  return it->second->isEmpty();
}

bool ResourceModel::isResourceConstrained(const Vertex *v) const
{
  return ! isUnlimited(v);
}

std::set<const Vertex*> ResourceModel::getFilteredVertices(std::function<bool(const ReservationTable*)> predicate) const
{
  std::set<const Vertex*> filteredVertices;
  for (const auto& p : vertexMap)
    if (predicate(p.second))
      filteredVertices.insert(p.first);
  return filteredVertices;
}

std::set<const Vertex*> ResourceModel::getUnlimitedVertices() const
{
  return getFilteredVertices([] (const ReservationTable* rt) -> bool { return rt->isEmpty(); });
}

std::set<const Vertex*> ResourceModel::getResourceConstrainedVertices(const ReservationTable* reservationTable) const
{
  return getFilteredVertices([reservationTable] (const ReservationTable* rt) -> bool { return rt == reservationTable; });
}

}
