#include <HatScheT/ResourceModel.h>

#include <algorithm>
#include <iostream>
#include <iomanip>



namespace HatScheT
{

ResourceModel::ResourceModel()
{

}

ostream& operator<<(ostream& os, const ResourceModel& rm)
{
  for(auto it:rm.resources)
  {
    Resource* r = &it;

    if(r->getLimit() > 0) os << "Resource Model has limited resource: " << r->getName() << " with limit " << r->getLimit() << ", latency "
                             << r->getLatency() << ", blockingTime " << r->getBlockingTime() << endl;
    if(r->getLimit() == -1) os << "Resource Model has unlimited resource: " << r->getName() << " with latency "
                               << r->getLatency() << ", blockingTime " << r->getBlockingTime() << endl;
  }

  //if(rm.vertexMap.size() == 0) os << "No vertices are registered to this resource model!" << endl;

  /*for(auto it:rm.vertexMap)
  {
    const Vertex* v = it.first;
    const ReservationTable* rt = it.second;

    if(rt->isEmpty() == false)
    {
      Resource* r = rt->getSingleResource();
      os << "Vertex " << v->getName() << " is registered to resource " << r->name << " with latency of " << rt->latency << endl;
    }
  }*/

  return os;
}

void ResourceModel::registerVertex(const Vertex *v, const Resource *r)
{
  this->registrations.insert({v, r});
}

const Resource* ResourceModel::getResource(const Vertex *v) const
{
  const auto it = this->registrations.find(v);
  if(it == this->registrations.end()) throw new Exception("ResourceModel.getResource: vertex not registered " + v->getName());
  else return it->second;
}

int ReservationTable::getLimit() const
{
  /*int limit = 0;

  for(auto it:this->table)
  {
    Resource* r = it.first;

    //unlimited
    if(limit == -1 ) continue;
    //init with first real limit
    if(limit == 0) limit = r->getLimit();
    else if(limit > 0)
    {
      if(r->getLimit() < limit) limit = r->getLimit();
    }
  }

  return limit;*/
}

int ReservationTable::getLatency() const
{
  /*int latency = 0;

  for(auto it:this->table)
  {
    Resource* r = it.first;

    if(latency < (r->getLatency()+it.second)) latency = r->getLatency()+it.second;
  }

  return latency;*/
}

ReservationBlock& ReservationTable::makeReservationBlock(Resource *r, int startTime)
{
  return *(this->blocks.emplace(this->blocks.end(),r, startTime));
}

Resource &ResourceModel::makeResource(string name, int limit, int latency, int blockingTime)
{
  return *(this->resources.emplace(this->resources.end(),name, limit, latency, blockingTime));
}

ReservationTable &ResourceModel::makeReservationTable(string name)
{
  return *(this->tables.emplace(this->tables.end(), name));
}

/*
ostream& operator<<(ostream& os, const ResourceModel& rm)
{
  for(auto it:rm.resources)
  {
    Resource* r = &it;

    os << "Resource Model has resource: " << r->name << " with " << r->availableUnits << " available units" << endl;
  }

  if(rm.vertexMap.size() == 0) os << "No vertices are registered to this resource model!" << endl;

  for(auto it:rm.vertexMap)
  {
    const Vertex* v = it.first;
    const ReservationTable* rt = it.second;

    if(rt->isEmpty() == false)
    {
      Resource* r = rt->getSingleResource();
      os << "Vertex " << v->getName() << " is registered to resource " << r->name << " with latency of " << rt->latency << endl;
    }
  }

  for(auto it: rm.reservationTables)
  {
    ReservationTable* rm = &it;

    rm->dump();
  }

  return os;
}

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

bool ResourceModel::resourceExists(string name) const
{
  for(const Resource &r:resources)
  {
    if(r.name == name) return true;
  }

  return false;
}

Resource* ResourceModel::getResource(Vertex* v) const
{
  const auto it = vertexMap.find(v);

  if(it != vertexMap.end())
  {
    const ReservationTable* rt = it->second;

    for(auto it2:resRtRelations)
    {
      if(rt == it2.second)
      {
        return it2.first;
      }
    }
  }

  throw Exception("ResourceModel.getResource: Requested vertex not found: " + v->getName());
}

Resource* ResourceModel::getResource(string name) const
{
  for(auto it:resources)
  {
    Resource* r = &it;

    if(r->name == name) return r;
  }

  throw Exception("ResourceModel.getResource: Requested Resource not found: " + name);
}

Resource* ResourceModel::makeResource(std::string name)
{
  return makeResource(name, 1);
}

Resource* ResourceModel::makeResource(std::string name, int available)
{
  return &*(resources.emplace(resources.end(), resources.size(), name, available));
}

const ReservationTable* ResourceModel::getEmptyReservationTableByLatency(int l) const
{
  for(const ReservationTable &r:reservationTables)
  {
    if(r.latency == l && r.isEmpty() == true) return &r;
  }

  throw Exception("ResourceModel.getEmptyReservationTablesByLatency: Requested latency not found: " + to_string(l));
}

ReservationTable* ResourceModel::makeReservationTable(int latency)
{
  return &*(reservationTables.emplace(reservationTables.end(), reservationTables.size(), latency));
}

ReservationTable* ResourceModel::makeEmptyReservationTable(int latency)
{
  return makeReservationTable(latency);
}

ReservationTable* ResourceModel::makeSimpleReservationTable(int latency, Resource* resource)
{
  auto rt = makeReservationTable(latency);
  rt->addReservation(resource);
  this->resRtRelations.insert(make_pair(resource, rt));
  return rt;
}

ReservationTable* ResourceModel::makeBlockReservationTablee(int latency, Resource* resource, int duration)
{
  auto rt = makeReservationTable(latency);
  rt->addReservation(resource, 0, duration);
  this->resRtRelations.insert(make_pair(resource, rt));
  return rt;
}

ReservationTable* ResourceModel::getRelatedRtByResName(std::string name) const
{
  for(auto it:resRtRelations)
  {
    Resource* r = it.first;
    if(r->name == name) return it.second;
  }

  throw Exception("ResourceModel.getRelatedRtByName: Requested Reservation Table to Resource not found: " + name);
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

int ResourceModel::getLatency(const Vertex *v) const
{
  const auto it = vertexMap.find(v);
  if (it == vertexMap.end())
    throw new Exception("Unregistered vertex");

  return it->second->latency;
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
}*/

}
