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
  for (auto p : reservationTable)
    res.insert(p.first);
  return res;
}

std::list<BlockReservation> ReservationTable::getReservations(Resource* resource) const
{
  auto it = reservationTable.find(resource);
  return it != reservationTable.end() ? it->second : std::list<BlockReservation>();
}

bool ReservationTable::usesResourceAt(Resource* resource, int timestep) const
{
  return getResourceDemandAt(resource, timestep) > 0;
}

int ReservationTable::getResourceDemandAt(Resource* resource, int timestep) const
{
  auto it = reservationTable.find(resource);
  if (it == reservationTable.end())
    return false;

  auto blocks = it->second;
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

BlockReservation* ReservationTable::getSingleReservation() const
{
  if (reservationTable.size() != 1)
    return nullptr;
  auto blocks = reservationTable.begin()->second;
  if (blocks.size() != 1)
    return nullptr;
  return &*blocks.begin();
}

bool ReservationTable::isEmpty() const
{
  return reservationTable.empty();
}

bool ReservationTable::isSimple() const
{
  if (BlockReservation* block = getSingleReservation())
    return block->start == 0 && block->duration == 1;
  return false;
}

bool ReservationTable::isBlock() const
{
  if (BlockReservation* block = getSingleReservation())
    return block->start == 0 && block->duration > 1;
  return false;
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

  for (auto p : reservationTable) {
    auto* resource = p.first;
    cout << setw(nameColumnWidth) << (resource->name + "(" + to_string(resource->id) + ")").substr(0, 20) << " |";
    for (int t = 0; t < latency; ++t)
      cout << setw(3) << getResourceDemandAt(resource, t) << " |";
    cout << endl;
  }
  cout << hline << endl;
}
}
