#include <HatScheT/ResourceModel.h>

#include <algorithm>
#include <iostream>
#include <iomanip>



namespace HatScheT
{

ResourceModel::ResourceModel()
{

}

ResourceModel::~ResourceModel()
{
  //for(auto r:resources) delete r;

}

ostream& operator<<(ostream& os, const ReservationBlock& rb)
{
  string limit;
  if(rb.resource->getLimit() == -1) limit = "inf";
  else limit = to_string(rb.resource->getLimit());

  os << "\t" << "ReservationBlock of Resource " << rb.resource->getName() << " with limit " << limit << ", latency " << rb.resource->getLatency()
     << ", blockingTime " << rb.resource->getBlockingTime() << " and startTime " << rb.startTime << endl;

  return os;
}

ostream& operator<<(ostream& os, const ReservationTable& rt)
{
  os << "Resource model has reservation table: " << rt.name << endl;

  for(auto it:rt.blocks)
  {
    const ReservationBlock* rb = it;

    os << *rb;
  }
  return os;
}

ostream& operator<<(ostream& os, const ResourceModel& rm)
{
  os << "------------------------------------------------------------------------------------" << endl;
  os << "-------------------------------Printing resource model------------------------------" << endl;
  os << "------------------------------------------------------------------------------------" << endl;

  for(auto it:rm.resources)
  {
    const Resource* r = it;

    if(r->getLimit() > 0) os << "Resource Model has limited resource: " << r->getName() << " with limit " << r->getLimit() << ", latency "
                             << r->getLatency() << ", blockingTime " << r->getBlockingTime() << endl;
    if(r->getLimit() == -1) os << "Resource Model has unlimited resource: " << r->getName() << " with latency "
                               << r->getLatency() << ", blockingTime " << r->getBlockingTime() << endl;
  }

  os << "------------------------------------------------------------------------------------" << endl;


  for(auto it:rm.tables)
  {
    const ReservationTable* rt = it;
    os << *rt;
  }

  os << "------------------------------------------------------------------------------------" << endl;

  for(auto it:rm.registrations)
  {
    const Vertex* v = it.first;
    const Resource* rt = it.second;
    os << "Registered vertex " << v->getName() << " with " << rt->getName() << endl;
  }

  os << "------------------------------------------------------------------------------------" << endl;

  return os;
}

void ResourceModel::registerVertex(const Vertex *v, const Resource *r)
{
  this->registrations.insert({v, r});
}

const Resource *ResourceModel::getResource(const Vertex *v) const
{
  const auto it = this->registrations.find(v);
  if(it == this->registrations.end()) throw new Exception("ResourceModel.getResource: vertex not registered " + v->getName());
  else return it->second;
}

int ReservationTable::getLimit() const
{
  int limit = 0;

  for(auto it:this->blocks)
  {
    ReservationBlock* rb = it;
    const Resource* r = rb->getResource();

    //unlimited
    if(r->getLimit() == -1 ) continue;
    //init with first real limit
    if(limit == 0) limit = r->getLimit();
    else if(limit > 0)
    {
      if(r->getLimit() < limit) limit = r->getLimit();
    }
  }

  return limit;
}

int ReservationTable::getLatency() const
{
  int latency = 0;

  for(auto it:this->blocks)
  {
    ReservationBlock* rb = it;
    const Resource* r = rb->getResource();

    if(latency < (r->getLatency()+rb->getStartTime())) latency = r->getLatency()+rb->getStartTime();
  }

  return latency;
}

ReservationBlock &ReservationTable::makeReservationBlock(Resource *r, int startTime)
{
  //currently disabled/not supported
  throw new Exception("ReservationTable.makeReservationBlock: ERROR makeReservationBlock currently disabled/not supported!");

  /*ReservationBlock* rb = new ReservationBlock(r,startTime);
  this->blocks.push_back(rb);
  return *rb;*/
}

Resource &ResourceModel::makeResource(string name, int limit, int latency, int blockingTime)
{
  Resource* r = new Resource(name, limit, latency, blockingTime);
  this->resources.push_back(r);

  return *r;
}

ReservationTable &ResourceModel::makeReservationTable(string name)
{
  //currently disabled/not supported
  throw new Exception("ResourceModel.makeReservationTable: ERROR makeReservationTable currently disabled/not supported!");

  /*ReservationTable* rt = new ReservationTable(name);
  this->tables.push_back(rt);

  return *rt;*/
}

ReservationBlock& ResourceModel::makeReservationBlock(ReservationTable *rt, Resource *r, int startTime)
{
  //currently disabled/not supported
  throw new Exception("ResourceModel.makeReservationBlock: ERROR makeReservationBlock currently disabled/not supported!");

  //return rt->makeReservationBlock(r,startTime);
}

Resource *ResourceModel::getResource(string name) const
{
  for(auto it:this->resources)
  {
    Resource* r = it;

    if(r->getName() == name) return r;
  }
  throw new Exception("MoovacScheduler.constructProblem: Could not find resource with name " + name);
}

bool ResourceModel::isEmpty()
{
  if(this->resources.size() == 0) return true;
  return false;
}

set<const Vertex*> ResourceModel::getVerticesOfResource(const Resource *r) const
{
  set<const Vertex*> vertices;

  for(auto it:this->registrations)
  {
    const Resource* rr = it.second;
    if(rr==r) vertices.insert(it.first);
  }

  return vertices;
}

int ResourceModel::getNoOfVerticesRegisteredToResource(Resource *r) const
{
  int count = 0;

  for(auto it:this->registrations)
  {
    const Resource* rr = it.second;
    if(r==rr) count++;
  }

  return count;
}

int ResourceModel::getMaxLatency() const
{
  int maxLat = 0;

  for(auto it:this->registrations)
  {
    const Resource* rr = it.second;

    if(rr->getLatency()>maxLat)
    {
      maxLat = rr->getLatency();
    }
  }

  return maxLat;
}

int ResourceModel::getVertexLatency(const Vertex *v) const
{
  for(auto it:this->registrations)
  {
    const Vertex* rv = it.first;
    const Resource* rr = it.second;

    if(rv==v)
    {
      return rr->getLatency();
    }
  }

  throw new Exception("ResourceModel.getVertexLatency: vertex not registered " + v->getName());
}

int ResourceModel::getVertexLatency(Vertex *v) const
{
  for(auto it:this->registrations)
  {
    const Vertex* rv = it.first;
    const Resource* rr = it.second;

    if(rv==v)
    {
      return rr->getLatency();
    }
  }

  throw new Exception("ResourceModel.getVertexLatency: vertex not registered " + v->getName());
}

}
