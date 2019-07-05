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
  this->registrations.clear();
  for(auto r:resources) delete r;
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
  os << "------------------------------- Resource Model -------------------------------------" << endl;
  os << "------------------------------------------------------------------------------------" << endl;

  for(auto it:rm.resources)
  {
    Resource* r = it;
    int reg = rm.getNumVerticesRegisteredToResource(r);

    if(r->getLimit() > 0) os << "Resource Model has limited resource: " << r->getName() << " with limit/vertices " << r->getLimit() << "/" << reg << ", latency "
                             << r->getLatency() << ", blockingTime " << r->getBlockingTime() << ", physicalDelay " << r->getPhysicalDelay() << endl;
    if(r->getLimit() == -1) os << "Resource Model has unlimited resource: " << r->getName() << " with latency "
                               << r->getLatency() << ", blockingTime " << r->getBlockingTime() << ", physicalDelay " << r->getPhysicalDelay() << endl;
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
  this->registrations[v] = r;
}

bool ResourceModel::resourceExists(std::string name) {
  for(auto it = this->resourcesBegin(); it != this->resourcesEnd(); ++it){
    Resource* r = *it;
    if(name == r->getName()) return true;
  }
  return false;
}

void Resource::setLimit(int l)
{
  if(this->name=="special_loop" && l!=1) throw Exception(this->name + ".setLimit: ERORR it is not allowed to limit other than 1 to this resource!");
  if(this->blockingTime==0 && l!=-1) {
    cout << this->name << ".setLimit: WARNING setting this resource limit to a limited value and a blocking time of 0 was detected! Blocking time set to 1!";
    this->blockingTime = 1;
  }
  if(this->latency==0 && this->phyDelay==0.0f && l!=-1){
    //ToDo (patrick) ist this warning really neccessary?
    //cout << this->name << ".setLimit: WARNING you should not limit a resource with a latency and physical delay of 0 as its not describing real hardware!" << endl;
    //throw Exception(this->name + ".setLimit: ERORR it is not allowed to limit resource with a latency and physical delay of 0!");
  }
  this->limit=l;}

const Resource *ResourceModel::getResource(const Vertex *v) const
{
  const auto it = this->registrations.find(v);
  if(it == this->registrations.end()) throw HatScheT::Exception("ResourceModel.getResource: vertex not registered " + v->getName());
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
  throw HatScheT::Exception("ReservationTable.makeReservationBlock: ERROR makeReservationBlock currently disabled/not supported!");

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
  throw HatScheT::Exception("ResourceModel.makeReservationTable: ERROR makeReservationTable currently disabled/not supported!");

  /*ReservationTable* rt = new ReservationTable(name);
  this->tables.push_back(rt);

  return *rt;*/
}

Resource *ResourceModel::getResource(string name) const
{
  for(auto it:this->resources)
  {
    Resource* r = it;

    if(r->getName() == name) return r;
  }
  throw HatScheT::Exception("ResourceModel.getResource: Could not find resource with name " + name);
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

int ResourceModel::getNumVerticesRegisteredToResource(Resource *r) const
{
  int count = 0;

  for(auto it:this->registrations)
  {
    const Resource* rr = it.second;
    if(r==rr) count++;
  }

  return count;
}

int ResourceModel::getNumVerticesRegisteredToResource(const Resource *r) const
{
  int count = 0;

  for(auto it:this->registrations)
  {
    const Resource* rr = it.second;
    if(r==rr) count++;
  }

  return count;
}

double Resource::getHardwareCost(string n) {
  if ( this->hardwareCost.find(n) == this->hardwareCost.end() ) {
    return 0.0f;
  } else {
    return this->hardwareCost[n];
  }
}

void Resource::addHardwareCost(string n, double c)
{
  this->hardwareCost.insert(make_pair(n,c));
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

  throw HatScheT::Exception("ResourceModel.getVertexLatency: vertex not registered " + v->getName());
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

  throw HatScheT::Exception("ResourceModel.getVertexLatency: vertex not registered " + v->getName());
}

}
