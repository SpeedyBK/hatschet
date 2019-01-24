/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)

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

#include "HatScheT/utility/Utility.h"
#include "HatScheT/scheduler/ASAPScheduler.h"
#include "HatScheT/scheduler/ULScheduler.h"
#include "HatScheT/utility/Verifier.h"
#include <limits>
#include <cmath>
#include <map>
#include <ios>
#include <fstream>
#include <ctime>
#include <cstddef>
#include <iomanip>

#include "HatScheT/scheduler/ASAPScheduler.h"

#ifdef USE_SCALP
#include "HatScheT/base/ILPSchedulerBase.h"
#include "HatScheT/scheduler/ilpbased/ASAPILPScheduler.h"
#include "HatScheT/utility/Verifier.h"
#include "HatScheT/scheduler/ULScheduler.h"
#endif

namespace HatScheT {

int Utility::getNoOfResConstrVertices(ResourceModel *rm, Graph *g)
{
  int count = 0;
  for(auto it=g->verticesBegin(); it!= g->verticesEnd(); ++it){
      Vertex* v = *it;
      const Resource* r = rm->getResource(v);
      if(r->getLimit()>0) count++;
  }
  return count;
}

bool Utility::everyVertexisRegistered(HatScheT::Graph &g, HatScheT::ResourceModel &rm) {
  for(auto it = g.verticesBegin(); it != g.verticesEnd(); ++it){
    Vertex* v = *it;

    rm.getResource(v);
  }
  return true;
}

int Utility::getNoOfInputsWithoutRegs(Graph *g, const Vertex *v)
{
  int no=0;

  for(auto it=g->edgesBegin(); it!=g->edgesEnd(); ++it)
  {
    Edge* e = *it;
    Vertex* dstV = &e->getVertexDst();

    if(dstV==v && e->getDistance()==0) no++;
  }

  return no;
}

int Utility::getNoOfInputs(Graph *g, const Vertex *v)
{
  int no=0;

  for(auto it=g->edgesBegin(); it!=g->edgesEnd(); ++it)
  {
    Edge* e = *it;
    Vertex* dstV = &e->getVertexDst();

    if(dstV==v) no++;
  }

  return no;
}

int Utility::getNoOfOutputsWithoutDistance(Graph *g, const Vertex *v)
{
  int outputs = 0;

  for(auto it=g->edgesBegin(); it!=g->edgesEnd();++it)
  {
    Edge* e = *it;
    Vertex* vSrc = &e->getVertexSrc();

    if(vSrc==v && e->getDistance()==0) outputs++;
  }

  return outputs;
}

int Utility::getNoOfOutputs(Graph *g, const Vertex *v)
{
  int outputs = 0;

  for(auto it=g->edgesBegin(); it!=g->edgesEnd();++it)
  {
    Edge* e = *it;
    Vertex* vSrc = &e->getVertexSrc();

    if(vSrc==v) outputs++;
  }

  return outputs;
}

bool Utility::IIisRational(double II) {
  double intpart;
  if(modf(II, &intpart) != 0.0) {
    return true;
  }
  return false;
}

double Utility::calcMinII(double minResII, double minRecII)
{
  if(minResII>minRecII) return minResII;

  return minRecII;
}
#ifdef USE_SCALP
double Utility::calcResMII(ResourceModel *rm, Target* t)
{
  double resMII = 1.0f;
  //standard case without using a hardware target
  if(t == nullptr){
    for(auto it=rm->resourcesBegin(); it!=rm->resourcesEnd(); ++it){
      Resource* r = *it;
      //skip unlimited resources
      if(r->getLimit()==-1)continue;
      int opsUsingR = rm->getNumVerticesRegisteredToResource(r);
      int avSlots = r->getLimit();

      if(avSlots<=0) throw HatScheT::Exception("Utility.calcResMII: avSlots <= 0 : " + to_string(avSlots));
      double tempMax = ((double)opsUsingR)/((double)avSlots); // + (double)(opsUsingR % avSlots != 0); < -- ?? patrick

      if(tempMax>resMII) resMII=tempMax;
    }
  }
  //a target is provided
  else{
    //throw Exception("Utility.calcResMII: ERROR calculating resMinII using a target is not supported yet");
    cout << "Utility.calcResMII: WARNING calculating resMinII using a target is not supported yet" << endl;
  }

  return resMII;
}

int Utility::getCriticalPath(HatScheT::Graph *g, HatScheT::ResourceModel *rm) {
  map<Resource*, int> restoreLimits;
  //set all resource unlimited for critical path calculation, store old values
  for(auto it = rm->resourcesBegin(); it != rm->resourcesEnd(); ++it){
    Resource* r = *it;
    if(r->isUnlimited() == true) continue;

    restoreLimits.insert(make_pair(r,r->getLimit()));
    r->setLimit(-1);
  }

  //do critical path calculation
  ASAPScheduler asap(*g,*rm);
  asap.schedule();
  int limitlat = asap.getScheduleLength();

  ASAPILPScheduler ilp(*g,*rm,  {"CPLEX", "Gurobi", "SCIP"});
  ilp.setMaxLatencyConstraint(limitlat);
  ilp.schedule();

  int criticalPath = ilp.getScheduleLength();

  //restore old resource model
  for(auto it = restoreLimits.begin(); it != restoreLimits.end(); ++it){
    Resource* r = it->first;
    r->setLimit(it->second);
  }

  return criticalPath;
}

  pair<int,int> Utility::splitRational(double x) {
    //int[] A =new int[12];
    //int[] B = new int[12];
    //int[] k = new int[12];
    int A0 = 1;
    int A1 = 0;
    int B0 = 0;
    int B1 = 1;
    for(int i = 0; i <1000; i++) {
      //k[i+2] = (int) ((double)1/x);
      int k = (int) ((double)1/x);
      //A[i+2] = A[i] + k[i+2] * A[i+1];
      int temp = A1;
      A1 = A0 + k*A1;
      A0 = temp;
      //B[i+2] = B[i] + k[i+2] * B[i+1];
      temp = B1;
      B1 = B0 + k*B1;
      B0 = temp;
      x = ((double)1/x) - k;
      if (abs(x)<=1.0E-3) {
        return make_pair(A1,B1);
      };
    }

    return make_pair(0,0);
  }

  int Utility::safeRoundDown(double x) {
    if (x>=0) {
      double error = x - (int) x;
      if (error < 0.999) return (int) x;
      return (int) ceil(x);
    }
    double error = (int) x -x;
    if (error < 0.001) return (int) x;
    return (int) floor(x);
  }

int Utility::calcMaxII(Graph *g, ResourceModel *rm) {
  //determine minimum possible latency
  HatScheT::ASAPScheduler asap(*g,*rm);
  asap.schedule();
  int criticalPath = asap.getScheduleLength();

  if(g->getNumberOfVertices() > 200) {
    HatScheT::verifyModuloSchedule(*g,*rm, asap.getSchedule(),asap.getScheduleLength());
    return criticalPath;
  }

  //get optimal critical path using asap ilp scheduler
  HatScheT::ASAPILPScheduler* asapilp = new HatScheT::ASAPILPScheduler(*g, *rm, {"CPLEX", "Gurobi", "SCIP"});
  asapilp->setMaxLatencyConstraint(criticalPath);
  asapilp->schedule();
  criticalPath = asapilp->getScheduleLength();

  delete asapilp;

  //error in non ilp-based asap detected
  //starting new asap ilp
  if(criticalPath <= 0){
    HatScheT::ASAPILPScheduler* asapilp = new HatScheT::ASAPILPScheduler(*g, *rm, {"CPLEX", "Gurobi", "SCIP"});
    asapilp->setMaxLatencyConstraint(g->getNumberOfVertices() * ( rm->getMaxLatency() + 1));
    asapilp->schedule();
    criticalPath = asapilp->getScheduleLength();

    delete asapilp;
  }

  return criticalPath;
}

double Utility::calcRecMII(Graph *g, ResourceModel *rm)
{
  ScaLP::Solver solver({"CPLEX", "Gurobi"});

  // construct decision variables
  auto II = ScaLP::newRealVariable("II", 0, ScaLP::INF());
  std::map<Vertex *, ScaLP::Variable> t;
  for (auto it = g->verticesBegin(), end = g->verticesEnd(); it != end; it++) {
    auto v = *it;
    t[v] = ScaLP::newRealVariable("t_" + to_string(v->getId()), 0, std::numeric_limits<int>::max());
  }

  // construct constraints
  for (auto it = g->edgesBegin(), end = g->edgesEnd(); it != end; it++) {
    auto e = *it;
    auto i = &e->getVertexSrc();
    auto j = &e->getVertexDst();

    solver << ((t[i] + rm->getVertexLatency(i) + e->getDelay() - t[j] - (e->getDistance()*II)) <= 0);
  }

  // construct objective
  solver.setObjective(ScaLP::minimize(II));

  auto status = solver.solve();
  if (status != ScaLP::status::OPTIMAL && status != ScaLP::status::FEASIBLE) {
    cout << "Utility.calcRecMII: ERROR No solution found!" << endl;
    throw HatScheT::Exception("RecMII computation failed!");
  }

  return max(1.0, (double)(solver.getResult().objectiveValue)); // RecMII could be 0 if instance has no backedges.
}

#endif
int Utility::sumOfStarttimes(std::map<Vertex *, int> &startTimes)
{
  int sum = 0;

  for(auto it=startTimes.begin();it!=startTimes.end();++it){
    sum+=it->second;
  }

  return sum;
}

bool Utility::resourceAvailable(std::map<Vertex *, int> &startTimes, ResourceModel* rm, const Resource *r, Vertex *checkV, int timeStep)
{
  //unlimited
  if(r->getLimit()==-1) return true;

  int instancesUsed = 0;

  for(auto it=startTimes.begin(); it!=startTimes.end(); ++it){
    if(it->second == timeStep){
      Vertex* v = it->first;
      if(checkV != v && rm->getResource(v) == r) instancesUsed++;
    }
  }

  if(instancesUsed < r->getLimit()) return true;
  return false;
}

bool Utility::edgeIsInGraph(Graph *g, const Edge *e)
{
  for(auto it=g->edgesBegin();it!=g->edgesEnd();++it){
    Edge* iterE = *it;
    if(iterE==e) return true;
  }
  return false;
}

bool Utility::isInput(Graph *g, Vertex *v)
{
  for(auto it=g->edgesBegin();it!=g->edgesEnd();++it){
      Edge* e = *it;
      Vertex* vDst = &e->getVertexDst();

      if(v==vDst && e->getDistance()==0) return false;
  }

  return true;
}

bool Utility::existEdgeBetweenVertices(Graph* g, Vertex* Vsrc, Vertex* Vdst)
{
    for(auto it=g->edgesBegin();it!=g->edgesEnd();++it){
        Edge* iterE = *it;
        Vertex* iterSrc = &iterE->getVertexSrc();
        Vertex* iterDst = &iterE->getVertexDst();

        if ((iterSrc==Vsrc) && (iterDst==Vdst)){
            return true;
        }
        if ((iterSrc==Vdst) && (iterDst==Vsrc)){
            return true;
        }
    }
    return false;
}


bool Utility::occurrencesAreConflictFree(Occurrence *occ1, Occurrence *occ2)
{
  vector<Vertex*> occ1Set = occ1->getVertices();

  for(auto it:occ1Set){
    Vertex* v = it;

    if(occ2->vertexIsNew(v)==false){
      return false;
    }
  }
  return true;
}

bool Utility::vertexInOccurrence(Occurrence *occ, Vertex *v)
{
  vector<Vertex*> vVec = occ->getVertices();

  for(auto it:vVec){
    Vertex* vIter = it;
    if(vIter==v) return true;
  }
  return false;
}

bool Utility::resourceModelAndTargetValid(HatScheT::ResourceModel &rm, HatScheT::Target &t) {
  for(auto it = rm.resourcesBegin(); it != rm.resourcesEnd(); ++it){
    Resource* r  = *it;
    //skip unlimited resources
    //TODO: is this still valid?? valid in this context?? (patrick)
    //if(r->getLimit() == -1) continue;
    map<string,double>& costs = r->getHardwareCosts();

    for(auto it2 = costs.begin(); it2 != costs.end(); ++it2) {
      if (t.elementExists(it2->first) == false) {
        cout << "Utility.resourceModelAndTargetValid: ERROR demanded hardware element '" << it2->first << "' of resource '" << r->getName() << "' is not available on this target:" << endl;
        cout << t << endl;
        return false;
      }
    }
  }

  return true;
}

int Utility::calcUsedOperationsOfBinding(map<const Vertex *, int> &binding, ResourceModel& rm, Resource *r)
{
  int opsUsed=0;
  //unlimited resources used in paralell
  if(r->getLimit()<=0){
    return rm.getNumVerticesRegisteredToResource(r);
  }

  vector<bool> usedOp;
  usedOp.resize(r->getLimit());
  for(int i=0;i<usedOp.size();i++){
    usedOp[i]=false;
  }

  for(auto it:binding){
    const Vertex* v = it.first;
    if(rm.getResource(v)==r){
      int bind = binding[v];
      usedOp[bind]=true;
    }
  }

  for(int i=0;i<usedOp.size();i++){
    if(usedOp[i]==true) opsUsed++;
  }

  return opsUsed;
}

void Utility::printBinding(map<const Vertex *, int> &binding, ResourceModel& rm)
{
  cout << "-------Print Binding Start-------" << endl;
  for(auto it:binding){
    const Vertex* v = it.first;
    const Resource* r = rm.getResource(v);
    cout << "Vertex " << v->getName() << " bound to unit " << it.second << " of Resource " << r->getName() <<" with limit " << r->getLimit() << endl;
  }
  cout << "-------Print Binding Finished-------" << endl;
}

void Utility::printSchedule(map<Vertex *, int> &schedule)
{
  cout << "Utility::printSchedule: Start" << endl;
  for(auto it:schedule){
    Vertex* v = it.first;
    cout << "Scheduled " << v->getName() << " at " << it.second << endl;
  }
  cout << "Utility::printSchedule: Finished" << endl;
}

bool Utility::allInputsAreRegisters(Graph *g, Vertex *v)
{
  for(auto it=g->edgesBegin(); it!= g->edgesEnd(); ++it){
    Edge* e = *it;
    if(e->getDistance()==0 && &e->getVertexDst()==v) return false;
  }
  return true;
}

bool Utility::vertexInOccurrenceSet(OccurrenceSet *occS, Vertex *v)
{
  set<Occurrence*> occSet = occS->getOccurrences();

  for(auto it:occSet){
    Occurrence* occIter = it;

    if(Utility::vertexInOccurrence(occIter,v)) return true;
  }
  return false;
}

bool Utility::occurenceSetsAreConflictFree(OccurrenceSet *occs1, OccurrenceSet *occs2)
{
  set<Occurrence*> occs1Set = occs1->getOccurrences();
  set<Occurrence*> occs2Set = occs2->getOccurrences();

  for(auto it:occs1Set){
    Occurrence* occ = it;
    for(auto it2:occs2Set){
      Occurrence* occ2 = it2;

      if(Utility::occurrencesAreConflictFree(occ,occ2)==false) return false;
    }
  }

  return true;
}

	std::map<const Vertex *, int> Utility::getSimpleBinding(map<Vertex *, int> sched, ResourceModel *rm, int II) {
  std::map<const Vertex *, int> binding;
  std::map<const Resource*, std::map<int, int>> resourceCounters;

  for(auto it : sched) {
    auto v = it.first;
    const Resource* res = rm->getResource(v);
    if(res->getLimit()<0) {
		if(resourceCounters[res].find(0) != resourceCounters[res].end()) {
			resourceCounters[res][0]++;
		}
		else {
			resourceCounters[res][0] = 0;
		}
		binding[v] = resourceCounters[res][0];
    } else {
		int time = it.second % II;
		if(resourceCounters[res].find(time) != resourceCounters[res].end()) {
			resourceCounters[res][time]++;
		}
		else {
			resourceCounters[res][time] = 0;
		}
		if(resourceCounters[res][time] >= res->getLimit())
			throw HatScheT::Exception("Utility::getSimpleBinding: found resource conflict while creating binding");
		binding[v] = resourceCounters[res][time];
    }
  }
  return binding;
}
#ifdef USE_SCALP
std::map<const Vertex *, int> Utility::getILPMinRegBinding(map<Vertex *, int> sched, Graph *g, ResourceModel *rm, int II, std::list<std::string> sw, int timeout) {
  // create solver
  if(sw.empty()) sw = {"Gurobi","CPLEX","SCIP","LPSolve"};
  auto solver = ScaLP::Solver(sw);
  solver.quiet = true;
  solver.threads = 1;
  solver.timeout = timeout;

  // create vertex-binding variables
  std::map<Vertex*,std::vector<ScaLP::Variable>> vertexVariables;
  std::map<const Resource*,std::vector<ScaLP::Variable>> registerVariables;
  std::map<const Resource*,std::list<Vertex*>> sameResources;
  std::map<const Resource*,int> resourceLimits;
  for(auto &it : sched){
	auto res = rm->getResource(it.first);
	try{
	  sameResources.at(res).emplace_back(it.first);
	}
	catch(std::out_of_range&){
	  sameResources[res] = {it.first};
	  registerVariables[res] = std::vector<ScaLP::Variable>();
	}
	int limit = res->getLimit();
	if(limit<0) {
	  // check how many resources were allocated by scheduling algorithm
	  try{
		limit = resourceLimits.at(res);
	  }
	  catch(std::out_of_range&){
		limit = 0;
		for(auto &it2 : sched){
		  if(rm->getResource(it2.first) == res) limit++;
		}
		resourceLimits[res] = limit;
	  }
	}
	vertexVariables[it.first] = std::vector<ScaLP::Variable>();
	for(auto i = 0; i<limit; i++){
	  vertexVariables[it.first].emplace_back(ScaLP::newIntegerVariable(it.first->getName()+"_"+to_string(i),0,1));
	}
  }

  // create register variables
  for(auto &it : registerVariables){
	auto res = it.first;
	auto limit = res->getLimit();
	for(int i=0; i<limit; i++){
	  auto var = ScaLP::newIntegerVariable(res->getName()+to_string(i),0,ScaLP::INF());
	  it.second.emplace_back(var);
	}
  }

  // calculate lifetimes
  std::map<Vertex*,int> lifetimes;
  for(auto &it : g->Edges()){
	auto* src = &it->getVertexSrc();
	auto* dst = &it->getVertexDst();
	int tempLifetime = sched[dst] - sched[src] - rm->getResource(src)->getLatency() + (II * it->getDistance());
	if(tempLifetime>lifetimes[src]) lifetimes[src] = tempLifetime;
  }

  // check, which vertices can potetially be bind to the same resource at the same control step
  std::map<const Resource*,std::map<int,std::list<Vertex*>>> potentiallySame;
  for(auto &it : sched){
	auto vert = it.first;
	auto timepoint = it.second % II;
	auto res = rm->getResource(vert);
	if(potentiallySame.find(res)==potentiallySame.end())
	  potentiallySame[res] = std::map<int,std::list<Vertex*>>();
	try{
	  potentiallySame[res].at(timepoint).emplace_back(vert);
	}
	catch(std::out_of_range&){
	  potentiallySame[res][timepoint] = {vert};
	}
  }

  // create constraints: bind each vertex to EXACTLY one resource
  for(auto &it1 : vertexVariables){
	ScaLP::Term t;
	for(auto &it2 : it1.second){
	  t += it2;
	}
	auto c = ScaLP::Constraint(t == 1);
	solver << c;
  }

  // create constraints: bind AT MOST one vertex to each resource in each control step
  for(auto &it1 : potentiallySame){
	auto res = it1.first;
	for(auto &it2 : it1.second){
	  for(int resourceCounter = 0; resourceCounter<res->getLimit(); resourceCounter++){
		ScaLP::Term t;
		for(auto &it3 : it2.second){
		  t += vertexVariables[it3][resourceCounter];
		}
		auto c = ScaLP::Constraint(t <= 1);
		solver << c;
	  }
	}
  }

  // create edge register constraints
  for(auto &it1 : registerVariables){
	auto res = it1.first;
	for(int resoureCounter = 0; resoureCounter<it1.second.size(); resoureCounter++){
	  auto var = it1.second[resoureCounter];
	  for(auto &it2 : sameResources[res]){
	  	auto c = ScaLP::Constraint(var - (lifetimes[it2] * vertexVariables[it2][resoureCounter]) >= 0);
		solver << c;
	  }
	}
  }

  // minimize the sum of all registers
  ScaLP::Term obj;
  for(auto &it1 : registerVariables){
	for(auto &it2 : it1.second){
	  obj += it2;
	}
  }
  solver.setObjective(ScaLP::minimize(obj));

  // solve and put results into binding map
  try{
    auto status = solver.solve();
    if(status != ScaLP::status::OPTIMAL && status != ScaLP::status::FEASIBLE && status != ScaLP::status::TIMEOUT_FEASIBLE){
	  cout << "Utility::getILPMinRegBinding: didn't find solution, returning simple binding" << endl;
	  return Utility::getSimpleBinding(sched,rm,II);
  	}
  }
  catch(ScaLP::Exception& e){
  	throw HatScheT::Exception("Utility::getILPMinRegBinding: caught ScaLP exception: "+std::string(e.what()));
  }

  auto results = solver.getResult().values;
  map<const Vertex*,int> binding;
  for(auto &it1 : vertexVariables){
	auto vertex = it1.first;
	for(int i = 0; i < (int)it1.second.size(); i++){
	  auto &it2 = it1.second[i];
	  auto val = results[it2];
	  cout << "#q# result for variable '" << it2 << "': " << val << endl;
	  if(val==1.0){
		binding[vertex] = i;
	  }
	}
  }

  return binding;
}
#endif	 
}
