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
#include <HatScheT/utility/writer/DotWriter.h>

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

int Utility::getCyclesOfLongestPath(HatScheT::Graph *g, HatScheT::ResourceModel *rm, double II) {
  int length = 0;

  //identify inputs
  vector<Vertex*> inputs;
  for(auto it = g->verticesBegin(); it != g->verticesEnd(); ++it){
    Vertex* v = *it;

    if(g->getPredecessors(v).size() == 0) inputs.push_back(v);
  }

  // check if there are vertices, which only have input edges with distance>0
  if(inputs.empty()) {
    for(auto v : g->Vertices()) {
      bool pushMeBack = true;
      for(auto e : g->Edges()) {
        if(&e->getVertexDst() != v) continue;
        if(e->getDistance()<1) {
          pushMeBack = false;
          break;
        }
      }
      if(pushMeBack) inputs.emplace_back(v);
    }
  }

  //find seed edges
  vector<const Edge*> seeds;
  //iterate over inputs
  for(auto it = inputs.begin(); it != inputs.end(); ++it){
    Vertex* in = *it;

    set<Vertex*> succ = g->getSuccessors(in);

    for(auto it2 : succ) {
      list<const Edge*> s = g->getEdges(in,it2);

      for(auto it3 : s) {
        seeds.push_back(it3);
      }
    }
  }

  //start recursion
  for(auto it : seeds) {
    vector<Vertex*> visited;
    int l = rm->getVertexLatency(&it->getVertexSrc());
    l += std::ceil(II*it->getDistance());
    Utility::cycle(it, visited, l, g, rm, II);

    if(l > length) length = l;
  }

  return length;
}

void Utility::cycle(const HatScheT::Edge *e, vector<HatScheT::Vertex *>& visited, int &currLength, Graph* g, ResourceModel* rm, double II) {
  Vertex* succ = &e->getVertexDst();

  //check whether succ was already visited (cycle ends)
  for(auto it : visited) {
    if(succ == it)
      return;
  }

  //put to visited
  visited.push_back(succ);
  //increase length
  currLength += rm->getVertexLatency(succ);

  //collect all outgoing edges
  set<Vertex*> s = g->getSuccessors(succ);
  vector<const Edge*> outgoing;
  for(auto it : s) {
    list<const Edge*> o = g->getEdges(succ, it);

    for(auto it2 : o) {
      outgoing.push_back(it2);
    }
  }

  //check if outport found
  if(outgoing.size() == 0) return;

  //continue recursion
  for(auto it : outgoing) {
    currLength += std::ceil(II*it->getDistance());
    Utility::cycle(it, visited, currLength, g, rm, II);
  }
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

int Utility::getILPASAPScheduleLength(HatScheT::Graph *g, HatScheT::ResourceModel *rm) {
  HatScheT::ASAPScheduler a(*g,*rm);
  a.schedule();
  int criticalPath = a.getScheduleLength();

  ASAPILPScheduler asap(*g,*rm,  {"CPLEX", "Gurobi", "SCIP"});
  asap.setMaxLatencyConstraint(criticalPath);
  asap.schedule();

  return asap.getScheduleLength();
}

int Utility::getASAPScheduleLength(HatScheT::Graph *g, HatScheT::ResourceModel *rm) {
  HatScheT::ASAPScheduler a(*g,*rm);
  a.schedule();

  return a.getScheduleLength();
}

int Utility::getASAPNoHCScheduleLength(HatScheT::Graph *g, HatScheT::ResourceModel *rm) {
  map<HatScheT::Resource*, int> limits;

  //save old limits
  for(auto it = rm->resourcesBegin(); it != rm->resourcesEnd(); ++it){
    HatScheT::Resource* r = *it;
    limits.insert(make_pair(r, r->getLimit()));

    r->setLimit(-1);
  }

  //determine schedule
  ASAPScheduler asap(*g,*rm);
  asap.schedule();
  int lat = asap.getScheduleLength();

  //restore old limits
  for(auto it = rm->resourcesBegin(); it != rm->resourcesEnd(); ++it){
    HatScheT::Resource* r = *it;

    r->setLimit(limits[r]);
  }

  return lat;
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

int Utility::calcAbsolutMaxII(Graph *g, ResourceModel *rm) {
  map<Resource*, int> limit_map;

  //store initial values
  for(auto it = rm->resourcesBegin(); it != rm->resourcesEnd(); ++it){
    Resource* r = *it;

    limit_map.insert(make_pair(r, r->getLimit()));

    //set temporarily to limit 1
    if(r->getName() != "special_virtual") r->setLimit(1);
  }

  //determine minimum possible latency
  HatScheT::ASAPScheduler asap(*g,*rm);
  asap.schedule();
  int criticalPath = asap.getScheduleLength();

  if(g->getNumberOfVertices() > 200) {
    if(!HatScheT::verifyModuloSchedule(*g,*rm, asap.getSchedule(),asap.getII())) {
      throw HatScheT::Exception("Utility.calcMaxII: ASAP scheduler found invalid result!");
    }
  }

  else {
    //get optimal critical path using asap ilp scheduler
    HatScheT::ASAPILPScheduler *asapilp = new HatScheT::ASAPILPScheduler(*g, *rm, {"CPLEX", "Gurobi", "SCIP"});
    asapilp->setMaxLatencyConstraint(criticalPath);
    asapilp->schedule();
    criticalPath = asapilp->getScheduleLength();

    delete asapilp;

    //error in non ilp-based asap detected
    //starting new asap ilp
    if (criticalPath <= 0) {
      HatScheT::ASAPILPScheduler *asapilp = new HatScheT::ASAPILPScheduler(*g, *rm, {"CPLEX", "Gurobi", "SCIP"});
      asapilp->setMaxLatencyConstraint(g->getNumberOfVertices() * (rm->getMaxLatency() + 1));
      asapilp->schedule();
      criticalPath = asapilp->getScheduleLength();

      delete asapilp;
    }
  }

  //restore values
  for(auto it = rm->resourcesBegin(); it != rm->resourcesEnd(); ++it){
    Resource* r = *it;
    r->setLimit(limit_map[r]);
  }

  return criticalPath;
}

int Utility::calcMaxII(Graph *g, ResourceModel *rm) {
  //determine minimum possible latency
  HatScheT::ASAPScheduler asap(*g,*rm);
  asap.schedule();
  int criticalPath = asap.getScheduleLength();

  if(g->getNumberOfVertices() > 200) {
    if(!HatScheT::verifyModuloSchedule(*g,*rm, asap.getSchedule(),asap.getII())) {
      throw HatScheT::Exception("Utility.calcMaxII: ASAP scheduler found invalid result!");
    }
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
  ScaLP::Solver solver({"Gurobi", "CPLEX"});

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
    cout << "Utility.calcRecMII: ScaLP::STATUS " << status << endl;
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

void Utility::printRationalIIMRT(map<HatScheT::Vertex *, int> sched, vector<map<const HatScheT::Vertex *, int> > ratIIbindings,
                                 HatScheT::ResourceModel *rm, int modulo, vector<int> initIntervalls) {
  for(auto it = rm->resourcesBegin(); it != rm->resourcesEnd(); ++it){
    const Resource* r = *it;

    //operation start time -> operation, unit
    map<int,vector< pair<string,int> > > MRT;

    //insert MRT information over modulo slots
    for(int i = 0; i < modulo; i++){
      vector<pair<string, int> > slot;
      for(auto it : sched){
        auto v = it.first;
        if(r != rm->getResource(v)) continue;

        //track the offset for the samples
        int offset=0;
        //iterate over samples
        for(int j = 0; j < initIntervalls.size(); j++){
          //increase offset
          if(j>0) offset+=initIntervalls[j-1];

          int moduloTime = (it.second+offset) % modulo;

          if(moduloTime != i) continue;
          slot.push_back(make_pair(v->getName() + "_s" + to_string(j), ratIIbindings[j][v]));
        }
      }
      MRT.insert(make_pair(i,slot));
    }

    //do the printing
    cout << "-----" << endl;
    cout << "MRT after binding for resource " << r->getName() << " (limit " << r->getLimit() << ")" << endl;
    for(auto it:MRT){
      cout << to_string(it.first) << ": ";

      for(auto it2:it.second){
        cout << "(" << it2.first << ", unit " << to_string(it2.second) << ") ";
      }
      if(it.second.size()==0) cout << "-----";
      cout << endl;
    }
    cout << "-----" << endl;
  }
}

  std::pair<int, int> Utility::getSampleIndexAndOffset(int distance, int sample, int samples, int modulo) {
    int sampleIndex = sample;
    int offset = 0;
    while(distance>0) {
      if(sampleIndex==0) {
        sampleIndex = samples-1;
        offset += modulo;
      }
      else {
        --sampleIndex;
      }
      --distance;
    }
    return std::make_pair(sampleIndex,offset);
  }

  std::pair<Graph*, map<Vertex*, Vertex*> >  Utility::transposeGraph(Graph *g) {

    auto *h = new Graph();
    std::map<Vertex*,Vertex*> m;

    //Copy all verticies from graph g to graph h. And creating a map with the corresponding verticies.
    //(gVertex : hVertex).
    for (auto V:g->Vertices()){
      m[V] = &h->createVertex(V->getId());
    }

    //Creating the edges
    for (auto E:g->Edges()){
      h->createEdge(*m[&E->getVertexDst()], *m[&E->getVertexSrc()], E->getDistance(), E->getDependencyType());
    }


    return make_pair(h,m);
  }

  bool Utility::iscyclic(Graph *g) {

    map <Vertex*, bool> visited;
    map <Vertex*, bool> recStack;

    for (auto &v : g->Vertices()){
      visited[v] = false;
      recStack[v] = false;
    }

    for (auto &it : g->Vertices()){
      if (iscyclicHelper(g, it, visited, recStack)){
        return true;
      }
    }

    return false;
  }

  bool Utility::iscyclicHelper(Graph *g, Vertex *V, map<Vertex *, bool> &visited, map<Vertex *, bool> &recStack) {

    if (!visited[V]) {

      visited[V] = true;
      recStack[V] = true;

      for (auto &it : g->getSuccessors(V)) {
        if (!visited[it] && iscyclicHelper(g, it, visited, recStack)){
          return true;
        }
        else if (recStack[it]){
          return true;
        }
      }
    }

    recStack[V] = false;
    return false;

  }

	int Utility::hFunction(double n, double M, int tau) {
		if(tau == 0) return (int)ceil(n/M);
		else return hFunction(n - ceil(n / M), M - 1, tau - 1);
	}

	std::pair<Graph *, ResourceModel *> Utility::unrollGraph(Graph *g, ResourceModel *resourceModel, int samples) {
		auto* g_unroll(new Graph);
		auto* rm_unroll(new ResourceModel);

		for(auto res : resourceModel->Resources()) {
			rm_unroll->makeResource(res->getName(),res->getLimit(),res->getLatency(),res->getBlockingTime());
		}

		for(auto v : g->Vertices()) {
			for(int s=0; s<samples; ++s) {
				auto& newVertex = g_unroll->createVertex();
				newVertex.setName(v->getName()+"_"+to_string(s));
				auto originalResource = resourceModel->getResource(v);
				rm_unroll->registerVertex(&newVertex,rm_unroll->getResource(originalResource->getName()));
			}
		}

		for(auto e : g->Edges()) {
			auto srcName = e->getVertexSrc().getName();
			auto dstName = e->getVertexDst().getName();

			int distance = e->getDistance();
			int offset = 0;

			// adjust distance/offset so distance < samples
			while(distance>samples) {
				distance -= samples;
				++offset;
			}

			for(int s=0; s<samples; ++s) {
				// adjust distance again (only once)
				int sourceSampleNumber = s - distance;
				int edgeOffset = offset;
				if(sourceSampleNumber < 0) {
					sourceSampleNumber += samples;
					++edgeOffset;
				}

				// create edge
				Vertex* srcVertex = nullptr;
				Vertex* dstVertex = nullptr;

				for(auto v : g_unroll->Vertices()) {
					if(v->getName() == srcName + "_" + to_string(sourceSampleNumber))
						srcVertex = v;
					if(v->getName() == dstName + "_" + to_string(s))
						dstVertex = v;
				}

				g_unroll->createEdge(*srcVertex,*dstVertex,edgeOffset,e->getDependencyType());
			}
		}

		return {g_unroll, rm_unroll};
	}

	double Utility::getNumberOfEquivalent2x1Muxs(int numFUConnections, Graph *g, ResourceModel *rm) {
		for (auto &r : rm->Resources()) {
			// number of FUs for this resource
			int numFUs;
			if (r->isUnlimited()) {
				numFUs = rm->getNumVerticesRegisteredToResource(r);
			}
			else {
				numFUs = r->getLimit();
			}
			// number of inputs for each FU
			int numInputs = 0;
			for (auto &v : rm->getVerticesOfResource(r)) {
				auto numVertexInputs = g->getPredecessors(v).size();
				if (numVertexInputs > numInputs) numInputs = numVertexInputs;
			}
			// subtract the number of input ports * the number of FUs
			// e.g. an FU has 2 inputs to port number 1 and 6 inputs to port number 2
			// then one 2x1 mux is needed for port 1 and 5 2x1 muxs are needed on port number 2
			// this means that #2x1 muxs = 1+5 = 6 = 2+6-#ports = 2+6-2
			// since this holds for each implemented FU, we must multiply the number of inputs with the number of FUs
			numFUConnections -= (numInputs * numFUs);
		}
		return numFUConnections;
	}

	std::pair<int, int> Utility::getMaxRegsAndMuxs(Graph *g, ResourceModel *rm, std::map<Vertex *, int> times, int II) {
		int maxRegs = 0;
		int maxConnections = 0;
		int maxMuxs = 0;
		// upper bound for the number of registers
		std::map<const HatScheT::Vertex*, int> sortedLifetimesVertices;
		std::map<const HatScheT::Resource*, std::list<int>> sortedLifetimesResources;
		try {
			for (auto &e : g->Edges()) {
				if (!e->isDataEdge()) continue;
				auto *vSrc = &e->getVertexSrc();
				auto *vDst = &e->getVertexDst();
				auto *rSrc = rm->getResource(vSrc);
				auto *rDst = rm->getResource(vDst);
				auto tSrc = times.at(vSrc);
				auto tDst = times.at(vDst);
				auto latSrc = rm->getVertexLatency(vSrc);
				auto distance = e->getDistance();
				auto lifetime = tDst - tSrc - latSrc + (II * distance);
				if (sortedLifetimesVertices[vSrc] < lifetime) sortedLifetimesVertices[vSrc] = lifetime;
			}
			for (auto &it : sortedLifetimesVertices) {
				auto &v = it.first;
				auto res = rm->getResource(v);
				auto &lifetime = it.second;
				sortedLifetimesResources[res].emplace_front(lifetime);
				sortedLifetimesResources[res].sort();
			}
			for (auto &it : sortedLifetimesResources) {
				auto &res = it.first;
				int numIt;
				if (res->isUnlimited()) {
					numIt = rm->getNumVerticesRegisteredToResource(res);
				}
				else {
					numIt = res->getLimit();
				}
				auto &sortedStuff = it.second;
				auto listIt = sortedStuff.end();
				int regs = 0;
				for (int i=0; i<numIt; i++) {
					listIt--;
					regs += (*listIt);
				}
				maxRegs += regs;
			}
		}
		catch (std::out_of_range&) {
			throw Exception("Utility::getMaxRegsAndMuxs: allocation or schedule corrupt");
		}
		// upper bound for the number of interconnect lines
		for (auto &e : g->Edges()) {
			if (!e->isDataEdge()) continue;
			maxConnections += 1;
		}
		// calculate upper bound for multiplexers from upper bound of interconnect lines
		maxMuxs = (int)HatScheT::Utility::getNumberOfEquivalent2x1Muxs(maxConnections, g, rm);
		return {maxRegs, maxMuxs};
	}

	double Utility::getNumberOfFUConnections(int num2x1Muxs, Graph *g, ResourceModel *rm) {
		for (auto &r : rm->Resources()) {
			// skip unlimited resources because they never need any muxs
			if (r->isUnlimited()) continue;
			// calculate the number of inputs for this FU
			int numInputs = 0;
			for (auto &v : rm->getVerticesOfResource(r)) {
				auto numVertexInputs = g->getPredecessors(v).size();
				if (numVertexInputs > numInputs) numInputs = numVertexInputs;
			}
			// add the number of input ports * the number of FUs
			// e.g. an FU has 2 inputs to port number 1 and 6 inputs to port number 2
			// then one 2x1 mux is needed for port 1 and 5 2x1 muxs are needed on port number 2
			// this means that #connections = 2+6 = 8 = 1+5+#ports = 1+5+2
			// since this holds for each implemented FU, we must multiply the number of inputs with the number of FUs
			num2x1Muxs += (numInputs * r->getLimit());
		}
		return num2x1Muxs;
	}

}
