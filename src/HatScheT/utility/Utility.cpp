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

vector<std::map<const Vertex*,int> > Utility::getBruteForceRatIIBinding(map<HatScheT::Vertex *, int> sched,
                                 HatScheT::Graph *g, HatScheT::ResourceModel *rm, int modulo, vector<int> initIntervalls,
                                 map<HatScheT::Edge *, pair<int, int>> edgePortMappings) {
  //generate cotainer for bindings
  vector<std::map<const Vertex*,int> > ratIIBindings;



  return ratIIBindings;
}

vector<std::map<const Vertex*,int> > Utility::getILPBasedRatIIBinding(map<Vertex*, int> sched, Graph* g,
  ResourceModel* rm,  int modulo, vector<int> initIntervalls, std::list<std::string> sw , int timeout){
  // create and setup solver
  if(sw.empty()) sw = {"Gurobi","CPLEX","SCIP","LPSolve"};
  auto solver = ScaLP::Solver(sw);
  solver.quiet = true;
  solver.threads = 1;
  solver.timeout=timeout;

  int samples = initIntervalls.size();

  vector<std::map<const Vertex*,int> > ratIIBindings;
  ratIIBindings.resize(samples);

  // samples, vertex, resource unit (true/false)
  vector<map<const Vertex*, vector<ScaLP::Variable > > > binding_vars;
  binding_vars.resize(samples);

  //fill binding_vars
  for(int i = 0; i < samples; i++) {
    for (auto it = g->verticesBegin(); it != g->verticesEnd(); ++it) {
      Vertex *v = *it;
      const Resource *r = rm->getResource(v);
      //skip unlimited
      //TODO handle unlimited resources
      //TODO fix their binding variables using '==constraint' ?
      if (r->getLimit() == -1) continue;

      //generate ilp variables for binding
      vector<ScaLP::Variable> vars;
      for (int j = 0; j < r->getLimit(); j++) {
        vars.push_back(ScaLP::newBinaryVariable("t'" + v->getName() + "_s" + to_string(i) + "_r" + to_string(j)));
      }

      binding_vars[i].insert(make_pair(v, vars));
    }
  }

  //add explicitness constraint(1) from the paper
  for(auto it : binding_vars){
    for(auto it2 : it) {
      //TODO get here all vertices that use samples in this time step and combine them
      ScaLP::Term expl;
      for(auto it3 : it2.second) {
        expl += it3;
      }
      solver.addConstraint(expl == 1);
    }
  }

  //sort the vertices by the time slot they are scheduled
  //map<int, vector<const Vertex*> > v_timesorted;

  //add binding constraints
  //respect the MRT in respect to modulo slots and resource constraints
  for(auto it = rm->resourcesBegin(); it != rm->resourcesEnd(); ++it){
    const Resource* r = *it;
    set<const Vertex*> verts = rm->getVerticesOfResource(r);

    //sort all ILP variables that fall into the same modulo time slot
    map<int, vector<ScaLP::Variable > > variables_timesorted;

    //use scalp term for every hardware unit and MRT time slot
    vector<vector<ScaLP::Term> > slot_terms;
    for(int i = 0; i < r->getLimit(); i++) {
      vector<ScaLP::Term> terms;
      for (int j = 0; j < modulo; j++) {
        ScaLP::Term t;
        terms.push_back(t);
      }
      slot_terms.push_back(terms);
    }

    //add the ScaLP::Variables to the corresponding ScaLP::Term
    //this describes the MRT
    for(auto it2 : verts) {
      const Vertex* v = it2;
      int modulo_slot = 0;
      for(auto it3: sched) {
        Vertex* v2 = it3.first;
        if(v==v2) {
          int distance = 0;
          for(int i = 0; i < samples; i++){
            if(i>0) distance+=initIntervalls[i-1];

            modulo_slot = (it3.second + distance) % modulo;
            for(int j = 0; j < r->getLimit(); j++){
              slot_terms[j][modulo_slot] += binding_vars[i][v][j];
            }
          }
        }
      }

    }
    //every hardware unit can only perform one operation each time step
    //according to (3) / (4) in the paper
    //question : what about blocking time > 1 ? this should have harder constraints than a fixed '1' (patrick)
    for(auto it2: slot_terms){
      for(auto it3 : it2){
        solver.addConstraint(it3 <= 1);
      }
    }
  }

  //TODO add MUX constraints
  //this is an attempt for two layer if->else using big-M
  //for all inports -> boolean variable if there is a mux input needed for this input edge
  vector<vector<ScaLP::Variable > > mux_inputs;
  //for all inports: boolean variable if mux is needed at all
  vector<ScaLP::Variable> mux_number;

  //TODO model mux_inputs
  for(auto it = g->verticesBegin(); it != g->verticesEnd(); ++it){
    Vertex* v = *it;
    set<Vertex*> pred = g->getPredecessors(v);

    //TODO experimental, debugging reasons
    if(pred.size() > 1) continue;
    list<const Edge*> edges = g->getEdges(v, *pred.begin());
    const Edge* e = *edges.begin();

    //TODO if
  }

  //TODO model total mux number

  //TODO add objective

  //write lp file
  solver.writeLP("binding.lp");

  //solve the problem
  ScaLP::status stat = solver.solve();
  cout << "Utility.getILPBasedRatIIBinding: The binding problem was solved: " << stat << endl;
  cout << solver.getResult() << endl;

  //add the solution to return container and print the binding
  //TODO include check for consistency / feasible binding
  int sample=0;
  for(auto it : binding_vars){
    for(auto it2 : it) {
      const Vertex* v = it2.first;
      int unit = 0;
      for(auto it3 : it2.second) {
        if(solver.getResult().values[it3] == true) {
          //add to solution structure
          ratIIBindings[sample][v] = unit;

          cout << "Bound " << v->getName() << "_s" << to_string(sample) << " to unit " << to_string(unit) << endl;
        }
        else unit++;
      }
    }
    sample++;
  }

  Utility::printRationalIIMRT(sched, ratIIBindings, rm, modulo, initIntervalls);

  //throw error, this binding function is not finished yet
  cout << "Utility.getILPBasedRatIIBinding: this binding function is not finished yet" << endl;
  throw Exception("Utility.getILPBasedRatIIBinding: this binding function is not finished yet");

  return ratIIBindings;
}

vector<std::map<const Vertex*,int> > Utility::getSimpleRatIIBinding(map<HatScheT::Vertex *, int> sched,
                                                                    HatScheT::ResourceModel *rm, int modulo,
                                                                    vector<int> initIntervalls) {
  vector<std::map<const Vertex*,int> > ratIIBindings;
  int samples = initIntervalls.size();
  ratIIBindings.resize(samples);

  std::map<const Resource*, std::map<int, int>> resourceCounters;

  for(auto it : sched) {
    auto v = it.first;
    const Resource* res = rm->getResource(v);
    //the offset for the respective sample based on the initInterval vector
    int offset = 0;
    //iterate over the samples
    for(int i = 0; i < samples; i++) {
      //increase the offset
      if(i>0) offset+=initIntervalls[i-1];

      if (res->getLimit() < 0) {
        //for unlimited resources, each sample gets bound to the same unit, each operation gets implemented by its own unit
        if (resourceCounters[res].find(0) != resourceCounters[res].end()) {
          if(offset==0) resourceCounters[res][0]++;
        } else {
          resourceCounters[res][0] = 0;
        }
        ratIIBindings[i][v] = resourceCounters[res][0];
      } else {
        int time = (it.second + offset) % modulo;
        if (resourceCounters[res].find(time) != resourceCounters[res].end()) {
          resourceCounters[res][time]++;
        } else {
          resourceCounters[res][time] = 0;
        }
        if (resourceCounters[res][time] > res->getLimit())
          throw HatScheT::Exception(
            "Utility::getSimpleRatIIBinding: found resource conflict while creating binding for resource "
            + res->getName() + "(limit " + to_string(res->getLimit()) + " )");
        ratIIBindings[i][v] = resourceCounters[res][time];
      }

    }
  }

  return ratIIBindings;
}

#ifdef USE_SCALP
std::map<const Vertex *, int> Utility::getMUXOptimalBinding(map<Vertex *, int> sched, ResourceModel *rm, int II) {
    cout << "starting mux optimal binding" << endl;
    std::map<const Vertex *, int> binding;

    // create solver
    auto s = ScaLP::Solver({"Gurobi","CPLEX","SCIP","LPSolve"});

    //work for jorge
    //stream constraints into solver and solve!

    throw HatScheT::Exception("Utility.getMUXOptimalBinding: this method is not working currently!! use getSimpleBinding!");
    return binding;
}
#endif

std::map<const Vertex *, int> Utility::getSimpleBinding(map<Vertex *, int> sched, ResourceModel *rm, int II) {
  std::map<const Vertex *, int> binding;
  std::map<const Resource*, std::map<int, int>> resourceCounters;

  for(auto it : sched) {
    auto v = it.first;
    const Resource* res = rm->getResource(v);
    if(res->getLimit()<0) {
      binding[v] = resourceCounters[res][0];
      resourceCounters[res][0]++;
    } else {
      int time = it.second % II;
      binding[v] = resourceCounters[res][time];
      if(resourceCounters[res][time] >= res->getLimit())
          throw HatScheT::Exception("Utility::getSimpleBinding: found resource conflict while creating binding for resource "
          + res->getName() + "(limit " + to_string(res->getLimit()) + " )");
      resourceCounters[res][time]++;
    }
  }
  return binding;
}
#ifdef USE_SCALP
std::map<const Vertex *, int> Utility::getILPMinRegBinding(map<Vertex *, int> sched, Graph *g, ResourceModel *rm, int II, std::list<std::string> sw, int timeout) {

  // container to return
  map<const Vertex*,int> binding;
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
  std::map<const Resource*,int> unlimitedResourceCounter;
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
	  binding[it.first] = unlimitedResourceCounter[res];
	  unlimitedResourceCounter[res]++;
	  continue;
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
  	cout << "Utility::getILPMinRegBinding: caught ScaLP exception: '" << std::string(e.what()) << "' returning simple binding";
  	return Utility::getSimpleBinding(sched,rm,II);
  }

  auto results = solver.getResult().values;
  for(auto &it1 : vertexVariables){
	auto vertex = it1.first;
	for(int i = 0; i < (int)it1.second.size(); i++){
	  auto &it2 = it1.second[i];
	  auto val = results[it2];
	  if(val==1.0){
		binding[vertex] = i;
	  }
	}
  }

  return binding;
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

#endif
}
