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

#ifdef USE_SCALP
double Utility::calcMinII(double minResII, double maxResII)
{
  if(minResII>maxResII) return minResII;

  return maxResII;
}

double Utility::calcResMII(Graph *g, ResourceModel *rm, Target* t)
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

  }

  return resMII;
}

int Utility::calcMaxII(Graph *g, ResourceModel *rm)
{
  //determine minimum possible latency
  HatScheT::ASAPScheduler asap(*g,*rm);
  asap.schedule();
  int criticalPath = asap.getScheduleLength();

  if(g->getNumberOfVertices() > 200) return criticalPath;

  //get optimal critical path using asap ilp scheduler
  HatScheT::ASAPILPScheduler* asapilp = new HatScheT::ASAPILPScheduler(*g, *rm, {"CPLEX", "Gurobi", "SCIP"});
  asapilp->setMaxLatencyConstraint(criticalPath);
  asapilp->schedule();
  criticalPath = asapilp->getScheduleLength();

  delete asapilp;

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

  return max(1.0, (solver.getResult().objectiveValue)); // RecMII could be 0 if instance has no backedges.
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

bool Utility::edgeIsInGraph(Graph *g, Edge *e)
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

}
