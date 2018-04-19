#include "HatScheT/utility/Utility.h"
#include "ScaLP/Solver.h"

#include <limits>
#include <cmath>
#include <map>

namespace HatScheT {

bool Utility::examplUtilityFunction(ResourceModel *rm, Graph *g)
{
  return true;
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

int Utility::calcMinII(ResourceModel *rm, Graph *g)
{
<<<<<<< HEAD
  int resMII = Utility::calcResMII(rm,g);
  int recMII = Utility::calcRecMII(rm,g);
=======
  int resMII = Utility::calcResMII(rm, g);
  int recMII = Utility::calcRecMII(rm, g);
>>>>>>> 5645e279967fd8a43960ab7ed06be6e9e4824f00

  if(resMII>recMII) return resMII;

  return recMII;
}

int Utility::calcResMII(ResourceModel *rm, Graph *g)
{
  int resMII = 1;

  for(auto it=rm->resourcesBegin(); it!=rm->resourcesEnd(); ++it){
    Resource* r = *it;
    //skip unlimited resources
    if(r->getLimit()==-1)continue;
    int opsUsingR = rm->getNoOfVerticesRegisteredToResource(r);
    int avSlots = r->getLimit();

    if(avSlots<=0) throw new Exception("Utility.calcResMII: avSlots <= 0 : " + to_string(avSlots));
    int tempMax = opsUsingR/avSlots + (opsUsingR % avSlots != 0);

    if(tempMax>resMII) resMII=tempMax;
  }

  return resMII;
}

int Utility::calcMaxII(SchedulerBase *sb)
{
  sb->schedule();;
  return sb->getScheduleLength();
}

<<<<<<< HEAD
int Utility::calcRecMII(ResourceModel *rm,Graph *g)
=======
int Utility::calcRecMII(ResourceModel *rm, Graph *g)
>>>>>>> 5645e279967fd8a43960ab7ed06be6e9e4824f00
{
  ScaLP::Solver solver({"CPLEX", "Gurobi"});

  // construct decision variables
  auto II = ScaLP::newIntegerVariable("II", 0, std::numeric_limits<int>::max());
  std::map<Vertex *, ScaLP::Variable> t;
  for (auto it = g->verticesBegin(), end = g->verticesEnd(); it != end; it++) {
    auto v = *it;
    t[v] = ScaLP::newIntegerVariable("t_" + to_string(v->getId()), 0, std::numeric_limits<int>::max());
  }

<<<<<<< HEAD
  for(auto it=g->edgesBegin(); it!=g->edgesEnd(); it++){         
    Edge* e = *it;
    Vertex* vSrc = &e->getVertexSrc();
    if((e->getDistance()+rm->getVertexLatency(vSrc)) > recMII) recMII = e->getDistance()+rm->getVertexLatency(vSrc);
=======
  // construct constraints
  for (auto it = g->edgesBegin(), end = g->edgesEnd(); it != end; it++) {
    auto e = *it;
    auto i = &e->getVertexSrc();
    auto j = &e->getVertexDst();

    solver << ((t[i] + rm->getVertexLatency(i) + e->getDelay() - t[j] - (e->getDistance()*II)) <= 0);
>>>>>>> 5645e279967fd8a43960ab7ed06be6e9e4824f00
  }

  // construct objective
  solver.setObjective(ScaLP::minimize(II));

  auto status = solver.solve();
  if (status != ScaLP::status::OPTIMAL && status != ScaLP::status::FEASIBLE)
    throw new Exception("RecMII computation failed!");

  return max(1, (int) std::round(solver.getResult().objectiveValue)); // RecMII could be 0 if instance has no backedges.
}

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
