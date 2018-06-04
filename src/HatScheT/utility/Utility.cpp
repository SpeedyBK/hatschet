#include "HatScheT/utility/Utility.h"
#include "ScaLP/Solver.h"
#include "HatScheT/MoovacMinRegScheduler.h"
#include "HatScheT/ModuloSDCScheduler.h"
#include "HatScheT/scheduler/ASAPScheduler.h"
#include "HatScheT/scheduler/ULScheduler.h"
#include "HatScheT/Verifier.h"
#include <limits>
#include <cmath>
#include <map>
#include <ios>
#include <fstream>
#include <ctime>
#include <cstddef>
#include <iomanip>

namespace HatScheT {

bool Utility::examplUtilityFunction(ResourceModel *rm, Graph *g)
{
  return true;
}

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

int Utility::calcMinII(ResourceModel *rm, Graph *g)
{
  int resMII = Utility::calcResMII(rm,g);
  int recMII = Utility::calcRecMII(rm,g);

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


int Utility::calcRecMII(ResourceModel *rm, Graph *g)
{
  ScaLP::Solver solver({"CPLEX", "Gurobi"});

  // construct decision variables
  auto II = ScaLP::newIntegerVariable("II", 0, std::numeric_limits<int>::max());
  std::map<Vertex *, ScaLP::Variable> t;
  for (auto it = g->verticesBegin(), end = g->verticesEnd(); it != end; it++) {
    auto v = *it;
    t[v] = ScaLP::newIntegerVariable("t_" + to_string(v->getId()), 0, std::numeric_limits<int>::max());
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
    throw new Exception("RecMII computation failed!");
  }

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

void Utility::compareRegisterUsage(Graph &g, ResourceModel &resourceModel, std::list<string> solverWishlist, string logFileName)
{
  HatScheT::SchedulerBase* scheduler;
  int moovacII=0;
  int minRegII=0;
  int moovacRegs=0;
  int minRegRegs=0;
  int moovacSL=0;
  int minRegSL=0;
  bool moovacVerified=false;
  bool minRegVerified=false;
  std::map<const Vertex *, int>  moovacBinding;
  std::map<const Vertex *, int>  minRegBinding; 

  for(int i = 0; i < 2;i++){
    //select scheduler
    if(i==0){
      scheduler = new HatScheT::MoovacScheduler(g, resourceModel, solverWishlist);
      MoovacScheduler* schedulerPtr= dynamic_cast<MoovacScheduler*>(scheduler);
      schedulerPtr->setSolverTimeout(300);
    }
    if(i==1){
      scheduler = new HatScheT::MoovacMinRegScheduler(g, resourceModel, solverWishlist);
      MoovacMinRegScheduler* schedulerPtr= dynamic_cast<MoovacMinRegScheduler*>(scheduler);
      schedulerPtr->setSolverTimeout(300);
      schedulerPtr->setMaxLatencyConstraint(moovacSL);
    }

    //do the scheduling
    scheduler->schedule();

    if(i==0){
        MoovacScheduler* schedulerPtr= dynamic_cast<MoovacScheduler*>(scheduler);
        moovacVerified = HatScheT::verifyModuloSchedule(g, resourceModel, scheduler->getStartTimes(), scheduler->getII());
        moovacII = scheduler->getII();
        moovacRegs = schedulerPtr->getNoOfImplementedRegisters();
        moovacSL = schedulerPtr->getScheduleLength();
        moovacBinding = schedulerPtr->getBindings();
    }
    if(i==1){
        MoovacMinRegScheduler* schedulerPtr= dynamic_cast<MoovacMinRegScheduler*>(scheduler);
        minRegVerified = HatScheT::verifyModuloSchedule(g, resourceModel, scheduler->getStartTimes(), scheduler->getII());
        minRegII = scheduler->getII();
        minRegRegs = schedulerPtr->getNoOfImplementedRegisters();
        minRegSL = schedulerPtr->getScheduleLength();
        minRegBinding = schedulerPtr->getBindings();
    }
    }

    //find graph name
    string graphstr = g.getName();
    std::size_t found = graphstr.find_last_of("/\\");
    graphstr = graphstr.substr(found+1);

    //find set name
    string setstr = g.getName();
    std::size_t found1 = setstr.find_last_of("/\\");
    setstr = setstr.substr(0,found1);
    std::size_t found2 = setstr.find_last_of("/\\");
    setstr = setstr.substr(found2+1);

    //
    double regSavedPer = (double)(moovacRegs-minRegRegs)/(double)moovacRegs;
    double slDiffPer = (double)(moovacSL-minRegSL)/(double)moovacSL;

    //log general data
    std::ofstream log(logFileName, std::ios_base::app | std::ios_base::out);
    log << setstr << ";"  << graphstr << ";" << moovacII-minRegII << ";" << moovacRegs << ";" << minRegRegs << ";"
        << setprecision(2) << fixed << regSavedPer << ";" << setprecision(2) << fixed << slDiffPer << ";" << moovacSL << ";" << minRegSL  << ";";
    //log binding data
    for(auto it=resourceModel.resourcesBegin();it!=resourceModel.resourcesEnd();++it){
      const Resource* r = *it;
      if(r->getLimit()<=0) continue;

      int moovacCount = Utility::calcUsedOperationsOfBinding(moovacBinding,resourceModel,const_cast<Resource*>(r));
      int minRegCount = Utility::calcUsedOperationsOfBinding(minRegBinding,resourceModel,const_cast<Resource*>(r));
      log << r->getName() << ";" << to_string(moovacCount-minRegCount) << ";";
    }
    log << endl;
    log.close();
}

void Utility::adaptiveScheduling(Graph &g, ResourceModel &resourceModel, std::list<string> solverWishlist, string logFileName)
{
  int noOfVertices = g.getNumberOfVertices();
  HatScheT::SchedulerBase* scheduler;
  string schedulerUsed = "";

  if(noOfVertices < 100){
    scheduler = new HatScheT::MoovacScheduler(g, resourceModel, solverWishlist);
    schedulerUsed = "Moovac";
  }
  else if(100 <= noOfVertices && noOfVertices <= 150){
    scheduler = new HatScheT::ModuloSDCScheduler(g, resourceModel, solverWishlist);
    schedulerUsed = "ModuloSDC";
  }
  else if(noOfVertices > 150){
    scheduler = new HatScheT::ULScheduler(g, resourceModel);
    schedulerUsed = "ULScheduler";
  }

  //measure time and schedule
  clock_t begin = clock();
  scheduler->schedule();
  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

  //verify schedule
  bool verified = false;
  bool schedFound = false;
  if(scheduler->getII()!=-1) {
    schedFound = true;
    verified = HatScheT::verifyModuloSchedule(g, resourceModel, scheduler->getStartTimes(), scheduler->getII());
  }

  //find graph name
  string graphstr = g.getName();
  std::size_t found = graphstr.find_last_of("/\\");
  graphstr = graphstr.substr(found+1);

  //find set name
  string setstr = g.getName();
  std::size_t found1 = setstr.find_last_of("/\\");
  setstr = setstr.substr(0,found1);
  std::size_t found2 = setstr.find_last_of("/\\");
  setstr = setstr.substr(found2+1);

  //log data
  std::ofstream log(logFileName, std::ios_base::app | std::ios_base::out);
  log << setstr << ";"  << graphstr << ";" << scheduler->getII() << ";" << to_string(elapsed_secs) << ";" << scheduler->getScheduleLength() << ";" << schedFound
      << ";" << verified << ";" << schedulerUsed << ";" << noOfVertices << ";" << endl;
  log.close();
}

void Utility::evaluateSchedulers(Graph &g, ResourceModel &resourceModel, std::list<string> solverWishlist, std::string logFileName)
{
  string logNameInsert = logFileName;
  HatScheT::SchedulerBase* scheduler;

  for(int i = 0; i < 6;i++){
    //reset logfilename
    logFileName = logNameInsert;

    //select scheduler
    if(i==0){
      logFileName += "ASAP.txt";
      scheduler = new HatScheT::ASAPScheduler(g,resourceModel);
    }
    if(i==1){
      logFileName += "ULScheduler.txt";
      scheduler = new HatScheT::ULScheduler(g, resourceModel);
    }
    if(i==2){
      logFileName += "ModuloSDC1min.txt";
      scheduler = new HatScheT::ModuloSDCScheduler(g, resourceModel, solverWishlist);
      ModuloSDCScheduler* schedulerPtr= dynamic_cast<ModuloSDCScheduler*>(scheduler);
      schedulerPtr->setSolverTimeout(60);
    }
    if(i==3){
      logFileName += "ModuloSDC5min.txt";
      scheduler = new HatScheT::ModuloSDCScheduler(g, resourceModel, solverWishlist);
      ModuloSDCScheduler* schedulerPtr= dynamic_cast<ModuloSDCScheduler*>(scheduler);
      schedulerPtr->setSolverTimeout(300);
    }
    if(i==4){
      logFileName += "Moovac1Min.txt";
      scheduler = new HatScheT::MoovacScheduler(g, resourceModel, solverWishlist);
      MoovacScheduler* schedulerPtr= dynamic_cast<MoovacScheduler*>(scheduler);
      schedulerPtr->setSolverTimeout(60);
    }
    if(i==5){
      logFileName += "Moovac5Min.txt";
      scheduler = new HatScheT::MoovacScheduler(g, resourceModel, solverWishlist);
      MoovacScheduler* schedulerPtr= dynamic_cast<MoovacScheduler*>(scheduler);
      schedulerPtr->setSolverTimeout(300);
    }

    //measure time and schedule
    clock_t begin = clock();
    scheduler->schedule();
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

    //verify schedule
    bool verified = false;
    bool schedFound = false;
    if(scheduler->getII()!=-1) {
      schedFound = true;
      verified = HatScheT::verifyModuloSchedule(g, resourceModel, scheduler->getStartTimes(), scheduler->getII());
    }

    //do normalization
    int minII = Utility::calcMinII(&resourceModel,&g);
    int foundII = scheduler->getII();
    int denominator = 1+ foundII - minII;
    float normalizedII = (float)1/denominator;

    //find graph name
    string str = g.getName();
    std::size_t found = str.find_last_of("/\\");
    str = str.substr(found+1);

    //find set name
    string setstr = g.getName();
    std::size_t found1 = setstr.find_last_of("/\\");
    setstr = setstr.substr(0,found1);
    std::size_t found2 = setstr.find_last_of("/\\");
    setstr = setstr.substr(found2+1);

    //log data
    std::ofstream log(logFileName, std::ios_base::app | std::ios_base::out);
    log << setstr << ";" << str << ";" << minII << ";" << foundII << ";" << to_string(elapsed_secs) << ";" << scheduler->getScheduleLength() << ";"
    << setprecision(3) << fixed << normalizedII << ";" << g.getNumberOfVertices() << ";" << g.getNumberOfEdges() << endl;
    log.close();
  }
}

int Utility::calcUsedOperationsOfBinding(map<const Vertex *, int> &binding, ResourceModel& rm, Resource *r)
{
  int opsUsed=0;
  //unlimited resources used in paralell
  if(r->getLimit()<=0){
    return rm.getNoOfVerticesRegisteredToResource(r);
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
