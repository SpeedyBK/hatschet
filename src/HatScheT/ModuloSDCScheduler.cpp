#include "utility/Utility.h"
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/ModuloSDCScheduler.h>
#include <HatScheT/Graph.h>
#include <HatScheT/Verifier.h>
#include <ctime>
#include <map>
#include <vector>
#include <algorithm>
#include <ctime>
#include <numeric>
#include <chrono>

HatScheT::MRT::MRT(HatScheT::ResourceModel& r, int ii)
: rm(&r), II(ii)
{
  std::cout << "Create MRT with II=" << II << std::endl;
  for(auto it=r.resourcesBegin(); it != r.resourcesEnd(); ++it)
  {
    if((*it)->getLimit()<0) continue;
    data.emplace(*it,std::vector<std::vector<HatScheT::Vertex*>>(II));
    for(int i=0;i<II;++i)
    {
      data.at(*it)[i]=std::vector<HatScheT::Vertex*>((*it)->getLimit());
    }
  }
}

static void printMRT(const HatScheT::MRT& mrt)
{
  std::cout << "MRT:" << std::endl;
  for(auto&p:mrt.data)
  {
    std::cout << "  " << p.first->getName() << ": " << std::endl;
    for(unsigned int i=0; i<p.second.size();++i)
    {
      std::cout << "    " << std::to_string(i) << ": ";
      for(auto&o:p.second[i])
      {
        if(o!=nullptr) std::cout << o->getName() << ", ";
      }
      std::cout << std::endl;

    }

  }
  std::cout << "MRT end" << std::endl;
}

bool HatScheT::MRT::resourceConflict(HatScheT::Vertex* v,int time)
{
  time%=II;
  auto r = rm->getResource(v);
  auto vs = rm->getVerticesOfResource(r);

  if(r->getLimit()>=0)
  {
    int count=0;
    for(auto&p:data[r][time]){
      if(p!=nullptr) ++count;
      //cant add the vertix to the same slot twice
      if(p==v) return true;
    }

    bool b = std::any_of(data[r][time].begin(), data[r][time].end(), [](HatScheT::Vertex* a){return a==nullptr;});
    for(int i=0;i<=time;++i)
    {
      for(Vertex*p:data[r][i])
      {
        if(p==nullptr or p==v) continue;
        auto o = vs.find(p);
        if(o!=vs.end())
        {
          if(i==time && count>=r->getLimit())
          {
            b=false;
            break;
          }
        }
      }
      if(b) break;

    }
    return not b;
  }
  else return false;
}

bool HatScheT::MRT::update(HatScheT::Vertex* i, int time)
{
  time%=II;
  //dont try to manipulate vertices from a unlimited resource as it will corrupt the MRT
  auto r = rm->getResource(i);
  if(r->getLimit()<=0) return true;
  auto& v = data.at(r).at(time);
  auto it = std::find(v.begin(),v.end(),nullptr);

  for(auto it:v){
    if(i==it){    
      return false;
    }
  }

  if(it!=v.end())
  {
    *it=i;
    return true;
  }
  else
  {
    std::cerr << "update can't add the instruction, no room left (this should never happen)" << std::endl;
    return false;
  }
}

bool HatScheT::MRT::remove(HatScheT::Vertex* i)
{
  auto r = rm->getResource(i);
  //dont try to remove vertices from a unlimited resource as it will corrupt the MRT
  if(r->getLimit()<=0) return true;

  for(auto& v:data[r])
  {
    for(auto&a:v)
    {
      if(a==i) a=nullptr;
    }
  }

  return true;
}

HatScheT::ModuloSDCScheduler::ModuloSDCScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist)
  : SchedulerBase(g,resourceModel)
  , ILPSchedulerBase(solverWishlist)
  , mrt({resourceModel,1})
{
  this->verbose = true;
  this->minII = this->computeMinII(&g,&resourceModel);
  HatScheT::ASAPScheduler asap(g,resourceModel);
  this->maxII = Utility::calcMaxII(&asap);
  if (minII >= maxII) maxII = minII+1;
}

bool HatScheT::MRT::vertexIsIn(Vertex *v)
{
  for(auto& it:data)
  {
    for(auto& it2:it.second)
    {
      for(auto& it3:it2)
      {
        if(it3==v) return true;
      }
    }
  }

  return false;
}

HatScheT::Vertex* HatScheT::MRT::getInstructionAt(unsigned int i, const Resource* r)
{
  std::cout << "getInstruction at " << std::to_string(i) << ": ";// << std::endl;
  for(auto&p:data)
  {
    if(p.second.size()>=i && p.first==r)
    {
      for(auto&o:p.second[i])
      {
        if(o!=nullptr)
        {
          std::cout << "returning instruction " << o->getName() << std::endl;
          return o;
        }
      }
    }
    else
    {
      continue;
    }
  }
  std::cout << "NO INSTRUCTION" << std::endl;

  return nullptr;
}

void HatScheT::ModuloSDCScheduler::constructProblem()
{
  this->solver->reset();

  if(this->threads>0) this->solver->threads = this->threads;
  this->solver->quiet=this->solverQuiet;
  this->solver->timeout=this->solverTimeout;

  for(auto c:this->baseConstraints) *this->solver << c;
  for(auto c:this->constraints) *this->solver << c;

  setObjective();
}

static bool feasible(ScaLP::status stat)
{
  return stat == ScaLP::status::OPTIMAL
    or   stat == ScaLP::status::FEASIBLE
    or   stat == ScaLP::status::TIMEOUT_FEASIBLE;
}

bool HatScheT::ModuloSDCScheduler::solveBasicWithConstraint(ScaLP::Constraint&& c)
{
  /*this->solver->reset();

  if(this->threads>0) this->solver->threads = this->threads;
  this->solver->quiet=this->solverQuiet;
  this->solver->timeout=this->solverTimeout;

  for(auto c:this->baseConstraints) *this->solver << c;
  *this->solver << c;
*/
  setObjective();

  return feasible(this->solver->solve());
}

bool HatScheT::ModuloSDCScheduler::dependencyConflict(std::map<Vertex*,int>& prevSched, Vertex* I, int time)
{
  for(auto it=this->g.edgesBegin(); it!=this->g.edgesEnd(); ++it)
  {
    Edge* e = *it;
    Vertex* d = &(*it)->getVertexDst();
    if(I == d)
    {
      Vertex* s = &(*it)->getVertexSrc();
      auto r = this->resourceModel.getResource(s);
      if(prevSched[s]+r->getLatency()+e->getDelay()>time+this->II*e->getDistance())
      {
        std::cout << "Dependency conflict between " << s->getName() << " and " << d->getName() << std::endl;
        return true;
      }
    }
  }

  for(auto it=this->g.edgesBegin(); it!=this->g.edgesEnd(); ++it)
  {
    Edge* e = *it;
    Vertex* d = &e->getVertexSrc();
    if(I == d)
    {
      Vertex* s = &(*it)->getVertexDst();
      auto r = this->resourceModel.getResource(d);
      if(time+r->getLatency()+e->getDelay()>prevSched[s]+this->II*e->getDistance())
      {
        std::cout << "Dependency conflict between " << s->getName() << " and " << d->getName() << std::endl;
        return true;
      }
    }
  }

  return false;
}

static ScaLP::Variable getVariable(std::map<HatScheT::Vertex*,ScaLP::Variable>& variables, HatScheT::Vertex* i)
{
  return variables.at(i);
}

void HatScheT::ModuloSDCScheduler::createBaseConstraints(int II)
{
  for(std::set<Edge*>::iterator it = this->g.edgesBegin() ; it!=this->g.edgesEnd();++it)
  {
    Edge* e = (*it);
    Vertex* a = &e->getVertexSrc();
    Vertex* b = &e->getVertexDst();

    ScaLP::Variable va = getVariable(variables,a);
    ScaLP::Variable vb = getVariable(variables,b);
    auto vaLatency = this->resourceModel.getResource(a)->getLatency();
    this->baseConstraints.emplace_back(va + vaLatency + e->getDelay() - vb <= II * e->getDistance() );
  }
}

static std::map<HatScheT::Vertex*,int> solveLP(ScaLP::Solver& s, HatScheT::Graph& g, std::map<HatScheT::Vertex*,ScaLP::Variable>& variables, std::vector<ScaLP::Constraint>& cons, std::vector<ScaLP::Constraint>& bcons)
{
  for(auto& c:bcons) s << c;
  for(auto& c:cons) s << c;
  s.writeLP("eee.lp");
  auto rrr = s.solve();
  //std::cout << rrr << std::endl;
  std::map<HatScheT::Vertex*,int> m;

  if(feasible(rrr/*s.solve()*/))
  {
    auto result = s.getResult();

    for(auto it = g.verticesBegin(); it!=g.verticesEnd();++it)
    {
      auto it2 = result.values.find(getVariable(variables,*it));
      if(it2!=result.values.end())
      {
       // cout << (*it)->getName() << " : " << it2->second << endl;
        m.emplace(*it,static_cast<int>(it2->second));
      }
    }
  }
  return m;
}

static void createVariables(std::map<HatScheT::Vertex*,ScaLP::Variable>& variables,HatScheT::Graph& g)
{
  for(auto it = g.verticesBegin(); it!=g.verticesEnd();++it)
  { // TODO: remove upper bound
    ScaLP::Variable t_i = ScaLP::newIntegerVariable((*it)->getName(),0,10000);
    variables.emplace(*it,t_i);
  }
}

static unsigned int addLayerRec(HatScheT::Graph& g, std::map<HatScheT::Vertex*,unsigned int>& res, HatScheT::Vertex* v, HatScheT::Vertex* c)
{
  unsigned int count=0;
  for(auto e = g.edgesBegin(); e!=g.edgesEnd();++e)
  {
    if((*e)->getDistance()==0) continue;
    auto edgeSrc = &(*e)->getVertexSrc();
    auto edgeDst = &(*e)->getVertexDst();
    if(c == edgeSrc and v!=edgeDst)
    { // successor without loop
      count+=1+addLayerRec(g,res,v,edgeDst);
    }
  }
  return count;
}

static std::map<HatScheT::Vertex*,unsigned int> createASAPPerturbation(HatScheT::Graph& g, HatScheT::ResourceModel& rm)
{
  std::map<HatScheT::Vertex*,unsigned int> res;
  HatScheT::ASAPScheduler asap(g,rm);
  asap.schedule();

  for(auto v = g.verticesBegin(); v!=g.verticesEnd();++v){
    HatScheT::Vertex* ver = *v;
    int prio = std::abs(asap.getScheduleLength() - asap.getStartTime(*ver));
    res.emplace(*v,prio);
  }
  return res;
}

static std::map<HatScheT::Vertex*,unsigned int> createPerturbation(HatScheT::Graph& g)
{
  std::map<HatScheT::Vertex*,unsigned int> res;
  for(auto v = g.verticesBegin(); v!=g.verticesEnd();++v)
  {
    res.emplace(*v,addLayerRec(g,res,*v,*v)); // TODO: FIXME: wrong?
  }

  for(auto&p:res)
  {
    std::cout << "Perturbation: " << p.first->getName() << " = " << std::to_string(p.second) << std::endl;
  }

  return res;
}

bool HatScheT::ModuloSDCScheduler::sched(int II, int budget, const std::map<HatScheT::Vertex*,unsigned int>& priority)
{
  setObjective();

  const std::map<Vertex*,int> asap = solveLP(*(this->solver),this->g,this->variables,this->constraints,this->baseConstraints);

  if(asap.empty()){
    std::cerr << "Can't find ASAP-Schedule (current II is too small (?))" << std::endl;
    return false;
  }

  std::map<Vertex*,int> prevSched;
  for(auto&p:asap){
    prevSched.insert(p);
  }

  Queue schedQueue([&priority](Vertex* a,Vertex* b){return priority.at(a)<priority.at(b);});

  for(std::list<Resource*>::iterator it=this->resourceModel.resourcesBegin();it!=this->resourceModel.resourcesEnd();++it)
  {
    //handle vertices without resource constraints
    auto vs = this->resourceModel.getVerticesOfResource(*it);
    if((*it)->getLimit()<=0) continue;

    //handle vertices with resource constraints
    for(const HatScheT::Vertex* v:vs){
      this->neverScheduled.emplace(const_cast<Vertex*>(v),true);
      schedQueue.push(const_cast<Vertex*>(v));
    }
  }

  int b = budget;

  std::chrono::time_point<std::chrono::system_clock> time_Start = std::chrono::system_clock::now();

  for(; not schedQueue.empty() and b>=0; --b)
  {
    std::chrono::time_point<std::chrono::system_clock> time_It = std::chrono::system_clock::now();
    std::time_t time_var_it = std::chrono::system_clock::to_time_t(time_It);
    auto diff = std::chrono::duration_cast<std::chrono::seconds>(time_It-time_Start);
    auto secondsRun = diff.count();
    if(secondsRun>this->solver->timeout){
      cout << "Timeout for II " << this->II << endl;
      cout << "#operations left on schedQueue: " << schedQueue.size() << endl;
      return false;
    }
    auto i = schedQueue.top();
    schedQueue.pop();
    std::cout << "#### Begin of Iteration " << (budget-b+1) << " at II " << this->II << " at time " << std::ctime(&time_var_it) << " with " << i->getName() << std::endl;
    std::cout << "Elapsed time left is " << secondsRun << " (sec) with timeout " << this->solverTimeout << " (sec)" << std::endl;

    //DIRTY HACK
    //there is a bug that will put vertices that are already in the mrt back to schedule queue
    if(mrt.vertexIsIn(i)==true) continue;

    if(this->verbose==true) std::cout << "Current Instruction: " << i->getName() << std::endl;
    if(this->verbose==true) cout << "budget b : " << b << endl;

    int asapI=0;
    {
      auto it = asap.find(i);
      if(it!=asap.end()) asapI = it->second;
      else asapI = 0;
    }

    int time = 0;
    auto it = prevSched.find(i);
    if(it==prevSched.end())
    {
      time = asapI;
      if(this->verbose==true) std::cout << "use asap-time: " << time << std::endl;
    }
    else
    {     
      time = it->second;
      if(this->verbose==true) std::cout << "use scheduled-time: " << time << std::endl;
    }
    ScaLP::Variable t_i = getVariable(variables,i);

    if(this->verbose==true) printMRT(mrt);
    if(mrt.resourceConflict(i,time)==false && mrt.update(i,time)==true)
    {    
      if(this->verbose==true) std::cout << "Add (no resource conflict) " << t_i << " == " << time << std::endl;
      constraints.push_back(t_i == time);
      prevSched[i]=time;
    }
    else
    {
      if(this->verbose==true) std::cout << "Add (conflict detected) " << t_i << " >= " << (time+1) << std::endl;
      constraints.push_back(t_i >= time+1);
      this->constructProblem();
      this->setObjective();

      auto res = solveLP(*this->solver,this->g,this->variables,this->constraints, this->baseConstraints);

      if(not res.empty())
      {
        if(this->verbose==true) std::cout << "Put back " << i->getName() << std::endl;
        schedQueue.push(i);
        prevSched.swap(res);
      }
      else
      {
        if(this->verbose==true) std::cout << "backtrack because of " << i->getName() << std::endl;
        constraints.pop_back(); // remove >= Constraint
        backtracking(schedQueue, prevSched,i,asapI,time,II);
        this->constructProblem();

        //this->setObjective();
        auto a =  solveLP(*this->solver,this->g,this->variables,this->constraints, this->baseConstraints);

        if(not a.empty()) prevSched.swap(a);
      }

      if(this->writeLPFile) this->solver->writeLP(to_string(this->II));
    }

    if(this->resourceModel.getResource(i)->getLimit()>-1){
      if(this->mrt.vertexIsIn(i)==false) schedQueue.push(i);
    }
    std::cout << "#### End of Iteration\n\n" << std::endl;
  }

  std::cout << "Result for II " << this->II << std::endl;

  for(auto&p:prevSched)
  {
    if(this->verbose==true) std::cout << p.first->getName() << " = " << p.second << std::endl;
  }

  std::cout << "Final ";
  printMRT(mrt);

  if(schedQueue.empty() && verifyModuloSchedule(this->g,this->resourceModel,prevSched,this->II)==true)
  {
    this->solver->writeLP("finallp" + to_string(this->II));
    startTimes=prevSched;
    std::cout << "success" << std::endl;
    if(verifyModuloSchedule(this->g,this->resourceModel,prevSched,this->II)==false)
      cout << "ERROR (not detected) wrong prevSched stored as solution" << endl;
    return true;
  }
  else
  {
    std::cout << "No success" << std::endl;
    return false;
  }
}

static void removeAllConstraintsOf(std::vector<ScaLP::Constraint>& cons, ScaLP::Variable&& v)
{
  auto fun = [&v](const ScaLP::Constraint& c)
  {
    return c.term.sum.find(v)!=c.term.sum.end();
  };
  cons.erase(std::remove_if(cons.begin(),cons.end(),fun),cons.end());
}

void HatScheT::ModuloSDCScheduler::backtracking(Queue& schedQueue, std::map<Vertex*,int>& prevSched, HatScheT::Vertex* I, int asapTime, int time, int II)
{
  if(this->verbose==true) std::cout << "begin backtracking(" << I->getName() << " , " << time << ")" << std::endl;
  int minTime=0;
  int evictTime=0;
  for(minTime = asapTime;minTime<=time;++minTime)
  {
    if(solveBasicWithConstraint(getVariable(variables,I)==minTime)) break;
  }

  if(this->verbose==true) std::cout << "minTime:"  << minTime << std::endl;

  auto it = prevSched.find(I);
  if(neverScheduled[I] or minTime >= it->second)
  {
    if(this->verbose==true) std::cout << "use minTime " << minTime<< std::endl;
    evictTime = minTime;
  }
  else
  {
    if(this->verbose==true) std::cout << "use prevSched: " << (it->second+1) << std::endl;
    evictTime = it->second+1;
  }

  if(this->verbose==true) std::cout << "BACKTRACKING at evict time " << evictTime << std::endl;
  if(mrt.resourceConflict(I,evictTime))
  {
    if(this->verbose==true) cout << "resource conflict detected of " << I->getName() << " at " << evictTime << endl;
    // all Instructions that overlap with I
    std::set<Vertex*> evictInsts;
    //for(unsigned int i=0;evictTime%II>=i;++i)
    {
      //std::cout << "Check slot " << std::to_string(i) << std::endl;
      // get a instruction that overlaps with I
      Vertex* evictInst = mrt.getInstructionAt((evictTime%II)/*-1*/,this->resourceModel.getResource(I));
      if(evictInst!=nullptr)
      {
        evictInsts.insert(evictInst);
      }
    }
    if(not evictInsts.empty())
    {
      for(Vertex*evictInst:evictInsts)
      {
        if(this->verbose==true) std::cout << "Resource conflict in backtracking for: " << evictInst->getName() << std::endl;
        if(this->verbose==true) cout << "Removing all constraints of " << evictInst->getName() << endl;
        removeAllConstraintsOf(constraints,getVariable(variables,evictInst));
        mrt.remove(evictInst);
        schedQueue.emplace(evictInst);
        if(this->verbose==true) cout << evictInst->getName() << " put back to sched queue" << endl;
      }
    }
    else
    {
      if(this->verbose==true) std::cout << "No instruction found (this should never happen)" << std::endl;
    }

    // // Single Instruction
    //Vertex* evictInst = nullptr; //mrt.getInstructionAt(evictTime % II);
    //for(unsigned int i=0;evictTime%II>=i;++i)
    //{
    //  // get a instruction that overlaps with I
    //  evictInst = mrt.getInstructionAt((evictTime%II)-1,i);
    //  if(evictInst!=nullptr) break;
    //}
    //if(evictInst!=nullptr)
    //{
    //std::cout << "Resource conflict in backtracking for: " << evictInst->getName() << std::endl;
    //removeAllConstraintsOf(constraints,getVariable(variables,evictInst));
    //mrt.remove(evictInst);
    //}
    //else
    //{
    //  std::cout << "No instruction found (this should never happen)" << std::endl;
    //}
  }

  if(this->verbose==true) std::cout << "Check for dependency conflict at slot: " << (evictTime%II) << std::endl;
  if(dependencyConflict(prevSched,I,evictTime))
  {
    if(this->verbose==true) std::cout << "Dependency conflict." << std::endl;
    for(auto& p:prevSched)
    {
      const Resource* rp = this->resourceModel.getResource(p.first);
      //continue for unlimited resource
      if(rp->getLimit()<=0) continue;
      removeAllConstraintsOf(constraints,getVariable(variables,p.first));
      mrt.remove(p.first);
      schedQueue.emplace(p.first);
      if(this->verbose==true) cout << "put back " << p.first->getName() << endl;
    }
  }
  else
  {
    if(this->verbose==true) std::cout << "No dependency conflict" << std::endl;
  }


  if(mrt.update(I,evictTime)==true){
    if(this->verbose==true) std::cout << "add constraint: " << I->getName() << " == " << std::to_string(evictTime)  << std::endl;
    constraints.push_back(getVariable(variables,I)==evictTime);
    prevSched[I]=evictTime;
    this->neverScheduled[I]=false;
  }

  else{
    schedQueue.emplace(I);
    cout << "put back at end of backtracking: " << I->getName() << endl;
  }

  std::cout << "end backtracking" << std::endl;
}

void HatScheT::ModuloSDCScheduler::schedule()
{
  this->timeoutCounter = 0;
  this->totalTime = 0;
  this->variables.clear();
  createVariables(variables,g);
  unsigned int budget = 6*this->g.getNumberOfVertices();
  cout << "ModuloSDCScheduler::schedule: createPerturbation Start!" << endl;
  clock_t begin = clock();
  //std::map<Vertex*,unsigned int> priority= createPerturbation(this->g);
  std::map<Vertex*,unsigned int> priority= createASAPPerturbation(this->g,this->resourceModel);
  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "ModuloSDCScheduler::schedule: createPerturbation Finished after " << elapsed_secs << " seconds!" << endl;

  for(unsigned int ii=this->minII;ii<=this->maxII;++ii)
  {
    // cleanup and preparations
    this->solver->reset();
    this->constraints.clear();
    this->neverScheduled.clear();
    this->solver->timeout = this->solverTimeout;
    createBaseConstraints(ii);
    this->mrt = MRT(this->resourceModel,ii);
    this->II = ii;    

    cout << "Starting new iteration of ModuloSDC for II " << this->II << " with timeout " << this->solver->timeout << "(sec)" << endl;
    if(sched(ii,budget,priority))
    {
      std::cout << "FOUND for II=" << ii << std::endl;     
      break; // found
    }
  }
}

void HatScheT::ModuloSDCScheduler::setObjective()
{
  ScaLP::Term t;
  for(auto&p:variables)
  {
    t+=p.second;
  }
  this->solver->setObjective(ScaLP::minimize(t));
}
