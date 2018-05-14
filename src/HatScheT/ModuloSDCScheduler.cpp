
#include <HatScheT/ModuloSDCScheduler.h>
#include <HatScheT/Graph.h>

#include <memory>
#include <map>
#include <vector>
#include <algorithm>

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
};

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
  std::cout << "Limit for resource conflict: " << r->getLimit() << std::endl;
  std::cout << "Vertex: " << v->getName() << std::endl;
  std::cout << "Time: " << time << std::endl;
  if(r->getLimit()>=0)
  {
    int count=0;
    for(auto&p:data[r][time]) if(p!=nullptr) ++count;
    std::cout << "Resource count at time " << time << ": " << count << std::endl;
    bool b = std::any_of(data[r][time].begin(), data[r][time].end(), [](HatScheT::Vertex* a){return a==nullptr;});
    for(unsigned int i=0;i<=time;++i)
    {
      for(Vertex*p:data[r][i])
      {
        if(p==nullptr or p==v) continue;
        auto o = vs.find(p);
        if(o!=vs.end())
        {
          std::cout << "AAAAAA:" << std::to_string(i) << "  " << time << "   " << r->getLatency() << std::endl;
          if(i<time-r->getLatency())
          {
            std::cout << "EEE: " << (*o)->getName() << " conflicts with " << v->getName() << std::endl;
            b=false;
            break;
          }


        }
      }
      if(b) break;

    }
    if(b) std::cout << "No Resource-Conflict" << std::endl;
    else  std::cout << "Resource-Conflict" << std::endl;
    return not b;
  }
  else
  {
    std::cout << "Resource count at time " << time << ": no limit" << std::endl;
    return false;
  }
}

bool HatScheT::MRT::update(HatScheT::Vertex* i, int time)
{
  time%=II;
  std::cout << "Update MRT: Add " << i->getName() << " to slot " << time << std::endl;
  auto r = rm->getResource(i);
  auto& v = data.at(r).at(time);
  auto it = std::find(v.begin(),v.end(),nullptr);
  if(it!=v.end())
  {
    *it=i;
      std::cout << "add " << i->getName() << " to the mrt for " << r->getName() << " at " << time   << std::endl;
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
  for(auto& v:data[r])
  {
    for(auto&a:v)
    {
      if(a==i) a=nullptr;
    }
  }
  std::cout << "MRT: " << i->getName() << " is removed." << std::endl;
  return true;;
}

HatScheT::ModuloSDCScheduler::ModuloSDCScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist, unsigned int maxII)
  : SchedulerBase(g,resourceModel)
  , ILPSchedulerBase(solverWishlist)
  , mrt({resourceModel,1})
  , minII(1)
  , maxII(maxII)
{

}

HatScheT::Vertex* HatScheT::MRT::getInstructionAt(unsigned int i, unsigned int t)
{
  std::cout << "getInstruction at " << std::to_string(i) << ": ";// << std::endl;
  for(auto&p:data)
  {
    if(p.second.size()>=i and p.second[i].size()>=0)
    {
      for(auto&o:p.second[i])
      {
        if(o!=nullptr and this->rm->getResource(o)->getLatency()>=t)
        {
          std::cout << o->getName() << std::endl;
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
  this->solver->reset();

  if(this->threads>0) this->solver->threads = this->threads;
  this->solver->quiet=this->solverQuiet;
  this->solver->timeout=this->solverTimeout;

  for(auto c:this->baseConstraints) *this->solver << c;
  *this->solver << c;

  setObjective();

  return feasible(this->solver->solve());
}

bool HatScheT::ModuloSDCScheduler::dependencyConflict(std::map<Vertex*,int>& prevSched, Vertex* I, int time)
{
  for(auto it=this->g.edgesBegin(); it!=this->g.edgesEnd(); ++it)
  {
    Vertex* d = &(*it)->getVertexDst();
    if(I == d)
    {
      Vertex* s = &(*it)->getVertexSrc();
      auto r = this->resourceModel.getResource(s);
      if(prevSched[s]+r->getLatency()>=time)
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
    Vertex* a = &(*it)->getVertexSrc();
    Vertex* b = &(*it)->getVertexDst();
    ScaLP::Variable va = getVariable(variables,a);
    ScaLP::Variable vb = getVariable(variables,b);
    auto vaLatency = this->resourceModel.getResource(a)->getLatency();
    std::cout << "II: " << II << std::endl;
    std::cout << "distance: " <<  (*it)->getDistance() << std::endl;
    this->baseConstraints.emplace_back(va + vaLatency - vb <= II * (*it)->getDistance() );
  }
  
}

static std::unique_ptr<std::map<HatScheT::Vertex*,int>> solveLP(ScaLP::Solver& s, HatScheT::Graph& g, std::map<HatScheT::Vertex*,ScaLP::Variable>& variables, std::vector<ScaLP::Constraint>& cons, std::vector<ScaLP::Constraint>& bcons)
{
  for(auto& c:bcons) s << c;
  for(auto& c:cons) s << c;
  s.writeLP("eee.lp");
  auto rrr = s.solve();
  std::cout << rrr << std::endl;
  if(feasible(rrr/*s.solve()*/))
  {
    std::unique_ptr<std::map<HatScheT::Vertex*,int>> m(new std::map<HatScheT::Vertex*,int>());
    auto result = s.getResult();

    for(auto it = g.verticesBegin(); it!=g.verticesEnd();++it)
    {
      auto it2 = result.values.find(getVariable(variables,*it));
      if(it2!=result.values.end())
      {
        m->emplace(*it,static_cast<int>(it2->second));
      }
    }
    return m;
  }
  else
  {
    // TODO: Error, should not happen.
    return nullptr;
  }
}

static bool solveFeasible(ScaLP::Solver& s)
{
  auto r = s.solve();
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  std::cout << s.getResult() << std::endl;
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  return feasible(r);
}

static void createVariables(std::map<HatScheT::Vertex*,ScaLP::Variable>& variables,HatScheT::Graph& g)
{
  for(auto it = g.verticesBegin(); it!=g.verticesEnd();++it)
  { // TODO: remove upper bound
    ScaLP::Variable t_i = ScaLP::newIntegerVariable((*it)->getName(),0,10000);
    variables.emplace(*it,t_i);
  }
}

void addLayerRec(HatScheT::Graph& g, std::map<HatScheT::Vertex*,unsigned int>& res, HatScheT::Vertex* v, HatScheT::Vertex* c)
{
  for(auto e = g.edgesBegin(); e!=g.edgesEnd();++e)
  {
    auto edgeSrc = &(*e)->getVertexSrc();
    auto edgeDst = &(*e)->getVertexDst();
    if(c == edgeSrc and v!=edgeDst and (*e)->getDistance()==0)
    { // successor without loop
      res[v]+=1;
      addLayerRec(g,res,v,edgeDst);
    }
  }

};

static std::map<HatScheT::Vertex*,unsigned int> createPerturbation(HatScheT::Graph& g)
{
  std::map<HatScheT::Vertex*,unsigned int> res;
  for(auto v = g.verticesBegin(); v!=g.verticesEnd();++v)
  {
    res.emplace(*v,0);
  }
  for(auto v = g.verticesBegin(); v!=g.verticesEnd();++v)
  {
    addLayerRec(g,res,*v,*v); // TODO: FIXME: wrong
  }

  for(auto&p:res)
  {
    std::cout << "Perturbation: " << p.first->getName() << " = " << std::to_string(p.second) << std::endl;
  }

  return res;
}

bool HatScheT::ModuloSDCScheduler::sched(int II, int budget)
{
  this->mrt = MRT(this->resourceModel,II);
  this->solver->reset();
  this->constraints.clear();
  this->neverScheduled.clear();

  setObjective();
  std::unique_ptr<std::map<Vertex*,int>> asap = solveLP(*(this->solver),this->g,this->variables,this->constraints,this->baseConstraints);
  if(not asap)
  {
    std::cerr << "Can't find ASAP-Schedule (current II is too small (?))" << std::endl;
    return false;
  }
  std::map<Vertex*,int> prevSched;

  std::map<Vertex*,unsigned int> priority; //= createPerturbation(this->g);
  for(auto&p:*asap)
  {
    if(p.first->getName()=="b") priority.emplace(p.first,3);
    if(p.first->getName()=="a") priority.emplace(p.first,2);
    if(p.first->getName()=="d") priority.emplace(p.first,1);
    if(p.first->getName()=="c") priority.emplace(p.first,0);
  }
  Queue schedQueue([&priority](Vertex* a,Vertex* b){return priority[a]<priority[b];});

  for(std::list<Resource*>::iterator it=this->resourceModel.resourcesBegin();it!=this->resourceModel.resourcesEnd();++it)
  {
    if((*it)->getLimit()<=0) continue;
    auto vs = this->resourceModel.getVerticesOfResource(*it);
    for(const HatScheT::Vertex* v:vs) schedQueue.push(const_cast<Vertex*>(v));
  }

  int b = budget;
  for(; not schedQueue.empty() and b>=0; --b)
  {
    std::cout << "#### Begin of Iteration " << (budget-b+1) << std::endl;
    auto i = schedQueue.top();
    schedQueue.pop();

    std::cout << "Current Instruction: " << i->getName() << std::endl;

    int asapI=0;
    {
      auto it = asap->find(i);
      if(it!=asap->end()) asapI = it->second;
      else asapI = 0;
    }

    int time = 0;
    auto it = prevSched.find(i);
    if(it==prevSched.end())
    {
      time = asapI;
      std::cout << "use asap-time: " << time << std::endl;
    }
    else
    {
      std::cout << "use scheduled-time: " << time << std::endl;
      time = it->second;
    }
    ScaLP::Variable t_i = getVariable(variables,i);

    printMRT(mrt);
    if(not mrt.resourceConflict(i,time))
    {
      std::cout << "Add " << t_i << " == " << time << std::endl;
      constraints.push_back(t_i == time);
      mrt.update(i,time);
      prevSched[i]=time;
    }
    else
    {
      std::cout << "Add " << t_i << " >= " << (time+1) << std::endl;
      constraints.push_back(t_i >= time+1);
      this->constructProblem();
      this->setObjective();

      auto res = solveLP(*this->solver,this->g,this->variables,this->constraints, this->baseConstraints);
      if(res!=nullptr)
      {
        std::cout << "Put back " << i->getName() << std::endl;
        schedQueue.push(i);
        //prevSched[i]=time+1;
        prevSched = std::move(*(res.release()));

      }
      else
      {
        std::cout << "backtrack because of " << i->getName() << std::endl;
        constraints.pop_back(); // remove >= Constraint
        backtracking(schedQueue, prevSched,i,asapI,time,II);
        this->constructProblem();
        this->setObjective();
        auto a =  solveLP(*this->solver,this->g,this->variables,this->constraints, this->baseConstraints);/* prevSched= ?*/
        if(a!=nullptr) prevSched = std::move(*(a.release()));
      }

      if(this->writeLPFile) this->solver->writeLP(to_string(this->II));
    }
    std::cout << "#### End of Iteration\n\n" << std::endl;
  }

  std::cout << "Result: " << std::endl;
  for(auto&p:prevSched)
  {
    std::cout << p.first->getName() << " = " << p.second << std::endl;
  }

  std::cout << "Final ";
  printMRT(mrt);

  if(schedQueue.empty())
  {
    startTimes=prevSched;
    std::cout << "success" << std::endl;
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
  std::cout << "begin backtracking(" << I->getName() << " , " << time << ")" << std::endl;
  int minTime=0;
  int evictTime=0;
  for(minTime = asapTime;minTime<=time;++minTime)
  {
    //this->constructProblem();
    if(solveBasicWithConstraint(getVariable(variables,I)==minTime)) break;
  }

  std::cout << "minTime:"  << minTime << std::endl;

  auto it = prevSched.find(I);
  if(neverScheduled[I] or minTime >= it->second)
  {
    std::cout << "use minTime " << minTime<< std::endl;
    evictTime = minTime;
  }
  else
  {
    std::cout << "use prevSched: " << (it->second+1) << std::endl;
    evictTime = it->second+1;
  }

  std::cout << "BACKTRACKING" << std::endl;
  if(mrt.resourceConflict(I,evictTime))
  {
    // all Instructions that overlap with I
    std::set<Vertex*> evictInsts;
    for(unsigned int i=0;evictTime%II>=i;++i)
    {
      std::cout << "Check slot " << std::to_string(i) << std::endl;
      // get a instruction that overlaps with I
      Vertex* evictInst = mrt.getInstructionAt((evictTime%II)-1,i);
      if(evictInst!=nullptr)
      {
        evictInsts.insert(evictInst);
      }
    }
    if(not evictInsts.empty())
    {
      for(Vertex*evictInst:evictInsts)
      {
        std::cout << "Resource conflict in backtracking for: " << evictInst->getName() << std::endl;
        removeAllConstraintsOf(constraints,getVariable(variables,evictInst));
        mrt.remove(evictInst);
      }
    }
    else
    {
      std::cout << "No instruction found (this should never happen)" << std::endl;
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

  std::cout << "Check for dependency conflict at slot: " << (evictTime%II) << std::endl;
  if(dependencyConflict(prevSched,I,evictTime))
  {
    std::cout << "Dependency conflict." << std::endl;
    for(auto& p:prevSched)
    {
      removeAllConstraintsOf(constraints,getVariable(variables,p.first));
      mrt.remove(p.first);
      schedQueue.push(p.first);
    }
  }
  else
  {
    std::cout << "No dependency conflict" << std::endl;
  }

  std::cout << "add constraint: " << I->getName() << " == " << std::to_string(evictTime)  << std::endl;
  constraints.push_back(getVariable(variables,I)==evictTime);
  mrt.update(I,evictTime);
  prevSched[I]=evictTime;

  std::cout << "end backtracking" << std::endl;
}

void HatScheT::ModuloSDCScheduler::schedule()
{
  this->timeoutCounter = 0;
  this->totalTime = 0;
  //this->II = this->minII;
  this->variables.clear();
  createVariables(variables,g);

  for(unsigned int ii=1/*minII*/;ii<=maxII;++ii)
  {
    baseConstraints.clear();
    createBaseConstraints(ii);
    if(sched(ii,6))
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
