
#include <HatScheT/ModuloSDCScheduler.h>
#include <HatScheT/Graph.h>

#include <memory>
#include <map>
#include <vector>
#include <algorithm>

HatScheT::MRT::MRT(HatScheT::ResourceModel& r, int II)
: rm(&r)
{
  for(auto it=r.resourcesBegin(); it != r.resourcesEnd(); ++it)
  { // TODO: remove magic numbers (8)
    data.emplace(*it,std::vector<std::vector<HatScheT::Vertex*>>(8));
    for(int i=1;i<=II;++i)
      data.at(*it).emplace_back(std::vector<HatScheT::Vertex*>(8));
  }
};

bool HatScheT::MRT::resourceConflict(HatScheT::Vertex* i,int time)
{
  auto r = rm->getResource(i);
  std::cout << "Limit for resource conflict: " << r->getLimit() << std::endl;
  std::cout << "Vertex: " << i->getName() << std::endl;
  std::cout << "Time: " << time << std::endl;
  if(not data[r].empty() and r->getLimit()>=0)
    std::cout << "Resource count at time " << time << ": " << data[r][time].size() << std::endl;
  else
    std::cout << "Resource count at time " << time << ": no limit" << std::endl;
  bool tmp = not data[r].empty() and r->getLimit()>0 and data[r][time].size() >= r->getLimit();
  if(tmp)
    std::cout << "Resource-Conflict " << std::endl;
  else
    std::cout << "No Resource-Conflict" << std::endl;
  return tmp; //not data[r].empty() and data[r][time].size() >= r->getLimit();
}

bool HatScheT::MRT::update(HatScheT::Vertex* i, int time)
{
  std::cout << "Update MRT: Add " << i->getName() << " to slot " << time << std::endl;
  auto r = rm->getResource(i);
  data.at(r).at(time).push_back(i);
  //data[r][time].push_back(i);
  std::cout << "add " << i->getName() << " to the mrt for " << r->getName() << " at " << time   << std::endl;

  return data[r][time].size() <= r->getLimit();
}

bool HatScheT::MRT::remove(HatScheT::Vertex* i)
{
  auto r = rm->getResource(i);
  for(auto& v:data[r])
  {
    auto fun = [&i](const HatScheT::Vertex* a)
    {
      return a==i;
    };
    v.erase(std::remove_if(v.begin(),v.end(),fun),v.end());
  }
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

HatScheT::Vertex* HatScheT::MRT::getInstructionAt(unsigned int i)
{
  std::cout << "getInstruction" << std::endl;
  for(auto&p:data)
  {
    if(p.second.size()>=i and p.second[i].size()>0)
    {
      std::cout << "GOT: " << p.second[i][p.second[i].size()-1]->getName() << std::endl;
      return p.second[i][p.second[i].size()-1];
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

bool HatScheT::ModuloSDCScheduler::solveBasicWithConstraint(ScaLP::Constraint&& c)
{
  this->solver->reset();

  if(this->threads>0) this->solver->threads = this->threads;
  this->solver->quiet=this->solverQuiet;
  this->solver->timeout=this->solverTimeout;

  for(auto c:this->baseConstraints) *this->solver << c;
  *this->solver << c;

  return true;
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
      if(r->getLimit()>0 and prevSched[s]+r->getLatency()>=time)
      {
        //auto lat = r->getLatency();
        //if(prevSched[s]+lat>=time)
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

static bool feasible(ScaLP::status stat)
{
  return stat == ScaLP::status::OPTIMAL
    or   stat == ScaLP::status::FEASIBLE
    or   stat == ScaLP::status::TIMEOUT_FEASIBLE;
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
  //return feasible(s.solve());
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
    std::cout << "PPPPPPPPP: " << p.first->getName() << " = " << std::to_string(p.second) << std::endl;
  }

  return res;
}

bool HatScheT::ModuloSDCScheduler::sched(int II, int budget)
{
  this->solver->reset();
  this->constraints.clear();

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

  // // show the queue
  //std::cout << "Queue:" << std::endl;
  //while(not schedQueue.empty())
  //{
  //  std::cout << schedQueue.top()->getName() << std::endl;
  //  schedQueue.pop();
  //}
  // return false;

  for(int b=budget; not schedQueue.empty() and b>=0; --b)
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

    if(not mrt.resourceConflict(i,time%II))
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

      auto res = solveLP(*this->solver,this->g,this->variables,this->constraints, this->baseConstraints);/* prevSched= ?*/
      if(res!=nullptr)
      //if(solveFeasible(*this->solver))
      {
        std::cout << "Put back " << i->getName() << std::endl;
        schedQueue.push(i);
        //prevSched[i]=time+1;
        prevSched = std::move(*(res.release()));

      }
      else
      {
        std::cout << "backtrack because of " << i->getName() << std::endl;
        constraints.pop_back();
        backtracking(schedQueue, prevSched,i,asapI,time,II);
        this->constructProblem();
        auto a =  solveLP(*this->solver,this->g,this->variables,this->constraints, this->baseConstraints);/* prevSched= ?*/
        if(a!=nullptr) prevSched = std::move(*(a.release()));
      }

      if(this->writeLPFile) this->solver->writeLP(to_string(this->II));
    }
    std::cout << "#### End of Iteration" << std::endl;
  }

  std::cout << "Result: " << std::endl;
  for(auto&p:prevSched)
  {
    std::cout << p.first->getName() << " = " << p.second << std::endl;
  }

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

void HatScheT::ModuloSDCScheduler::backtracking(Queue& schedQueue, std::map<Vertex*,int>& prevSched, HatScheT::Vertex* I, int asapI, int time, int II)
{

  int minTime=0;
  int evictTime=0;
  for(minTime = asapI;minTime<=time;++minTime)
  {
    //this->constructProblem();
    if(solveBasicWithConstraint(getVariable(variables,I)==minTime)) break;
  }


  auto it = prevSched.find(I);
  if(it==prevSched.end() or minTime >= it->second)
  {
    evictTime = minTime;
  }
  else
  {
    evictTime = it->second+1;
  }

  std::cout << "BACKTRACKING" << std::endl;
  if(mrt.resourceConflict(I,evictTime%II))
  {
    Vertex* evictInst = mrt.getInstructionAt(evictTime % II);
    std::cout << "Resource conflict in backtracking for: " << evictInst->getName() << std::endl;
    removeAllConstraintsOf(constraints,getVariable(variables,evictInst));
    mrt.remove(evictInst);
  }

  if(dependencyConflict(prevSched,I,evictTime))
  {
    for(auto& p:prevSched)
    {
      removeAllConstraintsOf(constraints,getVariable(variables,p.first));
      mrt.remove(p.first);
      schedQueue.push(p.first);
    }
  }

  constraints.push_back(getVariable(variables,I)==evictTime);
  mrt.update(I,evictTime);
  prevSched[I]=evictTime;

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
