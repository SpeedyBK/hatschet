/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Thomas Schönwälder, Patrick Sittel (thomas.schoenwaelder@student.uni-kassel.de, sittel@uni-kassel.de)

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
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/scheduler/ilpbased/ModuloSDCScheduler.h>
#include <HatScheT/Graph.h>
#include <HatScheT/utility/Verifier.h>
#include <ctime>
#include <map>
#include <vector>
#include <algorithm>
#include <ctime>
#include <numeric>
#include <chrono>
#include <math.h>

HatScheT::MRT::MRT(HatScheT::ResourceModel& r, int ii)
: rm(&r), II(ii)
{
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
  if(r->getLimit()<=0) {
    cout << "Warning: tried to remove a vertex of unlimited resource from MRT : " << i->getName() << "! This should never happen!" << endl;
    return true;
  }

  for(auto& v:data[r])
  {
    for(auto&a:v)
    {
      if(a==i){
        a=nullptr;
        return true;
      }
    }
  }

  return false;
}

HatScheT::ModuloSDCScheduler::ModuloSDCScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist)
  : SchedulerBase(g,resourceModel)
  , ILPSchedulerBase(solverWishlist)
  , mrt({resourceModel,1})
{
  this->quiet = false;
  this->userdef_budget = -1;
  this->timesOutOfBudget = 0;
  this->computeMinII(&g,&resourceModel);
  this->minII = ceil(this->minII);
  this->computeMaxII(&g, &resourceModel);
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
        if(it3==nullptr) continue;
        if(it3==v) {
          printMRT(*this);
          return true;
        }
      }
    }
  }

  return false;
}

HatScheT::Vertex* HatScheT::MRT::getInstructionAt(unsigned int i, const Resource* r)
{
  for(auto&p:data)
  {
    if(p.second.size()>=i && p.first==r)
    {
      for(auto&o:p.second[i])
      {
        if(o!=nullptr)
        {
          return o;
        }
      }
    }
    else
    {
      continue;
    }
  }

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
  for(Edge*e:this->g.Edges())
  {
    Vertex* d = &e->getVertexDst();
    if(I == d)
    {
      Vertex* s = &e->getVertexSrc();
      auto r = this->resourceModel.getResource(s);
      if(prevSched[s]+r->getLatency()+e->getDelay()>time+this->II*e->getDistance())
      {
        return true;
      }
    }
  }

  for(Edge*e:this->g.Edges())
  {
    Vertex* d = &e->getVertexSrc();
    if(I == d)
    {
      Vertex* s = &e->getVertexDst();
      auto r = this->resourceModel.getResource(d);
      if(time+r->getLatency()+e->getDelay()>prevSched[s]+this->II*e->getDistance())
      {
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
  for(Edge*e:this->g.Edges())
  {
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
  auto rrr = s.solve();
  std::map<HatScheT::Vertex*,int> m;

  if(feasible(rrr))
  {
    auto result = s.getResult();

    for(HatScheT::Vertex*v:g.Vertices())
    {
      auto it2 = result.values.find(getVariable(variables,v));
      if(it2!=result.values.end())
      {
        m.emplace(v,static_cast<int>(it2->second));
      }
    }
  }
  return m;
}

static void createVariables(std::map<HatScheT::Vertex*,ScaLP::Variable>& variables,HatScheT::Graph& g)
{
  for(HatScheT::Vertex*v:g.Vertices())
  {
    ScaLP::Variable t_i = ScaLP::newIntegerVariable(v->getName(),0,10000);
    variables.emplace(v,t_i);
  }
}

static unsigned int addLayerRec(HatScheT::Graph& g, std::map<HatScheT::Vertex*,unsigned int>& res, HatScheT::Vertex* v, HatScheT::Vertex* c)
{
  unsigned int count=0;
  for(HatScheT::Edge*e:g.Edges())
  {
    if(e->getDistance()!=0) continue;
    auto edgeSrc = &e->getVertexSrc();
    auto edgeDst = &e->getVertexDst();

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

  for(HatScheT::Vertex*v:g.Vertices())
  {
    int prio = std::abs(asap.getScheduleLength() - asap.getStartTime(*v));
    res.emplace(v,prio);
  }
  return res;
}

static std::map<HatScheT::Vertex*,unsigned int> createPerturbation(HatScheT::Graph& g)
{
  std::map<HatScheT::Vertex*,unsigned int> res;
  for(HatScheT::Vertex*v:g.Vertices())
  {
    res.emplace(v,addLayerRec(g,res,v,v));
  }

  return res;
}

bool HatScheT::ModuloSDCScheduler::sched(int budget, const std::map<HatScheT::Vertex*,unsigned int>& priority,  const std::map<HatScheT::Vertex*,int>& asap)
{
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
    if(secondsRun>this->solver->timeout) return false;

    auto i = schedQueue.top();
    schedQueue.pop();
    if(this->quiet==true) std::cout << "#### Begin of Iteration " << (budget-b+1) << " at II " << this->II << " at time " << std::ctime(&time_var_it) << " with " << i->getName() << std::endl;
    if(this->quiet==true) std::cout << "Elapsed run time is " << secondsRun << " (sec) with timeout " << this->solverTimeout << " (sec)" << std::endl;

    if(this->resourceModel.getResource(i)->getLimit()==-1) continue;

    if(this->quiet==true) std::cout << "Current Instruction: " << i->getName() << std::endl;
    if(this->quiet==true) cout << "budget b : " << b << endl;

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
      if(this->quiet==true) std::cout << "use asap-time: " << time << " for " << i->getName() << std::endl;
    }
    else
    {     
      time = it->second;
      if(this->quiet==true) std::cout << "use scheduled-time: " << time << " for " << i->getName() << std::endl;
    }
    ScaLP::Variable t_i = getVariable(variables,i);

    if(this->quiet==true) printMRT(mrt);
    if(mrt.resourceConflict(i,time)==false)
    {
      if(mrt.update(i,time)==true){
        if(this->quiet==true) std::cout << "Add (no resource conflict) " << t_i << " == " << time << std::endl;
        constraints.push_back(t_i == time);
        prevSched[i]=time;
        this->neverScheduled[i]=false;
      }
    }
    else
    {
      if(this->quiet==true) std::cout << "Add (conflict detected) " << t_i << " >= " << (time+1) << std::endl;
      this->constraints.push_back(t_i >= time+1);
      this->constructProblem();

      auto res = solveLP(*this->solver,this->g,this->variables,this->constraints, this->baseConstraints);

      if(not res.empty())
      {
        if(this->quiet==true) std::cout << "Put back " << i->getName() << std::endl;
        schedQueue.push(i);
        prevSched.swap(res);
      }
      else
      {
        if(this->quiet==true) std::cout << "backtrack because of " << i->getName() << std::endl;
        this->constraints.pop_back(); // remove >= Constraint
        backtracking(schedQueue, prevSched,i,asapI,time,II);
        this->constructProblem();

        auto a =  solveLP(*this->solver,this->g,this->variables,this->constraints, this->baseConstraints);

        if(not a.empty()) prevSched.swap(a);
      }

      if(this->writeLPFile) this->solver->writeLP(to_string(this->II));
    }

    if(this->quiet==true) std::cout << "#### End of Iteration\n\n" << std::endl;

    if(b==0  and schedQueue.empty()==false) this->timesOutOfBudget++;
  }

  if(this->quiet==true) std::cout << "Result for II " << this->II << std::endl;

  for(auto&p:prevSched)
  {
    if(this->quiet==true) std::cout << p.first->getName() << " = " << p.second << std::endl;
  }

  if(this->quiet==true) std::cout << "Final ";
  if(this->quiet==true) printMRT(mrt);

  if(schedQueue.empty() && verifyModuloSchedule(this->g,this->resourceModel,prevSched,this->II)==true)
  {
    startTimes=prevSched;
    if(this->quiet==true) std::cout << "success" << std::endl;
    if(verifyModuloSchedule(this->g,this->resourceModel,prevSched,this->II)==false)
      cout << "ERROR (not detected) wrong prevSched stored as solution" << endl;
    return true;
  }
  else
  {
    if(this->quiet==true) std::cout << "No success" << std::endl;
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
  if(this->quiet==true) std::cout << "begin backtracking(" << I->getName() << " , " << time << ")" << std::endl;
  int minTime=0;
  int evictTime=0;
  for(minTime = asapTime;minTime<=time;++minTime)
  {
    if(solveBasicWithConstraint(getVariable(variables,I)==minTime)) break;
  }

  if(this->quiet==true) std::cout << "minTime:"  << minTime << std::endl;

  auto it = prevSched.find(I);
  if(neverScheduled[I] or minTime >= it->second)
  {
    if(this->quiet==true) std::cout << "use minTime for evict " << minTime<< std::endl;
    evictTime = minTime;
  }
  else
  {
    if(this->quiet==true) std::cout << "use prevSched+1 for ecivt : " << (it->second+1) << std::endl;
    evictTime = it->second+1;
  }

  if(this->quiet==true) std::cout << "BACKTRACKING at evict time " << evictTime << std::endl;
  if(mrt.resourceConflict(I,evictTime))
  {
    if(this->quiet==true) cout << "resource conflict detected of " << I->getName() << " at " << evictTime << endl;
    // all Instructions that overlap with I
    Vertex* evictInst = mrt.getInstructionAt((evictTime%II)/*-1*/,this->resourceModel.getResource(I));
    if(evictInst!=nullptr)
    {
      if(this->quiet==true) std::cout << "Resource conflict in backtracking for: " << evictInst->getName() << std::endl;
      if(this->quiet==true) cout << "Removing all constraints of " << evictInst->getName() << endl;
      removeAllConstraintsOf(constraints,getVariable(variables,evictInst));
      if(mrt.remove(evictInst)==true)
      {
        schedQueue.emplace(evictInst);
        if(this->quiet==true)cout << evictInst->getName() << " put back to sched queue" << endl;
      }
    }
    else
    {
      if(this->quiet==true) std::cout << "No instruction found (this should never happen)" << std::endl;
    }
  }

  if(this->quiet==true) std::cout << "Check for dependency conflict at slot: " << (evictTime%II) << std::endl;
  if(dependencyConflict(prevSched,I,evictTime))
  {
    if(this->quiet==true) std::cout << "Dependency conflict." << std::endl;
    for(auto& p:prevSched)
    {
      const Resource* rp = this->resourceModel.getResource(p.first);
      //continue for unlimited resource
      if(rp->getLimit()<=0) continue;
      removeAllConstraintsOf(constraints,getVariable(variables,p.first));
      if(this->quiet==true) cout << "Removing from mrt : " << p.first->getName() << endl;

      if(mrt.remove(p.first)==true){
      schedQueue.emplace(p.first);
      if(mrt.vertexIsIn(p.first)==true){
       cout << "Warning: vertex should have been removed but is still in mrt " << p.first->getName() << endl;
       if(this->quiet==true) printMRT(mrt);
      }
      if(this->quiet==true) cout << "put back " << p.first->getName() << endl;
      }
    }
  }
  else
  {
    if(this->quiet==true) std::cout << "No dependency conflict" << std::endl;
  }


  if(mrt.update(I,evictTime)==true){
    if(this->quiet==true) std::cout << "add constraint: " << I->getName() << " == " << std::to_string(evictTime)  << std::endl;
    constraints.push_back(getVariable(variables,I)==evictTime);
    prevSched[I]=evictTime;
    this->neverScheduled[I]=false;
  }

  else{
    cout << "Warning: could not add to mrt at end of backtracking: " << I->getName() << "! This should never happen!" << endl;
  }

  if(this->quiet==true) std::cout << "end backtracking" << std::endl;
}

void HatScheT::ModuloSDCScheduler::schedule()
{
  this->timeouts = 0;
  this->totalTime = 0;
  this->variables.clear();
  createVariables(variables,g);
  int budget = 0;
  int b_fac = 6;

  if(this->userdef_budget < 0) {
    budget = b_fac*this->g.getNumberOfVertices();
    std::cout << "ModuloSDCScheduler: automatic set budget = " << budget << "(" << b_fac << " * " << budget << ")" << std::endl;
  }
  else{
    budget = this->userdef_budget;
    std::cout << "ModuloSDCScheduler: user defined budget = " << budget << std::endl;
  }

  //set maxRuns, e.g., maxII - minII, iff value if not -1
  if(this->maxRuns > 0){
    int runs = this->maxII - this->minII;
    if(runs > this->maxRuns) this->maxII = this->minII + this->maxRuns;
    std::cout << "ModuloSDCScheduler: maxII changed due to maxRuns value set by user!" << endl;
    std::cout << "ModuloSDCScheduler: min/maxII = " << minII << " " << maxII << std::endl;
  }

  std::map<Vertex*,unsigned int> priority;

  for(this->II=this->minII;this->II<=this->maxII;++this->II)
  {
    // cleanup and preparations
    this->solver->reset();
    this->constraints.clear();
    this->baseConstraints.clear();
    this->neverScheduled.clear();
    this->solver->timeout = this->solverTimeout;
    createBaseConstraints(this->II);
    this->mrt = MRT(this->resourceModel,this->II);

    // create asap-times
    setObjective();
    const std::map<Vertex*,int> asap = solveLP(*(this->solver),this->g,this->variables,this->constraints,this->baseConstraints);

    if(asap.empty()){
      std::cerr << "Can't find ASAP-Schedule (current II is too small (?))" << '\n'; // std::endl;
      continue;
    }   

    // create perturbation
    priority.clear();
    int max =0; 
    for(auto&p:asap)
    {   
      max = std::max(max,p.second);
    }   
    for(auto&p:asap)
    {
      int prio = std::abs(max - p.second);
      priority.emplace(p.first,prio);
    }

    cout << "Starting new iteration of ModuloSDC for II " << this->II << " with timeout " << this->solver->timeout << "(sec)" << endl;

    //timestamp
    this->begin = clock();
    //solve
    bool attempt = sched(budget,priority,asap);
    //timestamp
    this->end = clock();
    //log time
    if(this->solvingTime == -1.0) this->solvingTime = 0.0;
    this->solvingTime += (double)(this->end - this->begin)  / CLOCKS_PER_SEC;

    if(attempt==true)
    {
      scheduleFound=true;
      std::cout << "FOUND for II=" << this->II << " after " << this->solvingTime << " seconds" << std::endl;
      break; // found
    }
    else this->timeouts++;
  }
  if(scheduleFound==false) this->II = -1;
  std::cout << "ModuloSDCScheduler: total times out of budget = " << this->timesOutOfBudget << std::endl;
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
