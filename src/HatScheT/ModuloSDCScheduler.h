#pragma once

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>

#include <queue>
#include <functional>
#include <vector>

namespace HatScheT
{
  using Queue = std::priority_queue<Vertex*,std::vector<Vertex*>,std::function<bool(Vertex*,Vertex*)>>;
  /*!
   * \brief The MRT class
   */
  class MRT
  {
    public:
      MRT(HatScheT::ResourceModel& r, int II);

      bool resourceConflict(HatScheT::Vertex* i,int time);
      bool update(HatScheT::Vertex* i, int time);
      bool remove(HatScheT::Vertex* i);
      HatScheT::Vertex* getInstructionAt(unsigned int i, const Resource *r);
      bool vertexIsIn(Vertex* v);

      std::map<const HatScheT::Resource*,std::vector<std::vector<HatScheT::Vertex*>>> data;
      HatScheT::ResourceModel* rm;

      int II=0;
  };

  class ModuloSDCScheduler :public SchedulerBase, public ILPSchedulerBase, public ModuloSchedulerBase
  {
    public:
    ModuloSDCScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
    virtual void schedule();
    virtual int getII(){return this->II;}
    virtual void constructProblem();
    virtual void setObjective();

    private:
    std::vector<ScaLP::Constraint> baseConstraints;
    std::vector<ScaLP::Constraint> constraints;
    std::map<Vertex*,ScaLP::Variable> variables;
    std::map<Vertex*,bool> neverScheduled;
    MRT mrt;
    bool sched(int II, int budget);
    void backtracking(Queue& schedQueue, std::map<Vertex*,int>& prevsched, HatScheT::Vertex* I, int asapI, int time, int II);
    bool resourceConflict(HatScheT::Vertex* I, int time);
    bool dependencyConflict(std::map<Vertex*,int>& prevsched, HatScheT::Vertex* I, int time);
    void createBaseConstraints(int II);
    bool solveBasicWithConstraint(ScaLP::Constraint&& c);
  };

}
