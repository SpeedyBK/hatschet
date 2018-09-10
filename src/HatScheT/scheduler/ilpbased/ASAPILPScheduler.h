//
// Created by Patrick Sittel on 10.09.18.
//


#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>

namespace HatScheT
{

class ASAPILPScheduler:  public SchedulerBase, public ILPSchedulerBase
{
public:
    ASAPILPScheduler(Graph& g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);
    virtual void schedule();

protected:
    virtual void setGeneralConstraints();
    virtual void constructProblem();
    virtual void setObjective();
    void setVectorVariables();

    vector<ScaLP::Variable> ti;
    vector<ScaLP::Variable> ri;
    map<const Vertex*, unsigned int> tIndices;
    map<const Vertex*, unsigned int> rIndices;
};

}



