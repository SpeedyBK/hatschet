//
// Created by sittel on 15/11/19.
//

#include "UnrollRationalIIScheduler.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/Verifier.h"
#include "math.h"

#include "HatScheT/scheduler/ilpbased/MoovacScheduler.h"
#include "HatScheT/scheduler/ilpbased/ModuloSDCScheduler.h"
#include "HatScheT/scheduler/ilpbased/EichenbergerDavidson97Scheduler.h"
#include "HatScheT/scheduler/dev/ModSDC.h"

namespace HatScheT {

  UnrollRationalIIScheduler::UnrollRationalIIScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist)
  : RationalIISchedulerLayer(g, resourceModel), ILPSchedulerBase(solverWishlist) {

    this->s_start = -1;
    this->m_start = -1;
    this->solverWishlist = solverWishlist;

    this->scheduleFound = false;
    this->scheduler = ED97;

    this->computeMinII(&g,&resourceModel);
    this->minII = ceil(this->minII);
    this->computeMaxII(&g,&resourceModel);
    if (this->minII >= this->maxII) this->maxII = this->minII+1;

  }

  void UnrollRationalIIScheduler::schedule(){
    this->scheduleFound = false;

    //check if start values ar set, if not auto set to minRatII
    if(this->m_start == -1 or this->s_start == -1) {
      throw Exception("UnrollRationalIIScheduler::schedule: S or M not set, not supported yet!");
    }

    int attempts = 1;

    while(attempts <= this->maxRuns and this->scheduleFound == false) {
      Graph g_unrolled;
      ResourceModel rm_unrolled;

      //unroll the input graph according to s and m
      this->unroll(g_unrolled, rm_unrolled, this->s_start);

      HatScheT::SchedulerBase *scheduler;

      switch (this->scheduler) {
        case SchedulerType::MOOVAC:
          scheduler = new HatScheT::MoovacScheduler(g_unrolled,rm_unrolled, this->solverWishlist);
          if(this->solverTimeout > 0) ((HatScheT::MoovacScheduler*) scheduler)->setSolverTimeout(this->solverTimeout);
          if(this->maxLatencyConstraint > 0)
            ((HatScheT::MoovacScheduler*) scheduler)->setMaxLatencyConstraint(this->maxLatencyConstraint);
          ((HatScheT::MoovacScheduler*) scheduler)->setThreads(this->threads);
          ((HatScheT::MoovacScheduler*) scheduler)->setSolverQuiet(this->solverQuiet);
          ((HatScheT::MoovacScheduler*) scheduler)->setMaxRuns(1);
          break;
         case SchedulerType::MODULOSDC:
          scheduler = new HatScheT::ModSDC(g_unrolled,rm_unrolled, this->solverWishlist);
          if(this->solverTimeout > 0) ((HatScheT::ModSDC*) scheduler)->setSolverTimeout(this->solverTimeout);
          if(this->maxLatencyConstraint > 0)
            ((HatScheT::ModSDC*) scheduler)->setMaxLatencyConstraint(this->maxLatencyConstraint);
          ((HatScheT::ModSDC*) scheduler)->setThreads(this->threads);
          ((HatScheT::ModSDC*) scheduler)->setSolverQuiet(this->solverQuiet);
          ((HatScheT::ModSDC*) scheduler)->setMaxRuns(1);
          break;
        case SchedulerType::ED97:
          scheduler = new HatScheT::EichenbergerDavidson97Scheduler(g_unrolled,rm_unrolled, this->solverWishlist);
          if(this->solverTimeout > 0) ((HatScheT::EichenbergerDavidson97Scheduler*) scheduler)->setSolverTimeout(this->solverTimeout);
          if(this->maxLatencyConstraint > 0)
            ((HatScheT::EichenbergerDavidson97Scheduler*) scheduler)->setMaxLatencyConstraint(this->maxLatencyConstraint);
          ((HatScheT::EichenbergerDavidson97Scheduler*) scheduler)->setThreads(this->threads);
          ((HatScheT::EichenbergerDavidson97Scheduler*) scheduler)->setSolverQuiet(this->solverQuiet);
          ((HatScheT::EichenbergerDavidson97Scheduler*) scheduler)->setMaxRuns(1);
          break;
      }

      scheduler->schedule();

      switch(this->scheduler) {
        case SchedulerType::MOOVAC:
          this->stat = ((HatScheT::MoovacScheduler*) scheduler)->getScaLPStatus();
          this->solvingTime = ((HatScheT::MoovacScheduler*) scheduler)->getSolvingTime();
          break;
        case SchedulerType::MODULOSDC:
          this->stat = ((HatScheT::ModSDC*) scheduler)->getScaLPStatus();
          this->solvingTime = ((HatScheT::ModSDC*) scheduler)->getSolvingTime();
          break;
        case SchedulerType::ED97:
          this->stat = ((HatScheT::EichenbergerDavidson97Scheduler*) scheduler)->getScaLPStatus();
          this->solvingTime = ((HatScheT::EichenbergerDavidson97Scheduler*) scheduler)->getSolvingTime();
          break;
      }

      if(scheduler->getScheduleFound() == true) {
        this->II = scheduler->getII();

        this->scheduleFound = true;

        //TODO change this when iteration is implemented!!!
        this->modulo = this->m_start;
        this->samples = this->s_start;

        this->fillSolutionStructure(scheduler,&g_unrolled,&rm_unrolled);
      }

      delete scheduler;
      attempts++;
    }

  }

  void UnrollRationalIIScheduler::unroll(Graph& g_unrolled, ResourceModel& rm_unrolled, int s) {
    Graph *new_g = &g_unrolled;
    ResourceModel *new_rm = &rm_unrolled;

    map<Vertex *, vector<Vertex *> > mappings;

    for (auto it = this->g.verticesBegin(); it != this->g.verticesEnd(); ++it) {
      HatScheT::Vertex *v = *it;
      const HatScheT::Resource *r = this->resourceModel.getResource(v);

      Resource *r_new;

      vector<Vertex *> v_mapping;

      if (new_rm->resourceExists(r->getName()) == true) {
        r_new = new_rm->getResource(r->getName());
      } else {
        r_new = &new_rm->makeResource(r->getName(), r->getLimit(), r->getLatency(), r->getBlockingTime());
      }

      for (int i = 0; i < s; i++) {
        Vertex *v_new = &new_g->createVertex();
        v_new->setName(v->getName() + "_" + to_string(i));
        new_rm->registerVertex(v_new, r_new);
        v_mapping.push_back(v_new);
      }

      mappings.insert(make_pair(v, v_mapping));
    }

    for (auto it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it) {
      Edge *e = *it;
      Vertex *v_src = &e->getVertexSrc();
      Vertex *v_dst = &e->getVertexDst();

      vector<Vertex *> v_src_mappings = mappings[v_src];
      vector<Vertex *> v_dst_mappings = mappings[v_dst];

      for (int i = 0; i < v_src_mappings.size(); i++) {
        if (e->getDistance() == 0)
          new_g->createEdge(*v_src_mappings[i], *v_dst_mappings[i], 0, e->getDependencyType());
        else {
          int distance = e->getDistance();

          if (distance > i) {
            int new_distance = distance - i;
            int new_port = new_distance % s;
            new_distance = ceil((double) new_distance / (double) distance);

            new_g->createEdge(*v_src_mappings[new_port], *v_dst_mappings[i], new_distance, e->getDependencyType());
          } else {
            new_g->createEdge(*v_src_mappings[i - distance], *v_dst_mappings[i], 0, e->getDependencyType());
          }
        }
      }
    }
  }

  void UnrollRationalIIScheduler::fillSolutionStructure(SchedulerBase* scheduler, Graph* g_unrolled, ResourceModel* rm_unrolled) {
    auto schedUnrolled =  scheduler->getSchedule();

    // create a map for each sample insertion
    this->startTimesVector.clear();
    for(int s=0; s<this->samples; ++s) {
      this->startTimesVector.emplace_back(std::map<Vertex*, int>());
    }

    // names of vertices in unrolled graph correspond to vertices in original graph as
    // "*original_name*_0" ... "*original_name*_N" where N = #samples-1
    for(auto v : this->g.Vertices()) {
      for(int s=0; s<this->samples; ++s) {
        auto vertexName = v->getName()+"_"+to_string(s);
        Vertex* vertexUnrolled = nullptr;

        // find vertex in unrolled graph - there is NO function getVertexByName :(
        for(auto v2 : g_unrolled->Vertices()) {
          if(v2->getName() == vertexName) {
            vertexUnrolled = v2;
            break;
          }
        }

        // insert schedule time into startTimesVector
        this->startTimesVector[s][v] = schedUnrolled[vertexUnrolled];
      }
    }

    // verify schedules
    // verify "normal" modulo schedule with unrolled graph and resource model
    bool verify1 = HatScheT::verifyModuloSchedule(*g_unrolled,*rm_unrolled,schedUnrolled,this->modulo);
    // verify rational II modulo schedule with original graph and resource model
    bool verify2 = HatScheT::verifyRationalIIModuloSchedule(this->g,this->resourceModel,this->startTimesVector,this->samples,this->modulo);
    if(verify1 != verify2) {
      std::cout << "UnrollRationalIIScheduler::fillSolutionStructure: verifier for integer II modulo schedule != verifier for rational II modulo schedule -> one of them is buggy!" << std::endl;
    }
    if(!verify1) {
      if(!this->quiet) {
        std::cout << "UnrollRationalIIScheduler::fillSolutionStructure: Attention! Invalid schedule detected!" << std::endl;
      }
    }
  }
}