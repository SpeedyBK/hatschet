//
// Created by sittel on 15/11/19.
//

#include "UnrollRationalIIScheduler.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/Verifier.h"
#include <cmath>

#include "HatScheT/scheduler/ilpbased/MoovacScheduler.h"
#include "HatScheT/scheduler/ilpbased/ModuloSDCScheduler.h"
#include "HatScheT/scheduler/ilpbased/EichenbergerDavidson97Scheduler.h"
#include "HatScheT/scheduler/ilpbased/SuchaHanzalek11Scheduler.h"
#include "HatScheT/scheduler/graphBased/PBScheduler.h"
#include "HatScheT/scheduler/dev/ModSDC.h"
#ifdef USE_CADICAL
#include "HatScheT/scheduler/dev/SATScheduler.h"
#endif

namespace HatScheT {

  UnrollRationalIIScheduler::UnrollRationalIIScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist)
  : RationalIISchedulerLayer(g, resourceModel), ILPSchedulerBase(solverWishlist), solverWishlist(solverWishlist),
    scheduler(ED97)
  {

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
          auto sampleIndexOffset = Utility::getSampleIndexAndOffset(distance,i,s,this->modulo);
          auto index = sampleIndexOffset.first;
          auto newDistance = sampleIndexOffset.second / this->modulo;
          new_g->createEdge(*v_src_mappings[index], *v_dst_mappings[i], newDistance, e->getDependencyType());


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
        this->startTimesVector[s][v] = round(schedUnrolled[vertexUnrolled]);
      }
    }
  }

	void UnrollRationalIIScheduler::scheduleIteration() {
    Graph g_unrolled;
    ResourceModel rm_unrolled;

    //unroll the input graph according to s and m
    this->unroll(g_unrolled, rm_unrolled, this->samples);

    HatScheT::SchedulerBase *schedulerBase;

    switch (this->scheduler) {
      case SchedulerType::MOOVAC:
        schedulerBase = new HatScheT::MoovacScheduler(g_unrolled,rm_unrolled, this->solverWishlist);
        if(this->solverTimeout > 0) ((HatScheT::MoovacScheduler*) schedulerBase)->setSolverTimeout(this->solverTimeout);
        if(this->maxLatencyConstraint > 0)
          ((HatScheT::MoovacScheduler*) schedulerBase)->setMaxLatencyConstraint(this->maxLatencyConstraint);
        ((HatScheT::MoovacScheduler*) schedulerBase)->setThreads(this->threads);
        ((HatScheT::MoovacScheduler*) schedulerBase)->setSolverQuiet(this->solverQuiet);
        ((HatScheT::MoovacScheduler*) schedulerBase)->setMaxRuns(1);
        break;
      case SchedulerType::MODULOSDC:
        schedulerBase = new HatScheT::ModSDC(g_unrolled,rm_unrolled, this->solverWishlist);
        if(this->solverTimeout > 0) ((HatScheT::ModSDC*) schedulerBase)->setSolverTimeout(this->solverTimeout);
        if(this->maxLatencyConstraint > 0)
          ((HatScheT::ModSDC*) schedulerBase)->setMaxLatencyConstraint(this->maxLatencyConstraint);
        ((HatScheT::ModSDC*) schedulerBase)->setThreads(this->threads);
        ((HatScheT::ModSDC*) schedulerBase)->setSolverQuiet(this->solverQuiet);
        ((HatScheT::ModSDC*) schedulerBase)->setMaxRuns(1);
        break;
      case SchedulerType::ED97:
        schedulerBase = new HatScheT::EichenbergerDavidson97Scheduler(g_unrolled,rm_unrolled, this->solverWishlist);
        if(this->solverTimeout > 0) ((HatScheT::EichenbergerDavidson97Scheduler*) schedulerBase)->setSolverTimeout(this->solverTimeout);
        if(this->maxLatencyConstraint > 0)
          ((HatScheT::EichenbergerDavidson97Scheduler*) schedulerBase)->setMaxLatencyConstraint(this->maxLatencyConstraint);
        ((HatScheT::EichenbergerDavidson97Scheduler*) schedulerBase)->setThreads(this->threads);
        ((HatScheT::EichenbergerDavidson97Scheduler*) schedulerBase)->setSolverQuiet(this->solverQuiet);
        ((HatScheT::EichenbergerDavidson97Scheduler*) schedulerBase)->setMaxRuns(1);
        break;
      case SchedulerType::SUCHAHANZALEK:
        schedulerBase = new HatScheT::SuchaHanzalek11Scheduler(g_unrolled,rm_unrolled, this->solverWishlist);
        if(this->solverTimeout > 0) ((HatScheT::SuchaHanzalek11Scheduler*) schedulerBase)->setSolverTimeout(this->solverTimeout);
        if(this->maxLatencyConstraint > 0)
          ((HatScheT::SuchaHanzalek11Scheduler*) schedulerBase)->setMaxLatencyConstraint(this->maxLatencyConstraint);
        ((HatScheT::SuchaHanzalek11Scheduler*) schedulerBase)->setThreads(this->threads);
        ((HatScheT::SuchaHanzalek11Scheduler*) schedulerBase)->setSolverQuiet(this->solverQuiet);
        ((HatScheT::SuchaHanzalek11Scheduler*) schedulerBase)->setMaxRuns(1);
        break;
    	case SchedulerType::PBS:
				schedulerBase = new HatScheT::PBScheduler(g_unrolled,rm_unrolled, this->solverWishlist);
				if(this->solverTimeout > 0) ((HatScheT::PBScheduler*) schedulerBase)->setSolverTimeout(this->solverTimeout);
				if(this->maxLatencyConstraint > 0)
					((HatScheT::PBScheduler*) schedulerBase)->setMaxLatencyConstraint(this->maxLatencyConstraint);
				((HatScheT::PBScheduler*) schedulerBase)->setThreads(this->threads);
				((HatScheT::PBScheduler*) schedulerBase)->setSolverQuiet(this->solverQuiet);
				((HatScheT::PBScheduler*) schedulerBase)->setMaxRuns(1);
				break;
    	case SchedulerType::SAT:
#ifdef USE_CADICAL
    	schedulerBase = new HatScheT::SATScheduler(g_unrolled, rm_unrolled);
				if(this->solverTimeout > 0) ((HatScheT::SATScheduler*) schedulerBase)->setSolverTimeout(this->solverTimeout);
				if(this->maxLatencyConstraint > 0)
					((HatScheT::SATScheduler*) schedulerBase)->setMaxLatencyConstraint(this->maxLatencyConstraint);
				((HatScheT::SATScheduler*) schedulerBase)->setMaxRuns(1);
#else
				throw Exception("UnrollRationalIIScheduler: CaDiCaL needed for SATScheduler");
#endif
    }

    schedulerBase->setQuiet(this->quiet);
    schedulerBase->schedule();

    switch(this->scheduler) {
      case SchedulerType::MOOVAC:
        this->stat = ((HatScheT::MoovacScheduler*) schedulerBase)->getScaLPStatus();
        this->solvingTime = ((HatScheT::MoovacScheduler*) schedulerBase)->getSolvingTime();
        break;
      case SchedulerType::MODULOSDC:
        this->stat = ((HatScheT::ModSDC*) schedulerBase)->getScaLPStatus();
        this->solvingTime = ((HatScheT::ModSDC*) schedulerBase)->getSolvingTime();
        break;
      case SchedulerType::ED97:
        this->stat = ((HatScheT::EichenbergerDavidson97Scheduler*) schedulerBase)->getScaLPStatus();
        this->solvingTime = ((HatScheT::EichenbergerDavidson97Scheduler*) schedulerBase)->getSolvingTime();
        break;
			case SchedulerType::SUCHAHANZALEK:
				this->stat = ((HatScheT::SuchaHanzalek11Scheduler*) schedulerBase)->getScaLPStatus();
				this->solvingTime = ((HatScheT::SuchaHanzalek11Scheduler*) schedulerBase)->getSolvingTime();
				break;
			case SchedulerType::PBS:
				this->stat = ((HatScheT::PBScheduler*) schedulerBase)->getScaLPStatus();
				this->solvingTime = ((HatScheT::PBScheduler*) schedulerBase)->getSolvingTime();
				break;
    }

    if(schedulerBase->getScheduleFound() == true) {
      this->scheduleFound = true;

      this->fillSolutionStructure(schedulerBase,&g_unrolled,&rm_unrolled);
    }

    delete schedulerBase;
	}
}