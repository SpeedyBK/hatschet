//
// Created by sittel on 15/11/19.
//

#include "UnrollRationalIIScheduler.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/Verifier.h"
#include <cmath>

#include "HatScheT/scheduler/ilpbased/MoovacScheduler.h"
#include "HatScheT/scheduler/ilpbased/EichenbergerDavidson97Scheduler.h"
#include "HatScheT/scheduler/ilpbased/SuchaHanzalek11Scheduler.h"
#include "HatScheT/scheduler/graphBased/PBScheduler.h"
#include "HatScheT/scheduler/ilpbased/ModSDC.h"
#ifdef USE_CADICAL
#include "HatScheT/scheduler/satbased/SATScheduler.h"
#include "HatScheT/scheduler/satbased/SATCombinedScheduler.h"
#endif

namespace HatScheT {

  UnrollRationalIIScheduler::UnrollRationalIIScheduler(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist, int M, int S)
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
    // fill start times container with start times for the first sample
    this->startTimes = startTimesVector[0];
  }

	void UnrollRationalIIScheduler::scheduleIteration() {
    Graph g_unrolled;
    ResourceModel rm_unrolled;

    //unroll the input graph according to s and m
    std::map<Vertex*, std::vector<Vertex*>> vertexMappings;
    if (!this->quiet) {
    	std::cout << "UnrollRationalIIScheduler: start unrolling graph by a factor of '" << this->samples << "' now" << std::endl;
    }
    Utility::unroll(&g_unrolled, &rm_unrolled, this->samples, this->modulo, &this->g, &this->resourceModel, &vertexMappings, this->quiet);
    //this->unroll(g_unrolled, rm_unrolled, this->samples);
		if (!this->quiet) {
			std::cout << "UnrollRationalIIScheduler: finished unrolling - start scheduling now" << std::endl;
		}

    HatScheT::SchedulerBase *schedulerBase;

    switch (this->scheduler) {
      case SchedulerType::MOOVAC:
        schedulerBase = new HatScheT::MoovacScheduler(g_unrolled,rm_unrolled, this->solverWishlist, this->modulo);
        if(this->solverTimeout > 0) ((HatScheT::MoovacScheduler*) schedulerBase)->setSolverTimeout(this->solverTimeout);
        if(this->maxLatencyConstraint > 0)
          ((HatScheT::MoovacScheduler*) schedulerBase)->setMaxLatencyConstraint(this->maxLatencyConstraint);
        ((HatScheT::MoovacScheduler*) schedulerBase)->setThreads(this->threads);
        ((HatScheT::MoovacScheduler*) schedulerBase)->setSolverQuiet(this->solverQuiet);
        ((HatScheT::MoovacScheduler*) schedulerBase)->setMaxRuns(1);
        break;
      case SchedulerType::MODULOSDC:
        schedulerBase = new HatScheT::ModSDC(g_unrolled,rm_unrolled, this->solverWishlist, this->modulo);
        if(this->solverTimeout > 0) ((HatScheT::ModSDC*) schedulerBase)->setSolverTimeout(this->solverTimeout);
        if(this->maxLatencyConstraint > 0)
          ((HatScheT::ModSDC*) schedulerBase)->setMaxLatencyConstraint(this->maxLatencyConstraint);
        ((HatScheT::ModSDC*) schedulerBase)->setThreads(this->threads);
        ((HatScheT::ModSDC*) schedulerBase)->setSolverQuiet(this->solverQuiet);
        ((HatScheT::ModSDC*) schedulerBase)->setMaxRuns(1);
        break;
      case SchedulerType::ED97:
        schedulerBase = new HatScheT::EichenbergerDavidson97Scheduler(g_unrolled,rm_unrolled, this->solverWishlist, this->modulo);
        if(this->solverTimeout > 0) ((HatScheT::EichenbergerDavidson97Scheduler*) schedulerBase)->setSolverTimeout(this->solverTimeout);
        if(this->maxLatencyConstraint > 0)
          ((HatScheT::EichenbergerDavidson97Scheduler*) schedulerBase)->setMaxLatencyConstraint(this->maxLatencyConstraint);
        ((HatScheT::EichenbergerDavidson97Scheduler*) schedulerBase)->setThreads(this->threads);
        ((HatScheT::EichenbergerDavidson97Scheduler*) schedulerBase)->setSolverQuiet(this->solverQuiet);
        ((HatScheT::EichenbergerDavidson97Scheduler*) schedulerBase)->setMaxRuns(1);
        break;
      case SchedulerType::SUCHAHANZALEK:
        schedulerBase = new HatScheT::SuchaHanzalek11Scheduler(g_unrolled,rm_unrolled, this->solverWishlist, this->modulo);
        if(this->solverTimeout > 0) ((HatScheT::SuchaHanzalek11Scheduler*) schedulerBase)->setSolverTimeout(this->solverTimeout);
        if(this->maxLatencyConstraint > 0)
          ((HatScheT::SuchaHanzalek11Scheduler*) schedulerBase)->setMaxLatencyConstraint(this->maxLatencyConstraint);
        ((HatScheT::SuchaHanzalek11Scheduler*) schedulerBase)->setThreads(this->threads);
        ((HatScheT::SuchaHanzalek11Scheduler*) schedulerBase)->setSolverQuiet(this->solverQuiet);
        ((HatScheT::SuchaHanzalek11Scheduler*) schedulerBase)->setMaxRuns(1);
        break;
    	case SchedulerType::PBS:
				schedulerBase = new HatScheT::PBScheduler(g_unrolled,rm_unrolled, this->solverWishlist, this->modulo);
				if(this->solverTimeout > 0) ((HatScheT::PBScheduler*) schedulerBase)->setSolverTimeout(this->solverTimeout);
				if(this->maxLatencyConstraint > 0)
					((HatScheT::PBScheduler*) schedulerBase)->setMaxLatencyConstraint(this->maxLatencyConstraint);
				((HatScheT::PBScheduler*) schedulerBase)->setThreads(this->threads);
				((HatScheT::PBScheduler*) schedulerBase)->setSolverQuiet(this->solverQuiet);
				((HatScheT::PBScheduler*) schedulerBase)->setMaxRuns(1);
				break;
    	case SchedulerType::SAT:
#ifdef USE_CADICAL
    	schedulerBase = new HatScheT::SATScheduler(g_unrolled, rm_unrolled, this->modulo);
				if(this->solverTimeout > 0) ((HatScheT::SATScheduler*) schedulerBase)->setSolverTimeout(this->solverTimeout);
				if(this->maxLatencyConstraint > 0)
					((HatScheT::SATScheduler*) schedulerBase)->setMaxLatencyConstraint(this->maxLatencyConstraint);
				((HatScheT::SATScheduler*) schedulerBase)->setMaxRuns(1);
				((HatScheT::SATScheduler*) schedulerBase)->setLatencyOptimizationStrategy(SATScheduler::LatencyOptimizationStrategy::REVERSE_LINEAR);
#else
				throw Exception("UnrollRationalIIScheduler: CaDiCaL needed for SATScheduler");
#endif
			case SchedulerType::SATCOMBINED:
#ifdef USE_CADICAL
				schedulerBase = new HatScheT::SATCombinedScheduler(g_unrolled, rm_unrolled, this->modulo);
				if(this->solverTimeout > 0) ((HatScheT::SATCombinedScheduler*) schedulerBase)->setSolverTimeout(this->solverTimeout);
				if(this->maxLatencyConstraint > 0)
					((HatScheT::SATCombinedScheduler*) schedulerBase)->setMaxLatencyConstraint(this->maxLatencyConstraint);
				((HatScheT::SATCombinedScheduler*) schedulerBase)->setMaxRuns(1);
#else
				throw Exception("UnrollRationalIIScheduler: CaDiCaL needed for SATCombinedScheduler");
#endif
    }

    schedulerBase->setQuiet(this->quiet);
    schedulerBase->schedule();

    auto moduloSchedulerBase = (HatScheT::ModuloSchedulerBase*) schedulerBase;
    std::pair<bool, bool> objectivesOptimal = {false, false};
    if (moduloSchedulerBase != nullptr) {
    	objectivesOptimal = moduloSchedulerBase->getObjectivesOptimal();
    }
    this->secondObjectiveOptimal = objectivesOptimal.second;

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
    	case SchedulerType::SAT:
    		if (this->scheduleFound) {
					if (this->secondObjectiveOptimal) {
						this->stat = ScaLP::status::OPTIMAL;
					}
					else {
						this->stat = ScaLP::status::TIMEOUT_FEASIBLE;
					}
    		}
    		else {
    			if (objectivesOptimal.first) {
						this->stat = ScaLP::status::INFEASIBLE;
    			}
    			else {
						this->stat = ScaLP::status::TIMEOUT_INFEASIBLE;
    			}
    		}
#ifdef USE_CADICAL
				this->solvingTime = ((HatScheT::SATScheduler*) schedulerBase)->getSolvingTime();
#else
				throw Exception("UnrollRationalIIScheduler: CaDiCaL needed for SATScheduler");
#endif
    		break;
			case SchedulerType::SATCOMBINED:
				if (this->scheduleFound) {
					if (this->secondObjectiveOptimal) {
						this->stat = ScaLP::status::OPTIMAL;
					}
					else {
						this->stat = ScaLP::status::TIMEOUT_FEASIBLE;
					}
				}
				else {
					if (objectivesOptimal.first) {
						this->stat = ScaLP::status::INFEASIBLE;
					}
					else {
						this->stat = ScaLP::status::TIMEOUT_INFEASIBLE;
					}
				}
#ifdef USE_CADICAL
				this->solvingTime = ((HatScheT::SATCombinedScheduler*) schedulerBase)->getSolvingTime();
#else
				throw Exception("UnrollRationalIIScheduler: CaDiCaL needed for SATCombinedScheduler");
#endif
				break;
    }

    // track optimality of first objective (i.e., II)
    if (this->stat == ScaLP::status::TIMEOUT_INFEASIBLE) {
    	this->firstObjectiveOptimal = false;
    }
		// track optimality of second objective (i.e., schedule length)
    if (this->stat == ScaLP::status::OPTIMAL) {
    	this->secondObjectiveOptimal = true;
    }
    else {
    	this->secondObjectiveOptimal = false;
    }

    if(schedulerBase->getScheduleFound()) {
      this->scheduleFound = true;

      this->fillSolutionStructure(schedulerBase,&g_unrolled,&rm_unrolled);
    }

    delete schedulerBase;
	}
}