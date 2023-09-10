//
// Created by bkessler on 10/10/22.
//
#ifndef HATSCHET_SCCSCHEDULERTEMPLATE_H
#define HATSCHET_SCCSCHEDULERTEMPLATE_H

#if defined(USE_Z3) || defined(USE_SCALP)

#include <iostream>
#include <memory>
#include <deque>

#include <HatScheT/layers/IterativeModuloSchedulerLayer.h>
#include "HatScheT/utility/subgraphs/SCC.h"

namespace HatScheT {
  /*!
   * Template for 2-stage-schedulers. The scheduling problem is devided in 2 Parts. The first part is searching
   * schedules for complex SCCs. Which determines the minimum II a problem can be scheduled for. If a problem is
   * feasible, a heuristic schedule for the found II is saved. If this first scheduler does not time out, it will
   * alway find the optimal II, but latency of the saved schedule is heuristic.
   * The second stage scheduler trys to minimize the Latency of the found heuristic schedule.
   */
  class SCCSchedulerTemplate : public IterativeModuloSchedulerLayer {

  public:
    /*!
     * Enum class to switch the schedulers.
     */
    enum class scheduler {
      MOOVAC, ED97, SMT, SAT, SH11, MODSDC, SMTCDL, SDS, NONE
    };
    /*!
     * sccExpandMode:
     * - optimal: Expands the complex SCCs as far as possible and then adds one II to the found value.
     * - fast: Expands the complex SCCs as far as possible and then adds 1 for each SCCs to the found Value.
     *         Probaly not a correct method, but worked fine for all problems in MachSuite, CHStone and Origami.
     * - automatic: Just lets the SCC-Scheduler search for latency bounds.
     */
    enum class sccExpandMode {
      optimal, fast, automatic
    };

    /*!
     * Constructor, need the typical stuff a scheduler in Hatschet needs, as well as two enums to select the SCC-Scheduler
     * and the Latency(final)-Schduler
     * @param Graph g DFG of the scheduleproblem
     * @param resourceModel of the scheduleproblem
     * @param sccScheduler enum to select which moduloscheduler is used to schedule the complex SCCs.
     * @param finalScheduler enum to select which moduloscheduler is used to minimize latency.
     * @param II Initiation Interval
     */
    SCCSchedulerTemplate(Graph &g, ResourceModel &resourceModel, scheduler sccScheduler, scheduler finalScheduler,
                         double II = -1);

    /*!
     * Setter for the expand mode. (Line 30)
     */
    void setExpandMode(sccExpandMode expandMode);

    /*!
     * Sets the timeout value for the SCC-Scheduler. The latencyscheduler uses the time that is left when the
     * SCC-Scheduler is done.
     * @param seconds Timeout in seconds.
     */
    void setSolverTimeout(double seconds) override;

    /*!
     * Sets how many threads the solver in the backend can use.
     * @param t
     */
    void setThreads(int t) { this->threads = t; }

    /*!
     * Quite Obvious.
     * @return Name of the Scheduler
     */
    string getName() override { return "SCC-Scheduler"; }

  protected:
    /*!
     * Virtual function from IterativeModuloSchedulerLayer. Is called before the schedule loop starts. So everything
     * needed to init a scheduler should be done here. In this case:
     * - modifies the resourcemodel
     * - computes SCCs and sorts them by typ.
     * - build the graph of all complex SCCs.
     */
    void scheduleInit() override;

    /*!
     * Virtual function from IterativeModuloSchedulerLayer. Is called after the schedule loop ends. So everything
     * needed to cleanup after scheduling should be done here. In this case:
     * - clean up the resourcemodel
     * - set schedule length bound to original value
     */
    void scheduleCleanup() override;

    /*!
     * Is called in the scheduleloop. So everything which has to be done in an iteration of the scheduleloop should be
     * done here. In this case:
     * - Calculates Starttimes for complex SCCs.
     * - Schedules the complex SCC Graph and updates the MRT.
     * - If a schedule for complex SCCs is found, it builds a heuristic schedule around it.
     * - If a schedule is found and a latency-scheduler is selected, it trys to find an optimal Schedule.
     */
    void scheduleIteration() override;

  private:
    /*!
     * Function to cast shared pointers of a base class to shared pointers of derived classes to set specific settings
     * @param ssptr pointer of base class
     */
    void sharedPointerCastAndSetup(shared_ptr<IterativeModuloSchedulerLayer> &ssptr);

    /*!
     * Checks If SMT or ILP is availible. SMT is first choice.
     */
#if defined(USE_Z3)

    /*!
     * Calculates starttimes and expands the SCCs as far as posible. Uses z3...
     */
    void calcStarttimesPerComplexSCC();

#elif defined(USE_SCALP)
    /*!
     * Same as above but with ScaLP
     */
    void calcStarttimesPerComplexSCCScaLP();
#endif

    /*!
     * Sets ressourced with more functional Unit than operation of this resource to unlimited.
     */
    void modifyResourceModel();

    /*!
     * Resets a previously modified RM back to original Values.
     */
    void resetResourceModel();

    /*!
     * Function to partition a graph into SCCs. Creates a mapping between vertices and SCCs to look up to which SCC a
     * vertex belongs. Sorts SCCs topologically.
     */
    void computeSCCs();

    /*!
     * The name says almost everything. But it counts the complex SCCs as well.
     */
    void sortSCCsByType();

    /*!
     * This function takes all the complex SCCs and puts them in a complex supergraph. This is done since all complex
     * SCCs have to be scheduled at once to satisfy ressource constraints.
     */
    void buildComplexGraph();

    /*!
     * Function to schedule a complex supergraph.
     * @return a relative schedule for the complex supergraph.
     */
    map<Vertex *, int> scheduleComplexGraph();

    /*!
     * Updates the MRT after a schedule for the complex supergraph is found.
     */
    void finalizeComplexSchedule();

    /*!
     * Computes relative schedules for basic SCCs. Builds Graphs from SCCs, and uses sdcSchedule() to schedule them.
     * @param sc the SCCs which should be scheduled.
     * @return relative schedule of the SCC.
     */
    map<Vertex *, int> computeBasicSchedules(SCC *sc);

    /*!
     * Uses the Bellman-Ford based SDC-Alorithm from Utility::getSDCAsapAndAlapTimes to schedule the basic SCCs.
     * @param gr Graph of the SCCs
     * @param rm Resourcemodell
     * @param _sccVertexToVertex mapping from Vertices in SCC-Graph to vertices in the original graph.
     * @return relative schedule for the basic SCC.
     */
    map<Vertex *, int> sdcSchedule(std::shared_ptr<Graph> &gr, std::shared_ptr<ResourceModel> &rm,
                                   map<Vertex *, Vertex *> &_sccVertexToVertex);

    /*!
     * This function takes the relative schedules and puts them together to a heuristic general Schedule.
     */
    void combineRelativeSchedules();

    /*!
     * I think this one is replaced by calcStarttimesPerComplexSCC(), and is still a leftover from times long ago... ;)
     * I'll leave it in until i'm sure that it is safe to remove it.
     * @return max latency of complex SCCs.
     */
    int expandSCC();

    /*!
     * Function to select the scheduler. Can be used to select SCC-scheduler and latency-scheduler.
     * @param schedEnum tells the function which scheduler should be used. If "None" is selected, it returns a nullptr.
     * @param gr DFG of the problem.
     * @param rm Resourcemodell of the scheduleproblem.
     * @return a shared ptr of IterativeModuloSchedulerLayer, pointing to the selected scheduler.
     */
    shared_ptr<IterativeModuloSchedulerLayer> selectSccScheduler(scheduler schedEnum, Graph &gr, ResourceModel &rm);

    /*!
     * Sorts SCCs by topological order. And searches Edges, that connect SCCs. The are saved for combining the schedules later.
     * @param tempsccs vector of all SCCs in the DFG
     * @return returns an inverse priorty of the SCCs based on that topological order.
     */
    map<SCC *, int> computeTopologicalSCCOrder(vector<SCC *> &tempsccs);

    struct priocmp {
      bool operator()(const pair<SCC *, int> x, const pair<SCC *, int> y) const {
          if (x.second < y.second) {
              return true;
          }
          if (x.second == y.second) {
              return x.first < y.first;
          }
          return false;
      }
    };

    /*!
     * Set of topologically sorted SCCs, uses the priocmp above.
     */
    set<pair<SCC *, int>, priocmp> topoSortedSccs;
    /*!
     * Variable to store which scheduler should be used as SCC-Scheduler
     */
    scheduler sccScheduler;
    /*!
     * Variable to store which scheduler should be used as final-Scheduler
     */
    scheduler finalScheduler;
    /*!
     * Inverse priority of the topological SCC-Order.
     */
    map<SCC *, int> inversePriority;
    /*!
     * Mapping to which SCC a vertex belongs.
     */
    map<Vertex *, SCC *> vertexToSCC;
    /*!
     * Relativ schedules for the Basic SCCs
     */
    map<SCC *, map<Vertex *, int>> basicRelSchedules;
    /*!
     * Relativ schedules for the Complex SCCs
     */
    map<SCC *, map<Vertex *, int>> complexRelSchedules;
    /*!
     * Map to store original ResourceLimits of the Ressource Model is modified.
     */
    map<Resource *, int> resourceLimits;
    /*!
     * Others would say this is the MRT.
     */
    map<pair<Resource *, int>, int> usedFUsInModSlot;
    /*!
     * Edges that connect different SCCs.
     */
    set<Edge *> connectingEdges;
    /*!
     * Maximum Latency for all expanded SCCs.
     */
    set<int> maxTimesSCC;
    /*!
     * ToDo: Remove
     */
    map<Vertex *, int> earliestStarttimes;
    map<Vertex *, int> latestStarttimes;
    /*!
     * Containers for complex and basic SCCs.
     */
    deque<SCC *> complexSCCs;
    deque<SCC *> basicSCCs;
    /*!
     * Supergraph of complex SCCs. And ressourcemodel for complex SCCs.
     */
    shared_ptr<Graph> complexGraph;
    shared_ptr<ResourceModel> complexRm;
    /*!
     * Mappings between Vertices in SCC-Graphs to Vertices in the original Graph and reverse.
     */
    map<Vertex *, Vertex *> sccVertexToVertex;
    map<Vertex *, Vertex *> vertexToSccVertex;
    /*!
     * Schedule for the complex supergraph.
     */
    map<Vertex *, int> bigComplexSchedule;
    /*!
     * Line 30
     */
    sccExpandMode sccMode;
    int numOfCmplxSCCs;
    /*!
     * How many threads solvers can use.
     */
    int threads;

    /*!
     * if the backbone scheduler(s) should use the schedule length estimation
     */
    bool backboneSchedulerBoundSL;

  };
}
#else
/*! SCC - Template needs either Z3-SMT-Solver or ScaLP to run!' !*/
throw (HatScheT::Exception("Either Z3 or ScaLP is needed for this Scheduler"));
#endif

#endif //HATSCHET_SCCSCHEDULERTEMPLATE_H
