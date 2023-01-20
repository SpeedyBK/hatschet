//
// Created by bkessler on 8/11/22.
//

//ToDo Will be replaced by SCC-Scheduler-Template...

#ifndef HATSCHET_SMTSCCSCHEDULER_H
#define HATSCHET_SMTSCCSCHEDULER_H

#pragma once
#ifdef USE_Z3

#include <iostream>
#include <memory>
#include <deque>

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include "HatScheT/utility/subgraphs/SCC.h"

namespace HatScheT {


  /*!
   * First Stage of SMT-Combined Scheduler. It is used to find schedules for complex SCCs which determine the minimum
   * feasible II. If it has found a schedule for the complex SCCs, then a heuristic Schedule is build around the SCCs Scheules
   */
  class SMTSCCScheduler : public SchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase {

  public:
    /*!
     * Determines which method is used to compute the max. Latency Constraint for the SCC-Scheduler.
     * Automatic: lets the SCC-Schduler search for Latency Bounds
     * fast: Expands the SCCs and adds 1 per complex SCC.
     * optimal: Expands the SCCs and adds II to this value.
     */
    enum class schedule_t {optimal, fast, automatic};
    /*!
     * Constructor for cases when no II is given and the Scheduler has to search for the optimal II
     * @param g Graph
     * @param resourceModel
     */
    SMTSCCScheduler(Graph &g, ResourceModel &resourceModel);
    /*!
     * Constructor for cases when II is given. The scheduler then searches a schedule for the given II
     * @param g
     * @param resourceModel
     * @param II
     */
    SMTSCCScheduler(Graph &g, ResourceModel &resourceModel, double II);
    /*!
     * Destructor, to delete some SCC-Pointers.
     */
    ~SMTSCCScheduler() override;
    /*!
     * Main scheduling Function.
     */
    void schedule() override;
    /*!
     * Method to set timeouts for the Z3-Solver
     * @param seconds
     */
    void setSolverTimeout(int seconds);
    /*!
     * Method to set the max latency constraint calculation mode.
     * @param schedulemode line 38
     */
    void setMode(schedule_t schedulemode);
    /*!
     * Getter for time budget
     * @return
     */
    int getTimeBudget() const { return timeBudget; }

  private:
    /*!
     * Priority value for SCCs
     */
    map<SCC*, int> inversePriority;
    /*!
     * Comperator to sort pairs of SCCs and INTs, to add them into a priority queue.
     */
    struct priocmp {
      bool operator() (const pair<SCC*,int> x, const pair<SCC*,int> y) const {
          if (x.second < y.second){
              return true;
          }
          if (x.second == y.second){
              return x.first < y.first;
          }
          return false;
      }
    };
    /*!
     * Function to partition a graph into SCCs. Creates a mapping between vertices and SCCs to look up to which SCC a
     * vertex belongs.
     */
    void computeSCCs();
    /*!
     * Function to determine the maximum latency of an SCC. Uses a SMT-Formulation to expand the SCCs as far as possible.
     * @param Graph gr is a graph build from the complex SCCs of a DFG.
     * @param rm Resourcemodel
     * @return the maximum latency a SCC can be expanded to.
     */
    int expandSCC(shared_ptr<Graph> &gr, shared_ptr<ResourceModel> &rm);
    /*!
     * Sets ressourced with more functional Unit than operation of this resource to unlimited.
     */
    void modifyResourceModel();
    /*!
     * Resets a previously modified RM back to original Values.
     */
    void resetResourceModel();
    /*!
     * Functions which combines all the relative schedules to a general heuristic Schedule. Optimal regarding II (if the are no timeouts)
     * but heuristic for latency.
     */
    void combineRelScheds();
    /*!
     * Time Limit given by the user.
     */
    int timeLimit;
    /*!
     * Internal time limit the Scheduler works with. Is updated when a schedule is found, so SMTCombined can work with it.
     */
    int timeBudget;
    /*!
     * Sorts SCCs by a topological order and creats a mapping between SCCs and their priority
     * @param tempsccs
     * @return
     */
    map<SCC*, int> computeTopologicalSCCOrder(vector<SCC*>&tempsccs);
    /*!
     * Maps each Vertex to the SCCs it belongs to.
     */
    map<Vertex*, SCC*> vertexToSCC;
    /*!
     * This function uses the an SDC Algorithm to search a schedule for the basic SCCs. This is possible since there are
     * only dependency constraints to satisfy. First a Graph for the given SCCs is Build and than Scheduled.
     * @param sc Basic SCCs
     * @return relative Schedule for basic SCCs
     */
    map<Vertex*, int> computeBasicSchedules(SCC* sc);
    /*!
     * Helperfunction for computeBasicSchedules which uses a shortest path algo to solve the SDC-System of basic SCCs
     * @param gr Graph of Basic SCCs
     * @param rm
     * @param sccVertexToVertex Mapping for Vertices in the SCCs Graph to the original Vertices.
     * @return relative Schedule for basic SCCs
     */
    map<Vertex*, int> sdcSchedule(std::shared_ptr<Graph>&gr, std::shared_ptr<ResourceModel>&rm, map<Vertex*, Vertex*>&sccVertexToVertex);
    /*!
     * Basic Realtive Schedules
     */
    map<SCC*, map<Vertex*, int>> basicRelSchedules;
    /*!
     * Build the complex Graph, uses expandSCC() to calc a max latency constraint and then uses SMT-Unary
     * to get a realtive schedule for the Complex SCC-Graphs.
     * @param complexSCCs
     */
    void computeComplexSchedule(deque<SCC*> &complexSCCs);
    /*!
     * Uses SMT Unary to Schedule complex SCCs.
     * @param gr Graph for the Complex SCCS
     * @param rm
     * @param sccVertexToVertex sccVertexToVertex Mapping for Vertices in the SCCs Graph to the original Vertices.
     * @return relative Schedule for cmplx SCCs
     */
    map<Vertex*, int> smtSchedule(std::shared_ptr<Graph>&gr, std::shared_ptr<ResourceModel>&rm, map<Vertex*, Vertex*>&sccVertexToVertex);
    /*!
     * Relative Schedules for complex SCCS
     */
    map<SCC*, map<Vertex*, int>> complexRelSchedules;
    /*!
     * Map to store original ResourceLimits of the Ressource Model is modified.
     */
    map<Resource*, int>resourceLimits;
    /*!
     * Others would say this is the MRT.
     */
    map<pair<Resource *, int>, int> usedFUsInModSlot;
    /*!
     * Topologicly Sorted SCCs
     */
    set<pair<SCC*,int>, priocmp> topoSortedSccs;
    /*!
     * Edges between SCCs
     */
    set<Edge*> connectingEdges;
    /*!
     * Boolean, II is given by a user, or the scheduler has to search for it.
     */
    bool iigiven;
    /*!
     * Instance of the Enum Class in line 38
     */
    schedule_t mode;
    /*!
     * Number of Complex SCCs.
     */
    int numOfCmplxSCCs;

  };

}

#endif //USE_Z3

#endif //HATSCHET_SMTSCCSCHEDULER_H
