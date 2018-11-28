//
// Created by nfiege on 22/11/18.
//

#ifndef HATSCHET_MODSDC_H
#define HATSCHET_MODSDC_H

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>

#include <vector>
#include <string>

namespace HatScheT
{
    class ModSDC : public SchedulerBase, public ILPSchedulerBase, public ModuloSchedulerBase
    {
    public:
        /*!
         * @brief ModSDC for scheduling in hatschet a graph (g), a resource model (rm) is needed
         * for solving the SDC problems, a solver wishlist (sw) to enable the ScaLP ILP backend is needed
         * (for example "Gurobi", "SCIP", "CPLEX")
         * @param g graph
         * @param rm resource model
         * @param sw solver wishlist
         */
        ModSDC(Graph& g, ResourceModel &rm, std::list<std::string> &sw);
        ~ModSDC() override;
        /*!
         * @brief schedule main method that runs the algorithm and determines a schedule
         */
        void schedule() override;
        /*!
         * the status of the ilp solver does not provide any information
         * about the quality of the solution, because many ilp problems are
         * solved for MRTs
         * @return the unknown status
         */
        ScaLP::status getScaLPStatus() override {return ScaLP::status::UNKNOWN;}
        /*!
         * @brief defines a new budget
         * @param newBudget
         */
        void setBudget(int &newBudget){this->budget = newBudget;}
        /*!
         * @brief getNumberOfConstrainedVertices
         * @param g graph which contains vertices
         * @param rm resource model that specifies for each vertex if it's limited
         * @return the number of resource constrained vertices
         */
        static int getNumberOfConstrainedVertices(Graph &g, ResourceModel &rm);
    private:
        /*!
         * @brief schedQueue from paper
         */
        std::list<Vertex*> schedQueue;
        /*!
         * @brief put instruction 'I' into this->schedQueue based in priority function in paper
         * @param I
         */
        void putIntoSchedQueue(Vertex* I);
        /*!
         * @brief map to store priority for scheduling queue
         */
        map<Vertex*, int> priorityForSchedQueue;
        /*!
         * @brief fill this->priorityForSchedQueue
         */
        void calculatePriorities();
        /*!
         * @brief one iteration of Modulo SDC algorithm
         * @param II the II to try and find a scheduling for
         * @param budget the number of max iterations to try for this II
         * @param initSchedule asap schedule without resource constraints as initiation for this algorithm
         * @return a pointer to a scheduling map; nullptr: no scheduling found for given II
         */
        bool modSDCIteration(const int &II, int budget);
        /*!
         * @brief creates scheduling queue based on initial asap scheduling
         * @param scheduleMe asap scheduling
         */
        void createSchedulingQueue(const std::map<Vertex*,int>& scheduleMe);
        /*!
         * @brief getInitialSchedule
         * @return an asap schedule without resource constraints
         */
        void createInitialSchedule();
        /*!
         * @brief initialize solver (i.e. reset etc.)
         */
        void initSolver();
        /*!
         * @brief constructProblem
         */
        void constructProblem() override;
        /*!
         * @brief setObjective
         */
        void setObjective() override;
        /*!
         * @brief this is needed for this->setObjective()
         */
        string uniqueVariableName;
        /*!
         * @brief find a name for this->uniqueVariableName
         */
        void setUniqueVariableName();
        /*!
         * @brief the number of iterations per II
         */
        int budget;
        /*!
         * @brief this sets a default budget according to metric in paper
         */
        void setDefaultBudget();
        /*!
         * @brief check if scheduling operation 'I' at time 't' has a resource conflict
         * @param I operation
         * @param t time
         * @return
         */
        bool hasResourceConflict(const Vertex* I, const int &t) const;
        /*!
         * @brief modulo reservation table; is only valid if modSDCIteration returned true
         */
        std::map<Vertex*, int> mrt;
        /*!
         * @brief equals SDC Times from paper
         */
        std::map<Vertex*, int> sdcTimes;
        /*!
         * @brief asap times for a schedule without resource constraints
         */
        std::map<Vertex*, int> asapTimes;
        /*!
         * @brief this map stores the scalp variables associated to each vertex inside the graph
         */
        std::map<Vertex*, ScaLP::Variable> scalpVariables;
        /*!
         * @brief search this->scalpVariables for sv and return the associated key
         * @param sv
         * @return
         */
        Vertex* getVertexFromVariable(const ScaLP::Variable& sv);
        /*!
         * @brief this fills scalpVariables according to input graph
         */
        void createScalpVariables();
        /*!
         * @brief this creates ILP constraints based on data dependency (end_i - start_j <= II * distance(i,j))
         */
        void createDataDependencyConstraints();
        /*!
         * @brief this creates all additional constraints stored in this->additionalConstraints
         */
        void createAdditionalConstraints();
        /*!
         * @brief additional constraints based on lines 7 and 10 (ModSDC algorithm)
         */
        std::map<Vertex*,ScaLP::Constraint*> additionalConstraints;
        /*!
         * @brief this basically just catches std::out_of_range errors when accessing additionalConstraints.at(v) to check if a constraint for the given vertex exists
         * @return the scalp constraint associated to vertex v
         */
        ScaLP::Constraint* getAdditionalConstraint(Vertex* v);
        /*!
         * @brief creates or overwrites an additional constraint (c) for a given vertexptr (v)
         * @param v
         * @param c
         */
        void createAdditionalConstraint(Vertex* v, ScaLP::Constraint& c);
        /*!
         * @brief this clears the constraints associated to the given vertexptr
         * @param v
         */
        void clearConstraintForVertex(Vertex* v);
        /*!
         * @brief this clears all additional constraints in this->additionalConstraints
         */
        void clearAllAdditionalConstraints();
        /*!
         * @brief this sets up scalp for the current II
         */
        void setUpScalp();
        /*!
         * @brief this does everything needed to solves the problem specified in this->solver
         * and stores results in this->sdcTimes
         * @return if the solver found a solution (ScaLP::status::FEASIBLE or ScaLP::status::OPTIMAL)
         */
        bool solveSDCProblem();
        /*!
         * @brief previously scheduled times (see paper for details)
         */
        std::map<Vertex*, int> prevSched;
        /*!
         * @brief get previously scheduled time for a given vertex; needed for backtracking
         * @param v
         * @return -1 if no prevSched has no entry for the given vertex
         */
        int getPrevSched(Vertex* v);
        /*!
         * @brief Backtracking algorithm from paper
         * @param I instruction
         * @param time time slot
         */
        void backtracking(Vertex* I, const int &time);
        /*!
         * @brief checks if a resource conflict exists when scheduling instruction 'I' at time slot 'evictTime'
         * @param I instruction
         * @param evictTime time slot
         * @return true if a conflict exists
         */
        bool resourceConflict(Vertex* I, const int &evictTime);
        /*!
         * @param t time slot
         * @return all vertices scheduled to the given time slot 't'
         */
        std::list<Vertex*> getVerticesAtTimeSlot(const int &t);
        /*!
         * @param I Instruction
         * @param evictTime
         * @return all instructions scheduled to the time slot evictTime mod this->II
         */
        std::list<Vertex*> getResourceConflicts(Vertex* I, const int &evictTime);
        /*!
         * @brief remove instruction 'evictInst' from this->mrt, clear additional constraints for this vertex and add to schedule queue
         * @param evictInst
         */
        void unscheduleInstruction(Vertex* evictInst);
        /*!
         * @param I instruction
         * @param evictTime
         * @return true if scheduling instruction 'I' at time 'evictTime' creates data dependency conflict
         */
        bool dependencyConflict(Vertex* I, const int &evictTime);
        /*!
         * @param i start of an edge (j is the destination, that doesn't have to be passed)
         * @paragm newStartTime_i new start time of instruction i
         * @paragm newStartTime_j new start time of instruction j
         * @param edgeDelay delay on edge between operation i and j
         * @param distance on edge between operation i and j
         * @return end_i - start_j <= II * distance(i,j)
         */
        bool dependencyConflictForTwoInstructions(Vertex* i, const int &newStartTime_i, const int &newStartTime_j, const int& edgeDelay, const int& edgeDistance);
        /*!
         * @brief fill map with start times for unconstrained instructions
         * after finding a valid schedule for all resource constrained ones
         */
        void fillStartTimesWithUnconstrainedInstructions();

    };
}

#endif //HATSCHET_MODSDC_H
