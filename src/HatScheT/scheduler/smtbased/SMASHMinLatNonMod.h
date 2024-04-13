//
// Created by Benjamin on 19.03.24.
//

#ifndef HATSCHET_SMASHMINLATNONMOD_H
#define HATSCHET_SMASHMINLATNONMOD_H

#if USE_Z3

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/Z3SchedulerBase.h>

namespace HatScheT {

    class SMASHMinLatNonMod : public SchedulerBase, public Z3SchedulerBase {

    public:

        SMASHMinLatNonMod(Graph& g, ResourceModel& rm);

        /*!
         * use this methode to solve the scheduling problem
         */
        void schedule() override;

        /*!
         * Overrides methode from schedulerbase-class and sets a timeout for Z3.
         * @param seconds Timeout in seconds.
         */
        void setSolverTimeout(double seconds) override;

        /*!
         * This function ist used to set a searchmode for the non-modulo-min-lat schdeduler.
         * It takes a string to determine the searchmode. RECOMENDATION: "ASAPHC"
         * Currenty two modes are implemented:
         * - "Auto":    Uses an iterative process to enable the scheduler to find it's latency bounds by itself. Basicly it
         *              solves the dependency constraints, sets up resource constraints up to the length of the schedule from
         *              the dependency constraints. Then it checks the resulting model. Usually the length of the schedule
         *              will increase so it sets up resource constraints up to this new length... This process repeats until
         *              the length of the schedule does not increase anymore.
         * - "ASAPHC":  Uses the heuristic ASAPHC-Schedule to determine the length to which resource constraints must be set.
         *              The length of the heuristic ASAPHC-Schedule is greater or equal to the length of an optimal non-mod
         *              schedule. So no iterative process is needed.
         * - "Manual":  Allowed the user to set an upper bound. (Not implemented yet!)
         * @param sreachMode
         */
        void setSearchMode(string &sreachMode) { mode = sreachMode; };

        /*!
         * A getter to get the information, if an optimal schedule is found. The scheduler guaranties an optimal schedule
         * unless Z3 times out at one point.
         * @return information if schedule is optimal.
         */
        [[nodiscard]] bool isLatencyOptimal() const { return latencyOptimal; };

    private:

      /*****************************
       * Main Scheduling Functions *
       *****************************/
        /*!
         * Main schedule-function if "Auto" is selected. Probably buggy, since in a few examples the schedule length
         * increased to infinity. Main reason to leave it in is for documenting. "ASAPHC" is the recommended schedule mode!
         */
        void scheduleAutoSearch();
        /*!
         * Main schedule-function if "ASAPHC" is selected. Recommended mode!
         */
        void scheduleASAPHCSearch();

      /***************
       * T-Variables *
       ***************/
        /*!
         * T-Variables are Z3-Integer-Expressions which contain the information in which timestep an operation is started.
         */
        map<Vertex*, z3::expr> tVariables;
        /*!
         * Function to generate one T-Variable for each vertex and save it in the tVariables-map.
         */
        void generateTVariables();
        /*!
         * Getter for T-Variables, takes a vertex ptr and returns the corresponding T-Variable.
         * @param vPtr
         * @return Pointer to the T-Variable corresponding to vPtr.
         */
        z3::expr* getTVariable(Vertex* vPtr);

      /***************
       * B-Variables *
       ***************/
       /*!
        * B-Variables are Z3-Boolean-Expresions which contain the information if an operation is started in a certain
        * timeslot. Stored in a map with Vertex* and timeslot as key.
        */
        map<std::pair<Vertex*, int>, z3::expr> bVariables;
       /*!
        * Function to generate one B-Variable for each vertex and timestep and save it in the bVariables-map.
        */
        void generateBVariables();
       /*!
        * Getter for T-Variables, takes a vertex ptr and returns the corresponding T-Variable.
        * @param vPtr
        * @param i
        * @return Pointer to the B-Variable corresponding to vPtr and timeslot i.
        */
        z3::expr *getBvariable(Vertex *v, int i);

      /*******************************************************************
       * Functions to set up constraints to model the scheduling problem *
       *******************************************************************/
        /*!
         * Prevents T-Variables to become negative since we define timeslot 0 as the starting point of our schedule.
         * @return Result after checking the constraints.
         */
        z3::check_result addNonNegativeConstraints();
        /*!
         * Uses the edges of graph g to model the dependency constraints in the following way:
         * t_oi − t_oj + L_oi ≤ 0
         * Edges with weight can be ignored for a non-modulo schedule. Since the previous iteration is already completed
         * when the current iteration starts and the data will be present.
         * @return Result after checking the constraints.
         */
        z3::check_result addDependencyConstraints();
        /*!
         * This function connects B- and T-Variables so boi,t is true exactly then operation oi is scheduled in timeslot t.
         * boi,t = (toi == t)
         * @return Result after checking the constraints.
         */
        z3::check_result addVariableConnections();
        /*!
         * Used to model resource constraints.
         * Sum of all boi,t variables for operation form a limited resource r must be below the limit of this resource
         * for each timestep t in the schedule.
         * @return Result after checking the constraints.
         */
        z3::check_result addResourceConstraints();
        /*!
         * Function that finds the latest starttime in a previously generated ASAPHC-Schedule and then generates a max
         * latency constraint in preventing each toi getting larger then the latest starttime found.
         * @return Result after checking the constraints.
         */
        z3::check_result findMaxLatencyConstraints();
        /*!
         * Function to minimize latency after a valid schedule has been found by adding toi < tmax constraints to the
         * solver for each toi.
         * @return Result after checking the constraints.
         */
        z3::check_result minimizeLatency();

      /*****************
       * Utility Stuff *
       *****************/
        /*!
         * Parses the latest starttime from a satisfied Z3 model.
         * @return Latest starttime in the schedule.
         */
        int getLatestStarttime();
        /*!
         * Function to perform a Z3-Check with enabled timetracking. It updates the time remaining and time used variables
         * as well.
         */
        void z3CheckWithTimeTracking();
        /*!
         * see: setSearchMode(string &sreachMode)
         */
        string mode;
        /*!
         * Used as latency bounds.
         */
        int acturalRow;
        int lastRow;
        /*!
         * Information if a schedule is optimal. Gets true when a Z3-Check returned unkown at some point. 
         */
        bool latencyOptimal;

    };

} // HatScheT

#endif

#endif //HATSCHET_SMASHMINLATNONMOD_H
