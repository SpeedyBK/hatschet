/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Benjamin Lagershausen-Keßler (blagershausen@uni-kassel.de)
    Date: 4/1/24.

    Copyright (C) 2024

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

#ifndef HATSCHET_SMASHSCHEDULER_H
#define HATSCHET_SMASHSCHEDULER_H

#pragma once

#ifdef USE_Z3

#include "HatScheT/layers/IterativeModuloSchedulerLayer.h"
#include "HatScheT/base/Z3SchedulerBase.h"
#include "HatScheT/utility/Utility.h"

namespace HatScheT {

    class SMASHScheduler : public IterativeModuloSchedulerLayer, public Z3SchedulerBase {

    public:

        SMASHScheduler(Graph &g, ResourceModel &resourceModel, int II = -1);
        /*!
         * Overrides methode from schedulerbase-class and sets a timeout for Z3.
         * @param seconds Timeout in seconds.
         */
        void setSolverTimeout(double seconds) override;
        /*!
         * Mainly for debugging.
         * @return Name of the scheduler
         */
        string getName() override { return "SMASH (SMt-Aided-ScHeduling)"; }

    private:

        /*****************************
         * Main Scheduling Functions *
         *****************************/
        /*!
         * Function used to do stuff that has to be done before the II search loop starts. In this scheduler just used
         * for debugging prints and passing the set solver timeout to the z3-solver.
         */
        void scheduleInit() override;
        /*!
         * Main scheduling function is call from inside the II search loop in the iterative modulo scheduler layer.
         */
        void scheduleIteration() override;

        /***************
         * T-Variables *
         ***************/
        /*!
         * T-Variables are Z3-Integer-Expresions which contain the information in which timestep an operation is startet.
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
         * t_oi − t_oj + L_oi - D_eij * II ≤ 0
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
         * for each moduloslot in the schedule.
         * @return Result after checking the constraints.
         */
        z3::check_result addResourceConstraints(int candidateII);
        /*!
         * Adds a max-latency constraint to the solver. For now this constraint can be set by the user, or by the
         * utility latency estimation.
         * @return Result after checking the constraints.
         */
        z3::check_result addMaxLatencyConstraint();
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

        int getScheduleLengthOfGivenSchedule(map<Vertex *, int> &vertexStarttimes, ResourceModel *rm);

        Utility::LatencyEstimation latencyEstimation;

        /*!
         * Used as latency bounds.
         */
        int actualLength;
        int lastLength;

    };

}

#endif //USE_Z3
#endif //HATSCHET_SMASHSCHEDULER_H
