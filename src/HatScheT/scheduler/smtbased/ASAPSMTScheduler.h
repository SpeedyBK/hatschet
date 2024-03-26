//
// Created by Benjamin on 19.03.24.
//

#ifndef HATSCHET_ASAPSMTSCHEDULER_H
#define HATSCHET_ASAPSMTSCHEDULER_H

#if USE_Z3

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/Z3SchedulerBase.h>

namespace HatScheT {

    class ASAPSMTScheduler : public SchedulerBase, public Z3SchedulerBase{

    public:

        ASAPSMTScheduler(Graph& g, ResourceModel& rm);

        /*!
         * use this methode to solve the scheduling problem
         */
        void schedule() override;

    private:

        /*!
         * Modeling Resource constraints:
         * First of all we define a matrix of integer Z3-Variables. First dimension is the time slot
         * of each operation. Second Dimension are the ressourses. So a graph like
         * A -> B
         * A -> B
         * B -> C
         * with the resources A, B, C will correspond to a matrix like
         *
         *      A   B   C
         * 1    2   0   0
         * 2    0   1   0
         * 3    0   0   1
         */

    };

} // HatScheT

#endif

#endif //HATSCHET_ASAPSMTSCHEDULER_H
