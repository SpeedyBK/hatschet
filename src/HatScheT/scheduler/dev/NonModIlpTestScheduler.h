//
// Created by Benjamin on 12.04.24.
//

#ifndef HATSCHET_NONMODILPTESTSCHEDULER_H
#define HATSCHET_NONMODILPTESTSCHEDULER_H

#pragma once

#include "HatScheT/base/SchedulerBase.h"

namespace HatScheT {

    class NonModIlpTestScheduler : public SchedulerBase {

    public:
        NonModIlpTestScheduler(Graph& g, ResourceModel &resourceModel, const std::list<std::string>& solverWishlist);

        void schedule() override;

        bool isLatencyOptimal() {return isOptimal;}

    private:

        std::list<std::string> sW;

        bool isOptimal;
    };

} // HatScheT

#endif //HATSCHET_NONMODILPTESTSCHEDULER_H
