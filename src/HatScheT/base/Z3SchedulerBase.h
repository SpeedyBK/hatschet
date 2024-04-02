//
// Created by bkessler on 11/8/22.
//

#ifdef USE_Z3

#include <z3++.h>
#include <iostream>
#include <string>

#ifndef HATSCHET_SMTSCHEDULERBASE_H
#define HATSCHET_SMTSCHEDULERBASE_H

namespace HatScheT {

  class Z3SchedulerBase {

  public:

    Z3SchedulerBase();

  protected:

    void setZ3Timeout(uint32_t seconds);

    void setZ3Threads(uint32_t threads);

    void setZ3Quiet(bool q){ this->z3Quiet = q; }

    void printZ3Params();

    void z3Reset();

    void setZ3OptimizerObjective(const std::string& objective, const z3::expr& objectiveTerm);

    z3::check_result z3Optimize();

    z3::check_result z3Check();

    z3::check_result getZ3Result() { return this->r; };

    z3::context c;

    z3::solver s;

    z3::optimize o;

    z3::model m;

    z3::params p;

    z3::check_result r;

  private:

    bool z3Quiet;

  };

}

#endif //HATSCHET_SMTSCHEDULERBASE_H
#endif //USE_Z3