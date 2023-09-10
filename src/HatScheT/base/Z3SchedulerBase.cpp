//
// Created by bkessler on 11/8/22.
//
#if defined(USE_Z3)

#include "Z3SchedulerBase.h"

namespace HatScheT {

  HatScheT::Z3SchedulerBase::Z3SchedulerBase() : m(this->c), p(this->c), s(this->c){
      this->z3Quiet = true;
      this->r = z3::unknown;
  }

  void Z3SchedulerBase::setZ3Timeout(uint32_t seconds) {
      this->p.set("timeout", seconds * 1000);
  }

  void Z3SchedulerBase::setZ3Threads(uint32_t threads) {
      this->p.set("threads", threads);
  }

  void Z3SchedulerBase::printZ3Params() {
        std::cout << std::endl << p << std::endl;
  }

  z3::check_result Z3SchedulerBase::z3Check() {
      s.set(p);
      if (!z3Quiet){
          std::cout << std::endl << "Checking system of " << s.assertions().size() << " assertions... " << std::endl;
          //Too many COUTs, remove comment if needed:
          //std::cout << s << std::endl;
      }
      z3::check_result res = s.check();
      if (res == z3::sat){
          m = s.get_model();
          if (!z3Quiet){
              std::cout << "Result:" << std::endl;
              std::cout << res << std::endl;
              //Too many COUTs, remove comment if needed:
              //std::cout << "Model:" << std::endl;
              //std::cout << m << std::endl;
          }
      } else if (res == z3::unsat) {
          if (!z3Quiet) {
              std::cout << "Result:" << std::endl;
              std::cout << res << std::endl;
              std::cout << "Unsat-Core:" << std::endl;
              std::cout << s.unsat_core() << std::endl;
          }
      } else {
          if (!z3Quiet) {
              std::cout << "Result:" << std::endl;
              std::cout << res << std::endl;
              std::cout << "Reason Unknown:" << std::endl;
              std::cout << s.reason_unknown() << std::endl;
          }
      }
      this->r = res;
      return res;
  }

  void Z3SchedulerBase::z3Reset() {
      s.reset();
  }
}
#endif