//
// Created by bkessler on 4/25/24.
//

#include "SMASHIntegerScheduler.h"

#if USE_Z3

namespace HatScheT {

  SMASHIntegerScheduler::SMASHIntegerScheduler(Graph &g, ResourceModel &resourceModel, int II)
      : IterativeModuloSchedulerLayer(g, resourceModel, II) {

  }

  void SMASHIntegerScheduler::scheduleInit() {
      cout << "Start Scheduling!" << endl;
  }

  void SMASHIntegerScheduler::scheduleIteration() {

      cout << "generateModuloAndYVariables..." << endl;
      generateModuloAndYVariables();

      cout << "boundYVariables..." << endl;
      boundYVariables();

      cout << "boundMVariables..." << endl;
      boundMVariables();

      cout << "addDependencyConstraints..." << endl;
      addDependencyConstraints();

      cout << "generateResourceVariables..." << endl;
      generateResourceVariables();

      cout << "boundRVariables..." << endl;
      boundRVariables();

      cout << "addHardwareConstraints..." << endl;
      addHardwareConstraints();

      cout << "Solving!" << endl;
      z3Check();

      if (getZ3Result() == z3::sat) {
          parseSchedule();
      }else{
          z3Reset();
      }

      scheduleFound = (getZ3Result() == z3::sat);

  }

  void SMASHIntegerScheduler::generateModuloAndYVariables() {

      mVariables.clear();
      yVariables.clear();
      for (auto &vIt: g.Vertices()) {
          std::stringstream mname;
          mname << "m_" << vIt->getName();
          z3::expr m(c.int_const(mname.str().c_str()));
          mVariables.insert(std::make_pair(vIt, m));
          std::stringstream yname;
          yname << "y_" << vIt->getName();
          z3::expr y(c.int_const(yname.str().c_str()));
          yVariables.insert(std::make_pair(vIt, y));
      }

  }

  void SMASHIntegerScheduler::generateResourceVariables() {

      for (auto &rIt: resourceModel.Resources()) {
          if (rIt->isUnlimited()) {
              continue;
          }
          for (auto &vIt: resourceModel.getVerticesOfResource(rIt)) {
              std::stringstream name;
              name << "r_" << vIt->getName();
              z3::expr r(c.int_const(name.str().c_str()));
              rVariables.insert(std::make_pair((Vertex*)vIt, r));
          }
      }

  }

  void SMASHIntegerScheduler::boundYVariables() {

      for (auto &yIt : yVariables){
          z3::expr e = yIt.second >= 0 && yIt.second < 400;
          e = e.simplify();
          s.add(e);
      }

  }

  void SMASHIntegerScheduler::boundMVariables() {

      for (auto &mIt : mVariables){
          z3::expr e = mIt.second >= 0 && mIt.second < (int)II;
          e = e.simplify();
          s.add(e);
      }

  }

  void SMASHIntegerScheduler::addDependencyConstraints() {

      for (auto &eIt : g.Edges()){
          auto vSrc = (Vertex*)&eIt->getVertexSrc();
          auto vDst = (Vertex*)&eIt->getVertexDst();
          z3::expr mSrc = mVariables.at(vSrc);
          z3::expr ySrc = yVariables.at(vSrc);
          z3::expr mDst = mVariables.at(vDst);
          z3::expr yDst = yVariables.at(vDst);
          // m_oi + y_oi * II - m_oj - y_oj * II + L_oi - D_eij * II ≤ 0
          z3::expr constraint = (mSrc + ySrc * (int)II - mDst - yDst * (int)II + resourceModel.getVertexLatency(vSrc) + eIt->getDelay() - eIt->getDistance() * (int)II <= 0);
          constraint = constraint.simplify();
          s.add(constraint);
      }

  }

  void SMASHIntegerScheduler::addHardwareConstraints() {

      for (auto &rIt: resourceModel.Resources()) {
          if (rIt->isUnlimited()) {
              continue;
          }
          for (auto &vItI: resourceModel.getVerticesOfResource(rIt)) {
              for (auto &vItJ : resourceModel.getVerticesOfResource(rIt)){
                  if (vItI == vItJ){
                      continue;
                  }
                  auto moi = mVariables.at((Vertex*)vItI);
                  auto roi = rVariables.at((Vertex*)vItI);
                  auto moj = mVariables.at((Vertex*)vItJ);
                  auto roj = rVariables.at((Vertex*)vItJ);
                  z3::expr constraint = z3::nand(moi == moj, roi == roj);
                  constraint = constraint.simplify();
                  s.add(constraint);
              }
          }
      }

  }

  void SMASHIntegerScheduler::boundRVariables() {

      for (auto &rIt : rVariables){
          z3::expr e = rIt.second >= 0 && rIt.second < resourceModel.getResource(rIt.first)->getLimit();
          s.add(e);
      }

  }

  void SMASHIntegerScheduler::parseSchedule() {

      for (auto &vIt : g.Vertices()){
          int moi = m.eval(mVariables.at(vIt)).get_numeral_int();
          int yoi = m.eval(yVariables.at(vIt)).get_numeral_int();
          int t = moi + yoi * (int)II;
          startTimes.insert(std::make_pair(vIt, t));
      }

  }


} // Hatschet

#endif