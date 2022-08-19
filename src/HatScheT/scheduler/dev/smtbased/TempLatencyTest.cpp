//
// Created by bkessler on 8/9/22.
//

#include <iostream>
#include <cmath>
#include <chrono>
#include <deque>
#include <algorithm>
#include <fstream>

#include "TempLatencyTest.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/scheduler/ASAPScheduler.h"

namespace HatScheT {

  TempLatencyTest::TempLatencyTest(Graph &g, ResourceModel &resourceModel) : SchedulerBase(g, resourceModel) {
      computeMinII(&g, &resourceModel);
      minII = ceil(minII);
      computeMaxII(&g, &resourceModel);
  }

  void TempLatencyTest::schedule() {

      cout << _graphstr << endl;
      cout << _resstr << endl;
      cout << minII << endl;
      cout << maxII << endl;
      cout << "Calculate minLatency with ILP: " << endl;

      double tempII = minII;
      int minLAT = 0;
      deque<result> ilpMinLatResults;
      do{
          result r = minLatencyWithILP(tempII);
          ilpMinLatResults.push_back(r);
          minLAT = r.minLat;
          cout << "II: " << tempII << " minLat: " <<  minLAT << " Solving Time: " << r.solvingTime << endl;
          tempII += 1;
      } while ((int)tempII < minLAT);

      double totalSolvingTimeILP = 0;
      for (auto &r : ilpMinLatResults){
          totalSolvingTimeILP += r.solvingTime;
          cout << "Algorithm: " << r.algo << endl;
          cout << "II: " << r.II << endl;
          cout << "Solving Time: " << r.solvingTime << endl;
          cout << "minLat: " << r.minLat << endl;
          cout << "maxLat: " << r.maxLat << endl << endl;
      }

      cout << "Calculate minLatency with Custom Algo: " << endl;

      tempII = minII;
      minLAT = 0;
      deque<result> customMinLatResults;
      do{
          result r = minLatencyWithCustom(tempII);
          customMinLatResults.push_back(r);
          minLAT = r.minLat;
          cout << "II: " << tempII << " minLat: " <<  minLAT << " Solving Time: " << r.solvingTime << endl;
          tempII += 1;
      } while ((int)tempII < minLAT);

      double totalSolvingTimeCustom = 0;
      for (auto &r : customMinLatResults){
          totalSolvingTimeCustom += r.solvingTime;
          cout << "Algorithm: " << r.algo << endl;
          cout << "II: " << r.II << endl;
          cout << "Solving Time: " << r.solvingTime << endl;
          cout << "minLat: " << r.minLat << endl;
          cout << "maxLat: " << r.maxLat << endl << endl;
      }

      cout << "Calculate maxLatency with Custom Algo: " << endl;

      tempII = minII;
      int maxLAT = 0;
      deque<result> customMaxLatResults;
      do{
          result r = maxLatencyWithCustom(tempII);
          customMaxLatResults.push_back(r);
          maxLAT = r.maxLat;
          cout << "II: " << tempII << " maxLat: " <<  maxLAT << " Solving Time: " << r.solvingTime << endl;
          tempII += 1;
      } while ((int)tempII < maxLAT);

      double totalSolvingTimeMaxCustom = 0;
      for (auto &r : customMaxLatResults){
          totalSolvingTimeMaxCustom += r.solvingTime;
          cout << "Algorithm: " << r.algo << endl;
          cout << "II: " << r.II << endl;
          cout << "Solving Time: " << r.solvingTime << endl;
          cout << "minLat: " << r.minLat << endl;
          cout << "maxLat: " << r.maxLat << endl << endl;
      }

      tempII = minII;
      minLAT = 0;
      deque<result> sdcMinLatResults;
      do{
          result r = minLatencyWithSDC(tempII);
          sdcMinLatResults.push_back(r);
          minLAT = r.minLat;
          cout << "II: " << tempII << " minLat: " <<  minLAT << " Solving Time: " << r.solvingTime << endl;
          tempII += 1;
      } while ((int)tempII < minLAT);

      double totalSolvingTimesdc = 0;
      for (auto &r : sdcMinLatResults){
          totalSolvingTimesdc += r.solvingTime;
          cout << "Algorithm: " << r.algo << endl;
          cout << "II: " << r.II << endl;
          cout << "Solving Time: " << r.solvingTime << endl;
          cout << "minLat: " << r.minLat << endl;
          cout << "maxLat: " << r.maxLat << endl << endl;
      }

      tempII = minII;
      minLAT = 0;
      deque<result> asapMinLatResults;
      do{
          result r = minLatencyWithASAP();
          asapMinLatResults.push_back(r);
          minLAT = r.minLat;
          cout << "II: " << tempII << " minLat: " <<  minLAT << " Solving Time: " << r.solvingTime << endl;
          tempII += 1;
      } while ((int)tempII < minLAT);

      double totalSolvingTimeasap = 0;
      for (auto &r : asapMinLatResults){
          totalSolvingTimeasap += r.solvingTime;
          cout << "Algorithm: " << r.algo << endl;
          cout << "II: " << r.II << endl;
          cout << "Solving Time: " << r.solvingTime << endl;
          cout << "minLat: " << r.minLat << endl;
          cout << "maxLat: " << r.maxLat << endl << endl;
      }

      tempII = minII;
      maxLAT = 0;
      deque<result> oppermannMaxLatResults;
      do{
          result r = maxLatencyOppermann();
          oppermannMaxLatResults.push_back(r);
          maxLAT = r.maxLat;
          cout << "II: " << tempII << " maxLat: " <<  maxLAT << " Solving Time: " << r.solvingTime << endl;
          tempII += 1;
      } while ((int)tempII < minLAT);

      double totalSolvingTimeOppermann = 0;
      for (auto &r : oppermannMaxLatResults){
          totalSolvingTimeOppermann += r.solvingTime;
          cout << "Algorithm: " << r.algo << endl;
          cout << "II: " << r.II << endl;
          cout << "Solving Time: " << r.solvingTime << endl;
          cout << "minLat: " << r.minLat << endl;
          cout << "maxLat: " << r.maxLat << endl << endl;
      }

      tempII = minII;
      maxLAT = 0;
      deque<result> eichenbergerMaxLatResults;
      do{
          result r = maxLatencyEichenberger(tempII);
          eichenbergerMaxLatResults.push_back(r);
          maxLAT = r.maxLat;
          cout << "II: " << tempII << " maxLat: " <<  maxLAT << " Solving Time: " << r.solvingTime << endl;
          tempII += 1;
      } while ((int)tempII < minLAT);

      double totalSolvingTimeEichenberger = 0;
      for (auto &r : eichenbergerMaxLatResults){
          totalSolvingTimeEichenberger += r.solvingTime;
          cout << "Algorithm: " << r.algo << endl;
          cout << "II: " << r.II << endl;
          cout << "Solving Time: " << r.solvingTime << endl;
          cout << "minLat: " << r.minLat << endl;
          cout << "maxLat: " << r.maxLat << endl << endl;
      }

      cout << "Total Solving Time minLAT ILP: " << totalSolvingTimeILP << endl;
      cout << "Total Solving Time minLAT Custom: " << totalSolvingTimeCustom << endl;
      cout << "Total Solving Time minLAT SDC: " << totalSolvingTimesdc << endl;
      cout << "Total Solving Time minLAT ASAP: " << totalSolvingTimeasap << endl;
      cout << "Total Solving Time maxLat Custom: " << totalSolvingTimeMaxCustom << endl;
      cout << "Total Solving Time maxLat Oppermann: " << totalSolvingTimeOppermann << endl;
      cout << "Total Solving Time maxLat Eichenberger: " << totalSolvingTimeEichenberger<< endl;

      cout << endl;

      list<deque<result>*>datalist;
      datalist.push_back(&ilpMinLatResults);
      datalist.push_back(&customMinLatResults);
      datalist.push_back(&customMaxLatResults);
      datalist.push_back(&sdcMinLatResults);
      datalist.push_back(&asapMinLatResults);
      datalist.push_back(&oppermannMaxLatResults);
      datalist.push_back(&eichenbergerMaxLatResults);

      openFileAndCollectData(datalist);

      cout << "Number of Vertices: " << g.getNumberOfVertices() << endl;

      throw(HatScheT::Exception("No further Actions needed!"));
  }

  TempLatencyTest::result TempLatencyTest::minLatencyWithILP(double InitI) {

      result r;
      r.algo = "ILPminLat";
      r.maxLat = -1;
      r.II = InitI;

      auto start_t = std::chrono::high_resolution_clock::now();
      auto aslap = Utility::getSDCAsapAndAlapTimes(&g, &resourceModel, InitI);
      auto minLatILP = Utility::getMinLatency(&g, &resourceModel,
                                              aslap.first, aslap.second,
                                              (int)InitI, {"Gurobi"},
                                              300, -1,
                                              1, true);
      unordered_map<Vertex*, Vertex*> helperMap;
      unordered_map<Vertex*, int> tMaxUnordered;
      for (auto &v : aslap.second){
          helperMap[v.first] = v.first;
          tMaxUnordered[v.first] = v.second;
      }
      r.minLat = minLatILP + Utility::getSDCScheduleLength(tMaxUnordered, helperMap, &resourceModel);
      auto end_t = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count();
      auto t = ((double)duration / 1000);

      r.solvingTime = t;

      return r;

  }

  TempLatencyTest::result TempLatencyTest::minLatencyWithCustom(double InitI) {

      result r;
      r.algo = "CustomMinLat";
      r.maxLat = -1;
      r.II = InitI;

      auto start_t = std::chrono::high_resolution_clock::now();
      auto aslap = Utility::getSDCAsapAndAlapTimes(&g, &resourceModel, InitI);
      auto minLatCustom = Utility::getLatencyEstimation(&g, &resourceModel, InitI, Utility::latencyBounds::minLatency);
      r.minLat = minLatCustom.minLat;
      auto end_t = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count();
      auto t = ((double)duration / 1000);

      r.solvingTime = t;

      return r;

  }

  TempLatencyTest::result TempLatencyTest::maxLatencyWithCustom(double InitI) {
      result r;
      r.algo = "CustomMaxLat";
      r.minLat = -1;
      r.II = InitI;

      auto start_t = std::chrono::high_resolution_clock::now();
      auto aslap = Utility::getSDCAsapAndAlapTimes(&g, &resourceModel, InitI);
      auto maxLatCustom = Utility::getLatencyEstimation(&g, &resourceModel, InitI, Utility::latencyBounds::maxLatency);
      r.maxLat = maxLatCustom.maxLat;
      auto end_t = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count();
      auto t = ((double)duration / 1000);
      r.solvingTime = t;
      return r;
  }

  TempLatencyTest::result TempLatencyTest::minLatencyWithSDC(double InitI) {
      result r;
      r.algo = "SDCminLat";
      r.maxLat = -1;
      r.II = InitI;

      auto start_t = std::chrono::high_resolution_clock::now();
      auto aslap = Utility::getSDCAsapAndAlapTimes(&g, &resourceModel, InitI);
      unordered_map<Vertex*, Vertex*> helperMap;
      unordered_map<Vertex*, int> tMaxUnordered;
      for (auto &v : aslap.second){
          helperMap[v.first] = v.first;
          tMaxUnordered[v.first] = v.second;
      }
      r.minLat = Utility::getSDCScheduleLength(tMaxUnordered, helperMap, &resourceModel);
      auto end_t = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count();
      auto t = ((double)duration / 1000);

      r.solvingTime = t;

      return r;
  }

  TempLatencyTest::result TempLatencyTest::minLatencyWithASAP() {
      result r;
      r.algo = "ASAPminLat";
      r.maxLat = -1;
      r.II = -1;

      auto start_t = std::chrono::high_resolution_clock::now();
      map<Resource*, int> limits;
      for (auto &res : resourceModel.Resources()){
          limits[res] = res->getLimit();
          res->setLimit(UNLIMITED, false);
      }
      ASAPScheduler asap(g, resourceModel);
      asap.schedule();
      r.minLat = asap.getScheduleLength();
      for (auto &res : resourceModel.Resources()){
          res->setLimit(limits.at(res), false);
      }
      auto end_t = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count();
      auto t = ((double)duration / 1000);

      r.solvingTime = t;

      return r;
  }

  void TempLatencyTest::writeData(deque<result>& data, ofstream &of) {
      auto comp = [](const result &a, const result &b){ return a.II < b.II; };
      std::sort(data.begin(), data.end(), comp);

      for (auto &r : data){
          cout << r.II << "," << r.algo << "," << r.minLat << "," << r.maxLat << "," << r.solvingTime << ";\n";
          of << r.II << "," << r.algo << "," << r.minLat << "," << r.maxLat << "," << r.solvingTime << ";\n";
      }
  }

  void TempLatencyTest::openFileAndCollectData(list<deque<result> *> &data) {

      std::string delimiter = "/";

      size_t pos = 0;
      std::string token;
      while ((pos = _graphstr.find(delimiter)) != std::string::npos) {
          token = _graphstr.substr(0, pos);
          _graphstr.erase(0, pos + delimiter.length());
      }

      std::string delimiter2 = "/";

      size_t pos2 = 0;
      std::string token2;
      while ((pos2 = _resstr.find(delimiter2)) != std::string::npos) {
          token2 = _resstr.substr(0, pos2);
          _resstr.erase(0, pos2 + delimiter2.length());
      }

      string s;
      for (auto itr = _resstr.begin(); itr < _resstr.end() - 4; ++itr) {
          s += *itr;
      }

      string path = "SMTBenchmark/latency/" + token + "_" + s + ".csv";
      cout << path << endl;
      try {
          ofstream of;
          of.open(path);
          if (of.is_open()) {
              of << "#" << token + "_" + _graphstr << ";\n";
              of << "#" << token + "_" + _resstr << ";\n";
              of << "#" << "Number of Vertices: " << g.getNumberOfVertices() << ";\n";
              of << "II,Algorithm,minLat,maxLat,solvingTime;\n";
              for (auto &d : data) {
                  writeData(*d, of);
              }
              of.close();
          } else {
              throw (HatScheT::Exception("File not created"));
          }
      } catch (std::ofstream::failure &writeErr) {
          cout << path << endl;
          throw (HatScheT::Exception("File not created"));
      }
  }

  TempLatencyTest::result TempLatencyTest::maxLatencyOppermann() {

      result r;
      r.algo = "MaxLatOppermann";
      r.minLat = -1;
      r.II = -1;

      // use upper limit from Equation (6) in:
      // [1] J. Oppermann, M. Reuter-Oppermann, L. Sommer, A. Koch, and O. Sinnen,
      // ‘Exact and Practical Modulo Scheduling for High-Level Synthesis’,
      // ACM Transactions on Reconfigurable Technology and Systems, vol. 12, no. 2, p. 26.

      auto start_t = std::chrono::high_resolution_clock::now();
      int maxLatency = 0;
      for (auto &v : this->g.Vertices()) {
          int maxChainingDelay = 0;
          for (auto &e : this->g.Edges()) {
              if (&e->getVertexSrc() != v) continue;
              auto d = e->getDelay();
              if (d > maxChainingDelay) maxChainingDelay = d;
          }
          maxLatency += (this->resourceModel.getVertexLatency(v) + maxChainingDelay);
      }
      for (auto &res : this->resourceModel.Resources()) {
          if (res->isUnlimited()) continue;
          auto numRegistrations = this->resourceModel.getNumVerticesRegisteredToResource(res);
          auto limit = (float) res->getLimit();
          for (int i = 0; i < numRegistrations; i++) {
              maxLatency += (int) floor(((float) i) / limit);
          }
      }
      if (!this->quiet) {
          std::cout << "SATScheduler: computed maximum latency " << maxLatency << std::endl;
      }
      r.maxLat = maxLatency;
      auto end_t = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count();
      auto t = ((double)duration / 1000);

      r.solvingTime = t;

      return r;
  }

  TempLatencyTest::result TempLatencyTest::maxLatencyEichenberger(double InitI) {

      result r;
      r.algo = "MaxLatEichenberger";
      r.minLat = -1;
      r.II = InitI;

      auto start_t = std::chrono::high_resolution_clock::now();
      int numOfOps = g.getNumberOfVertices();
      int latOfOp = 0;
      for (auto &res : resourceModel.Resources()){
          if (res->getLatency() > latOfOp){
              latOfOp = res->getLatency();
          }
      }
      r.maxLat = numOfOps * (latOfOp + InitI - 1);
      auto end_t = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count();
      auto t = ((double)duration / 1000);

      r.solvingTime = t;

      return r;
  }

}


