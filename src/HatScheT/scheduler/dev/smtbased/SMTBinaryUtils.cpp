//
// Created by bkessler on 7/23/22.
//

#include <iomanip>
#include <algorithm>
#include <queue>
#include "SMTBinaryUtils.h"

namespace HatScheT{


  SMTBinaryUtils::SMTBinaryUtils(map<Vertex *, int> &earliestTimes, map<Vertex *, int> &latestTimes, ResourceModel &rm,
                                 int currentII, int minLatency) : rm(rm) {

      cout << "Constructor of SMTBinaryUtils!" << endl;

      this->currentII = currentII;
      this->minLatency = minLatency;

      createMatrix(earliestTimes, latestTimes, minLatency);

      for (auto &vp : earliestTimes){
          vpSet.insert(vp.first);
      }

      cout << "Constructor Done!" << endl;

  }

  void SMTBinaryUtils::printMatrix(map<pair<Vertex*, int>, bool>& matrix) {
      cout << "MinLAT: " << minLatency << endl;
      for (auto &r : rm.Resources()) {
          if (r->isUnlimited()) {
              continue;
          }
          for (auto &v : rm.getVerticesOfResource(r)) {
              cout << setw(18) << v->getName() << ": ";
              for (int i = 0; i < minLatency; i++) {
                  if (i % currentII == 0) {
                      cout << " ";
                  }
                  cout << matrix.at(std::make_pair((Vertex *) v, i));
              }
              cout << " // " << availibleSlots.at((Vertex*)v) << endl;
          }
          cout << setw(22);
          for (int i = 0; i < minLatency; i++){
              if (i % currentII == 0){
                  cout << "-";
              }
              cout << "-";
          }
          cout << endl;
          cout << setw(22);
          for (auto &i : usedFUinModSlot.at(r)){
              cout << i;
          }
          cout << endl << endl;
      }
  }

  void SMTBinaryUtils::createMatrix(map<Vertex *, int> &earliestTimes, map<Vertex *, int> &latestTimes, int minLat) {
      cout << "Creating Starttimes Matrix:" << endl;
      for (auto &vipair : earliestTimes) {
          int tAsap = earliestTimes.at(vipair.first);
          int tAlap = latestTimes.at(vipair.first);
          for (int i = 0; i < tAsap; i++) {
              vertexTimeslot[{vipair.first, i}] = false;
          }
          for (int i = tAsap; i <= tAlap; i++) {
              vertexTimeslot[{vipair.first, i}] = true;
          }
          for (int i = tAlap + 1; i < minLat; i++) {
              vertexTimeslot[{vipair.first, i}] = false;
          }
      }
  }

  void SMTBinaryUtils::printAvailibleSlots(map<Vertex *, int>& availSlots) {
      for (auto &it : availSlots){
          cout << it.first->getName() << ": " << it.second << endl;
      }
  }

  void SMTBinaryUtils::printUsedFUInModSlot(map<Resource *, vector<int>> &used) {
      for (auto &r : used) {
          cout << r.first->getName() << ": " << endl;
          for (auto &ms : r.second) {
              cout << ms << " ";
          }
          cout << endl;
      }
  }

  void SMTBinaryUtils::reduceToOneInterval() {

      for (int x = 0; x < currentII; x++) {
          for (auto &vp : vpSet) {
              if (vertexTimeslot.at({vp, x})) {
                  for (int i = x + 1; i < minLatency; i++) {
                      if ((i % currentII) != x) {
                          continue;
                      }
                      vertexTimeslot.at({vp, i}) = false;
                  }
              } else {
                  for (int i = x + 1; i < minLatency; i++) {
                      if ((i % currentII) != x) {
                          continue;
                      }
                      if (vertexTimeslot.at({vp, i})) {
                          vertexTimeslot.at({vp, i}) = false;
                          vertexTimeslot.at({vp, x}) = true;
                      }
                  }
              }
          }
      }
  }

  void SMTBinaryUtils::mainfunc() {
      reduceToOneInterval();
      countFUsAndSlots();
      printMatrix(vertexTimeslot);
      calcMinLatency();
  }

  void SMTBinaryUtils::countFUsAndSlots() {
      for (auto &r : rm.Resources()){
          if (r->isUnlimited()){
              continue;
          }
          vector<int> usedFUofR;
          usedFUofR.resize(currentII);
          usedFUinModSlot[r] = usedFUofR;
          auto verticesOfThisResource = rm.getVerticesOfResource(r);
          for (auto &cvp : verticesOfThisResource) {
              auto vp = (Vertex*) cvp;
              availibleSlots[vp] = 0;
              for (int i = 0; i < currentII; i++) {
                  if (!vertexTimeslot.at({vp, i})) {
                      continue;
                  }
                  usedFUinModSlot.at(r).at(i)++;
                  availibleSlots.at(vp)++;
              }
          }
      }
  }

  void SMTBinaryUtils::calcMinLatency() {
      for (auto &r: rm.Resources()){
          if (ableToContinue(r)){
              cout << "true... continue..." << endl;
              continue;
          }

          cout << "false... Opimize FUs... " << endl;

          for (int x = 0; x < currentII; x++) {
              priority_queue<VertexTuple, vector<VertexTuple>, availableComp> pq;
              for (auto &cvp : rm.getVerticesOfResource(r)) {
                  auto vp = (Vertex *) cvp;
                  VertexTuple vt = {vp, r, x, &usedFUinModSlot.at(r).at(x), &availibleSlots.at(vp)};
                  pq.push(vt);
              }
              //Debugging
              vector<VertexTuple>tempVec;
              while(!pq.empty()){
                  auto temp = pq.top();
                  pq.pop();
                  cout << temp.v->getName() << " " << temp.slot << " " << *temp.FUsInModslot << " " << *temp.availSlots << endl;
              }
              for (auto &debugIt : tempVec){
                  pq.push(debugIt);
              }
              //Debugging Ende
              int usedFUsAtActualSlot = 0;
              while (usedFUinModSlot.at(r).at(x) > r->getLimit()){
                  priority_queue<pair<int, int>, vector<pair<int, int>>, FUsUsedComp> fupq;
                  int limit = r->getLimit();
                  for (int i = 0; i < currentII; i++){
                      auto p = std::make_pair(i, limit - usedFUinModSlot.at(r).at(i));
                      fupq.push(p);
                  }

              }
          }
      }
  }

  bool SMTBinaryUtils::ableToContinue(Resource *r) {
      if (r->isUnlimited()) {
          return true;
      }
      auto limit = (const int) r->getLimit();
      return std::all_of(usedFUinModSlot.at(r).begin(), usedFUinModSlot.at(r).end(), [limit](int i){ return i <= limit;});
  }
}