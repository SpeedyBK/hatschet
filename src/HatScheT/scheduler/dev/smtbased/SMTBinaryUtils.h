//
// Created by bkessler on 7/23/22.
//

#ifndef HATSCHET_SMTBINARYUTILS_H
#define HATSCHET_SMTBINARYUTILS_H

#include <map>
#include <vector>

#include <HatScheT/Vertex.h>
#include <HatScheT/ResourceModel.h>

namespace HatScheT {

  class SMTBinaryUtils {

  public:

      SMTBinaryUtils(map<Vertex*, int>&earliestTimes, map<Vertex*, int>&latestTimes, ResourceModel& rm, int currentII, int minLatency);

      void mainfunc();

      void printMatrix(map<pair<Vertex*, int>, bool>& matrix);

      void createMatrix(map<Vertex*, int>&earliestTimes, map<Vertex*, int>&latestTimes, int minLat);

      static void printAvailibleSlots(map<Vertex*, int>& availSlots);

      static void printUsedFUInModSlot(map<Resource*, vector<int>>& used);

      void reduceToOneInterval();

      void countFUsAndSlots();

      void calcMinLatency();

      bool ableToContinue(Resource *r);

      struct VertexTuple {
          Vertex* v;
          Resource* r;
          int slot;
          int* FUsInModslot;
          int* availSlots;
          bool inSlotBelowLimit;
      };

      struct availableComp {
          bool operator()(VertexTuple const &a, VertexTuple const &b)
          const noexcept {
              return a.availSlots < b.availSlots;
          }
      };

      struct FUsUsedComp {
          bool operator()(pair<int, int> const &a, pair<int, int> const &b)
          const noexcept {
              return a.second < b.second;
          }
      };

  private:

      ResourceModel& rm;

      int currentII;
      int minLatency;

      map<pair<Vertex*, int>, bool> vertexTimeslot;

      map<Vertex*, int> availibleSlots;
      map<Resource*, vector<int>> usedFUinModSlot;

      set<Vertex*> vpSet;


  };
}

#endif //HATSCHET_SMTBINARYUTILS_H
