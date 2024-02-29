#pragma once

#include <HatScheT/layers/IterativeModuloSchedulerLayer.h>
#include <HatScheT/scheduler/ASAPScheduler.h>
#include <HatScheT/utility/SDCSolverIncremental.h>
#include <HatScheT/utility/Utility.h>
#include <vector>

namespace HatScheT
{
    class NonIterativeModuloScheduler : public IterativeModuloSchedulerLayer
    {
        public:
            NonIterativeModuloScheduler(Graph& graph, ResourceModel& resourceModel, double II = -1); //: IterativeModuloSchedulerLayer(graph, resourceModel, II) { };

        protected:
            /*!
             * \brief Schedule Iteration for one II.
             */
            void scheduleIteration() override;

            void scheduleInit() override;

        private:
            class CycleVec : public std::vector<Vertex*>
            {
                public:
                    int slack;
                    bool operator < (const CycleVec& vec) const;
            };

            class PathVec : public std::vector<Vertex*>
            {
                public:
                    int length;
                    bool operator < (const PathVec& vec) const;
            };

            class TrackVec : public std::vector<std::pair<Vertex*, Edge*>>
                {
                public:
                    int length;
                };

            class OrderedWeight
            {
                public:
                    int weight = -1;
                    bool ordered = false;
                    OrderedWeight(int w, bool o);
            };

            bool custom = false;
            bool firstRun = true;

            std::map<Vertex*, OrderedWeight> wMap;
            std::map<Vertex*, std::map<Vertex*, bool>> pathBools;

            std::map<Vertex*, std::vector<PathVec>> cache;
            std::vector<Vertex*> orderedForwardInstructions;
            std::vector<std::pair<Edge*, PathVec>> cyclePaths;

            std::vector<CycleVec> getCycleAndSlacks(Vertex* currentInst, Vertex* backEdgeInst, int partialSlack, int backEdgeDistance, std::map<Vertex*, bool>& vMap);
            std::vector<PathVec> getBackwardsPathsAndLengths(Vertex* currentInst, int partialLength, std::map<Vertex*, bool>& vMap);
            std::vector<PathVec> getForwardsPathsAndLengths(Vertex* currentInst, int partialLength, std::map<Vertex*, bool>& vMap);
            std::vector<Vertex*> establishTopologicalOrderForCycle(Vertex* currentInst, std::map<Vertex*, OrderedWeight>& wMap);
            std::vector<Vertex*> establishTopologicalOrderForPath(Vertex* currentInst, std::map<Vertex*, OrderedWeight>& wMap);
            std::map<int, int> getASAPNonRestrictedStartTimes(int curII);
            void propagateIncrement(std::map<Vertex*, int>& increments, Vertex* instruction, int increment);
            bool isSourceForCurrentLoop(Vertex& instruction);
            bool isSinkForCurrentLoop(Vertex& instruction);

            std::vector<PathVec> getPathsAndCycles(Vertex* currentInst, int currentLength, std::map<Edge*, std::vector<PathVec>>& currentCyclePaths, std::map<Vertex*, bool>& vMap);
            std::vector<PathVec> getPathsAndCyclesCached(Vertex* currentInst, int currentLength, std::map<Edge*, std::vector<PathVec>>& currentCyclePaths, std::map<Vertex*, bool>& vMap, std::map<Vertex*, std::vector<PathVec>>& cache);

            int getLatency(Vertex* vertex) { return this->resourceModel.getResource(vertex)->getLatency(); }

    };
}