//
// Created by nfiege on 5/20/22.
//

#ifndef HATSCHET_SATSCCSCHEDULER_H
#define HATSCHET_SATSCCSCHEDULER_H

#ifdef USE_CADICAL

#include <cadical.hpp>
#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <string>
#include <map>
#include <utility>
#include <memory>
#include <HatScheT/scheduler/satbased/CaDiCaLTerminator.h>
#include <HatScheT/utility/subgraphs/SCC.h>

namespace HatScheT {
	class SATSCCScheduler : public SchedulerBase, public ModuloSchedulerBase {
	public:
		SATSCCScheduler(Graph& g, ResourceModel &resourceModel, int II);
		~SATSCCScheduler() override;
		void schedule() override;
		void setSolverTimeout(unsigned int newTimeoutInSec);
		bool getIIFeasible() const;
		double getSolvingTime() const;

        map <Vertex*, pair<int, int>> printVertexStarttimes();

	private:
		std::map<const Resource*, std::map<int, int>> MRT;
		void initMRT();
		void computeSCCs();
		void createSCCGraphsAndRMs();
		void computeEarliestAndLatestStartTimes();
		std::pair<std::map<Vertex*, int>, std::map<Vertex*, int>> getMinMaxSCCStartTimes(const std::list<Vertex*> &sccVertices, const std::list<Edge*> &sccEdges, const int &maxLat);
		void computeComplexSCCSchedule();
		void computeBasicSCCSchedules();
		void orderSCCs();
		void computeFinalSchedule();
		void postProcessSchedule();
		Graph complexSCCG;
		ResourceModel complexSCCR;
		int numBasicSCCs;
		std::vector<std::shared_ptr<Graph>> basicSCCG;
		std::vector<std::shared_ptr<ResourceModel>> basicSCCR;
		unsigned int solverTimeout;
		double solvingTime;
		bool IIFeasible;
		std::map<Vertex*, int> earliestStartTimes;
		std::map<Vertex*, int> latestStartTimes;
		std::map<Vertex*, int> latestStartTimeDifferences;
		std::map<SCC*, int> sccMaxLat;
		int sccGraphMaxLat;
		std::map<Vertex*, int> relativeSchedule;
		std::map<Vertex*, Vertex*> vertexToSCCVertexMap;
		std::map<Vertex*, Vertex*> sccVertexToVertexMap;
		std::vector<std::vector<SCC*>> topologicallySortedSCCs;
		std::vector<SCC*> sccs;
	};
}
#endif

#endif //HATSCHET_SATSCCSCHEDULER_H
