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

	private:
		std::map<const Resource*, std::map<int, int>> MRT;
		void initMRT();
		void computeSCCs();
		void computeSCCSchedule();
		void computeEarliestAndLatestStartTimes();
		void orderSCCs();
		void computeFinalSchedule();
		void postProcessSchedule();
		unsigned int solverTimeout;
		double solvingTime;
		bool IIFeasible;
		std::map<Vertex*, int> earliestStartTimes;
		std::map<Vertex*, int> latestStartTimes;
		std::map<Vertex*, int> latestStartTimeDifferences;
		std::map<SCC*, int> sccMaxLat;
		int sccGraphMaxLat;
		std::map<Vertex*, int> relativeSchedule;
		std::map<Vertex*, SCC*> sccVertexToSCCMap;
		std::map<Vertex*, Vertex*> vertexToSCCVertexMap;
		std::map<Vertex*, Vertex*> sccVertexToVertexMap;
		std::vector<std::vector<SCC*>> topologicallySortedSCCs;
		std::vector<SCC*> sccs;
	};
}
#endif

#endif //HATSCHET_SATSCCSCHEDULER_H
