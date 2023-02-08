//
// Created by nfiege on 2/6/23.
//

#ifndef HATSCHET_ILPSCHEDULELENGTHESTIMATION_H
#define HATSCHET_ILPSCHEDULELENGTHESTIMATION_H

#ifdef USE_SCALP
#include <HatScheT/Graph.h>
#include <HatScheT/ResourceModel.h>
#include <ScaLP/Solver.h>
#include <memory>
#include <map>
#include <list>
#include <string>

namespace HatScheT {
	class ILPScheduleLengthEstimation {
	public:
		ILPScheduleLengthEstimation(Graph* g, ResourceModel* rm, const std::list<std::string>& sw, const int &threads=1);

		void setQuiet(const bool &q) { this->quiet = q; }

		void estimateMinSL(const int &II, const int &timeBudget = -1);
		void estimateMaxSL(const int &II, const int &timeBudget = -1);
		void calcScheduleFromMaxSLEstimation(const int &II, const int &timeBudget = -1);

		bool minSLEstimationFound() const { return this->minSL >= 0; }
		bool maxSLEstimationFound() const { return this->maxSL >= 0; }
		bool scheduleFound() const { return !this->schedule.empty(); }
		bool timeoutEncountered() const { return this->timeout; }

		int getMinSLEstimation() const { return this->minSL; }
		int getMaxSLEstimation() const { return this->maxSL; }
		std::map<Vertex*, int> getSchedule() const;

		std::map<Vertex*, int> getASAPTimesSDC() const;
		std::map<Vertex*, int> getALAPTimeDiffsSDC() const;
		int getSDCScheduleLength() const { return this->sdcSL; }

	private:
		std::map<const Vertex*, int> ASAPTimesSDC;
		std::map<const Vertex*, int> ALAPTimesSDC;
		std::map<const Vertex*, int> ASAPTimesMaxSL;
		std::map<const Vertex*, int> ALAPTimesMaxSL;
		std::map<const Vertex*, int> ALAPTimeDiffsSDC;
		std::map<const Vertex*, int> schedule;
		int minSL = -1;
		int maxSL = -1;
		int sdcSL = -1;
		bool timeout = false;

		std::map<const Resource*, bool> resourceUnlimited;
		std::map<const Vertex*, bool> vertexUnlimited;
		std::map<const Resource*, int> maxDelay;

		Graph* g;
		ResourceModel* rm;
		std::unique_ptr<ScaLP::Solver> s;
		std::list<std::string> sw;
		int threads;
		bool quiet = true;
	};
}

#endif //USE_SCALP

#endif //HATSCHET_ILPSCHEDULELENGTHESTIMATION_H
