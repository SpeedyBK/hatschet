//
// Created by nfiege on 2/6/23.
//

#include "ILPScheduleLengthEstimation.h"
#include <HatScheT/utility/Utility.h>

namespace HatScheT {
	ILPScheduleLengthEstimation::ILPScheduleLengthEstimation(HatScheT::Graph *g, HatScheT::ResourceModel *rm,
																													 const list<std::string> &sw, const int &threads) : g(g), rm(rm), sw(sw), threads(threads) {

		if (!this->quiet) std::cout << "ILPScheduleLengthEstimation::constructor: computing resource infos" << std::endl;
		for (auto &r : this->rm->Resources()) {
			auto vertices = this->rm->getVerticesOfResource(r);
			auto unlimited = (r->isUnlimited()) or (this->rm->getNumVerticesRegisteredToResource(r) <= r->getLimit());
			this->resourceUnlimited[r] = unlimited;
			for (auto &v : vertices) {
				this->vertexUnlimited[v] = unlimited;
			}
		}
		// max delay per resource
		for (auto &r : this->rm->Resources()) {
			if (this->resourceUnlimited.at(r)) {
				this->maxDelay[r] = 0;
				continue;
			}
			double limit = r->getLimit();
			double numVertices = this->rm->getNumVerticesRegisteredToResource(r);
			this->maxDelay[r] = (int)std::ceil(numVertices / limit);
		}
	}

	void ILPScheduleLengthEstimation::estimateMinSL(const int &II, const int &timeBudget) {
		// compute min/max SDC times
		auto minMaxTimes = Utility::getSDCAsapAndAlapTimes(this->g, this->rm, II);
		this->sdcSL = 0;
		for (auto &v : this->g->Vertices()) {
			this->ASAPTimesSDC[v] = minMaxTimes.first.at(v);
			this->ALAPTimesSDC[v] = minMaxTimes.second.at(v);
			if (this->ASAPTimesSDC[v] + this->rm->getVertexLatency(v) > this->sdcSL) {
				this->sdcSL = this->ASAPTimesSDC[v] + this->rm->getVertexLatency(v);
			}
		}
		// compute actual SL estimation
		this->s = std::unique_ptr<ScaLP::Solver>(new ScaLP::Solver(this->sw));
		if (this->threads > 0) {
			this->s->threads = (uint8_t) this->threads;
		}
		std::map<Vertex *, ScaLP::Variable> y;
		std::map<std::pair<Vertex *, int>, ScaLP::Variable> b;
		std::map<Vertex *, ScaLP::Variable> t;
		int offset = 0;
		auto tau = ScaLP::newIntegerVariable("tau", 0, ScaLP::INF());
		for (auto &v : this->g->Vertices()) {
			// check if tMin/tMax are set
			if (this->ASAPTimesSDC.find(v) == this->ASAPTimesSDC.end() or this->ALAPTimesSDC.find(v) == this->ALAPTimesSDC.end()) {
				throw Exception("ILPScheduleLengthEstimation::estimateMinSL: tMin/tMax missing for vertex '" + v->getName() + "'");
			}
			auto r = this->rm->getResource(v);
			if (this->resourceUnlimited.at(r)) {
				auto &tVar = t[v] = ScaLP::newIntegerVariable("t_" + v->getName(), 0, ScaLP::INF());
				this->s->addConstraint(tVar - tau <= this->ALAPTimesSDC.at(v));
				this->s->addConstraint(tVar >= this->ASAPTimesSDC.at(v));
			} else {
				auto &yVar = y[v] = ScaLP::newIntegerVariable("y_" + v->getName(), 0, ScaLP::INF());
				ScaLP::Term bSum;
				ScaLP::Term bSumWeighted;
				for (int m = 0; m < II; m++) {
					auto &bVar = b[{v, m}] = ScaLP::newBinaryVariable("b_" + v->getName() + "_" + std::to_string(m));
					bSum += bVar;
					bSumWeighted += (m * bVar);
				}
				this->s->addConstraint(bSum == 1);
				this->s->addConstraint(bSumWeighted + II * yVar - tau <= this->ALAPTimesSDC.at(v));
				this->s->addConstraint(bSumWeighted + II * yVar >= this->ASAPTimesSDC.at(v));
			}
			auto tMaxV = this->ALAPTimesSDC.at(v) + r->getLatency();
			if (offset < tMaxV) offset = tMaxV;
		}
		for (auto &r : this->rm->Resources()) {
			if (this->resourceUnlimited.at(r)) continue;
			auto vertices = this->rm->getVerticesOfResource(r);
			for (int m = 0; m < II; m++) {
				ScaLP::Term bSum;
				for (auto &vc : vertices) {
					auto *v = const_cast<Vertex *>(vc);
					bSum += b[{v, m}];
				}
				this->s->addConstraint(bSum <= r->getLimit());
			}
		}
		this->s->setObjective(ScaLP::minimize(tau));
		if (timeBudget > 0) {
			this->s->timeout = timeBudget;
		}
		auto stat = this->s->solve();
		this->timeout = stat == ScaLP::status::TIMEOUT_INFEASIBLE;
		if (stat == ScaLP::status::INFEASIBLE or stat == ScaLP::status::TIMEOUT_INFEASIBLE or
				stat == ScaLP::status::INVALID or stat == ScaLP::status::ERROR or
				stat == ScaLP::status::INFEASIBLE_OR_UNBOUND or stat == ScaLP::status::UNKNOWN) {
			std::cout << "ILPScheduleLengthEstimation::estimateMinSL: ScaLP failed to find min latency estimation in " << timeBudget << " sec" << std::endl;
			std::cout << "ILPScheduleLengthEstimation::estimateMinSL: ScaLP status: " << ScaLP::showStatus(stat) << std::endl;
			this->minSL = -1;
			return;
		}
		this->minSL = offset + (int) std::round(this->s->getResult().values.at(tau));
	}

	void ILPScheduleLengthEstimation::estimateMaxSL(const int &II, const int &timeBudget) {

		// set up solver
		this->s = std::unique_ptr<ScaLP::Solver>(new ScaLP::Solver(this->sw));
		if (this->threads > 0) {
			this->s->threads = this->threads;
		}
		// variables
		std::map<const Vertex*, ScaLP::Variable> t;
		std::map<const Vertex*, ScaLP::Variable> d;
		std::map<std::pair<const Vertex*, int>, ScaLP::Variable> p;
		for (auto &v : this->g->Vertices()) {
			t[v] = ScaLP::newIntegerVariable("t_"+std::to_string(v->getId()), 0.0, ScaLP::INF());
			d[v] = ScaLP::newIntegerVariable("d_"+std::to_string(v->getId()), 0.0, ScaLP::INF());
			auto *r = this->rm->getResource(v);
			if (this->resourceUnlimited.at(r)) continue;
			auto maxD = this->maxDelay.at(r);
			for (int delay=0; delay<maxD; delay++) {
				p[{v, delay}] = ScaLP::newBinaryVariable("p_"+std::to_string(v->getId())+"_"+std::to_string(delay));
			}
		}
		// constraints & objective
		ScaLP::Term objective;
		for (auto &v : this->g->Vertices()) {
			auto *r = this->rm->getResource(v);
			if (this->resourceUnlimited.at(r)) {
				this->s->addConstraint(d[v] == 0);
			}
			else {
				ScaLP::Term pSum;
				ScaLP::Term pSumWeighted;
				auto maxD = this->maxDelay.at(r);
				for (int delay=0; delay<maxD; delay++) {
					pSum += p[{v, delay}];
					pSumWeighted += (delay * p[{v, delay}]);
				}
				this->s->addConstraint(pSum == 1);
				this->s->addConstraint(pSumWeighted - d[v] >= 0);
				objective += pSumWeighted;
				objective -= d[v];
			}
		}
		for (auto &e : this->g->Edges()) {
			auto* v = &e->getVertexSrc();
			auto* w = &e->getVertexDst();
			auto* r = this->rm->getResource(v);
			auto l = r->getLatency();
			auto dist = e->getDistance();
			auto delay = e->getDelay();
			auto rhs = l + delay - (dist * II);
			auto tV = t.at(v);
			auto tW = t.at(w);
			auto dV = d.at(v);
			this->s->addConstraint(tW - tV - dV >= rhs);
		}
		for (auto &r : this->rm->Resources()) {
			if (this->resourceUnlimited.at(r)) continue;
			auto vertices = this->rm->getVerticesOfResource(r);
			auto maxD = this->maxDelay.at(r);
			auto limit = r->getLimit();
			for (int delay=0; delay<maxD; delay++) {
				ScaLP::Term pSum;
				for (auto &v : vertices) {
					pSum += p[{v, delay}];
				}
				this->s->addConstraint(pSum <= limit);
			}
		}
		// objective
		this->s->setObjective(ScaLP::minimize(objective));
		// compute solution
		//if (timeBudget > 0) this->s->timeout = timeBudget;
		//std::chrono::high_resolution_clock::time_point startTimer = std::chrono::high_resolution_clock::now();
		auto stat = this->s->solve();
		//std::chrono::high_resolution_clock::time_point endTimer = std::chrono::high_resolution_clock::now();
		//auto elapsedTimeSec = ((double)std::chrono::duration_cast<std::chrono::milliseconds>(endTimer - startTimer).count()) / 1000.0;
		this->timeout = stat == ScaLP::status::TIMEOUT_INFEASIBLE;
		if (stat == ScaLP::status::INFEASIBLE or stat == ScaLP::status::TIMEOUT_INFEASIBLE or
				stat == ScaLP::status::INVALID or stat == ScaLP::status::ERROR or
				stat == ScaLP::status::INFEASIBLE_OR_UNBOUND or stat == ScaLP::status::UNKNOWN) {
			std::cout << "ILPScheduleLengthEstimation::estimateMaxSL: ScaLP failed to find max latency estimation in " << timeBudget << " sec" << std::endl;
			std::cout << "ILPScheduleLengthEstimation::estimateMaxSL: ScaLP status: " << ScaLP::showStatus(stat) << std::endl;
			this->maxSL = -1;
			return;
		}
		auto solution = this->s->getResult().values;
		std::map<std::pair<const Resource*, int>, int> pseudoResourceConstraintDelays;
		std::map<const Vertex*, int> vertexDelays;
		std::map<const Resource*, int> delayDiffs;
		for (auto &r : this->rm->Resources()) {
			//if (this->resourceUnlimited.at(r)) continue;
			auto vertices = this->rm->getVerticesOfResource(r);
			delayDiffs[r] = 0;
			for (auto &v : vertices) {
				auto delay = (int)std::round(solution.at(d.at(v)));
				vertexDelays[v] = delay;
				pseudoResourceConstraintDelays[{r, delay}]++;
				if (r->isUnlimited()) continue;
				auto requestedDelay = 0;
				auto maxD = this->maxDelay.at(r);
				for (int it=0; it<maxD; it++) {
					requestedDelay += it * (int)std::round(solution.at(p.at({v, it})));
				}
				delayDiffs[r] += (requestedDelay - delay);
			}
			if (this->resourceUnlimited.at(r)) continue;
			// correct delays for cases where II > #vertices / FUs
			auto maxD = this->maxDelay.at(r);
			auto limit = r->getLimit();
			int tooMany = 0;
			for (int delay=0; delay<maxD; delay++) {
				tooMany += pseudoResourceConstraintDelays[{r, delay}];
				tooMany -= limit;
				if (tooMany >= 0) continue;
				if (!this->quiet) {
					std::cout << "ILPScheduleLengthEstimation::estimateMaxSL: correcting resource limit for delay=" << delay << " from " << pseudoResourceConstraintDelays[{r, delay}] << " to " << limit << std::endl;
				}
				tooMany = 0;
				pseudoResourceConstraintDelays[{r, delay}] = limit;
			}
			if (!this->quiet) {
				std::cout << "ILPScheduleLengthEstimation::estimateMaxSL: delays for resource '" << r->getName() << "':" << std::endl;
				for (int delay=0; delay<maxD; delay++) {
					std::cout << "   delay=" << delay << ": " << pseudoResourceConstraintDelays[{r, delay}] << std::endl;
				}
			}
		}
		int correctionTermSL = 0;
		for (auto &r : this->rm->Resources()) {
			correctionTermSL = std::max(correctionTermSL, delayDiffs[r]);
		}
		if (!this->quiet) {
			std::cout << "ILPScheduleLengthEstimation::estimateMaxSL: SL correction term = " << correctionTermSL << std::endl;
		}

#define SIMPLE_OPT 0
		// calc max latency estimation
		// set up solver
		this->s = std::unique_ptr<ScaLP::Solver>(new ScaLP::Solver(this->sw));
		if (this->threads > 0) {
			this->s->threads = this->threads;
		}
		// variables
		t.clear();
		d.clear();
		p.clear();
		auto objectiveVar = ScaLP::newIntegerVariable("objective", 0.0, ScaLP::INF());
		for (auto &v : this->g->Vertices()) {
			t[v] = ScaLP::newIntegerVariable("t2_"+std::to_string(v->getId()), 0.0, ScaLP::INF());
#if SIMPLE_OPT
			// nothing
#else
			d[v] = ScaLP::newIntegerVariable("d2_"+std::to_string(v->getId()), 0.0, ScaLP::INF());
			auto *r = this->rm->getResource(v);
			if (this->resourceUnlimited.at(r)) continue;
			auto maxD = this->maxDelay.at(r);
			for (int delay=0; delay<maxD; delay++) {
				if (pseudoResourceConstraintDelays[{r, delay}] == 0) continue;
				p[{v, delay}] = ScaLP::newBinaryVariable("p2_"+std::to_string(v->getId())+"_"+std::to_string(delay));
			}
#endif
		}
		// constraints & objective
		for (auto &v : this->g->Vertices()) {
			auto *r = this->rm->getResource(v);
#if 1
			if (this->g->hasNoZeroDistanceOutgoingEdges(v)) {
				this->s->addConstraint(objectiveVar - t[v] >= vertexDelays.at(v) + r->getLatency());
			}
#else
			if (this->g->hasNoZeroDistanceOutgoingEdges(v)) {
				this->s->addConstraint(objectiveVar - t[v] - d[v] - r->getLatency() >= 0);
			}
			if (this->resourceUnlimited.at(r)) {
				this->s->addConstraint(d[v] == 0);
			}
			else {
				ScaLP::Term pSum;
				ScaLP::Term pSumWeighted;
				auto maxD = this->maxDelay.at(r);
				for (int delay=0; delay<maxD; delay++) {
					if (pseudoResourceConstraintDelays.at({r, delay}) == 0) continue;
					pSum += p[{v, delay}];
					pSumWeighted += (delay * p[{v, delay}]);
				}
				this->s->addConstraint(pSum == 1);
				this->s->addConstraint(d[v] - pSumWeighted >= 0);
			}
#endif
		}
		for (auto &e : this->g->Edges()) {
			auto* v = &e->getVertexSrc();
			auto* w = &e->getVertexDst();
			auto* r = this->rm->getResource(v);
			auto l = r->getLatency();
			auto dist = e->getDistance();
			auto delay = e->getDelay();
			auto tV = t.at(v);
			auto tW = t.at(w);
#if 1
			auto rhs = l + delay + vertexDelays.at(v) - (dist * II);
			this->s->addConstraint(tW - tV >= rhs);
#else
			auto rhs = l + delay - (dist * II);
			auto dV = d.at(v);
			this->s->addConstraint(tW - tV - dV >= rhs);
#endif
		}
#if 1
#else
		for (auto &r : this->rm->Resources()) {
			if (this->resourceUnlimited.at(r)) continue;
			auto vertices = this->rm->getVerticesOfResource(r);
			auto maxD = this->maxDelay.at(r);
			for (int delay=0; delay<maxD; delay++) {
				auto numAllowedVertices = pseudoResourceConstraintDelays.at({r, delay});
				if (numAllowedVertices == 0) continue;
				ScaLP::Term pSum;
				for (auto &v : vertices) {
					pSum += p[{v, delay}];
				}
				this->s->addConstraint(pSum <= numAllowedVertices);
			}
		}
#endif
		// objective
		this->s->setObjective(ScaLP::minimize(objectiveVar));
		//if (timeout > 0) {
		//	auto remainingTime = ((int)timeout) - ((int)std::round(elapsedTimeSec));
		//	if (remainingTime < 0) {
		//		this->maxSL = -1;
		//		return;
		//	}
		//	this->s->timeout = remainingTime;
		//}
		this->s->timeout = timeBudget;
		stat = this->s->solve();
		this->timeout = stat == ScaLP::status::TIMEOUT_INFEASIBLE;
		if (stat == ScaLP::status::INFEASIBLE or stat == ScaLP::status::TIMEOUT_INFEASIBLE or
				stat == ScaLP::status::INVALID or stat == ScaLP::status::ERROR or
				stat == ScaLP::status::INFEASIBLE_OR_UNBOUND or stat == ScaLP::status::UNKNOWN) {
			std::cout << "ILPScheduleLengthEstimation::estimateMaxSL: ScaLP failed to find max latency estimation in " << timeBudget << " sec" << std::endl;
			std::cout << "ILPScheduleLengthEstimation::estimateMaxSL: ScaLP status: " << ScaLP::showStatus(stat) << std::endl;
			this->maxSL = -1;
			return;
		}
		solution = this->s->getResult().values;
		for (auto &v : this->g->Vertices()) {
			this->ASAPTimesMaxSL[v] = (int)std::round(solution.at(t.at(v)));
#if 1
			this->ALAPTimesMaxSL[v] = (int)std::round(solution.at(t.at(v)) + vertexDelays.at(v));
#else
			this->ALAPTimesMaxSL[v] = (int)std::round(solution.at(t.at(v))+solution.at(d.at(v)));
#endif
		}
		this->maxSL = correctionTermSL + (int)std::round(solution.at(objectiveVar));
	}

	void ILPScheduleLengthEstimation::calcScheduleFromMaxSLEstimation(const int &II, const int &timeBudget) {
		// clear container
		this->schedule.clear();
		// build MRT
		std::map<std::pair<const Resource*, int>, int> MRT;
		for (auto &r : this->rm->Resources()) {
			if (this->resourceUnlimited.at(r)) continue;
			auto limit = r->getLimit();
			// base MRT
			for (int m=0; m<II; m++) {
				MRT[{r, m}] = limit;
			}
		}
		// factor in vertices that have no delay due to pseudo resource constraint
		for (auto &v : this->g->Vertices()) {
			auto *r = this->rm->getResource(v);
			auto asap = this->ASAPTimesMaxSL.at(v);
			auto alap = this->ALAPTimesMaxSL.at(v);
			std::cout << "Vertex '" << v->getName() << "': " << asap << " <= t <= " << alap << std::endl;
			if (asap != alap) continue;
			this->schedule[v] = asap;
			if (this->resourceUnlimited.at(r)) continue;
			MRT[{r, asap % II}]--;
		}
		if (!this->quiet) {
			for (auto &it : MRT) {
				std::cout << "Resource '" << it.first.first->getName() << "' delay " << it.first.second << ": " << it.second << std::endl;
			}
		}
		// init solver
		this->s = std::unique_ptr<ScaLP::Solver>(new ScaLP::Solver(this->sw));
		if (this->threads > 0) {
			this->s->threads = this->threads;
		}
		// create variables and ensure exactly 1 schedule time assignment
		std::map<std::pair<const Vertex*, int>, ScaLP::Variable> var;
		std::map<std::pair<const Resource*, int>, std::vector<ScaLP::Variable>> varSorted;
		for (auto &v : this->g->Vertices()) {
			if (this->vertexUnlimited.at(v)) continue;
			auto asap = this->ASAPTimesMaxSL.at(v);
			auto alap = this->ALAPTimesMaxSL.at(v);
			if (asap == alap) continue;
			auto *r = this->rm->getResource(v);
			ScaLP::Term term;
			for (int t=asap; t<=alap; t++) {
				auto tempVar = ScaLP::newBinaryVariable("var_"+std::to_string(v->getId())+"_"+std::to_string(t));
				var[{v, t}] = tempVar;
				auto m = t % II;
				varSorted[{r, m}].emplace_back(tempVar);
				term += tempVar;
			}
			this->s->addConstraint(term == 1);
		}
		// resource constraints
		for (auto &r : this->rm->Resources()) {
			if (this->resourceUnlimited.at(r)) continue;
			for (int m=0; m<II; m++) {
				auto resVars = varSorted[{r, m}];
				if (resVars.empty()) continue;
				ScaLP::Term term;
				for (auto &it : resVars) {
					term += it;
				}
				this->s->addConstraint(term <= MRT[{r, m}]);
			}
		}
		// solve
		auto stat = this->s->solve();
		this->timeout = stat == ScaLP::status::TIMEOUT_INFEASIBLE;
		if (stat == ScaLP::status::INFEASIBLE or stat == ScaLP::status::TIMEOUT_INFEASIBLE or
				stat == ScaLP::status::INVALID or stat == ScaLP::status::ERROR or
				stat == ScaLP::status::INFEASIBLE_OR_UNBOUND or stat == ScaLP::status::UNKNOWN) {
			std::cout << "ILPScheduleLengthEstimation::calcScheduleFromMaxSLEstimation: ScaLP failed to find schedule in " << timeBudget << " sec" << std::endl;
			std::cout << "ILPScheduleLengthEstimation::calcScheduleFromMaxSLEstimation: ScaLP status: " << ScaLP::showStatus(stat) << std::endl;
			this->schedule.clear();
			return;
		}
		auto solution = this->s->getResult().values;
		for (auto &v : this->g->Vertices()) {
			if (this->vertexUnlimited.at(v)) continue;
			auto asap = this->ASAPTimesMaxSL.at(v);
			auto alap = this->ALAPTimesMaxSL.at(v);
			if (asap == alap) continue;
			bool foundTime = false;
			for (int t=asap; t<=alap; t++) {
				auto value = (int)std::round(solution.at(var.at({v, t})));
				if (value == 1) {
					this->schedule[v] = t;
					foundTime = true;
				}
			}
			if (not foundTime) {
				std::cout << "ILPScheduleLengthEstimation::calcScheduleFromMaxSLEstimation: failed to find time slot for vertex '" << v->getName() << "' -> that should never happen" << std::endl;
				throw HatScheT::Exception("ILPScheduleLengthEstimation::calcScheduleFromMaxSLEstimation: failed");
			}
		}
	}

	std::map<Vertex *, int> ILPScheduleLengthEstimation::getSchedule() const {
		std::map<Vertex *, int> returnMe;
		if (!this->scheduleFound()) return returnMe;
		for (auto &v : this->g->Vertices()) {
			returnMe[v] = this->schedule.at(v);
		}
		return returnMe;
	}

	std::map<Vertex *, int> ILPScheduleLengthEstimation::getASAPTimesSDC() const {
		std::map<Vertex *, int> returnMe;
		if (this->ASAPTimesSDC.empty()) return returnMe;
		for (auto &v : this->g->Vertices()) {
			returnMe[v] = this->ASAPTimesSDC.at(v);
		}
		return returnMe;
	}

	std::map<Vertex *, int> ILPScheduleLengthEstimation::getALAPTimeDiffsSDC() const {
		std::map<Vertex *, int> returnMe;
		if (this->ALAPTimesSDC.empty()) return returnMe;
		for (auto &v : this->g->Vertices()) {
			returnMe[v] = this->sdcSL - this->ALAPTimesSDC.at(v);
		}
		return returnMe;
	}
}