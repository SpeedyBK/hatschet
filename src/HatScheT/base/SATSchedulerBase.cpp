//
// Created by bkessler on 11/8/22.
//

#include "SATSchedulerBase.h"
#include <cmath>

namespace HatScheT {

	SATSchedulerBase::SATSchedulerBase(Graph &g, ResourceModel &resourceModel, int II) :
		IterativeModuloSchedulerLayer(g, resourceModel, II), terminator(0.0),
		los(LatencyOptimizationStrategy::REVERSE_LINEAR), linearJumpLength(-1), latencyLowerBound(-1),
		latencyUpperBound(-1), enableIIBasedLatencyLowerBound(true) {

	}

	void SATSchedulerBase::setLatencyOptimizationStrategy(const SATSchedulerBase::LatencyOptimizationStrategy &newLos) {
		this->los = newLos;
	}

	void SATSchedulerBase::setTargetLatency(const int &newTargetLatency) {
		this->minLatency = newTargetLatency;
		this->maxLatency = newTargetLatency;
		this->enableIIBasedLatencyLowerBound = false;
		this->minLatencyUserDef = true;
		this->maxLatencyUserDef = true;
	}

	void SATSchedulerBase::setEarliestStartTimes(const map<Vertex *, int> &newEarliestStartTimes) {
		bool foundAll = true;
		for (auto &v : this->g.Vertices()) {
			if (newEarliestStartTimes.find(v) == newEarliestStartTimes.end()) {
				std::cout << "SATSchedulerBase::setEarliestStartTimes: Warning: failed to find earliest start time for vertex '" << v->getName() << "' -> skipping assignment!" << std::endl;
				foundAll = false;
			}
		}
		if (!foundAll) return;
		this->earliestStartTime = newEarliestStartTimes;
	}

	void SATSchedulerBase::setLatestStartTimeDifferences(const map<Vertex *, int> &newLatestStartTimeDifferences) {
		bool foundAll = true;
		for (auto &v : this->g.Vertices()) {
			if (newLatestStartTimeDifferences.find(v) == newLatestStartTimeDifferences.end()) {
				std::cout << "SATSchedulerBase::setLatestStartTimeDifferences: Warning: failed to find earliest start time for vertex '" << v->getName() << "' -> skipping assignment!" << std::endl;
				foundAll = false;
			}
		}
		if (!foundAll) return;
		this->latestStartTimeDifferences = newLatestStartTimeDifferences;
	}

	bool SATSchedulerBase::computeNewLatencySuccess(const bool &lastSchedulingAttemptSuccessful) {
		if (!this->quiet) {
			std::cout << "SATSchedulerBase: computing new latency for last candidate latency=" << this->candidateLatency
								<< ", min latency=" << this->latencyLowerBound << ", max latency=" << this->latencyUpperBound
								<< " and last attempt success=" << lastSchedulingAttemptSuccessful << std::endl;
		}
		if (this->linearJumpLength < 0) {
			this->linearJumpLength = (int)std::ceil(std::sqrt(this->latencyUpperBound - this->latencyLowerBound) / 2.0);
		}
		// support a disabled second objective
		if (this->disableSecObj) {
			if (this->candidateLatency < 0) {
				this->candidateLatency = this->latencyUpperBound;
				return true;
			}
			else {
				return false;
			}
		}
		switch (this->los) {
			case REVERSE_LINEAR: {
				if (this->candidateLatency < 0) {
					// first attempt: try maximum latency
					this->candidateLatency = this->latencyUpperBound;
					return true;
				}
				else if (lastSchedulingAttemptSuccessful) {
					// last scheduling attempt was a success
					if (this->candidateLatency <= this->latencyLowerBound) {
						// and we reached the lower bound
						// -> we found the optimum
						return false;
					}
					else {
						// and we did not yet reach the lower bound
						// -> try again with the next latency...
						this->candidateLatency--;
						return true;
					}
				}
				else {
					// last scheduling attempt was a fail
					// -> II is either infeasible or we found the optimum
					return false;
				}
			}
			case LINEAR: {
				if (this->candidateLatency < 0) {
					// first attempt: try minimum latency
					this->candidateLatency = this->latencyLowerBound;
					return true;
				}
				else if (lastSchedulingAttemptSuccessful) {
					// last scheduling attempt was a success
					// -> we found the optimum
					return false;
				}
				else if (this->candidateLatency < this->latencyUpperBound) {
					// last scheduling attempt was a fail
					// and we did not reach the upper bound
					// -> try again with the next latency...
					this->candidateLatency++;
					return true;
				}
				else {
					// last scheduling attempt was a fail
					// and we reached the upper bound
					// -> II is infeasible
					return false;
				}
			}
			case LINEAR_JUMP: {
				if (this->candidateLatency < 0) {
					// first attempt: try the first jump after the minimum latency
					// because in practice, the minimum latency is only very rarely achievable
					this->candidateLatency = this->latencyLowerBound + this->linearJumpLength;
					if (this->candidateLatency > this->latencyUpperBound) this->candidateLatency = this->latencyUpperBound;
				}
				else if (lastSchedulingAttemptSuccessful) {
					// check if we found the optimum
					if (this->candidateLatency <= this->latencyLowerBound) return false;
					// we already got a valid solution
					// adjust upper bound
					this->latencyUpperBound = this->candidateLatency;
					// decrease latency to find optimum
					this->candidateLatency--;
				}
				else {
					// check if we found the optimum
					if (this->scheduleFound) return false;
					// check if II is infeasible
					if (this->candidateLatency >= this->latencyUpperBound) return false;
					// looks like we are still searching for a valid solution
					// adjust lower bound
					this->latencyLowerBound = this->candidateLatency+1;
					this->candidateLatency += this->linearJumpLength;
					if (this->candidateLatency > this->latencyUpperBound) this->candidateLatency = this->latencyUpperBound;
				}
				return true;
			}
			case LINEAR_JUMP_LOG: {
				// this is the same as the linear jump one, but we are using a logarithmic search
				// once we have found a solution (i.e., a "good" upper bound on the latency)
				if (this->candidateLatency < 0) {
					// first attempt: try minimum latency
					this->candidateLatency = this->latencyLowerBound;
					return true;
				}
				else if (this->scheduleFound) {
					// we got a solution -> do binary search to find optimum
					if (lastSchedulingAttemptSuccessful) {
						// last scheduling attempt was a success
						this->latencyUpperBound = this->candidateLatency;
						// -> floor(mean(lower bound, last latency))
						this->candidateLatency = floor(((double)this->candidateLatency + (double)this->latencyLowerBound) / 2.0);
					}
					else {
						// last scheduling attempt was a fail
						this->latencyLowerBound = this->candidateLatency+1;
						// -> ceil(mean(upper bound, last latency))
						this->candidateLatency = ceil(((double)this->candidateLatency + (double)this->latencyUpperBound) / 2.0);
					}
					auto successPair = this->latencyAttempts.insert(this->candidateLatency);
					return successPair.second;
				}
				else {
					// we do not have a solution, yet
					// check if II is infeasible
					if (this->candidateLatency >= this->latencyUpperBound) return false;
					// looks like we are still searching for a valid solution
					// -> keep jumping, baby
					// adjust lower bound
					this->latencyLowerBound = this->candidateLatency+1;
					this->candidateLatency += this->linearJumpLength;
					if (this->candidateLatency > this->latencyUpperBound) this->candidateLatency = this->latencyUpperBound;
					return true;
				}
			}
			case LOGARITHMIC: {
				if (this->candidateLatency < 0) {
					// first attempt: try minimum latency
					this->candidateLatency = this->latencyLowerBound;
				}
				else if (lastSchedulingAttemptSuccessful) {
					// last scheduling attempt was a success
					this->latencyUpperBound = this->candidateLatency;
					// -> floor(mean(lower bound, last latency))
					this->candidateLatency = floor(((double)this->candidateLatency + (double)this->latencyLowerBound) / 2.0);
				}
				else {
					// last scheduling attempt was a fail
					this->latencyLowerBound = this->candidateLatency;
					// -> ceil(mean(upper bound, last latency))
					this->candidateLatency = ceil(((double)this->candidateLatency + (double)this->latencyUpperBound) / 2.0);
				}
				auto successPair = this->latencyAttempts.insert(this->candidateLatency);
				return successPair.second;
			}
			case NO_LATENCY_OPTIMIZATION: {
				if (this->candidateLatency < 0) {
					this->candidateLatency = this->latencyUpperBound;
					return true;
				}
				else return false;
			}
		}
		// something went wrong ... ABORT!
		return false;
	}

	void SATSchedulerBase::scheduleIteration() {
		this->candidateII = (int)this->II;
		this->literalCounter = 0;
		this->clauseCounter = 0;
		this->secondObjectiveOptimal = true;
	}

	void SATSchedulerBase::scheduleInit() {
		if (!this->quiet) {
			std::cout << "SATSchedulerBase: start initializing scheduler" << std::endl;
		}
		// store info about resources
		for (auto &v : this->g.Vertices()) {
			auto r = this->resourceModel.getResource(v);
			this->vertexIsUnlimited[v] = r->isUnlimited();
			if (r->isUnlimited()) {
				this->resourceLimit[v] = (int)this->resourceModel.getNumVerticesRegisteredToResource(r);
			}
			else {
				this->resourceLimit[v] = r->getLimit();
			}
		}
		// simplify resource limits to save variables/clauses
		this->simplifyResourceLimits();
	}

	void SATSchedulerBase::scheduleCleanup() {
		this->restoreResourceLimits();
	}

	void SATSchedulerBase::simplifyResourceLimits() {
		for (auto &r : this->resourceModel.Resources()) {
			auto limit = r->getLimit();
			if (limit == UNLIMITED) continue; // skip unlimited resources because this is the dream scenario anyways
			auto numVertices = this->resourceModel.getNumVerticesRegisteredToResource(r);
			if (numVertices <= limit) {
				// save original limit to restore it later
				this->originalResourceLimits[r] = limit;
				// resource limit can be ignored
				r->setLimit(UNLIMITED, false);
			}
		}
	}

	void SATSchedulerBase::restoreResourceLimits() {
		for (auto &r : this->resourceModel.Resources()) {
			if (this->originalResourceLimits.find(r) == this->originalResourceLimits.end()) continue; // limit was not changed
			auto lim = this->originalResourceLimits.at(r);
			r->setLimit(lim, false);
		}
	}
}