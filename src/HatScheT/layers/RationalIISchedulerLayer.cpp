/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Martin Kumm, Patrick Sittel ({kumm, sittel}@uni-kassel.de)

    Copyright (C) 2018

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <HatScheT/layers/RationalIISchedulerLayer.h>
#include <HatScheT/utility/Utility.h>
#include <HatScheT/utility/Verifier.h>
#include <cmath>

namespace HatScheT
{

RationalIISchedulerLayer::RationalIISchedulerLayer(Graph &g, ResourceModel &resourceModel, int M, int S) :
	SchedulerBase(g, resourceModel), latencySequence() {
  	this->modulo = -1;
  	this->samples = -1;
	this->m_start = -1;
	this->s_start = -1;
	this->m_found = -1;
	this->s_found = -1;
	this->s_max = -1;

	this->maxLatencyConstraint = -1;
	this->maxRuns = 10;
	this->minRatIIFound = false;
	this->scheduleValid = false;
	this->verifySolution = true;

	pair<int, int> frac;
	if (M <= 0 and S <= 0) {
		// no M/S given by user
		this->computeMinII(&this->g, &this->resourceModel);
		frac = Utility::splitRational(this->minII);
	}
	else {
		// M/S given by user
		this->minII = ((double)M) / ((double)S);
		frac = {M, S};
	}
	this->integerMinII = (int)ceil(this->minII);

	if(!this->quiet) {
		cout << "rational min II is " << this->minII << endl;
		cout << "integer min II is " << this->integerMinII << endl;
		cout << "auto setting samples to " << frac.second << endl;
		cout << "auto setting modulo to " << frac.first << endl;
	}

	this->samples = frac.second;
	this->modulo = frac.first;
}

int RationalIISchedulerLayer::getScheduleLength() {
	int latency = 0;
	for (auto &it : this->startTimesVector) {
		for (auto &it2 : it) {
			auto tEnd = it2.second + this->resourceModel.getVertexLatency(it2.first);
			if (tEnd > latency) latency = tEnd;
		}
	}
	return latency;
}

std::list<pair<int, int>>
RationalIISchedulerLayer::getRationalIIQueue(int sMinII, int mMinII, int integerII, int sMax, int maxListSize) {

	std::list<std::pair<int,int>> moduloSamplePairs; // list of sorted M/S pairs

	if(sMax<0) sMax = sMinII;
	if(maxListSize<0) maxListSize = sMinII * mMinII;
	if(sMinII<=sMax) moduloSamplePairs.emplace_back(std::make_pair(mMinII,sMinII));

	if(maxListSize==1) return moduloSamplePairs;

	// II=4/3=8/6=12/9=... since 4/3 is the easiest to get a schedule for, all other fractions can be skipped!
	std::list<std::pair<int,int>> skipMe = {std::make_pair(mMinII,sMinII)};

	double rationalMinII = double(mMinII) / double(sMinII);

	for(int s=2; s<=sMax; ++s) {
		auto mMin = (int)ceil(rationalMinII * s);
		for(int m=mMin; m<integerII*s; ++m) {
			// check if M/S pair can be skipped
			bool skip = false;
			for(auto it : skipMe) {
				if(it.first == m and it.second == s) {
					skip = true;
					break;
				}
			}
			if(skip) continue;

			// insert all reducable fractions into skipMe
			for(int ss=2*s; ss<=sMax; ss+=s) {
				auto mm = m * (ss/s);
				skipMe.emplace_back(std::make_pair(mm,ss));
			}

			// insert m/s into sorted list
			bool inserted = false;
			for(auto it=moduloSamplePairs.begin(); it!=moduloSamplePairs.end(); ++it) {
				auto mTemp = it->first;
				auto stemp = it->second;
				if(((double)(m) / (double)(s)) < ((double)(mTemp) / (double)(stemp))) {
					moduloSamplePairs.insert(it,std::make_pair(m,s));
					inserted = true;
					break;
				}
			}
			if(!inserted) moduloSamplePairs.emplace_back(std::make_pair(m,s));
		}
	}

	if(moduloSamplePairs.size() > maxListSize) {
		moduloSamplePairs.resize((unsigned int)maxListSize);
	}

	return moduloSamplePairs;
}

	std::vector<int> RationalIISchedulerLayer::getOptimalInitiationIntervalSequence(int samples, int modulo, bool quiet) {
		std::vector<int> initIntervalTemp = {};
		auto lowerII = int(floor(double(modulo)/double(samples)));
		auto upperII = int(ceil(double(modulo)/double(samples)));

		auto noLowerIIs = (upperII * samples) - modulo;
		auto noUpperIIs = modulo - (lowerII * samples);
		if(noLowerIIs + noUpperIIs != samples) {
			std::cout << "number of lower IIs: " << noLowerIIs << std::endl;
			std::cout << "number of upper IIs: " << noUpperIIs << std::endl;
			std::cout << "#samples: " << samples << std::endl;
			std::cout << noLowerIIs << " + " << noUpperIIs << " != " << samples << std::endl;
			throw HatScheT::Exception("Something went wrong while calculating optimal latency sequence - that should never happen");
		}
		bool lowerDom = false;
		auto maxNo = noUpperIIs;
		auto minNo = noLowerIIs;
		if(noLowerIIs>=noUpperIIs) {
			lowerDom = true;
			maxNo = noLowerIIs;
			minNo = noUpperIIs;
		}

		int errAccumulator = 0;
		int latencyCounter = 0;
		if(lowerDom) latencyCounter -= lowerII;
		else latencyCounter -= upperII;

		if(!quiet) {
			std::cout << "  Lower II: " << lowerII << std::endl;
			std::cout << "  Upper II: " << upperII << std::endl;
			std::cout << "  number of lower IIs: " << noLowerIIs << std::endl;
			std::cout << "  number of upper IIs: " << noUpperIIs << std::endl;
			std::cout << "  dominance of lower IIs: " << lowerDom << std::endl;
		}

		for(int i=0; i<maxNo; ++i) {
			if(lowerDom) {
				latencyCounter += lowerII;
			}
			else {
				latencyCounter += upperII;
			}
			initIntervalTemp.emplace_back(latencyCounter);
			errAccumulator += minNo;
			if(!quiet) {
				std::cout << "    errAccumulator: " << errAccumulator << std::endl;
			}
			if(errAccumulator >= maxNo) {
				errAccumulator -= maxNo;
				if(lowerDom) {
					latencyCounter += upperII;
				}
				else {
					latencyCounter += lowerII;
				}
				initIntervalTemp.emplace_back(latencyCounter);
			}
		}

		if(!quiet) {
			std::cout << "IIs: < ";
			for(auto II : initIntervalTemp) {
				std::cout << II << " ";
			}
			std::cout << ">" << std::endl;
		}

		return initIntervalTemp;
	}

	std::vector<int> RationalIISchedulerLayer::getLatencySequenceFromInitiationIntervals(std::vector<int> &initIntervals, int M) {
		std::vector<int> latSeq;
		for(unsigned int i=0; i<initIntervals.size()-1; ++i) {
			latSeq.emplace_back(initIntervals[i + 1] - initIntervals[i]);
		}
		latSeq.emplace_back(M - initIntervals.back());
		return latSeq;
	}

	void RationalIISchedulerLayer::autoSetMAndS() {
		double minII = this->getMinII();
		this->integerMinII = (int)ceil(minII);

		if(this->samples<1 or this->modulo<1) {
			pair<int,int> frac =  Utility::splitRational(minII);
			if(!this->quiet) {
				cout << "------------------------" << endl;
				cout << "RationalIISchedulerLayer.autoSetMAndS: auto setting samples to " << frac.second << endl;
				cout << "RationalIISchedulerLayer.autoSetMAndS: auto setting modulo to " << frac.first << endl;
				cout << "------------------------" << endl;
			}
			this->samples = frac.second;
			this->modulo = frac.first;
		}
	}

	void RationalIISchedulerLayer::schedule() {
		if(!this->quiet) {
			std::cout << "RationalIISchedulerLayer::schedule: start scheduling" << std::endl;
		}
		this->scheduleFound = false;
		this->minRatIIFound = false;
		this->scheduleValid = false;

		//experimental auto set function for the start values of modulo and sample
		this->autoSetMAndS();
		this->s_start = this->samples;
		this->m_start = this->modulo;

		if(!this->quiet) {
			std::cout << "RationalIISchedulerLayer::schedule: maxLatencyConstraint: " << this->maxLatencyConstraint << std::endl;
			std::cout << "RationalIISchedulerLayer::schedule: modulo: " << this->modulo << std::endl;
			std::cout << "RationalIISchedulerLayer::schedule: samples: " << this->samples << std::endl;
		}

		if(this->samples <= 0) {
			throw HatScheT::Exception("RationalIISchedulerLayer.schedule : moduloClasses <= 0! Scheduling not possible!");
		}

		if(this->modulo <= 0) {
			throw HatScheT::Exception("RationalIISchedulerLayer.schedule : consideredModuloCycle <= 0! Scheduling not possible!");
		}

		if(!this->quiet) {
			cout << "------------------------" << endl;
			cout << "RationalIISchedulerLayer:schedule: start for " << this->g.getName() << endl;
			cout << "RationalIISchedulerLayer:schedule: maxLatency " << this->maxLatencyConstraint << endl;
			cout << "RationalIISchedulerLayer::schedule: recMinII is " << this->getRecMinII() << endl;
			cout << "RationalIISchedulerLayer::schedule: resMinII is " << this->getResMinII() << endl;
			cout << "------------------------" << endl;
		}

		// keep track of optimality
		this->firstObjectiveOptimal = true;
		this->secondObjectiveOptimal = true;

		auto msQueue = RationalIISchedulerLayer::getRationalIIQueue(this->s_start,this->m_start,(int)ceil(double(m_start)/double(s_start)),this->s_max,this->maxRuns);
		if(msQueue.empty()) {
			throw HatScheT::Exception("RationalIISchedulerLayer::schedule: empty M / S queue for mMin / sMin="+to_string(this->m_start)+" / "+to_string(this->s_start));
		}

		if(!this->quiet) cout << "RationalIISchedulerLayer::schedule: Found " << msQueue.size() << " valid rat II iteration values using SMax=" << this->s_max << endl;

		for(auto it : msQueue) {
			this->modulo = it.first;
			this->samples = it.second;
			if(!this->quiet) cout << "RationalIISchedulerLayer::schedule: building ilp problem for s / m : " << this->samples << " / " << this->modulo << endl;
			if(!this->quiet) cout << "RationalIISchedulerLayer::schedule: max. no. of iterations " << this->maxRuns << endl;

			this->scheduleIteration();
            updateSolvingTimeTotal();

			if(this->scheduleFound) {
				this->m_found = this->modulo;
				this->s_found = this->samples;
				this->II = (double)(this->modulo) / (double)(this->samples);
				this->minRatIIFound = this->II == this->minII;
				if(!this->verifySolution) break;
				this->scheduleValid = this->verifySchedule();
				if(!this->scheduleValid) {
					std::cout << "RationalIISchedulerLayer::schedule: Scheduler found invalid schedule!" << std::endl;
				}
				break;
			}
			else ++this->timeouts;
		}
	}

	bool RationalIISchedulerLayer::verifySchedule() {
		if(!this->quiet)
			std::cout << "RationalIISchedulerLayer::verifySchedule: start verifier for II=" << this->m_found << "/" << this->s_found << std::endl;

		if(this->g.isEmpty()) return true;

		bool verifyOriginal = verifyRationalIIModuloSchedule(this->g,this->resourceModel,this->startTimesVector,this->samples,this->modulo);

		if(!this->quiet) {
			if(verifyOriginal)
				std::cout << "Rational II schedule for original graph is verified" << std::endl;
			else
				std::cout << "Rational II schedule for original graph is NOT verified" << std::endl;
		}

		return verifyOriginal;
	}

	std::map<Edge *, vector<int> > RationalIISchedulerLayer::getRatIILifeTimes() {
		if(this->startTimesVector.empty()) throw HatScheT::Exception("RationalIISchedulerLayer::getRatIILifeTimes: cant return lifetimes! no startTimes determined!");
		if(this->II <= 0) throw HatScheT::Exception("RationalIIScheduler.getRatIILifeTimes: cant return lifetimes! no II determined!");

		std::map<Edge*,vector<int> > allLifetimes;

		for(auto it = this->g.edgesBegin(); it != this->g.edgesEnd(); ++it){
			Edge* e = *it;
			Vertex* vSrc = &e->getVertexSrc();
			Vertex* vDst = &e->getVertexDst();

			vector<int > lifetimes;

			for(int s = 0; s < this->samples; s++){
				auto sAndO = Utility::getSampleIndexAndOffset(e->getDistance(),s,this->samples,this->modulo);

				int lifetime = this->startTimesVector[s][vDst] - this->startTimesVector[sAndO.first][vSrc]
											 - this->resourceModel.getVertexLatency(vSrc) + sAndO.second;

				if(lifetime < 0) throw HatScheT::Exception("RationalIISchedulerLayer::getRatIILifeTimes: negative lifetime detected!");
				else lifetimes.push_back(lifetime);
			}

			allLifetimes.insert(make_pair(e, lifetimes));
		}
		return allLifetimes;
	}

	vector<std::map<const Vertex *, int> > &RationalIISchedulerLayer::getRationalIIBindings() {
		if(this->ratIIbindings.empty()) {
			this->ratIIbindings = Binding::getSimpleRationalIIBinding(this->startTimesVector,&this->resourceModel,this->modulo,this->samples);
		}
		return this->ratIIbindings;
	}

	void RationalIISchedulerLayer::disableVerifier() {
		this->verifySolution = false;
	}

}
