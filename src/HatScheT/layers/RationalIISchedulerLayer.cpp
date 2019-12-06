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

RationalIISchedulerLayer::RationalIISchedulerLayer(Graph &g, ResourceModel &resourceModel) :
	SchedulerBase(g, resourceModel), latencySequence() {
  this->modulo = -1;
  this->samples = -1;
	this->m_start = -1;
	this->s_start = -1;
	this->m_found = -1;
	this->s_found = -1;
	this->s_max = -1;

	this->maxLatencyConstraint = -1;
	this->maxRuns = 1;
	this->minRatIIFound = false;
	this->scheduleValid = false;

	this->computeMinII(&this->g, &this->resourceModel);
	this->integerMinII = (int)ceil(this->minII);

	pair<int, int> frac = Utility::splitRational(this->minII);

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

	for(auto it : this->startTimesVector) {
		int min = -1;
		int max = 0;

		for(auto it2 : it) {
		  //init
			if(min == -1) min = it2.second;

			if(it2.second < min)
				min = it2.second;
			if(it2.second + this->resourceModel.getVertexLatency(it2.first) > max)
				max = it2.second + this->resourceModel.getVertexLatency(it2.first);
		}

		int l = max - min;

		if(l > latency) latency = l;
	}

	return latency;
}

std::list<pair<int, int>>
RationalIISchedulerLayer::getRationalIIQueue(int sMinII, int mMinII, int integerII, int sMax, int maxListSize) {

	std::list<std::pair<int,int>> moduloSamplePairs; // list of sorted M/S pairs

	if(sMax<0) sMax = sMinII;
	if(maxListSize<0) maxListSize = sMinII * mMinII;
	/*if(sMinII<=sMax)*/ moduloSamplePairs.emplace_back(std::make_pair(mMinII,sMinII));

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

	void RationalIISchedulerLayer::autoSetMAndS() {
		double minII = this->getMinII();
		this->integerMinII = (int)ceil(minII);

		if(this->samples<1 or this->modulo<1) {
			pair<int,int> frac =  Utility::splitRational(minII);
			if(!this->quiet) {
				cout << "------------------------" << endl;
				cout << "RationalIISchedulerLayer.autoSetMAndS: auto setting samples to " << frac.second << endl;
				cout << "RationalIISchedulerLayer.autoSetMAndS:auto setting modulo to " << frac.first << endl;
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

		auto msQueue = RationalIISchedulerLayer::getRationalIIQueue(this->s_start,this->m_start,(int)ceil(double(m_start)/double(s_start)),this->s_max,this->maxRuns);
		if(msQueue.empty()) {
			throw HatScheT::Exception("RationalIISchedulerLayer::schedule: empty M / S queue for mMin / sMin="+to_string(this->m_start)+" / "+to_string(this->s_start));
		}

		cout << "RationalIISchedulerLayer::schedule: Found " << msQueue.size() << " valid rat II iteration values using SMax=" << this->s_max << endl;

		for(auto it : msQueue) {
			this->modulo = it.first;
			this->samples = it.second;
			/*if(!this->quiet)*/ cout << "RationalIISchedulerLayer::schedule: building ilp problem for s / m : " << this->samples << " / " << this->modulo << endl;
			/*if(!this->quiet)*/ cout << "RationalIISchedulerLayer::schedule: max. no. of iterations " << this->maxRuns << endl;

			this->scheduleIteration();

			if(this->scheduleFound) {
				this->m_found = this->modulo;
				this->s_found = this->samples;
				this->II = (double)(this->modulo) / (double)(this->samples);
				this->minRatIIFound = this->II == this->minII;
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
		if(this->g.isEmpty()) return true;
		////////////////////////////////////////////////////////////////////
		// VERIFY UNROLLED GRAPH WITH INTEGER II MODULO SCHEDULE VERIFIER //
		////////////////////////////////////////////////////////////////////

		// unroll graph and create corresponding resource model
		Graph g_unroll;
		ResourceModel rm_unroll;

		for(auto res : this->resourceModel.Resources()) {
			rm_unroll.makeResource(res->getName(),res->getLimit(),res->getLatency(),res->getBlockingTime());
		}

		for(auto v : this->g.Vertices()) {
			for(int s=0; s<this->samples; ++s) {
				auto& newVertex = g_unroll.createVertex();
				newVertex.setName(v->getName()+"_"+to_string(s));
				auto originalResource = this->resourceModel.getResource(v);
				rm_unroll.registerVertex(&newVertex,rm_unroll.getResource(originalResource->getName()));
			}
		}

		for(auto e : this->g.Edges()) {
			auto srcName = e->getVertexSrc().getName();
			auto dstName = e->getVertexDst().getName();

			int distance = e->getDistance();
			int offset = 0;

			// adjust distance/offset so distance < this->samples
			while(distance>this->samples) {
				distance -= this->samples;
				++offset;
			}

			for(int s=0; s<this->samples; ++s) {
				// adjust distance again (only once)
				int sourceSampleNumber = s - distance;
				int edgeOffset = offset;
				if(sourceSampleNumber < 0) {
					sourceSampleNumber += this->samples;
					++edgeOffset;
				}

				// create edge
				Vertex* srcVertex = nullptr;
				Vertex* dstVertex = nullptr;

				for(auto v : g_unroll.Vertices()) {
					if(v->getName() == srcName + "_" + to_string(sourceSampleNumber))
						srcVertex = v;
					if(v->getName() == dstName + "_" + to_string(s))
						dstVertex = v;
				}

				g_unroll.createEdge(*srcVertex,*dstVertex,edgeOffset,e->getDependencyType());
			}
		}

		std::map<Vertex*, int> unrolledSchedule;

		for(unsigned int s=0; s<this->samples; ++s) {
			for(auto it : this->startTimesVector[s]) {

				Vertex* v = nullptr;

				for(auto vIt : g_unroll.Vertices()) {
					if(vIt->getName() == it.first->getName() + "_" + to_string(s))
						v = vIt;
				}

				unrolledSchedule[v] = it.second;
			}
		}

		if(!this->quiet) {
			std::cout << "RationalIISchedulerLayer::verifySchedule: UNROLLED GRAPH:" << std::endl;
			std::cout << g_unroll << std::endl;
			std::cout << "RationalIISchedulerLayer::verifySchedule: UNROLLED RESOURCE MODEL:" << std::endl;
			std::cout << rm_unroll << std::endl;
		}

		bool verifyUnrolled = verifyModuloSchedule(g_unroll,rm_unroll,unrolledSchedule,this->modulo);

		bool verifyOriginal = verifyRationalIIModuloSchedule(this->g,this->resourceModel,this->startTimesVector,this->samples,this->modulo);

		if(!this->quiet) {
			if(verifyUnrolled)
				std::cout << "Integer II schedule for unrolled graph is verified" << std::endl;
			else
				std::cout << "Integer II schedule for unrolled graph is NOT verified" << std::endl;

			if(verifyOriginal)
				std::cout << "Rational II schedule for original graph is verified" << std::endl;
			else
				std::cout << "Rational II schedule for original graph is NOT verified" << std::endl;
		}

		if(verifyOriginal != verifyUnrolled) {
			std::cout << "RationalIISchedulerLayer::verifySchedule: ATTENTION! Verifier for unrolled graph is not identical to rational II verifier! Rational II verifier is buggy!" << std::endl;
		}

		return verifyUnrolled and verifyOriginal;
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

}
