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
#include <cmath>

namespace HatScheT
{

RationalIISchedulerLayer::RationalIISchedulerLayer(Graph &g, ResourceModel &resourceModel) : SchedulerBase(g, resourceModel) {
  this->modulo = -1;
  this->samples = -1;
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

	std::list<std::pair<int,int>> moduloSamplePairs = {std::make_pair(mMinII,sMinII)}; // list of sorted M/S pairs
	if(maxListSize==1) return moduloSamplePairs;

	// II=4/3=8/6=12/9=... since 4/3 is the easiest to get a schedule for, all other fractions can be skipped!
	std::list<std::pair<int,int>> skipMe = {std::make_pair(mMinII,sMinII)};

	if(sMax<0) sMax = sMinII;
	if(maxListSize<0) maxListSize = sMinII * mMinII;

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
				if(double(m) / double(s) < double(mTemp) / double(stemp)) {
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

}
