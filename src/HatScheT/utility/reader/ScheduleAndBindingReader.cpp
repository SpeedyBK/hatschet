//
// Created by nfiege on 6/7/21.
//

#include "ScheduleAndBindingReader.h"
#include <HatScheT/utility/Utility.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <sstream>

HatScheT::ScheduleAndBindingReader::ScheduleAndBindingReader(HatScheT::Graph *g, HatScheT::ResourceModel *rm) :
  g(g), rm(rm), modulo(-1), samples(-1), II(-1), schedule(), binding(){
}

void HatScheT::ScheduleAndBindingReader::read(const std::string &filepath) {
	ifstream f;

	f.open(filepath.c_str());
	if(!f.is_open()) {
		throw HatScheT::Exception(
			"ScheduleAndBindingReader::read: can't open file at location '"+filepath+"'");
	}

	std::string linebuffer;
	while(std::getline(f,linebuffer)) {
		std::cout << linebuffer << std::endl;
		// remove unnecessary chars
		linebuffer.erase(remove(linebuffer.begin(),linebuffer.end(),'\n'),linebuffer.end());
		linebuffer.erase(remove(linebuffer.begin(),linebuffer.end(),'\r'),linebuffer.end());
		// split into words
		std::string segment;
		std::vector<std::string> segments;
		std::stringstream linebufferStream(linebuffer);
		while(std::getline(linebufferStream,segment,';')) {
			segments.emplace_back(segment);
		}
		if(segments.empty()) continue;
		if(segments[0][0] == '#') {
			readHeader(linebuffer);
			continue;
		}
		if(segments.size() == 4 or segments.size() == 3) {
			// schedule/binding info
			if(this->isRatII()) {
				// rational-II schedule
				auto vertexName = segments[0];
				auto sample = stoi(segments[1]);
				auto cycle = stoi(segments[2]);
				auto fu = stoi(segments[3]);
				if(this->schedule.size() <= sample) this->schedule.resize(sample+1);
				if(this->binding.size() <= sample) this->binding.resize(sample+1);
				for(auto v : this->g->Vertices()) {
					if(v->getName() != vertexName) continue;
					this->schedule[sample][v] = cycle;
					this->binding[sample][v] = fu;
				}
			}
			else {
				// integer-II schedule
				auto vertexName = segments[0];
				auto cycle = stoi(segments[1]);
				auto fu = stoi(segments[2]);
				if(this->schedule.empty()) this->schedule.resize(1);
				if(this->binding.empty()) this->binding.resize(1);
				for(auto v : this->g->Vertices()) {
					if(v->getName() != vertexName) continue;
					this->schedule[0][v] = cycle;
					this->binding[0][v] = fu;
				}
			}
		}
	}

	// calc schedule length and validate it if it was set
	int scheduleLengthRead = -1;
	for (auto &v : this->g->Vertices()) {
		if (this->isRatII()) {
			// rational-II case
			for (auto s=0; s<this->samples; s++) {
				auto t = this->schedule.at(s).at(v) + this->rm->getVertexLatency(v);
				if (t > scheduleLengthRead) scheduleLengthRead = t;
			}
		}
		else {
			// integer-II case
			auto t = this->schedule.at(0).at(v) + this->rm->getVertexLatency(v);
			if (t > scheduleLengthRead) scheduleLengthRead = t;
		}
	}
	if (scheduleLengthRead != this->scheduleLength) {
		std::cout << "WARNING! ScheduleAndBindingReader: actual schedule length (" << scheduleLengthRead
		  << ") is not equal to the schedule length in the .csv file (" << this->scheduleLength << ")"
		  << " -> setting schedule length to the actual value!" << std::endl;
	}
	this->scheduleLength = scheduleLengthRead;

	// calc min number of registers to implement this schedule and validate it if it was set
	int minNumRegsRead;
	if (this->isRatII()) {
		minNumRegsRead = Utility::calcMinNumRegs(this->g, this->rm, this->schedule, this->modulo);
	}
	else {
		minNumRegsRead = Utility::calcMinNumRegs(this->g, this->rm, this->schedule[0], (int)this->II);
	}
	if (minNumRegsRead != this->minNumRegs and this->minNumRegs >= 0) {
		std::cout << "WARNING! ScheduleAndBindingReader: actual min #Regs (" << minNumRegsRead
							<< ") is not equal to the min #Regs in the .csv file (" << this->minNumRegs << ")"
							<< " -> setting min #Regs to the actual value!" << std::endl;
	}
	this->minNumRegs = minNumRegsRead;

	f.close();
}

std::map<HatScheT::Vertex *, int> HatScheT::ScheduleAndBindingReader::getIntegerIISchedule() const {
	if(this->schedule.size() != 1) {
		std::cout << "Did not read an integer-II schedule. Unable to return it" << std::endl;
	}
	return this->schedule[0];
}

std::vector<std::map<HatScheT::Vertex *, int>> HatScheT::ScheduleAndBindingReader::getRationalIISchedule() const {
	if(this->schedule.empty()) {
		std::cout << "Did not read a rational-II schedule. Unable to return it" << std::endl;
	}
	return this->schedule;
}

std::map<HatScheT::Vertex *, int> HatScheT::ScheduleAndBindingReader::getIntegerIIBinding() const {
	if(this->schedule.size() != 1) {
		std::cout << "Did not read an integer-II binding. Unable to return it" << std::endl;
	}
	return this->binding[0];
}

std::vector<std::map<HatScheT::Vertex *, int>> HatScheT::ScheduleAndBindingReader::getRationalIIBinding() const {
	if(this->binding.empty()) {
		std::cout << "Did not read a rational-II binding. Unable to return it" << std::endl;
	}
	return this->binding;
}

int HatScheT::ScheduleAndBindingReader::getSamples() const {
	return this->samples;
}

int HatScheT::ScheduleAndBindingReader::getModulo() const {
	return this->modulo;
}

void HatScheT::ScheduleAndBindingReader::readHeader(std::string &line) {
	std::string segment;
	std::vector<std::string> segments;
	std::stringstream linebufferStream(line);
	while(std::getline(linebufferStream,segment,' ')) {
		segments.emplace_back(segment);
	}
	if(segments.size() != 3) {
		// ignore this line
		return;
	}
	auto key = segments[1];
	auto arg = segments[2];
	if(key == "II") this->II = stod(arg);
	if(key == "modulo") this->modulo = stoi(arg);
	if(key == "samples") this->samples = stoi(arg);
	if(key == "scheduleLength") this->scheduleLength = stoi(arg);
	if(key == "minNumRegs") this->minNumRegs = stoi(arg);
}

bool HatScheT::ScheduleAndBindingReader::isRatII() const {
	return this->modulo >= 0 and this->samples >= 0;
}

double HatScheT::ScheduleAndBindingReader::getII() const {
	return this->II;
}

int HatScheT::ScheduleAndBindingReader::getScheduleLength() const {
	return this->scheduleLength;
}

int HatScheT::ScheduleAndBindingReader::getMinNumRegs() const {
	return this->minNumRegs;
}
