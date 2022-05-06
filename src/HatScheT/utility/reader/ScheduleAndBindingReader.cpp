//
// Created by nfiege on 6/7/21.
//

#include "ScheduleAndBindingReader.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <sstream>

HatScheT::ScheduleAndBindingReader::ScheduleAndBindingReader(HatScheT::Graph *g, HatScheT::ResourceModel *rm) :
  g(g), rm(rm), modulo(-1), samples(-1), II(-1), schedule(), binding(){
}

void HatScheT::ScheduleAndBindingReader::read(std::string filepath) {
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
		else if(segments.size() == 6) {
			// fu connection info
			fuConnection fuC;
			fuC.resourceSrc = segments[0];
			fuC.fuSrc = stoi(segments[1]);
			fuC.resourceDst = segments[2];
			fuC.fuDst = stoi(segments[3]);
			fuC.port = stoi(segments[4]);
			fuC.lifetimeRegs = stoi(segments[5]);
			this->fuConnections.emplace_back(fuC);
		}
	}

	f.close();
}

std::map<HatScheT::Vertex *, int> HatScheT::ScheduleAndBindingReader::getIntegerIISchedule() {
	if(this->schedule.size() != 1) {
		std::cout << "Did not read an integer-II schedule. Unable to return it" << std::endl;
	}
	return this->schedule[0];
}

std::vector<std::map<HatScheT::Vertex *, int>> HatScheT::ScheduleAndBindingReader::getRationalIISchedule() {
	if(this->schedule.empty()) {
		std::cout << "Did not read a rational-II schedule. Unable to return it" << std::endl;
	}
	return this->schedule;
}

std::map<HatScheT::Vertex *, int> HatScheT::ScheduleAndBindingReader::getIntegerIIBinding() {
	if(this->schedule.size() != 1) {
		std::cout << "Did not read an integer-II binding. Unable to return it" << std::endl;
	}
	return this->binding[0];
}

std::vector<std::map<HatScheT::Vertex *, int>> HatScheT::ScheduleAndBindingReader::getRationalIIBinding() {
	if(this->schedule.empty()) {
		std::cout << "Did not read a rational-II schedule. Unable to return it" << std::endl;
	}
	return this->binding;
}

int HatScheT::ScheduleAndBindingReader::getSamples() {
	return this->samples;
}

int HatScheT::ScheduleAndBindingReader::getModulo() {
	return this->modulo;
}

double HatScheT::ScheduleAndBindingReader::getII() {
	return this->II;
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
}

bool HatScheT::ScheduleAndBindingReader::isRatII() {
	return this->modulo >= 0 and this->samples >= 0;
}

vector<HatScheT::ScheduleAndBindingReader::fuConnection> HatScheT::ScheduleAndBindingReader::getFUConnections() {
	return this->fuConnections;
}
