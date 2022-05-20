//
// Created by nfiege on 03/12/19.
//

#include "ScheduleAndBindingWriter.h"
#include <HatScheT/utility/Exception.h>
#include <fstream>

namespace HatScheT {
	ScheduleAndBindingWriter::ScheduleAndBindingWriter(std::string path, std::vector<std::map<Vertex *, int>> &schedule,
																										 std::vector<std::map<const Vertex *, int>> &binding, int samples,
																										 int modulo, std::vector<fuConnection> fuConnections) :
		Writer(path), schedule(schedule), binding(binding), samples(samples), modulo(modulo), graphPath(""), rmPath(""),
		fuConnections(fuConnections), solvingTime(-1.0), scheduleLength(-1), minNumRegs(-1), minNumRegsChain(-1), minII(-1)
	{}

	ScheduleAndBindingWriter::ScheduleAndBindingWriter(std::string path, std::map<Vertex *, int> &schedule,
																										 std::map<const Vertex *, int> &binding, int II,
																										 std::vector<fuConnection> fuConnections) :
		Writer(path), schedule({schedule}), binding({binding}), samples(1), modulo(II), graphPath(""), rmPath(""),
		fuConnections(fuConnections), solvingTime(-1.0), scheduleLength(-1), minNumRegs(-1), minNumRegsChain(-1), minII(-1)
	{}

	void ScheduleAndBindingWriter::write() {
		// handle errors
		if(this->path.empty()) {
			throw HatScheT::Exception("ScheduleAndBindingWriter::write: specify path -> currently empty");
		}
		/*
		if(this->samples<1) {
			throw HatScheT::Exception("ScheduleAndBindingWriter::write: invalid number of samples provided ("+to_string(this->samples)+")");
		}
		if(this->modulo<1) {
			throw HatScheT::Exception("ScheduleAndBindingWriter::write: invalid cycle length provided ("+to_string(this->modulo)+")");
		}
		 */
		if(this->samples != this->binding.size()) {
			throw HatScheT::Exception("ScheduleAndBindingWriter::write: invalid binding container provided (#samples="+to_string(this->samples)+", binding size="+to_string(this->binding.size())+")");
		}
		if(this->samples != this->schedule.size()) {
			throw HatScheT::Exception("ScheduleAndBindingWriter::write: invalid schedule container provided (#samples="+to_string(this->samples)+", schedule size="+to_string(this->schedule.size())+")");
		}

		// open file
		ofstream file;
		file.open(this->path.c_str());
		if(!file.is_open()) {
			throw HatScheT::Exception("ScheduleAndBindingWriter::write: can't open file at location '"+this->path+"'");
		}

		// container where we store all lines of the csv file before writing them
		std::vector<fileLine> lines;

		// inst line container
		try {
			for(int s=0; s<this->samples; ++s) {
				for(auto &it : this->schedule.at((unsigned int)s)) {
					fileLine line;
					line.v = it.first;
					line.name = it.first->getName();
					line.cycle = it.second;
					line.sample = s;
					line.fu = this->binding.at((unsigned int)s).at(it.first);
					lines.emplace_back(line);
				}
			}
		}
		catch(...) {
			throw HatScheT::Exception("ScheduleAndBindingWriter::write: corrupt schedule/binding provided");
		}

		// header
		file << "# II " << ((double)this->modulo)/((double)this->samples) << std::endl;
		file << "# firstObjectiveOptimal " << this->objectivesOptimal.first << std::endl;
		file << "# secondObjectiveOptimal " << this->objectivesOptimal.second << std::endl;
		if(this->samples!=1) {
			file << "# samples " << this->samples << std::endl;
			file << "# modulo " << this->modulo << std::endl;
		}
		if(!this->graphPath.empty()) {
			file << "# graph " << this->graphPath << std::endl;
		}
		if(!this->graphPath.empty()) {
			file << "# resourceModel " << this->rmPath << std::endl;
		}
		if(this->solvingTime >= 0.0) {
			file << "# solvingTime " << this->solvingTime << std::endl;
		}
		if(this->scheduleLength >= 0) {
			file << "# scheduleLength " << this->scheduleLength << std::endl;
		}
		if (this->minNumRegs >= 0) {
			file << "# minNumRegs " << this->minNumRegs << std::endl;
		}
		if (this->minNumRegsChain >= 0) {
			file << "# minNumRegsChain " << this->minNumRegsChain << std::endl;
		}
		if (this->minII >= 0) {
			file << "# minII " << this->minII << std::endl;
		}
		if(this->samples==1) {
			// integer II -> information about sample can be omitted
			file << "# vertex;cycle;fu" << std::endl;
		}
		else {
			// rational II
			file << "# vertex;sample;cycle;fu" << std::endl;
		}

		// write all lines regarding schedule and fu binding info into file
		for(auto &line : lines) {
			if(this->samples==1) {
				// integer II -> information about sample can be omitted
				file << line.name << ";" << line.cycle << ";" << line.fu << std::endl;
			}
			else {
				// rational II
				file << line.name << ";" << line.sample << ";" << line.cycle << ";" << line.fu << std::endl;
			}
		}

		// new header for fu connections if needed
		if(this->fuConnections.empty()) {
			file.close();
			return;
		}

		file << "# resourceSrc;fuSrc;resourceDst;fuDst;port;lifetimeRegs" << std::endl;
		for(auto &it : this->fuConnections) {
			file << it.resourceSrc << ";" << it.fuSrc << ";" << it.resourceDst << ";" << it.fuDst << ";" << it.port << ";" << it.lifetimeRegs << std::endl;
		}

		// close file
		file.close();
	}
}