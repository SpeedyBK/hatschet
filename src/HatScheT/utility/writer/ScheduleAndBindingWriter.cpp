//
// Created by nfiege on 03/12/19.
//

#include "ScheduleAndBindingWriter.h"
#include <HatScheT/utility/Exception.h>
#include <fstream>

namespace HatScheT {
	ScheduleAndBindingWriter::ScheduleAndBindingWriter(std::string path, std::vector<std::map<Vertex *, int>> &schedule,
																										 std::vector<std::map<const Vertex *, int>> &binding, int samples,
																										 int modulo) :
		Writer(path), schedule(schedule), binding(binding), samples(samples), modulo(modulo), graphPath(""), rmPath("") {

	}

	ScheduleAndBindingWriter::ScheduleAndBindingWriter(std::string path, std::map<Vertex *, int> &schedule,
																										 std::map<const Vertex *, int> &binding, int II) :
		Writer(path), schedule({schedule}), binding({binding}), samples(1), modulo(II), graphPath(""), rmPath("") {

	}

	void ScheduleAndBindingWriter::write() {
		// handle errors
		if(this->path.empty()) {
			throw HatScheT::Exception("ScheduleAndBindingWriter::write: specify path -> currently empty");
		}
		if(this->samples<1) {
			throw HatScheT::Exception("ScheduleAndBindingWriter::write: invalid number of samples provided ("+to_string(this->samples)+")");
		}
		if(this->modulo<1) {
			throw HatScheT::Exception("ScheduleAndBindingWriter::write: invalid cycle length provided ("+to_string(this->modulo)+")");
		}
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
		if(this->samples==1) {
			// integer II -> information about sample can be omitted
			file << "# vertex;cycle;fu" << std::endl;
		}
		else {
			// rational II
			file << "# vertex;sample;cycle;fu" << std::endl;
		}

		// write all lines into file
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

		// close file
		file.close();
	}
}