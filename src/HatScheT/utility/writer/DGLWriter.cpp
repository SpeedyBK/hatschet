//
// Created by nfiege on 7/5/21.
//

#include "DGLWriter.h"
#include <fstream>
#include <set>

namespace HatScheT {
	DGLWriter::DGLWriter(std::string path, HatScheT::Graph *g, HatScheT::ResourceModel *rm, int II,
		map<Vertex *, int> &schedule) : Writer(path), g(g), rm(rm), II(II), schedule(schedule) {
		// nothing left to do here
	}

	void DGLWriter::write() {
		// calc norm factor
		double normFactor = 0.0;
		for (auto v : this->g->Vertices()) {
			normFactor += this->rm->getResource(v)->getLatency();
		}

		// write node data
		fstream f;
		std::string filename = this->path + "_nodedata.csv";
		f.open(filename);
		f << "II;lat_norm;res;lim;t;lat" << std::endl;
		auto numVertices = this->g->getNumberOfVertices();
		std::set<const Resource*> resources;
		for (int i=0; i<numVertices; i++) {
			auto *v = &this->g->getVertexById(i);
			auto *r = this->rm->getResource(v);
			double limit = r->getLimit();
			if (r->isUnlimited()) {
				limit = 10000.0; // this is more or less inifinity
			}
			resources.insert(r);
			f << (double)this->II / normFactor << ";"; // II (normed)
			f << (double)r->getLatency() / normFactor << ";"; // latency (normed)
			f << (double)(resources.size()-1) / normFactor << ";"; // resource index (normed)
			f << (double)limit / normFactor << ";"; // resource limit (normed)
			f << (double)this->schedule[v] << ";"; // control step (not normed)
			f << (double)r->getLatency() << std::endl; // latency (not normed)
		}
		f.close();

		// write edge data
		filename = this->path + "_edgedat.csv";
		f.open(filename);
		f << "II;lat_norm;res;lim;t;lat" << std::endl;
		for (auto e : this->g->Edges()) {
			if (e->getDependencyType() != Edge::DependencyType::Data) {
				throw HatScheT::Exception("Chaining Edges are not supported yet");
			}
			auto vsrc = e->getVertexSrc().getId();
			auto vdst = e->getVertexDst().getId();
			auto distance = e->getDistance();
			f << (double)vsrc << ";"; // src index (not normed)
			f << (double)vdst << ";"; // dst index (not normed)
			f << (double)distance / normFactor << std::endl; // distance (normed)
		}
		f.close();
	}
}