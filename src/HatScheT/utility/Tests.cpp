/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)

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

#include <ctime>
#include <iomanip>
#include "HatScheT/utility/Tests.h"
#include "HatScheT/utility/TreeBind.h"
#include "HatScheT/utility/OptimalIntegerIIGeneralizedBinding.h"
#include "HatScheT/utility/reader/GraphMLGraphReader.h"
#include "HatScheT/utility/reader/XMLResourceReader.h"
#include "HatScheT/utility/reader/XMLTargetReader.h"
#include "HatScheT/utility/writer/GraphMLGraphWriter.h"
#include "HatScheT/scheduler/ilpbased/MoovacMinRegScheduler.h"
#include "HatScheT/scheduler/ilpbased/ModuloSDCScheduler.h"
#include "HatScheT/scheduler/dev/IntegerIINonRectScheduler.h"
#include "HatScheT/scheduler/graphBased/PBScheduler.h"
#include "HatScheT/scheduler/ilpbased/EichenbergerDavidson97Scheduler.h"
#include "HatScheT/scheduler/ilpbased/RationalIIScheduler.h"
#include "HatScheT/scheduler/dev/UniformRationalIIScheduler.h"
#include "HatScheT/scheduler/dev/UniformRationalIISchedulerNew.h"
#include "HatScheT/scheduler/dev/NonUniformRationalIIScheduler.h"
#include "HatScheT/scheduler/dev/UnrollRationalIIScheduler.h"
#include "HatScheT/scheduler/ilpbased/RationalIISchedulerFimmel.h"
#include "HatScheT/ResourceModel.h"
#include "HatScheT/utility/writer/DotWriter.h"
#include "HatScheT/utility/writer/XMLResourceWriter.h"
#include "HatScheT/scheduler/ASAPScheduler.h"
#include "HatScheT/scheduler/ALAPScheduler.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/subgraphs/OccurrenceSetCombination.h"
#include "HatScheT/scheduler/graphBased/SGMScheduler.h"
#include "HatScheT/scheduler/ULScheduler.h"
#include "HatScheT/utility/Verifier.h"
#include "HatScheT/scheduler/dev/ModSDC.h"
#include "HatScheT/utility/subgraphs/KosarajuSCC.h"
#include "HatScheT/utility/subgraphs/SCC.h"
#include "HatScheT/scheduler/dev/DaiZhang19Scheduler.h"
#include "HatScheT/scheduler/dev/SDSScheduler.h"
#include "HatScheT/utility/FibonacciHeap.h"
#include "HatScheT/utility/SDCSolver.h"
#include "HatScheT/utility/writer/ScheduleAndBindingWriter.h"
#include <HatScheT/scheduler/ilpbased/RationalIIScheduler.h>
#include <HatScheT/scheduler/dev/ModuloQScheduler.h>
#include <HatScheT/scheduler/dev/SCCQScheduler.h>
#include <stdio.h>
#include <math.h>
#include <HatScheT/scheduler/dev/RationalIIModuloSDCScheduler.h>
#include <HatScheT/scheduler/dev/CombinedRationalIIScheduler.h>
#include <HatScheT/scheduler/dev/MinRegMultiScheduler.h>
#include <HatScheT/scheduler/satbased/SATScheduler.h>
#include <HatScheT/scheduler/satbased/SATMinRegScheduler.h>
#include <HatScheT/scheduler/dev/SMT/SMTModScheduler.h>

#ifdef USE_CADICAL
#include "cadical.hpp"
#endif

#ifdef USE_Z3
#include <z3++.h>
#endif


namespace HatScheT {

	bool Tests::moovacTest() {
#ifndef USE_XERCESC
		return true;
#else
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;
		HatScheT::XMLResourceReader readerRes(&rm);

		string resStr = "cTest/MoovacExampleRM.xml";
		string graphStr = "cTest/MoovacExample.graphml";
		readerRes.readResourceModel(resStr.c_str());

		HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
		readerGraph.readGraph(graphStr.c_str());

		if (g.getNumberOfVertices() != 6) {
			cout << "graph not read correctly! expected 6 vertices but got " << g.getNumberOfVertices() << endl;
			return false;
		}

		int maxLatencyConstraint = 18;
		HatScheT::MoovacScheduler sched(g, rm, {"CPLEX", "Gurobi", "SCIP", "LPSolve"});
		sched.setMaxLatencyConstraint(maxLatencyConstraint);
		sched.setSolverQuiet(false);

		cout << "starting moovac scheduling" << endl;
		sched.schedule();
		cout << "finished moovac scheduling" << endl;

		if (sched.getII() != 5) {
			cout << "Wrong II determined: " << sched.getII() << " instead of 5!" << endl;
			return false;
		}

		if (sched.getNoOfImplementedRegisters() != 9) {
			cout << "Wrong number of registers determined: " << sched.getNoOfImplementedRegisters() << " instead of 9!"
					 << endl;
			return false;
		}

		return true;
#endif
	}

	bool Tests::asapHCTest() {
#ifndef USE_XERCESC
		return true;
#else
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;
		HatScheT::XMLResourceReader readerRes(&rm);

		string resStr = "cTest/ASAPHCExampleRM.xml";
		string graphStr = "cTest/ASAPHCExample.graphml";
		readerRes.readResourceModel(resStr.c_str());

		HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
		readerGraph.readGraph(graphStr.c_str());

		HatScheT::ASAPScheduler asap(g, rm);
		asap.schedule();

		if (HatScheT::verifyModuloSchedule(g, rm, asap.getSchedule(), asap.getII())) return true;

		cout << "Tests::asapHCTest: Test Failed! Schedule is: " << endl;
		cout << g << endl;
		cout << rm << endl;
		HatScheT::Utility::printSchedule(asap.getSchedule());
		cout << "Tests::asapHCTest: asap HC failed verification!" << endl;

		return false;
#endif
	}

	bool Tests::alapHCTest() {
#ifndef USE_XERCESC
		return true;
#else
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;
		HatScheT::XMLResourceReader readerRes(&rm);

		string resStr = "cTest/ASAPHCExampleRM.xml";
		string graphStr = "cTest/ASAPHCExample.graphml";
		readerRes.readResourceModel(resStr.c_str());

		HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
		readerGraph.readGraph(graphStr.c_str());

		HatScheT::ALAPScheduler alap(g, rm);
		alap.schedule();

		if (HatScheT::verifyModuloSchedule(g, rm, alap.getSchedule(), alap.getII())) return true;

		cout << "Tests::alapHCTest: Test Failed! Schedule is: " << endl;
		cout << g << endl;
		cout << rm << endl;
		HatScheT::Utility::printSchedule(alap.getSchedule());
		cout << "Tests::alapHCTest: alap HC failed verification!" << endl;
		return false;
#endif
	}

	bool Tests::rationalMinIITest() {
#ifndef USE_XERCESC
		return true;
#else
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;
		HatScheT::XMLResourceReader readerRes(&rm);

		string resStr = "cTest/vanDongenRM.xml";
		string graphStr = "cTest/vanDongen.graphml";

		readerRes.readResourceModel(resStr.c_str());

		HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
		readerGraph.readGraph(graphStr.c_str());

		double resMinII = HatScheT::Utility::calcResMII(&rm);
		double recMinII = HatScheT::Utility::calcRecMII(&g, &rm);

		double epsilon1 = resMinII - ((double) 10) / ((double) 3);
		double epsilon2 = recMinII - ((double) 16) / ((double) 3);

		if (epsilon1 > 0.001f or epsilon1 < -0.001f) {
			cout << "resMinII is: " << to_string(resMinII) << " instead of 10/3 =" << to_string(((double) 10) / ((double) 3))
					 << endl;
			return false;
		}
		if (epsilon2 > 0.001f or epsilon2 < -0.001f) {
			cout << "recMinII is: " << to_string(recMinII) << " instead of 16/3 =" << to_string(((double) 16) / ((double) 3))
					 << endl;
			return false;
		}

		return true;
#endif
	}

	bool Tests::readWriteReadScheduleTest() {
#ifndef USE_XERCESC
		return true;
#else
		string resStr = "cTest/MoovacExampleRM.xml";
		string graphStr = "cTest/MoovacExample.graphml";

		HatScheT::ResourceModel rm;
		HatScheT::Graph g;

		//reader
		HatScheT::XMLResourceReader readerRes(&rm);
		readerRes.readResourceModel(resStr.c_str());
		HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
		readerGraph.readGraph(graphStr.c_str());

		//moovac original
		HatScheT::MoovacScheduler ms1(g, rm, {"CPLEX", "Gurobi", "SCIP", "LPSolve"});
		ms1.schedule();
		int IIOrg = ms1.getII();

		//writer
		string writePath = "test.graphml";
		string writeResourcePath = "test.xml";
		HatScheT::XMLResourceWriter writerResource(writeResourcePath, &rm);
		writerResource.write();
		HatScheT::GraphMLGraphWriter writerGraph(writePath, &g, &rm);
		writerGraph.write();

		//reader 2
		HatScheT::Graph g2;
		HatScheT::ResourceModel rm2;
		HatScheT::XMLResourceReader readerRes2(&rm2);
		readerRes2.readResourceModel(writeResourcePath.c_str());
		HatScheT::GraphMLGraphReader readerGraph2(&rm2, &g2);
		readerGraph2.readGraph(writePath.c_str());

		//moovac write read graph
		HatScheT::MoovacScheduler ms2(g2, rm2, {"CPLEX", "Gurobi", "SCIP", "LPSolve"});
		ms2.schedule();
		int IIWriteRead = ms2.getII();

		//cleanup
		if (remove("test.graphml") != 0) {
			cout << "Test.readWriteReadScheduleTest: Error deleting File during cleanup process: test.graphml!" << endl;
			return false;
		}
		if (remove("test.xml") != 0) {
			cout << "Test.readWriteReadScheduleTest: Error deleting File during cleanup process: test.xml!" << endl;
			return false;
		}

		if (IIOrg != IIWriteRead) {
			cout << "Test.readWriteReadScheduleTest: Error after write and read a differen II was determined!" << endl;
			cout << "Test.readWriteReadScheduleTest: Org II " << IIOrg << endl;
			cout << "Test.readWriteReadScheduleTest: WriteRead II " << IIWriteRead << endl;
			return false;
		}

		return true;
#endif
	}

	bool Tests::cpTest() {
#ifndef USE_XERCESC
		return true;
#else
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;
		HatScheT::XMLResourceReader readerRes(&rm);

		string resStr = "cTest/ASAPHCExampleRM.xml";
		string graphStr = "cTest/ASAPHCExample.graphml";
		readerRes.readResourceModel(resStr.c_str());

		HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
		readerGraph.readGraph(graphStr.c_str());

		int cp = Utility::getCriticalPath(&g, &rm);

		if (cp != 8) {
			cout << "Tests.cpTest: critital path calculated: " << cp << endl;
			cout << "Tests.cpTest: the correct value is 8! " << endl;
			return false;
		}
		for (auto it = rm.resourcesBegin(); it != rm.resourcesEnd(); ++it) {
			Resource *r = *it;
			if (r->isUnlimited() == true) {
				cout << "Tests.cpTest: unlimited resource detected after critical path calculation: " << r->getName() << endl;
				cout << "Tests.cpTest: This should never happen! Old limits should be restored by the utility function!"
						 << endl;
				return false;
			}
		}
		return true;
#endif
	}

	bool Tests::readTest() {
#ifndef USE_XERCESC
		return true;
#else
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;
		HatScheT::XMLResourceReader readerRes(&rm);
		HatScheT::Target target;
		HatScheT::XMLTargetReader targetReader(&target);

		string resStr = "cTest/ASAPHCExampleRM.xml";
		string graphStr = "cTest/ASAPHCExample.graphml";
		string fpgaStr = "cTest/virtex6.xml";
		readerRes.readResourceModel(resStr.c_str());

		HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
		readerGraph.readGraph(graphStr.c_str());

		targetReader.readHardwareTarget(fpgaStr.c_str());

		if (rm.getNumResources() != 3) {
			cout << "Incorrect no of resource read: " << rm.getNumResources() << " instead of 3!" << endl;
			return false;
		}

		for (auto it = rm.resourcesBegin(); it != rm.resourcesEnd(); ++it) {
			Resource *r = *it;

			if (r->getName() == "Adder") {
				if (r->getHardwareCost("LUT") != 275) {
					cout << "Incorrect LUT costs found for Adder: " << r->getHardwareCost("LUTS") << " instead of 275" << endl;
					return false;
				}
			}

			if (r->getName() == "Multiplier") {
				if (r->getHardwareCost("DSP") != 1) {
					cout << "Incorrect DSP costs found for Multiplier: " << r->getHardwareCost("DSPs") << " instead of 1" << endl;
					return false;
				}
			}

			if (r->getName() == "Gain") {
				if (r->getHardwareCost("LUT") != 64) {
					cout << "Incorrect LUT costs found for Gain: " << r->getHardwareCost("LUTS") << " instead of 64" << endl;
					return false;
				}
			}
		}

		if (g.getNumberOfVertices() != 11) {
			cout << "Incorrect no of vertices read: " << g.getNumberOfVertices() << " instead of 11!" << endl;
			return false;
		}

		if (target.getFamily() != "virtex6") {
			cout << "Incorrec family found: " << target.getFamily() << " instead of virtex6" << endl;
			return false;
		}

		if (target.getName() != "XC6VLX75T") {
			cout << "Incorrect name found: " << target.getName() << " instead of XC6VLX75T" << endl;
			return false;
		}

		if (target.getVendor() != "xilinx") {
			cout << "Incorrect vendor found: " << target.getVendor() << " instead of xilinx" << endl;
			return false;
		}

		if (target.getElement("LUT") != 46560) {
			cout << "Incorrect target LUTs found: " << target.getElement("LUT") << " instead of 46560" << endl;
			return false;
		}

		if (target.getElement("DSP") != 288) {
			cout << "Incorrect target DSP found: " << target.getElement("DSP") << " instead of 288" << endl;
			return false;
		}

		if (target.getElement("MEM") != 312) {
			cout << "Incorrect target MEMs found: " << target.getElement("MEM") << " instead of 312" << endl;
			return false;
		}

		return true;
#endif
	}

	bool Tests::moduloSDCTest() {
		try {
			HatScheT::ResourceModel rm;

			auto &load = rm.makeResource("load", 1, 2, 1);
			auto &add = rm.makeResource("add", -1, 0, 1);

			HatScheT::Graph g;

			Vertex &a = g.createVertex(1);
			Vertex &b = g.createVertex(2);
			Vertex &c = g.createVertex(3);
			Vertex &d = g.createVertex(4);

			a.setName("a");
			b.setName("b");
			c.setName("c");
			d.setName("d");

			g.createEdge(a, c, 0);
			g.createEdge(b, c, 0);
			g.createEdge(c, d, 0);
			g.createEdge(d, a, 1);

			rm.registerVertex(&a, &load);
			rm.registerVertex(&b, &load);
			rm.registerVertex(&c, &add);
			rm.registerVertex(&d, &load);

			HatScheT::ModuloSDCScheduler m{g, rm, {"CPLEX", "Gurobi", "SCIP", "LPSolve"}};
			m.setSolverQuiet(true);
			m.setQuiet(false);
			m.schedule();

			auto sch = m.getSchedule();

			bool result = true;
			for (auto &p:sch) {
				std::cout << p.first->getName() << " = " << p.second << std::endl;
			}

			if (verifyModuloSchedule(g, rm, sch, m.getII()) == false) return false;
			if (m.getII() != 4) return false;
			return result;
		}
		catch (HatScheT::Exception &e) {
			std::cout << e.msg << std::endl;
		}
		return false;
	}

	bool Tests::integerIINonRectTest() {
		try {
			HatScheT::ResourceModel rm;

			auto &load = rm.makeResource("load", 1, 2, 1);
			auto &add = rm.makeResource("add", -1, 0, 1);
			// create non-rectangular mrt for load resource
			load.setNonRectLimit(0, 1);
			load.setNonRectLimit(1, 1);
			load.setNonRectLimit(2, 1);
			load.setNonRectLimit(3, 0);

			HatScheT::Graph g;

			Vertex &a = g.createVertex(1);
			Vertex &b = g.createVertex(2);
			Vertex &c = g.createVertex(3);
			Vertex &d = g.createVertex(4);

			a.setName("a");
			b.setName("b");
			c.setName("c");
			d.setName("d");

			g.createEdge(a, c, 0);
			g.createEdge(b, c, 0);
			g.createEdge(c, d, 0);
			g.createEdge(d, a, 1);

			rm.registerVertex(&a, &load);
			rm.registerVertex(&b, &load);
			rm.registerVertex(&c, &add);
			rm.registerVertex(&d, &load);

			HatScheT::IntegerIINonRectScheduler m{g, rm, {"CPLEX", "Gurobi", "SCIP", "LPSolve"}};
			m.setSolverQuiet(true);
			m.setQuiet(false);
			m.setCandidateII(4);
			m.schedule();

			auto sch = m.getSchedule();

			for (auto &p:sch) {
				std::cout << p.first->getName() << " = " << p.second << std::endl;
			}

			if (verifyModuloSchedule(g, rm, sch, m.getII()) == false) {
				std::cout << "Detected invalid modulo schedule" << std::endl;
				return false;
			}
			if (m.getII() != 4) {
				std::cout << "Expected II=4 but got II=" << m.getII() << std::endl;
				return false;
			}
			if (m.getScheduleLength() != 6) {
				std::cout << "Expected latency=6 but got latency=" << m.getScheduleLength() << std::endl;
				return false;
			}
			return true;
		}
		catch (HatScheT::Exception &e) {
			std::cout << e.msg << std::endl;
		}
		return false;
	}

	bool Tests::apiTest() {
		HatScheT::ResourceModel rm;
		auto &limited = rm.makeResource("q", 1, 1, 1);
		auto &unlimited = rm.makeResource("r", -1, 1, 0);
		auto &chained = rm.makeResource("c", -1, 0, 0);

		HatScheT::Graph g;
		for (int i = 1; i <= 12; ++i)
			g.createVertex(i);

		std::vector<std::pair<int, int>> fwdEdges = {
			{1,  2},
			{1,  11},
			{2,  3},
			{2,  7},
			{3,  4},
			{4,  5},
			{5,  6},
			{7,  8},
			{8,  9},
			{10, 11},
			{11, 12},
			{12, 9}
		};

		std::vector<std::pair<int, int>> chainingEdges = {
			{3, 5},
			{3, 6},
			{4, 6},
			{6, 9}
		};

		for (auto fe : fwdEdges)
			g.createEdge(g.getVertexById(fe.first), g.getVertexById(fe.second), 0);
		for (auto ce : chainingEdges)
			g.createEdge(g.getVertexById(ce.first), g.getVertexById(ce.second), 1);

		for (int i = 1; i <= 12; ++i) {
			auto &v = g.getVertexById(i);
			switch (i) {
				case 3:
				case 4:
				case 5:
				case 6:
					rm.registerVertex(&v, &chained);
					break;
				case 7:
				case 9:
				case 11:
					rm.registerVertex(&v, &limited);
					break;
				default:
					rm.registerVertex(&v, &unlimited);
			}
		}

		HatScheT::DotWriter dw("apitest.dot", &g, &rm);
		dw.write();

		cout << "No actual API tests performed yet" << endl;
		return true;
	}

	bool Tests::ulSchedulerTest() {
#ifndef USE_XERCESC
		return true;
#else
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;
		HatScheT::XMLResourceReader readerRes(&rm);

		string resStr = "cTest/ASAPHCExampleRM.xml";
		string graphStr = "cTest/ASAPHCExample.graphml";
		readerRes.readResourceModel(resStr.c_str());

		HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
		readerGraph.readGraph(graphStr.c_str());

		HatScheT::ULScheduler uls(g, rm);
		uls.schedule();

		HatScheT::Utility::printSchedule(uls.getSchedule());
		std::map<Vertex *, int> &schedule = uls.getSchedule();
		int foundII = uls.getII();
		bool verified = HatScheT::verifyModuloSchedule(g, rm, schedule, foundII);

		if (verified == false) return false;
		if (uls.getScheduleLength() != 8) return false;

		return true;
#endif
	}

	bool Tests::moduloSDCTestFiege() {
		try {
			HatScheT::ResourceModel rm;

			auto &load = rm.makeResource("load", 3, 2, 1);
			auto &add = rm.makeResource("add", -1, 0, 1);

			HatScheT::Graph g;

			Vertex &a = g.createVertex(1);
			Vertex &b = g.createVertex(2);
			Vertex &c1 = g.createVertex(3);
			Vertex &d1 = g.createVertex(4);
			Vertex &e1 = g.createVertex(5);
			Vertex &f1 = g.createVertex(6);
			Vertex &g1 = g.createVertex(7);
			Vertex &c2 = g.createVertex(8);
			Vertex &d2 = g.createVertex(9);
			Vertex &e2 = g.createVertex(10);
			Vertex &f2 = g.createVertex(11);
			Vertex &g2 = g.createVertex(12);
			Vertex &h2 = g.createVertex(13);
			Vertex &i2 = g.createVertex(14);
			Vertex &j2 = g.createVertex(15);
			Vertex &k2 = g.createVertex(16);
			Vertex &l2 = g.createVertex(17);
			Vertex &m2 = g.createVertex(18);
			Vertex &n2 = g.createVertex(19);
			Vertex &o2 = g.createVertex(20);
			Vertex &p2 = g.createVertex(21);

			a.setName("a");
			b.setName("b");
			c1.setName("c1");
			d1.setName("d1");
			e1.setName("e1");
			f1.setName("f1");
			g1.setName("g1");
			c2.setName("c2");
			d2.setName("d2");
			e2.setName("e2");
			f2.setName("f2");
			g2.setName("g2");
			h2.setName("h2");
			i2.setName("i2");
			j2.setName("j2");
			k2.setName("k2");
			l2.setName("l2");
			m2.setName("m2");
			n2.setName("n2");
			o2.setName("o2");
			p2.setName("p2");

			g.createEdge(a, b, 0);
			g.createEdge(b, c1, 0);
			g.createEdge(b, c2, 0);
			g.createEdge(c1, d1, 0);
			g.createEdge(d1, e1, 0);
			g.createEdge(e1, f1, 0);
			g.createEdge(f1, c1, 1);
			g.createEdge(f1, g1, 0);
			g.createEdge(c2, d2, 0);
			g.createEdge(d2, e2, 0);
			g.createEdge(e2, f2, 0);
			g.createEdge(f2, c2, 1);
			g.createEdge(f2, g2, 0);
			g.createEdge(g2, h2, 0);
			g.createEdge(h2, i2, 0);
			g.createEdge(i2, j2, 0);
			g.createEdge(j2, k2, 0);
			g.createEdge(k2, h2, 1);
			g.createEdge(k2, l2, 0);
			g.createEdge(l2, m2, 0);
			g.createEdge(m2, h2, 1);
			g.createEdge(k2, n2, 0);
			g.createEdge(l2, o2, 0);
			g.createEdge(m2, p2, 0);

			rm.registerVertex(&a, &load);
			rm.registerVertex(&b, &add);
			rm.registerVertex(&c1, &load);
			rm.registerVertex(&d1, &load);
			rm.registerVertex(&e1, &load);
			rm.registerVertex(&f1, &load);
			rm.registerVertex(&g1, &load);
			rm.registerVertex(&c2, &load);
			rm.registerVertex(&d2, &load);
			rm.registerVertex(&e2, &load);
			rm.registerVertex(&f2, &load);
			rm.registerVertex(&g2, &load);
			rm.registerVertex(&h2, &load);
			rm.registerVertex(&i2, &load);
			rm.registerVertex(&j2, &load);
			rm.registerVertex(&k2, &load);
			rm.registerVertex(&l2, &load);
			rm.registerVertex(&m2, &load);
			rm.registerVertex(&n2, &add);
			rm.registerVertex(&o2, &add);
			rm.registerVertex(&p2, &add);

			std::list<std::string> solverList = {"CPLEX", "Gurobi", "SCIP", "LPSolve"};
			HatScheT::ModSDC m(g, rm, solverList);
			m.setPriorityType(PriorityHandler::priorityType::ALASUB);
			m.setSolverQuiet(true);
			m.schedule();

			auto sch = m.getSchedule();

			for (auto &p:sch) {
				std::cout << p.first->getName() << " = " << p.second << std::endl;
			}
			std::cout << "latency = " << m.getScheduleLength() << std::endl;

			if (!verifyModuloSchedule(g, rm, sch, (int) m.getII())) {
				return false;
			}

			auto binding = m.getBindings();
			cout << "Binding: " << endl;
			for (auto it : binding) {
				cout << it.first->getName() << ": " << rm.getResource(it.first)->getName() << " " << it.second << endl;
			}
		}
		catch (HatScheT::Exception &e) {
			std::cout << e.msg << std::endl;
		}
		return false;
	}

	bool Tests::rationalIISchedulerFimmelTest() {
#ifndef USE_XERCESC
		cout << "Tests::rationalIISchedulerFimmelTest: XERCESC parsing library is not active! This test is disabled!" << endl;
	return false;
#else
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;
		HatScheT::XMLResourceReader readerRes(&rm);

		string resStr = "cTest/vanDongenRM.xml";
		string graphStr = "cTest/vanDongen.graphml";
		readerRes.readResourceModel(resStr.c_str());

		HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
		readerGraph.readGraph(graphStr.c_str());

		HatScheT::RationalIISchedulerFimmel fimmel{g, rm, {"CPLEX", "Gurobi", "SCIP", "LPSolve"}};
		fimmel.schedule();

		cout << "Tests::rationalIISchedulerFimmelTest: expected II is 5.333..." << endl;
		cout << "Tests::rationalIISchedulerFimmelTest: found II is " << fimmel.getII() << endl;

		if (fimmel.getII() < 5.333 or fimmel.getII() >= 5.334) return false;
		else return true;
#endif
	}

	bool Tests::rationalIISchedulerTest() {
#ifndef USE_XERCESC
		cout << "Tests::rationalIISchedulerTest: XERCESC parsing library is not active! This test is disabled!" << endl;
		return false;
#else
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;
		HatScheT::XMLResourceReader readerRes(&rm);

		string resStr = "cTest/exampleRatII_RM.xml";
		string graphStr = "cTest/exampleRatII.graphml";
		readerRes.readResourceModel(resStr.c_str());

		HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
		readerGraph.readGraph(graphStr.c_str());

		HatScheT::RationalIIScheduler rii{g, rm, {"CPLEX", "Gurobi", "SCIP", "LPSolve"}};
		rii.setUniformScheduleFlag(true);
		rii.schedule();

		cout << "Tests::rationalIISchedulerTest: expected II is 5/3" << endl;
		cout << "Tests::rationalIISchedulerTest: found II " << rii.getM_Found() << "/" << rii.getS_Found() << endl;

		if (rii.getM_Found() != 5 or rii.getS_Found() != 3) return false;
		else return true;
#endif
	}

	bool Tests::compareModuloSchedulerTest() {
#ifndef USE_XERCESC
		cout << "Tests::compareModuloScheduler: XERCESC parsing library is not active! This test is disabled!" << endl;
		return false;
#else

		int modSDC_II = -1;
		int moovac_II = -1;
		int ED97_II = -1;
		int moovacminreg_II = -1;

		HatScheT::ResourceModel rm;
		HatScheT::Graph g;
		HatScheT::XMLResourceReader readerRes(&rm);

		string resStr = "cTest/iir_biquRM.xml";
		string graphStr = "cTest/iir_biqu.graphml";
		readerRes.readResourceModel(resStr.c_str());

		HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
		readerGraph.readGraph(graphStr.c_str());

		//------------
		HatScheT::ModuloSDCScheduler sdc{g, rm, {"CPLEX", "Gurobi", "SCIP", "LPSolve"}};
		sdc.schedule();
		modSDC_II = sdc.getII();
		//------------
		HatScheT::MoovacScheduler moovac{g, rm, {"CPLEX", "Gurobi", "SCIP", "LPSolve"}};
		moovac.schedule();
		moovac_II = moovac.getII();
		//------------
		HatScheT::MoovacMinRegScheduler minreg{g, rm, {"CPLEX", "Gurobi", "SCIP", "LPSolve"}};
		minreg.schedule();
		moovacminreg_II = minreg.getII();
		//------------
		HatScheT::EichenbergerDavidson97Scheduler ed97{g, rm, {"CPLEX", "Gurobi", "SCIP", "LPSolve"}};
		ed97.schedule();
		ED97_II = ed97.getII();

		cout << "Tests::compareModuloSchedulerTest: Expected II is 11" << endl;
		cout << "Tests::compareModuloSchedulerTest: ModuloSDC found II " << modSDC_II << endl;
		cout << "Tests::compareModuloSchedulerTest: MoovacScheduler found II " << moovac_II << endl;
		cout << "Tests::compareModuloSchedulerTest: MoovacMinRegScheduler found II " << moovacminreg_II << endl;
		cout << "Tests::compareModuloSchedulerTest: EichenbergerDavidson97Scheduler found II " << ED97_II << endl;

		if (modSDC_II != 11 or moovac_II != 11 or moovacminreg_II != 11 or ED97_II != 11) return false;
		else return true;
#endif
	}

	bool Tests::KosarajuTest() {

		HatScheT::Graph KosaGr;
		HatScheT::ResourceModel rm;

		auto &add = rm.makeResource("add", -1, 1, 1);
		auto &mult = rm.makeResource("mult", 1, 1, 1);

		//-----------------------------------------------------------------------------------------------------------------
		//Hardcoding the examplegraph from https://www.geeksforgeeks.org/strongly-connected-components/

		Vertex &A = KosaGr.createVertex(0);
		Vertex &B = KosaGr.createVertex(1);
		Vertex &C = KosaGr.createVertex(2);
		Vertex &D = KosaGr.createVertex(3);
		Vertex &E = KosaGr.createVertex(4);
		Vertex &F = KosaGr.createVertex(5);
		Vertex &G = KosaGr.createVertex(6);
		Vertex &H = KosaGr.createVertex(7);
		Vertex &I = KosaGr.createVertex(8);

		KosaGr.createEdge(A, B, 0);
		KosaGr.createEdge(B, C, 0);
		KosaGr.createEdge(C, D, 0);
		KosaGr.createEdge(C, E, 0);
		KosaGr.createEdge(D, A, 1);
		KosaGr.createEdge(E, F, 0);
		KosaGr.createEdge(F, G, 0);
		KosaGr.createEdge(G, E, 1);
		KosaGr.createEdge(H, G, 0);
		KosaGr.createEdge(H, I, 0);

		rm.registerVertex(&A, &add);
		rm.registerVertex(&B, &add);
		rm.registerVertex(&C, &mult);
		rm.registerVertex(&D, &add);
		rm.registerVertex(&E, &add);
		rm.registerVertex(&F, &add);
		rm.registerVertex(&G, &add);
		rm.registerVertex(&H, &add);
		rm.registerVertex(&I, &add);

		KosarajuSCC kscc(KosaGr);

		vector<SCC *> sccs = kscc.getSCCs();

		vector<list<int>> compValues;
		list<int> sccA = {7};
		compValues.push_back(sccA);
		list<int> sccB = {8};
		compValues.push_back(sccB);
		list<int> sccC = {1, 2, 3, 0};
		compValues.push_back(sccC);
		list<int> sccD = {5, 6, 4};
		compValues.push_back(sccD);

		for (auto &it : sccs) {
			cout << endl;
			it->printVertexStatus();
			auto v = it->getVerticesOfSCC();
			int i = 0;
			for (auto &itr : v) {
				cout << i << ": " << itr->getName() << " ";
				i++;
			}
			cout << endl;
		}

		vector<list<int>> testValues;
		list<int> value;

		for (auto &it : sccs) {
			auto vertexList = it->getVerticesOfSCC();
			value.clear();
			for (auto &itr : vertexList) {
				value.push_back(itr->getId());
			}
			testValues.push_back(value);
		}

		for (int j = 0; j < 4; j++) {
			if (compValues[j] != testValues[j]) {
				return false;
			}
		}

		return true;

	}
	//-----------------------------------------------------------------------------------------------------------------
	//Graph to test the build supergraph method.

	bool Tests::DaiZhangTest() {

		HatScheT::Graph Gr;
		HatScheT::ResourceModel rm;

		auto &red = rm.makeResource("red", 1, 1, 1);
		auto &blue = rm.makeResource("blue", 1, 1, 1);
		auto &green = rm.makeResource("green", -1, 1, 1);

		Vertex &A = Gr.createVertex(0);
		Vertex &B = Gr.createVertex(1);
		Vertex &C = Gr.createVertex(2);
		Vertex &D = Gr.createVertex(3);
		Vertex &E = Gr.createVertex(4);
		Vertex &F = Gr.createVertex(5);
		Vertex &G = Gr.createVertex(6);
		Vertex &H = Gr.createVertex(7);
		Vertex &I = Gr.createVertex(8);
		Vertex &J = Gr.createVertex(9);
		Vertex &K = Gr.createVertex(10);
		Vertex &L = Gr.createVertex(11);
		Vertex &M = Gr.createVertex(12);
		Vertex &N = Gr.createVertex(13);
		Vertex &O = Gr.createVertex(14);

		// SSC 0:
		Gr.createEdge(A, B, 0);
		Gr.createEdge(B, C, 1);
		Gr.createEdge(C, A, 0);

		// SCC 1:
		Gr.createEdge(D, E, 0);
		Gr.createEdge(E, F, 1);
		Gr.createEdge(F, D, 0);

		// SCC 2:
		Gr.createEdge(G, H, 0);
		Gr.createEdge(H, I, 1);
		Gr.createEdge(I, G, 0);

		// SCC 3:
		Gr.createEdge(J, K, 0);
		Gr.createEdge(K, L, 1);
		Gr.createEdge(L, J, 0);

		// SCC 4:
		Gr.createEdge(M, N, 0);
		Gr.createEdge(N, O, 1);
		Gr.createEdge(O, M, 0);

		//0-2
		Gr.createEdge(A, G, 0);
		//0-3
		Gr.createEdge(A, J, 0);
		//1-3
		Gr.createEdge(D, L, 0);
		//1-4
		Gr.createEdge(D, M, 0);
		//3-4
		Gr.createEdge(L, M, 0);
		//3-2
		Gr.createEdge(L, G, 0);

		//SCC 0
		rm.registerVertex(&A, &red);
		rm.registerVertex(&B, &blue);
		rm.registerVertex(&C, &blue);

		//SCC 1
		rm.registerVertex(&D, &green);
		rm.registerVertex(&E, &green);
		rm.registerVertex(&F, &green);

		//SCC 2
		rm.registerVertex(&J, &red);
		rm.registerVertex(&K, &blue);
		rm.registerVertex(&L, &blue);

		//SCC 4
		rm.registerVertex(&G, &green);
		rm.registerVertex(&H, &green);
		rm.registerVertex(&I, &green);

		//SCC 3
		rm.registerVertex(&M, &blue);
		rm.registerVertex(&N, &blue);
		rm.registerVertex(&O, &red);

		//------------------------------------------------------------------------------------------------------------------

		DaiZhang19Scheduler DaiZhang(Gr, rm, {"CPLEX", "Gurobi", "SCIP", "LPSolve"});

		DaiZhang.schedule();

		//SCC scc;

		//if(expected == true) return true
		//else return false

		return false;
	}

	bool Tests::DaiZhangTestTwo() {

#ifndef USE_XERCESC
		return true;
#else
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;
		HatScheT::XMLResourceReader readerRes(&rm);

		string resStr = "benchmarks/origami/iir_sos16_RM.xml";
		string graphStr = "benchmarks/origami/iir_sos16.graphml";
		//string resStr = "benchmarks/origami/fir_SAM_RM.xml";
		//string graphStr = "benchmarks/origami/fir_SAM.graphml";
		readerRes.readResourceModel(resStr.c_str());

		HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
		readerGraph.readGraph(graphStr.c_str());

		HatScheT::DaiZhang19Scheduler sched(g, rm, {"CPLEX", "Gurobi", "SCIP", "LPSolve"});
		//EichenbergerDavidson97Scheduler sched(g, rm, {"CPLEX", "Gurobi", "SCIP", "LPSolve"});

		cout << "starting DaiZhang Scheduler" << endl;
		sched.schedule();
		cout << "Finished DaiZhang Scheduler" << endl;

		return true;
#endif
		return false;
	}


	bool Tests::cadicalTest() {

#ifdef USE_CADICAL
		cout << "CaDiCaL-Test started..." << endl;

		int vars = 5;
		int solvable;
		list <int> solution;
		list <int> expectedSolution = {-1, 2, 3, -4, -5};

		CaDiCaL::Solver solver;

		solver.read_dimacs("SATTest/SATTest.cnf", vars, 1);

		solvable = solver.solve();

		if (solvable == 0) {
			cout << "Problem is not solved..." << endl;
			cout << "Problem is not solved, check the solver or the input file." << endl;
			return false;
		} else if (solvable == 10) {
			cout << "Problem is SATISFIABLE..." << endl;
		} else if (solvable == 20) {
			cout << "Problem is UNSATISFIABLE..." << endl;
			cout << "Problem UNSATISFIABLE, but should be SATISFIABLE, check the solver or the input file." << endl;
			return false;
		} else {
			cout << "Unexpected Return-Value from SAT-Solver" << endl;
			return false;
		}

		for (int i = 0; i < 5; i++){
			solution.push_back(solver.val(i+1)); //Solver variable starts at 1, since negated variables are shown as negative numbers.
		}

		for (auto &it : solution){
			cout << it << " ";
		}
		cout << endl;

		if (solution == expectedSolution){
			cout << "Zappel-Jupp and Narben-Johny are TRUE criminals... ARREST them!!!" << endl;
			cout << "Test passed.." << endl;
			return true;
		}else {
			cout << "Inspector, you are arresting the wrong guys.." << endl;
			cout << "Test failed, solver returned a wrong solution. " << endl;
			return false;
		}

#else
		//CaDiCaL not active! Test function disabled!
		return true;
#endif
	}


	bool Tests::rationalIIModuloQTest() {
		HatScheT::Graph g;
		HatScheT::ResourceModel rm;

		auto &red = rm.makeResource("red", 2, 2, 1);
		auto &green = rm.makeResource("green", 4, 1, 1);

		// critical resource (#vertices=5, limit=2)
		// loop: latency=6, distance=4
		Vertex &r0 = g.createVertex(0);
		Vertex &r1 = g.createVertex(1);
		Vertex &r2 = g.createVertex(2);
		Vertex &r3 = g.createVertex(3);
		Vertex &r4 = g.createVertex(4);
		rm.registerVertex(&r0, &red);
		rm.registerVertex(&r1, &red);
		rm.registerVertex(&r2, &red);
		rm.registerVertex(&r3, &red);
		rm.registerVertex(&r4, &red);
		g.createEdge(r0, r2, 0);
		g.createEdge(r1, r2, 0);
		g.createEdge(r2, r3, 0);
		g.createEdge(r3, r4, 0);
		g.createEdge(r3, r0, 4);

		// non-critical resource (#vertices=9, limit=4)
		// loop: latency=4, distance=2
		Vertex &g0 = g.createVertex(5);
		Vertex &g1 = g.createVertex(6);
		Vertex &g2 = g.createVertex(7);
		Vertex &g3 = g.createVertex(8);
		Vertex &g4 = g.createVertex(9);
		Vertex &g5 = g.createVertex(10);
		Vertex &g6 = g.createVertex(11);
		Vertex &g7 = g.createVertex(12);
		Vertex &g8 = g.createVertex(13);
		rm.registerVertex(&g0, &green);
		rm.registerVertex(&g1, &green);
		rm.registerVertex(&g2, &green);
		rm.registerVertex(&g3, &green);
		rm.registerVertex(&g4, &green);
		rm.registerVertex(&g5, &green);
		rm.registerVertex(&g6, &green);
		rm.registerVertex(&g7, &green);
		rm.registerVertex(&g8, &green);
		g.createEdge(g0, g2, 0);
		g.createEdge(g7, g2, 0);
		g.createEdge(g8, g2, 0);
		g.createEdge(g1, g2, 0);
		g.createEdge(g2, g3, 0);
		g.createEdge(g3, g4, 0);
		g.createEdge(g4, g1, 2);
		g.createEdge(g4, g5, 0);
		g.createEdge(g4, g6, 0);

		ModuloQScheduler m(g, rm, {"Gurobi", "CPLEX", "LPSolve", "SCIP"});
		m.setQuiet(false);
		//m.setMaxLatencyConstraint(100);
		m.schedule();
		//auto result = m.getSchedule();

		std::cout << "Tests::moduloQTest: finished scheduling - resulting control steps:" << std::endl;
		auto startTimesVector = m.getStartTimeVector();
		auto initIntervals = m.getInitiationIntervals();
		auto latencySequence = m.getLatencySequence();

		auto valid = m.getScheduleValid();
		if (!valid) {
			std::cout << "Tests::moduloQTest: invalid rational II modulo schedule found" << std::endl;
			return false;
		}

		for (unsigned int i = 0; i < initIntervals.size(); ++i) {
			auto l = initIntervals[i];
			auto startTimes = startTimesVector[i];
			std::cout << "Tests::moduloQTest: start times for insertion time=" << l << std::endl;
			for (auto it : startTimes) {
				std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
			}
		}

		return true;
	}

	bool Tests::ratIIVerifierWrongCausalityDetected() {

#ifndef USE_XERCESC
		cout << "Tests::rationalIISchedulerTest: XERCESC parsing library is not active! This test is disabled!" << endl;
	return false;
#else
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;
		HatScheT::XMLResourceReader readerRes(&rm);

		string resStr = "cTest/exampleRatII_RM.xml";
		string graphStr = "cTest/exampleRatII.graphml";
		readerRes.readResourceModel(resStr.c_str());

		HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
		readerGraph.readGraph(graphStr.c_str());

		HatScheT::RationalIIScheduler rii{g, rm, {"CPLEX", "Gurobi", "SCIP", "LPSolve"}};
		rii.setUniformScheduleFlag(true);
		rii.schedule();

		//making all resources unlimited to not trigger the resource MRT check
		for (auto it : rm.Resources()) {
			it->setLimit(-1);
		}

		//generating wrong latemcy sequence
		//<1 1 3 > should not be allowed

		vector<int> ls({1, 1, 3});


		bool ok = verifyRationalIIModuloSchedule2(g, rm, rii.getStartTimeVector(), ls, rii.getScheduleLength());

		if (ok == true) return false;
		else return true;
#endif
	}

	bool Tests::ratIIVerifierWrongMRTDetected() {

#ifndef USE_XERCESC
		cout << "Tests::rationalIISchedulerTest: XERCESC parsing library is not active! This test is disabled!" << endl;
	return false;
#else
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;
		HatScheT::XMLResourceReader readerRes(&rm);

		string resStr = "cTest/exampleRatII_RM.xml";
		string graphStr = "cTest/exampleRatII.graphml";
		readerRes.readResourceModel(resStr.c_str());

		HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
		readerGraph.readGraph(graphStr.c_str());

		HatScheT::RationalIIScheduler rii{g, rm, {"CPLEX", "Gurobi", "SCIP", "LPSolve"}};
		rii.setUniformScheduleFlag(true);
		rii.schedule();

		vector<std::map<Vertex *, int> > s = rii.getStartTimeVector();

		for (auto it:g.Vertices()) {
			Vertex *v = it;

			if (v->getName() == "A5") {
				s[0][v]++;
			}
		}

		bool ok = verifyRationalIIModuloSchedule2(g, rm, s, rii.getLatencySequence(), rii.getScheduleLength());

		if (ok == true) return false;
		else return true;
#endif
	}

	bool Tests::sdsSchedulerTest() {

#ifdef USE_CADICAL

		//ToDo Problems for: fir6dlms, fir_GM, fir_hilb, fir_lms, fir_SAM, fir_SHI, fir_srg, iir4

		double timetoschedule = 0;

		/*
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;
		HatScheT::XMLResourceReader readerRes(&rm);

		//string resStr = "benchmarks/MachSuite/fft_strided/graph2_RM.xml";
		//string graphStr = "benchmarks/MachSuite/fft_strided/graph2.graphml";
		//string resStr = "benchmarks/MachSuite/sort_radix/graph14_RM.xml";
		//string graphStr = "benchmarks/MachSuite/sort_radix/graph14.graphml";
		//string resStr = "benchmarks/origami/fir_SAMRM.xml";
		//string graphStr = "benchmarks/origami/fir_SAM.graphml";
		string resStr = "benchmarks/origami/iir_biquRM.xml";
		string graphStr = "benchmarks/origami/iir_biqu.graphml";
		readerRes.readResourceModel(resStr.c_str());
		HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
		readerGraph.readGraph(graphStr.c_str());
		*/
		/*
		for(auto &it : rm.Resources()){
				it->setLimit(-1);
		}*/


		HatScheT::Graph g;
		HatScheT::ResourceModel rm;

		auto &ld = rm.makeResource("Load", 1, 1, 1);

		auto &add = rm.makeResource("Adder", -1, 1, 1);

		auto &st = rm.makeResource("Store", -1, 1, 1);

		//Load operations:
		Vertex &A = g.createVertex(0);
		Vertex &B = g.createVertex(1);
		Vertex &C = g.createVertex(2);
		//Add operations:
		Vertex &D = g.createVertex(3);
		Vertex &E = g.createVertex(4);
		//Store operations:
		Vertex &F = g.createVertex(5);
		//Vertex &G = g.createVertex(6);

		//Load operations:
		rm.registerVertex(&A, &ld);
		rm.registerVertex(&B, &ld);
		rm.registerVertex(&C, &ld);
		//Add operations:
		rm.registerVertex(&D, &add);
		rm.registerVertex(&E, &add);
		//Store operations:
		rm.registerVertex(&F, &st);
		//rm.registerVertex(&G, &st);

		//Edges:
		g.createEdge(B, D, 0);
		g.createEdge(C, D, 0);
		g.createEdge(A, E, 0);
		g.createEdge(D, E, 0);
		g.createEdge(E, F, 0);
		g.createEdge(F, A, 1);

		cout << g << endl;
		cout << rm << endl;

		cout << "Number of Vertices:" << g.getNumberOfVertices() << ", Number of Edges: " << g.getNumberOfEdges() << endl;
		cout << "*******************************************************" << endl << endl;

		cout << "SDS:" << endl;
		list <string> sw = {"CPLEX","Gurobi", "SCIP", "LPSolve"};
		SDSScheduler sds(g, rm, sw);
		cout << "*******************************************************" << endl << endl;
		sds.setQuiet(false);
		sds.setBindingType('S');
		std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
		sds.schedule();
		std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
		std::chrono::nanoseconds timeSpan = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1);
		timetoschedule += (((double) timeSpan.count()) / 1000000000.0);
		auto sdssched = sds.getSchedule();
		cout << endl << endl << "Shedule: " << endl;
		for (auto &it : sdssched) {
			cout << it.first->getName() << " : " << it.second << endl;
		}
		if(verifyModuloSchedule(g, rm, sdssched, sds.getScheduleLength())){
			cout << endl << "SDSScheduler: Schedule is valid" << endl;
		}else {
			cout << endl << "SDSScheduler: Schedule is NOT valid" << endl;
		}

		cout << endl << "Time to get shedule: " << timetoschedule << endl;
		timetoschedule = 0;
		/*
		cout << endl << endl;

		cout << "ASAP:" << endl;
		ASAPScheduler asa (g,rm);
		asa.setQuiet(false);
		t1 = std::chrono::high_resolution_clock::now();
		asa.schedule();
		t2 = std::chrono::high_resolution_clock::now();
		timeSpan = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1);
		timetoschedule += (((double) timeSpan.count()) / 1000000000.0);
		auto sched = asa.getSchedule();

		for (auto &it : sched){
			cout << it.first->getName() << " : " << it.second << endl;
		}
		if (verifyModuloSchedule(g, rm, sched, asa.getScheduleLength())){
			cout << endl << "ASAP: Schedule is valid" << endl;
		}else {
			cout << endl << "ASAP: Schedule is NOT valid" << endl;
		}

		cout << endl << "Time to get shedule: " << timetoschedule << endl;*/
		return false;
#else
		//CaDiCaL not active! Test function disabled!
		return true;
#endif
	}

	bool Tests::rationalIISCCQTest() {
		HatScheT::Graph g;
		HatScheT::ResourceModel rm;

		auto &red = rm.makeResource("red", 3, 2, 1);
		auto &green = rm.makeResource("green", 5, 1, 1);

		// critical resource (#vertices=5, limit=3)
		// loop: latency=6, distance=5
		Vertex &r0 = g.createVertex(0);
		Vertex &r1 = g.createVertex(1);
		Vertex &r2 = g.createVertex(2);
		Vertex &r3 = g.createVertex(3);
		Vertex &r4 = g.createVertex(4);
		rm.registerVertex(&r0, &red);
		rm.registerVertex(&r1, &red);
		rm.registerVertex(&r2, &red);
		rm.registerVertex(&r3, &red);
		rm.registerVertex(&r4, &red);
		g.createEdge(r0, r2, 0);
		g.createEdge(r1, r2, 0);
		g.createEdge(r2, r3, 0);
		g.createEdge(r3, r4, 0);
		g.createEdge(r3, r0, 5);

		// non-critical resource (#vertices=22, limit=5)
		// loop: latency=4, distance=3
		Vertex &g0 = g.createVertex(5);
		Vertex &g1 = g.createVertex(6);
		Vertex &g2 = g.createVertex(7);
		Vertex &g3 = g.createVertex(8);
		Vertex &g4 = g.createVertex(9);
		Vertex &g5 = g.createVertex(10);
		Vertex &g6 = g.createVertex(11);
		Vertex &g7 = g.createVertex(12);
		Vertex &g8 = g.createVertex(13);
		Vertex &g9 = g.createVertex(14);
		Vertex &g10 = g.createVertex(15);
		Vertex &g11 = g.createVertex(16);
		Vertex &g12 = g.createVertex(17);
		Vertex &g13 = g.createVertex(18);
		Vertex &g14 = g.createVertex(19);
		Vertex &g15 = g.createVertex(20);
		Vertex &g16 = g.createVertex(21);
		Vertex &g17 = g.createVertex(22);
		Vertex &g18 = g.createVertex(23);
		Vertex &g19 = g.createVertex(24);
		Vertex &g20 = g.createVertex(25);
		Vertex &g21 = g.createVertex(26);
		rm.registerVertex(&g0, &green);
		rm.registerVertex(&g1, &green);
		rm.registerVertex(&g2, &green);
		rm.registerVertex(&g3, &green);
		rm.registerVertex(&g4, &green);
		rm.registerVertex(&g5, &green);
		rm.registerVertex(&g6, &green);
		rm.registerVertex(&g7, &green);
		rm.registerVertex(&g8, &green);
		rm.registerVertex(&g9, &green);
		rm.registerVertex(&g10, &green);
		rm.registerVertex(&g11, &green);
		rm.registerVertex(&g12, &green);
		rm.registerVertex(&g13, &green);
		rm.registerVertex(&g14, &green);
		rm.registerVertex(&g15, &green);
		rm.registerVertex(&g16, &green);
		rm.registerVertex(&g17, &green);
		rm.registerVertex(&g18, &green);
		rm.registerVertex(&g19, &green);
		rm.registerVertex(&g20, &green);
		rm.registerVertex(&g21, &green);
		g.createEdge(g0, g2, 0);
		g.createEdge(g7, g2, 0);
		g.createEdge(g8, g2, 0);
		g.createEdge(g9, g2, 0);
		g.createEdge(g1, g2, 0);
		g.createEdge(g2, g3, 0);
		g.createEdge(g3, g4, 0);
		g.createEdge(g4, g1, 3);
		g.createEdge(g4, g5, 0);
		g.createEdge(g4, g6, 0);

		SCCQScheduler m(g, rm, {"Gurobi", "CPLEX", "LPSolve", "SCIP"});
		m.setQuiet(false);
		m.setSolverTimeout(1);
		m.schedule();

		std::cout << "Tests::sccqtest: finished scheduling - resulting control steps:" << std::endl;
		auto startTimesVector = m.getStartTimeVector();
		auto initIntervals = m.getInitiationIntervals();
		auto latencySequence = m.getLatencySequence();

		auto valid = m.getScheduleValid();
		if (!valid) {
			std::cout << "Tests::sccqtest: invalid rational II modulo schedule found" << std::endl;
			return false;
		}

		if (m.getM_Found() != m.getM_Start() or m.getS_Found() != m.getS_Start()) {
			std::cout << "Tests::sccqtest: expected scheduler to find solution for II=" << m.getM_Start() << "/"
				<< m.getS_Start() << " but got II=" << m.getM_Found() << "/" << m.getS_Found() << std::endl;
			return false;
		}

		for (unsigned int i = 0; i < initIntervals.size(); ++i) {
			auto l = initIntervals[i];
			auto startTimes = startTimesVector[i];
			std::cout << "Tests::sccqtest: start times for insertion time=" << l << std::endl;
			for (auto it : startTimes) {
				std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
			}
		}

		return true;
	}

	bool Tests::integerIIPBTest() {
		HatScheT::Graph g;
		HatScheT::ResourceModel rm;

		auto &red = rm.makeResource("red", 2, 1, 1);
		auto &blue = rm.makeResource("blue", 1, 1, 1);

		Vertex &r0 = g.createVertex();
		Vertex &r1 = g.createVertex();
		Vertex &r2 = g.createVertex();
		Vertex &r3 = g.createVertex();
		Vertex &r4 = g.createVertex();
		rm.registerVertex(&r0, &red);
		rm.registerVertex(&r1, &red);
		rm.registerVertex(&r2, &red);
		rm.registerVertex(&r3, &red);
		rm.registerVertex(&r4, &red);
		Vertex &b0 = g.createVertex();
		Vertex &b1 = g.createVertex();
		Vertex &b2 = g.createVertex();
		rm.registerVertex(&b0, &blue);
		rm.registerVertex(&b1, &blue);
		rm.registerVertex(&b2, &blue);
		g.createEdge(r0, r2, 0);
		g.createEdge(r0, b2, 0);
		g.createEdge(r1, r2, 0);
		g.createEdge(r1, b0, 0);
		g.createEdge(b0, r2, 0);
		g.createEdge(b0, r3, 0);
		g.createEdge(b0, b1, 0);
		g.createEdge(r2, b2, 0);
		g.createEdge(r3, r4, 0);
		g.createEdge(b1, b2, 0);
		g.createEdge(b2, r4, 0);
		//g.createEdge(b2, r1, 1); // optional back edge

		std::cout << rm << std::endl;
		std::cout << g << std::endl;

		// create scheduler
		PBScheduler pbs(g, rm, {"Gurobi", "CPLEX", "LPSolve", "SCIP"});
		// set it up
		pbs.setQuiet(false);
		pbs.setSolverTimeout(1);
		pbs.setMaxRuns(1);
		pbs.maximalSubgraphSize = 3;
		// call scheduling function
		pbs.schedule();

		// get results
		auto solvingTime = pbs.getSolvingTime();
		std::cout << "solving time = " << solvingTime << std::endl;
		auto II = (int)pbs.getII();
		std::cout << "II = " << II << std::endl;
		auto latency = pbs.getScheduleLength();
		std::cout << "latency = " << latency << std::endl;
		auto schedule = pbs.getSchedule();

		std::cout << "Resulting schedule:" << std::endl;
		for (auto it : schedule) {
			std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
		}

		auto foundSolution = pbs.getScheduleFound();
		if (!foundSolution) {
			std::cout << "Tests::integerIIPBTest: scheduler failed to find solution" << std::endl;
			return false;
		}

		auto valid = verifyModuloSchedule(g, rm, schedule, II);
		if (!valid) {
			std::cout << "Tests::integerIIPBTest: invalid modulo schedule found" << std::endl;
			return false;
		}

		if (II != 3) {
			std::cout << "Tests::integerIIPBTest: expected scheduler to find solution for II=3 but got II=" << II << std::endl;
			return false;
		}

		return true;
	}

	bool Tests::uniformRationalIISchedulerTest() {
#ifndef USE_XERCESC
		cout << "Tests::uniformRationalIISchedulerTest: XERCESC parsing library is not active! This test is disabled!" << endl;
		return false;
#else
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;
		HatScheT::XMLResourceReader readerRes(&rm);

		string resStr = "cTest/exampleRatII_RM.xml";
		string graphStr = "cTest/exampleRatII.graphml";
		readerRes.readResourceModel(resStr.c_str());

		HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
		readerGraph.readGraph(graphStr.c_str());

		HatScheT::UniformRationalIIScheduler rii(g, rm, {"Gurobi", "CPLEX", "SCIP", "LPSolve"});
		rii.setQuiet(false);
		rii.schedule();
		auto valid = rii.getScheduleValid();
		if (!valid) {
			std::cout << "Scheduler found invalid solution" << std::endl;
			return false;
		}

		cout << "Tests::uniformRationalIISchedulerTest: expected II is 5/3" << endl;
		cout << "Tests::uniformRationalIISchedulerTest: found II " << rii.getM_Found() << "/" << rii.getS_Found() << endl;

		return (rii.getM_Found() == 5 and rii.getS_Found() == 3);
#endif
	}

	bool Tests::uniformRationalIISchedulerNewTest() {
#ifndef USE_XERCESC
		cout << "Tests::uniformRationalIISchedulerTest: XERCESC parsing library is not active! This test is disabled!" << endl;
		return false;
#else
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;
		HatScheT::XMLResourceReader readerRes(&rm);

		string resStr = "cTest/exampleRatII_RM.xml";
		string graphStr = "cTest/exampleRatII.graphml";
		readerRes.readResourceModel(resStr.c_str());

		HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
		readerGraph.readGraph(graphStr.c_str());

		HatScheT::UniformRationalIIScheduler rii(g, rm, {"Gurobi", "CPLEX", "SCIP", "LPSolve"});
		rii.setQuiet(false);
		rii.schedule();
		auto valid = rii.getScheduleValid();
		if (!valid) {
			std::cout << "Scheduler found invalid solution" << std::endl;
			return false;
		}

		cout << "Tests::uniformRationalIISchedulerTest: expected II is 5/3" << endl;
		cout << "Tests::uniformRationalIISchedulerTest: found II " << rii.getM_Found() << "/" << rii.getS_Found() << endl;

		return (rii.getM_Found() == 5 and rii.getS_Found() == 3);
#endif
	}

	bool Tests::nonUniformRationalIISchedulerTest() {
#ifndef USE_XERCESC
		cout << "Tests::uniformRationalIISchedulerTest: XERCESC parsing library is not active! This test is disabled!" << endl;
		return false;
#else
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;
		HatScheT::XMLResourceReader readerRes(&rm);

		string resStr = "cTest/exampleRatII_RM.xml";
		string graphStr = "cTest/exampleRatII.graphml";
		readerRes.readResourceModel(resStr.c_str());

		HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
		readerGraph.readGraph(graphStr.c_str());

		HatScheT::NonUniformRationalIIScheduler rii(g, rm, {"Gurobi", "CPLEX", "SCIP", "LPSolve"});
		rii.setQuiet(false);
		rii.schedule();
		auto valid = rii.getScheduleValid();
		if (!valid) {
			std::cout << "Scheduler found invalid solution" << std::endl;
			return false;
		}

		cout << "Tests::uniformRationalIISchedulerTest: expected II is 5/3" << endl;
		cout << "Tests::uniformRationalIISchedulerTest: found II " << rii.getM_Found() << "/" << rii.getS_Found() << endl;

		return (rii.getM_Found() == 5 and rii.getS_Found() == 3);
#endif
	}

	bool Tests::ratIIOptimalIterationTest() {
		//int mMinII = 11;
		//int sMinII = 10;
		int mMinII = 7;
		int sMinII = 5;
		double minII = double(mMinII) / double(sMinII);
		auto integerII = (int) ceil(double(mMinII) / double(sMinII));
		int sMax = -1;
		auto maxListSize = -1;

		auto solutions = RationalIISchedulerLayer::getRationalIIQueue(sMinII, mMinII, integerII, sMax, maxListSize);

		std::cout << "mMinII = " << mMinII << std::endl;
		std::cout << "sMinII = " << sMinII << std::endl;
		std::cout << "minII = " << minII << std::endl;
		std::cout << "integerII = " << integerII << std::endl;
		std::cout << "Queue size = " << solutions.size() << std::endl;
		std::cout << "M/S Queue:" << std::endl;
		for (auto it : solutions) {
			std::cout << "  M = " << it.first << ", S = " << it.second << ", M/S = " << double(it.first) / double(it.second)
								<< std::endl;
		}

		auto expectedSize = 6;
		if (solutions.size() != expectedSize) {
			std::cout << "Expected queue size of " << expectedSize << " but got queue size of " << solutions.size() << std::endl;
		}
		return solutions.size() == expectedSize;
	}

	bool Tests::ratIIUnrollSchedulerTest() {
#ifndef USE_XERCESC
		cout << "Tests::uniformRationalIISchedulerTest: XERCESC parsing library is not active! This test is disabled!" << endl;
		return false;
#else
		std::list<SchedulerType> intIISchedulers = {ED97, MODULOSDC, MOOVAC, SUCHAHANZALEK};
#ifdef USE_CADICAL
		intIISchedulers.emplace_back(SAT);
#endif
		for (auto intIIScheduler : intIISchedulers) {
			std::string schedulerName;

			switch (intIIScheduler) {
				case ED97:
					std::cout << "Tests::uniformRationalIISchedulerTest: intII scheduler: ED97" << std::endl;
					schedulerName = "ED97";
					break;
				case MODULOSDC:
					std::cout << "Tests::uniformRationalIISchedulerTest: intII scheduler: MODULOSDC" << std::endl;
					schedulerName = "MODULOSDC";
					break;
				case MOOVAC:
					std::cout << "Tests::uniformRationalIISchedulerTest: intII scheduler: MOOVAC" << std::endl;
					schedulerName = "MOOVAC";
					break;
				case SUCHAHANZALEK:
					std::cout << "Tests::uniformRationalIISchedulerTest: intII scheduler: SUCHAHANZALEK" << std::endl;
					schedulerName = "SUCHAHANZALEK";
					break;
				default: // SAT
					std::cout << "Tests::uniformRationalIISchedulerTest: intII scheduler: SAT" << std::endl;
					schedulerName = "SAT";
					break;
			}

			HatScheT::ResourceModel rm;
			HatScheT::Graph g;
			HatScheT::XMLResourceReader readerRes(&rm);

			string resStr = "benchmarks/Programs/vanDongen/vanDongenRM.xml";
			string graphStr = "benchmarks/Programs/vanDongen/vanDongen.graphml";
			readerRes.readResourceModel(resStr.c_str());

			HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
			readerGraph.readGraph(graphStr.c_str());

			HatScheT::UnrollRationalIIScheduler rii(g, rm, {"Gurobi", "CPLEX", "SCIP", "LPSolve"});
			rii.setIntIIScheduler(intIIScheduler);
			rii.setQuiet(false);
			rii.setSolverTimeout(30);
			rii.setThreads(1);
			rii.schedule();
			auto valid = rii.getScheduleValid();
			if (!valid) {
				std::cout << "Scheduler '" << schedulerName << "' found invalid solution" << std::endl;
				return false;
			}

			if (rii.getM_Found() != 16 or rii.getS_Found() != 3) {
				cout << "Tests::uniformRationalIISchedulerTest: expected II is 16/3" << endl;
				cout << "Tests::uniformRationalIISchedulerTest: found II " << rii.getM_Found() << "/" << rii.getS_Found()
						 << endl;
				return false;
			}
		}

		cout << "Tests::uniformRationalIISchedulerTest: TEST PASSED! All schedulers found expected II of 16/3" << endl;
		return true;
#endif
	}

	bool Tests::tcadExampleTest() {
		HatScheT::Graph g;
		HatScheT::ResourceModel rm;

		auto &red = rm.makeResource("red", 5, 1, 1);

		// non-critical resource (#vertices=7, limit=5) => resMinII = 7/5 = 1.4
		// loop: latency=3, distance=2 => recMinII = 3/2 = 1.5
		Vertex &r1 = g.createVertex(1);
		Vertex &r2 = g.createVertex(2);
		Vertex &r3 = g.createVertex(3);
		Vertex &r4 = g.createVertex(4);
		Vertex &r5 = g.createVertex(5);
		Vertex &r6 = g.createVertex(6);
		Vertex &r7 = g.createVertex(7);
		rm.registerVertex(&r1, &red);
		rm.registerVertex(&r2, &red);
		rm.registerVertex(&r3, &red);
		rm.registerVertex(&r4, &red);
		rm.registerVertex(&r5, &red);
		rm.registerVertex(&r6, &red);
		rm.registerVertex(&r7, &red);
		g.createEdge(r1, r2, 0);
		g.createEdge(r1, r5, 0);
		g.createEdge(r7, r2, 0);
		g.createEdge(r7, r5, 0);
		g.createEdge(r2, r3, 0);
		g.createEdge(r5, r6, 0);
		g.createEdge(r3, r4, 0);
		g.createEdge(r6, r4, 0);
		g.createEdge(r4, r2, 2);
		g.createEdge(r6, r5, 2);

		SCCQScheduler m(g, rm, {"Gurobi", "CPLEX", "LPSolve", "SCIP"});
		m.setQuiet(false);
		m.setSolverTimeout(1);
		m.schedule();

		std::cout << "Tests::tcadExampleTest: SCCQ finished scheduling - resulting control steps:" << std::endl;
		auto startTimesVector = m.getStartTimeVector();
		auto initIntervals = m.getInitiationIntervals();
		auto latencySequence = m.getLatencySequence();

		auto valid = m.getScheduleValid();
		if (!valid) {
			std::cout << "Tests::tcadExampleTest: SCCQ discovered invalid rational-II modulo schedule found" << std::endl;
			return false;
		}
		for (unsigned int i = 0; i < initIntervals.size(); ++i) {
			auto l = initIntervals[i];
			auto startTimes = startTimesVector[i];
			std::cout << "Tests::tcadExampleTest: SCCQ - start times for insertion time=" << l << std::endl;
			for (auto it : startTimes) {
				std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
			}
		}

		UniformRationalIIScheduler u(g, rm, {"Gurobi", "CPLEX", "LPSolve", "SCIP"});
		u.setQuiet(false);
		u.setSolverTimeout(1);
		u.schedule();

		std::cout << "Tests::tcadExampleTest: Uniform rational-II scheduler finished scheduling - resulting control steps:"
							<< std::endl;
		startTimesVector = u.getStartTimeVector();
		latencySequence = u.getLatencySequence();

		valid = u.getScheduleValid();
		if (!valid) {
			std::cout
				<< "Tests::tcadExampleTest: Uniform rational-II scheduler discovered invalid rational-II modulo schedule found"
				<< std::endl;
			return false;
		}
		auto insertionTime = 0;
		for (unsigned int i = 0; i < latencySequence.size(); ++i) {
			auto startTimes = startTimesVector[i];
			std::cout << "Tests::tcadExampleTest: Uniform rational-II scheduler - start times for insertion time="
								<< insertionTime << std::endl;
			for (auto it : startTimes) {
				std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
			}
			insertionTime += latencySequence[i];
		}

		std::cout << "Tests::tcadExampleTest: Test passed" << std::endl;
		return true;
	}

	bool Tests::maFiegeTest() {
		HatScheT::Graph g;
		HatScheT::ResourceModel rm;

		auto &res = rm.makeResource("res", 2, 2, 1);

		Vertex &v0 = g.createVertex(0);
		Vertex &v1 = g.createVertex(1);
		Vertex &v2 = g.createVertex(2);

		rm.registerVertex(&v0, &res);
		rm.registerVertex(&v1, &res);
		rm.registerVertex(&v2, &res);

		g.createEdge(v0, v1, 0);
		g.createEdge(v1, v2, 0);
		g.createEdge(v2, v0, 4);

		ASAPScheduler asap(g, rm);
		EichenbergerDavidson97Scheduler ed97(g, rm, {"Gurobi", "CPLEX"});
		NonUniformRationalIIScheduler ratIInon(g, rm, {"Gurobi", "CPLEX"});
		UniformRationalIIScheduler ratIIu(g, rm, {"Gurobi", "CPLEX"});

		asap.setQuiet(false);
		ed97.setQuiet(false);
		ratIInon.setQuiet(false);
		ratIIu.setQuiet(false);

		asap.schedule();
		ed97.schedule();
		ratIInon.schedule();
		ratIIu.schedule();

		auto b = Binding::getSimpleRationalIIBinding(ratIIu.getStartTimeVector(), &rm, ratIIu.getModulo(),
																								 ratIIu.getSamples());
		ScheduleAndBindingWriter w("ma_fiege_example_schedule.csv", ratIIu.getStartTimeVector(), b, ratIIu.getSamples(),
															 ratIIu.getModulo());
		w.write();

		return true;
	}

	bool Tests::iiSmallerOneTest() {
		HatScheT::Graph g;
		HatScheT::ResourceModel rm;

		int samples = 50;

		auto &res = rm.makeResource("res", samples, 2, 1);

		Vertex &v0 = g.createVertex(0);
		Vertex &v1 = g.createVertex(1);
		Vertex &v2 = g.createVertex(2);

		rm.registerVertex(&v0, &res);
		rm.registerVertex(&v1, &res);
		rm.registerVertex(&v2, &res);

		g.createEdge(v0, v2, 0);
		g.createEdge(v1, v2, 0);

		NonUniformRationalIIScheduler ratIInon(g, rm, {"Gurobi", "CPLEX"});
		UniformRationalIISchedulerNew ratIIu(g, rm, {"Gurobi", "CPLEX"});
		SCCQScheduler ratIIsccq(g, rm, {"Gurobi", "CPLEX"});

		ratIInon.setQuiet(false);
		ratIIu.setQuiet(false);
		ratIIsccq.setQuiet(false);

		ratIInon.setModulo(3);
		ratIInon.setSamples(samples);
		ratIIu.setModulo(3);
		ratIIu.setSamples(samples);
		ratIIsccq.setModulo(3);
		ratIIsccq.setSamples(samples);

		ratIInon.schedule();
		ratIIu.schedule();
		ratIIsccq.schedule();

		return true;
	}

	bool Tests::sccqFailTest() {
		HatScheT::Graph g;
		HatScheT::ResourceModel rm;

		auto &res0 = rm.makeResource("res0", 3, 2, 1);
		auto &res1 = rm.makeResource("res1", 3, 1, 1);

		Vertex &v0 = g.createVertex(0);
		Vertex &v1 = g.createVertex(1);
		Vertex &v2 = g.createVertex(2);
		Vertex &v3 = g.createVertex(3);

		Vertex &v4 = g.createVertex(4);
		Vertex &v5 = g.createVertex(5);
		Vertex &v6 = g.createVertex(6);
		Vertex &v7 = g.createVertex(7);

		rm.registerVertex(&v0, &res0);
		rm.registerVertex(&v1, &res0);
		rm.registerVertex(&v2, &res0);
		rm.registerVertex(&v3, &res0);

		rm.registerVertex(&v4, &res1);
		rm.registerVertex(&v5, &res1);
		rm.registerVertex(&v6, &res1);
		rm.registerVertex(&v7, &res1);

#if 0
		// NO UNIFORM SCHEDULE POSSIBLE
		g.createEdge(v0,v4,0);
		g.createEdge(v4,v1,0);
		g.createEdge(v1,v5,0);
		g.createEdge(v5,v2,0);
		g.createEdge(v2,v6,0);
		g.createEdge(v6,v3,0);
		g.createEdge(v3,v7,0);
		g.createEdge(v7,v0,8);
#else
#if 1
		// NO SCCQ SCHEDULE POSSIBLE
		g.createEdge(v0, v1, 0);
		g.createEdge(v1, v2, 0);
		g.createEdge(v2, v3, 0);
		g.createEdge(v3, v4, 0);
		g.createEdge(v4, v5, 0);
		g.createEdge(v5, v6, 0);
		g.createEdge(v6, v7, 0);
		g.createEdge(v7, v0, 8);
#else
		// SCHEDULE POSSIBLE
		g.createEdge(v0,v1,0);
		g.createEdge(v1,v2,0);
		g.createEdge(v2,v3,0);
		g.createEdge(v3,v4,0);
		g.createEdge(v4,v5,0);
		g.createEdge(v5,v6,0);
		g.createEdge(v6,v7,0);
		g.createEdge(v7,v0,8);
#endif
#endif

		SCCQScheduler sccq(g, rm, {"Gurobi"});
		//UniformRationalIISchedulerNew sccq(g,rm,{"Gurobi"});
		//NonUniformRationalIIScheduler sccq(g,rm,{"Gurobi"});

		sccq.setSolverTimeout(48 * 3600); // 48h timeout

		sccq.setQuiet(false);

		sccq.schedule();

		return true;
	}

	bool Tests::minIntIIFailTest() {
		HatScheT::Graph g;
		HatScheT::ResourceModel rm;

		auto &res0 = rm.makeResource("res0", 1, 2, 1);

		Vertex &v0 = g.createVertex(0);
		Vertex &v1 = g.createVertex(1);

		rm.registerVertex(&v0, &res0);
		rm.registerVertex(&v1, &res0);

		// NO SCHEDULE POSSIBLE FOR MIN II
		g.createEdge(v0, v1, 0);
		g.createEdge(v1, v0, 2);
		EichenbergerDavidson97Scheduler ed97(g, rm, {"Gurobi"});

		ed97.setQuiet(false);

		ed97.schedule();

		return true;
	}

	bool Tests::fibonacciTest() {

		cout << "INSERT AND EXTRACT_MIN TEST:" << endl << endl;
        clock_t start, end;
        double times[4];
		const int arraysize = 20; //should be an even number
		int numbers[arraysize];
		int sorted_numbers_from_heap[arraysize];
		int sorted_numbers_from_mapsort[arraysize];
        int sorted_numbers_from_std_sort[arraysize];

		srand(time(nullptr));
		for (int i = 0; i < arraysize; i++) {
			numbers[i] = rand() % 10000;
		}
		cout << "Numbers generates..." << endl;
		for (int i : numbers) {
			cout << i << " ";
		}

		/////// MapSort - Who knows if that exists allready? //////
        start = clock();
		map<int,list<int>>mapSortMap;
		for(auto &it : numbers){
		    mapSortMap[it].push_back(it);
		}

		int h = 0;
		for(auto &it : mapSortMap){
		    while (!it.second.empty()){
		        sorted_numbers_from_mapsort[h] = it.second.back();
		        it.second.pop_back();
		        h++;
		    }
		}
		end = clock();
        times[0] = double(end - start) / double(CLOCKS_PER_SEC);
        cout << endl << "MapSort done..." << endl;

		/////// Insert and Extract_min Test Begin ///////
        start = clock();
		//Creating a new Heap and fill it with numbers-Arrays content.
		auto *FIB = new FibonacciHeap<int>;
		for (int i : numbers) {
			FIB->push(i);
		}

		//Extracting the values from the heap again.
		int j = 0;
		while (!FIB->empty()) {
			sorted_numbers_from_heap[j] = FIB->get_extract_min();
			j++;
		}
        end = clock();
        times[1] = double(end - start) / double(CLOCKS_PER_SEC);
		cout << "Heap done..." << endl;

		//Sort with std::sort
		start = clock();
		//Sort
		for (int l = 0; l < arraysize; l++) {
            sorted_numbers_from_std_sort[l] = numbers[l];
        }
		std::sort(sorted_numbers_from_std_sort, sorted_numbers_from_std_sort+arraysize);
        end = clock();
        times[2] = double(end - start) / double(CLOCKS_PER_SEC);
        cout << "STD::Sort done..." << endl;

        start = clock();
        //Sort the numbers Array with a simple Bubble Sort
		for (int k = arraysize; k > 1; --k) {
			for (int j = 0; j < k - 1; ++j) {
				if (numbers[j] > numbers[j + 1]) {
					int temp = numbers[j];
					numbers[j] = numbers[j + 1];
					numbers[j + 1] = temp;
				}
			}
		}
        end = clock();
        times[3] = double(end - start) / double(CLOCKS_PER_SEC);
		cout << "Bubble Sort done..." << endl;

		//Display Results:
        cout << endl << "Sorted array by MapSort:" << endl;
        for (int it : sorted_numbers_from_mapsort) {
            cout << it << " ";
        }
		cout << endl << "Sorted array by F-Heap:" << endl;
		for (int it : sorted_numbers_from_heap) {
			cout << it << " ";
		}
        cout << endl << "Sorted array by std::sort:" << endl;
        for (int it : sorted_numbers_from_std_sort) {
            cout << it << " ";
        }
		cout << endl << "Sorted array by Bubble Sort:" << endl;
		for (int it : numbers) {
			cout << it << " ";
		}

		cout << endl;

		if (0 != memcmp(sorted_numbers_from_heap, numbers, sizeof(numbers))) {
			cout << endl << "Test failed!" << endl;
			return false;
		}
        if (0 != memcmp(sorted_numbers_from_std_sort, numbers, sizeof(numbers))) {
            cout << endl << "Test failed!" << endl;
            return false;
        }
        if (0 != memcmp(sorted_numbers_from_mapsort, numbers, sizeof(numbers))) {
            cout << endl << "Test failed!" << endl;
            return false;
        }

		cout << "Map-Sort: " << fixed << times[0] << setprecision(5) << " sec " << endl;
        cout << "Fibbo-Heap: " << fixed << times[1] << setprecision(5) << " sec " << endl;
        cout << "STD-Sort: " << fixed << times[2] << setprecision(5) << " sec " << endl;
        cout << "Bubble-Sort: " << fixed << times[3] << setprecision(5) << " sec " << endl;

		/////// Insert and Extract_min Test End ///////

		/////// Inserting Elements with Payload ///////
		auto *FIBO = new FibonacciHeap<int>;
		Vertex *vp = nullptr;
		for (int i = 0; i < 10; i++) {
			auto *v = new Vertex(9 - i);
			if (i == 5) {
				vp = v;
			}
			FIBO->push(i, v);
		}

		/////// Removing Elements from the Heap ///////
		do {
			auto v = (Vertex *) FIBO->topNode()->payload;
			auto k = FIBO->topNode()->key;
			FIBO->pop();
			cout << "Key: " << k << " Name: " << v->getName() << endl;
		} while (!FIBO->empty());

		delete FIBO;

		/////////////// Payload Test End //////////////

		cout << endl << "Test passed!" << endl;

		return true;
	}

	bool Tests::sdcSolverTest() {

		//Creating an instance of the solver.
		auto *solver = new SDCSolver();

		//Create a system of Single Difference Constraints.
		Vertex R(0);
		R.setName("Rolf");
		Vertex H(1);
		H.setName("Horst");
		Vertex G(2);
		G.setName("Gunter");
		Vertex E(3);
		E.setName("Emma");
		Vertex P(4);
		P.setName("Paul");

		//Example SDC-System
		list<SDCConstraint> constr;
		//Rolf - Horst <= 3
		constr.push_back(solver->create_sdc_constraint(&H, &R, 3));
		//Gunter - Horst <= -2
		constr.push_back(solver->create_sdc_constraint(&H, &G, -2));
		//Rolf - Gunter <= 3
		constr.push_back(solver->create_sdc_constraint(&G, &R, 3));
		//Gunter - Rolf <= 3
		constr.push_back(solver->create_sdc_constraint(&R, &G, -3));
		//Emma - Gunter <= 42
		constr.push_back(solver->create_sdc_constraint(&G, &E, -1));
		//Paul - Emma <= 2
		constr.push_back(solver->create_sdc_constraint(&E, &P, 4));

		//Adding the constraints to the solver
		for (auto &it : constr) {
			solver->add_sdc_constraint(it);
		}

		//Printing the system
		solver->print_Constraint_Graph();

		//Compute an initial solution for the given SDC-System.
		solver->compute_inital_solution();
		if (solver->get_solver_status() == 11) {
			cout << endl << "System is not feasible." << endl;
		} else if (solver->get_solver_status() == 10) {
			cout << endl << "System is feasible." << endl;
			auto spath = solver->get_solution();
			for (auto &it : spath) {
				cout << it.first->getName() << ": " << it.second << endl;
			}
		}
		cout << endl;

		//Adding a constraint
		SDCConstraint c = solver->create_sdc_constraint(&G, &P, -3);
		SDCConstraint d = solver->create_sdc_constraint(&P, &G, -4);

		//Using the incremental Algorithm to solve the new system.
		solver->add_to_feasible(c);
		cout << "Test" << endl;
		solver->add_to_feasible(d);

		//Checking the Solver Status and Print the Solution if feasible.
		if (solver->get_solver_status() == 21) {
			cout << endl << "System is not feasible." << endl;
		} else if (solver->get_solver_status() == 20) {
			cout << endl << "System is feasible." << endl;
			auto spath = solver->get_solution();
			for (auto &it : spath) {
				cout << it.first->getName() << ": " << it.second << endl;
			}
		}
		cout << endl;


		//Removing a constraint from the solver.
		//solver->remove_sdc_constraint(D, A);

		//Printing the system again.
		//solver->print_Constraint_Graph();

		delete solver;

		return false;
	}

	bool Tests::rationalIIModuloSDCTest() {
		{

			HatScheT::ResourceModel rm;
			HatScheT::Graph g;

			Vertex &A = g.createVertex();
            Vertex &B = g.createVertex();
            Vertex &C = g.createVertex();
            Vertex &D = g.createVertex();
            Vertex &E = g.createVertex();

            A.setName("A");
            B.setName("B");
            C.setName("C");
            D.setName("D");
            E.setName("E");

            g.createEdge(A,E,0);
            g.createEdge(B,E,0);
            g.createEdge(C,E,0);
            g.createEdge(E,D,0);
            g.createEdge(D,A,3);

            auto &add = rm.makeResource("add", -1,0,1);
            auto &load = rm.makeResource("load", 3,2,1);


            rm.registerVertex(&A,&load);
            rm.registerVertex(&B,&load);
            rm.registerVertex(&C,&load);
            rm.registerVertex(&D,&load);
            rm.registerVertex(&E,&add);

            cout<< rm;
            cout<< g;

            //UniformRationalIISchedulerNew schedulerUNIFORM (g,rm,{"Gurobi", "CPLEX", "SCIP", "LPSolve"});
            ModSDC schedulerMod (g,rm,{"Gurobi", "CPLEX", "SCIP", "LPSolve"});
            ModuloQScheduler test (g,rm,{"Gurobi", "CPLEX", "SCIP", "LPSolve"});
            RationalIIModuloSDCScheduler schedulerRationalSDC (g,rm,{"Gurobi", "CPLEX", "SCIP", "LPSolve"});
            //ASAPScheduler schedulerASAP (g,rm);
            //RationalIIModuloSDCScheduler scheduler (g,rm,{"Gurobi", "CPLEX", "SCIP", "LPSolve"});
            //schedulerRationalSDC.setSolverTimeout(30000);
            schedulerRationalSDC.setBudgetMultiplier(15);
            schedulerRationalSDC.setQuiet(false);
            schedulerRationalSDC.schedule();
            //auto &test = schedulerRationalSDC.getStartTimeVector();
            //auto s =schedulerMod.getSchedule();
            //for (auto it : s){
            //    cout << "Vertex: " << it.first->getName() << " Time: " << it.second <<endl;
            //}

			return true;
		}
	}

	bool Tests::ilpBasedIntIIBindingTestCong() {
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;

		auto &mem = rm.makeResource("mem", UNLIMITED, 0, 1);
		auto &add = rm.makeResource("add", 2, 1, 1);

		// resource: #vertices=3, limit=2
		Vertex &mem1 = g.createVertex(1);
		Vertex &mem2 = g.createVertex(2);
		Vertex &mem3 = g.createVertex(3);
		Vertex &mem4 = g.createVertex(4);
		Vertex &add1 = g.createVertex(5);
		Vertex &add2 = g.createVertex(6);
		Vertex &add3 = g.createVertex(7);
		rm.registerVertex(&mem1, &mem);
		rm.registerVertex(&mem2, &mem);
		rm.registerVertex(&mem3, &mem);
		rm.registerVertex(&mem4, &mem);
		rm.registerVertex(&add1, &add);
		rm.registerVertex(&add2, &add);
		rm.registerVertex(&add3, &add);
		auto *e0 = &g.createEdge(mem1, add1, 0);
		auto *e1 = &g.createEdge(mem2, add1, 0);
		auto *e2 = &g.createEdge(mem3, add2, 0);
		auto *e3 = &g.createEdge(mem4, add2, 0);
		auto *e4 = &g.createEdge(add1, add3, 0);
		auto *e5 = &g.createEdge(add2, add3, 0);

		// port assignment for each edge
		std::map<Edge*,int> portAssignments;
		portAssignments[e0] = 0;
		portAssignments[e1] = 1;
		portAssignments[e2] = 0;
		portAssignments[e3] = 1;
		portAssignments[e4] = 0;
		portAssignments[e5] = 1;

		std::map<Vertex*, int> sched;
		int II = 2;

		/*
		sched[&r1] = 0;
		sched[&r2] = 1;
		sched[&r3] = 0;
		sched[&r4] = 0;
		sched[&r5] = 4;
		sched[&r6] = 1;
		sched[&r7] = 5;
		 */

		// We actually do not need to schedule every time.
		HatScheT::EichenbergerDavidson97Scheduler ed97{g, rm, {"Gurobi", "CPLEX", "SCIP", "LPSolve"}};
		ed97.setSolverQuiet(true);
		ed97.setQuiet(false);
		ed97.setSolverTimeout(300);
		ed97.schedule();

		if(!ed97.getScheduleFound()) return false;
		sched = ed97.getSchedule();
		II = (int)ed97.getII();

		auto bind = Binding::getILPBasedIntIIBindingCong(sched, &g, &rm, II, portAssignments,
																										 {"Gurobi", "CPLEX", "SCIP", "LPSolve"}, 300);

		if(not verifyIntIIBinding(&g, &rm, sched, II, bind)) {
			std::cout << "Invalid binding detected - test failed" << std::endl;
			return false;
		}

		std::cout << "resource bindings:" << std::endl;
		for(auto &it : bind.resourceBindings) {
			std::cout << "  binding '" << it.first << "' to FU '" << *it.second.begin() << "'" << std::endl;
		}

		std::cout << "connections:" << std::endl;
		for (auto it : bind.connections) {
			std::cout << "  '" << std::get<0>(it) << "' (" << std::get<1>(it) << ") -> '" << std::get<2>(it) << "' (" << std::get<3>(it) << ") port " << std::get<4>(it) << std::endl;
		}

		return true;
	}

	bool Tests::rationalIICombinedSchedulerTest() {
#ifndef USE_XERCESC
		cout << "Tests::rationalIICombinedSchedulerTest: XERCESC parsing library is not active! This test is disabled!" << endl;
		return false;
#else
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;
		HatScheT::XMLResourceReader readerRes(&rm);

		string resStr = "benchmarks/origamiRatII/mat_inv/RM78.xml";
		string graphStr = "benchmarks/origamiRatII/mat_inv/mat_inv.graphml";
		readerRes.readResourceModel(resStr.c_str());

		HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
		readerGraph.readGraph(graphStr.c_str());

		std::list<std::string> sw = {"Gurobi"}; // {"Gurobi", "CPLEX", "SCIP", "LPSolve"}

		long timeout = 300;

		HatScheT::CombinedRationalIIScheduler comb{g, rm, sw};
		comb.setSolverQuiet(true);
		comb.disableVerifier();
		comb.setQuiet(false);
		comb.setSolverTimeout(timeout);
		comb.schedule();

		// compare results
		if(comb.getScheduleFound()) {
			std::cout << "Combined scheduler found solution with II=" << comb.getII() << " (" << comb.getM_Found() << "/"
			<< comb.getS_Found() << ")" << " and latency=" << comb.getScheduleLength() << std::endl;
		}
		else {
			std::cout << "Combined scheduler did not find solution" << std::endl;
		}
		std::cout << "Combined scheduler needed " << comb.getSolvingTime() << " sec with solver status "
		<< comb.getScaLPStatus() << std::endl;

		HatScheT::UniformRationalIISchedulerNew uni{g, rm, sw};
		uni.setSolverQuiet(true);
		uni.disableVerifier();
		uni.setQuiet(false);
		uni.setSolverTimeout(timeout);
		uni.schedule();

		if(uni.getScheduleFound()) {
			std::cout << "Optimal scheduler found solution with II=" << uni.getII() << " (" << uni.getM_Found() << "/"
								<< uni.getS_Found() << ")" << " and latency=" << uni.getScheduleLength() << std::endl;
		}
		else {
			std::cout << "Optimal uniform scheduler did not find solution" << std::endl;
		}
		std::cout << "Optimal uniform scheduler needed " << uni.getSolvingTime() << " sec with solver status "
							<< uni.getScaLPStatus() << std::endl;

		return true;
#endif
	}

	bool Tests::ilpBasedIntIIBindingTest() {

		// create scheduling problem
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;

		auto &memR = rm.makeResource("memR", UNLIMITED, 0, 1);
		auto &memW = rm.makeResource("memW", UNLIMITED, 0, 1);
		auto &mult = rm.makeResource("mult", 2, 1, 1);
		auto &cons = rm.makeResource("cons", UNLIMITED, 0, 1);

		auto &x0 = g.createVertex();
		x0.setName("x0");
		auto &x1 = g.createVertex();
		x1.setName("x1");
		auto &constant0_25 = g.createVertex();
		constant0_25.setName("constant0_25");
		auto &mult0 = g.createVertex();
		mult0.setName("mult0");
		auto &mult1 = g.createVertex();
		mult1.setName("mult1");
		auto &mult2 = g.createVertex();
		mult2.setName("mult2");
		auto &y0 = g.createVertex();
		y0.setName("y0");

		rm.registerVertex(&x0, &memR);
		rm.registerVertex(&x1, &memR);
		rm.registerVertex(&constant0_25, &cons);
		rm.registerVertex(&mult0, &mult);
		rm.registerVertex(&mult1, &mult);
		rm.registerVertex(&mult2, &mult);
		rm.registerVertex(&y0, &memW);

		std::map<Edge*,int> portAssignments;
		auto &e0 = g.createEdge(x0,mult0,0);
		portAssignments[&e0] = 0;
		auto &e1 = g.createEdge(mult2,mult0,4);
		portAssignments[&e1] = 1;
		auto &e2 = g.createEdge(x1,mult1,0);
		portAssignments[&e2] = 0;
		auto &e3 = g.createEdge(mult0,mult1,0);
		portAssignments[&e3] = 1;
		auto &e4 = g.createEdge(constant0_25,mult2,0);
		portAssignments[&e4] = 0;
		auto &e5 = g.createEdge(mult1,mult2,0);
		portAssignments[&e5] = 1;
		auto &e6 = g.createEdge(mult2,y0,0);
		portAssignments[&e6] = 0;

		// schedule that badboy
		std::list<std::string> sw = {"Gurobi"};
		int timeout = 300;
		std::map<Vertex*,int> sched;
		std::vector<std::map<Vertex*,int>> ratIISched;
		double intII = 2.0;
		double ratII = 1.5;
		int samples = 2;
		int modulo = 3;

		EichenbergerDavidson97Scheduler scheduler(g,rm,sw);
		scheduler.setQuiet(true);
		scheduler.setSolverTimeout(timeout);
		scheduler.schedule();
		sched = scheduler.getSchedule();
		intII = scheduler.getII();

		std::cout << "Integer-II Schedule:" << std::endl;
		for(auto it : sched) {
			std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
		}

		// rat-II schedule
		UniformRationalIISchedulerNew ratIIScheduler(g,rm,sw);
		ratIIScheduler.setQuiet(true);
		ratIIScheduler.setSolverTimeout(timeout);
		ratIIScheduler.schedule();
		ratIISched = ratIIScheduler.getStartTimeVector();
		ratII = ratIIScheduler.getII();
		samples = ratIIScheduler.getS_Found();
		modulo = ratIIScheduler.getM_Found();

		std::cout << "Rational-II Schedule:" << std::endl;
		for(int s=0; s<ratIISched.size(); s++) {
			std::cout << "  sample " << s << std::endl;
			for (auto it : ratIISched[s]) {
				std::cout << "    " << it.first->getName() << " - " << it.second << std::endl;
			}
		}

		// specify commutative operation types
		std::set<const Resource*> commutativeOps;
		commutativeOps.insert(&mult);

		// call binding function for integer IIs
		double wMux = 1.0;
		double wReg = 1.0;
		double maxMux = -1.0;
		double maxReg = -1.0;
		auto bind = Binding::getILPBasedIntIIBinding(sched,&g,&rm,(int)intII,wMux,wReg,portAssignments,maxMux,maxReg,commutativeOps,sw,timeout,true);
		std::cout << "Integer-II binding successfully computed" << std::endl;
		bool intIIValid = verifyIntIIBinding(&g,&rm,sched,(int)intII,bind,commutativeOps);
		std::cout << "Integer-II binding is " << (intIIValid?"":"not ") << "valid" << std::endl;
		std::cout << "Integer-II implementation multiplexer costs: " << bind.multiplexerCosts << std::endl;
		std::cout << "Integer-II implementation register costs: " << bind.registerCosts << std::endl;

		// call binding function for rational IIs
		auto ratIIBind = Binding::getILPBasedRatIIBinding(ratIISched,&g,&rm,samples,modulo,wMux,wReg,portAssignments,maxMux,maxReg,commutativeOps,sw,timeout,true);
		std::cout << "Rational-II binding successfully computed" << std::endl;
		bool ratIIValid = verifyRatIIBinding(&g,&rm,ratIISched,samples,modulo,ratIIBind,portAssignments,commutativeOps);
		std::cout << "Rational-II binding is " << (ratIIValid?"":"not ") << "valid" << std::endl;
		std::cout << "Rational-II implementation multiplexer costs: " << ratIIBind.multiplexerCosts << std::endl;
		std::cout << "Rational-II implementation register costs: " << ratIIBind.registerCosts << std::endl;
		return intIIValid and ratIIValid and (bind.multiplexerCosts+bind.registerCosts==15) and (ratIIBind.multiplexerCosts+ratIIBind.registerCosts==21);
	}

	bool Tests::optimalIntegerIIGeneralizedBindingTest() {
#ifndef USE_SCALP
		std::cout << "Tests::optimalIntegerIIGeneralizedBindingTest: build HatScheT with ScaLP to enable this test" << std::endl;
		return true;
#else
		// create HLS problem
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;

		// read ports
		auto &r1 = g.createVertex();
		r1.setName("r1");
		auto &r2 = g.createVertex();
		r2.setName("r2");
		auto &r3 = g.createVertex();
		r3.setName("r3");
		auto &r4 = g.createVertex();
		r4.setName("r4");

		// complex limited operations
		auto &c1 = g.createVertex();
		c1.setName("c1");
		auto &c2 = g.createVertex();
		c2.setName("c2");
		auto &c3 = g.createVertex();
		c3.setName("c3");
		auto &c4 = g.createVertex();
		c4.setName("c4");
		auto &c5 = g.createVertex();
		c5.setName("c5");

		// write ports
		auto &w1 = g.createVertex();
		w1.setName("w1");
		auto &w2 = g.createVertex();
		w2.setName("w2");
		auto &w3 = g.createVertex();
		w3.setName("w3");
		auto &w4 = g.createVertex();
		w4.setName("w4");

		// edges
		std::map<Edge*, std::pair<int,int>> portAssignments;
		auto &e1 = g.createEdge(r1, c1, 0, Edge::DependencyType::Data);
		portAssignments[&e1] = {0, 0};
		auto &e2 = g.createEdge(r2, c1, 0, Edge::DependencyType::Data);
		portAssignments[&e2] = {0, 1};
		auto &e3 = g.createEdge(r3, c2, 0, Edge::DependencyType::Data);
		portAssignments[&e3] = {0, 0};
		auto &e4 = g.createEdge(r4, c2, 0, Edge::DependencyType::Data);
		portAssignments[&e4] = {0, 1};
		auto &e5 = g.createEdge(c1, c3, 0, Edge::DependencyType::Data);
		portAssignments[&e5] = {1, 0};
		auto &e6 = g.createEdge(c2, c3, 0, Edge::DependencyType::Data);
		portAssignments[&e6] = {0, 1};
		auto &e7 = g.createEdge(c1, c4, 0, Edge::DependencyType::Data);
		portAssignments[&e7] = {0, 0};
		auto &e8 = g.createEdge(c2, c5, 0, Edge::DependencyType::Data);
		portAssignments[&e8] = {1, 1};
		auto &e9 = g.createEdge(c3, c4, 0, Edge::DependencyType::Data);
		portAssignments[&e9] = {0, 1};
		auto &e10 = g.createEdge(c3, c5, 0, Edge::DependencyType::Data);
		portAssignments[&e10] = {1, 0};
		auto &e11 = g.createEdge(c4, w1, 0, Edge::DependencyType::Data);
		portAssignments[&e11] = {0, 0};
		auto &e12 = g.createEdge(c4, w2, 0, Edge::DependencyType::Data);
		portAssignments[&e12] = {1, 0};
		auto &e13 = g.createEdge(c5, w3, 0, Edge::DependencyType::Data);
		portAssignments[&e13] = {0, 0};
		auto &e14 = g.createEdge(c5, w4, 0, Edge::DependencyType::Data);
		portAssignments[&e14] = {1, 0};

		// create resources
		auto &read = rm.makeResource("read",2,0,1);
		auto &write = rm.makeResource("write",2,0,1);
		auto &complex_operation = rm.makeResource("complex_operation",3,1,1);

		// register vertices
		rm.registerVertex(&r1, &read);
		rm.registerVertex(&r2, &read);
		rm.registerVertex(&r3, &read);
		rm.registerVertex(&r4, &read);
		rm.registerVertex(&w1, &write);
		rm.registerVertex(&w2, &write);
		rm.registerVertex(&w3, &write);
		rm.registerVertex(&w4, &write);
		rm.registerVertex(&c1, &complex_operation);
		rm.registerVertex(&c2, &complex_operation);
		rm.registerVertex(&c3, &complex_operation);
		rm.registerVertex(&c4, &complex_operation);
		rm.registerVertex(&c5, &complex_operation);

		// debug info
		std::cout << g << std::endl;
		std::cout << rm << std::endl;

		// schedule
		EichenbergerDavidson97Scheduler ed97(g,rm,{"Gurobi", "CPLEX", "LPSolve", "SCIP"});
		ed97.schedule();

		// print result
		if (!ed97.getScheduleFound()) {
			std::cout << "scheduler failed to find solution - this should never happen!" << std::endl;
			return false;
		}
		auto II = ed97.getII();
		auto sl = ed97.getScheduleLength();
		auto sched = ed97.getSchedule();

		std::cout << "found schedule with II=" << (int)II << " and latency=" << sl << std::endl;
		for (auto it : sched) {
			std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
		}

		OptimalIntegerIIGeneralizedBinding oigb(&g, &rm, sched, II, portAssignments, {}, {"Gurobi"});
		oigb.setQuiet(false);
		oigb.setAllowMultipleBindings(false);
		oigb.bind();
		Binding::BindingContainer b;
		oigb.getBinding(&b);
		std::cout << "binding finished!" << std::endl;

		auto valid = verifyIntIIBinding(&g, &rm, sched, II, b, {});
		if (!valid) {
			std::cout << "binding invalid" << std::endl;
			std::cout << b << std::endl;
			return false;
		}

		std::cout << "Test passed!" << std::endl;
		return true;
#endif
	}

  bool Tests::firSAMRatIIImplementationsTest() {
#ifndef USE_XERCESC
		cout << "Tests::firSAMRatIIImplementationsTest: XERCESC parsing library is not active! This test is disabled!" << endl;
		return false;
#else
		// read resource model and graph
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;
		string resStr = "benchmarks/origami/fir_SAMRM.xml";
		string graphStr = "benchmarks/origami/fir_SAM.graphml";
		HatScheT::XMLResourceReader readerRes(&rm);
		readerRes.readResourceModel(resStr.c_str());
		HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
		readerGraph.readGraph(graphStr.c_str());

		// create new resource model for minimum FUs with II=30/7=4.29 and II=9/2=4.5 (95.33% of max TP)
		ResourceModel rm307;
		ResourceModel rm92;
		for (auto *r : rm.Resources()) {
			auto vertices = rm.getVerticesOfResource(r);
			auto newLimit307 = (int)ceil(((double)vertices.size()) / (30.0/7.0));
			auto newLimit92 = (int)ceil(((double)vertices.size()) / (9.0/2.0));
			if(newLimit307 == 0) newLimit307 = 1;
			if(newLimit92 == 0) newLimit92 = 1;
			auto *r307 = &rm307.makeResource(r->getName(), newLimit307, r->getLatency(), r->getBlockingTime());
			auto *r92 = &rm92.makeResource(r->getName(), newLimit92, r->getLatency(), r->getBlockingTime());
			std::cout << "Resource '" << r->getName() << "' - 30/7 limit: " << newLimit307 << ", 9/2 limit: " << newLimit92 << std::endl;
			if (newLimit92 != newLimit307) {
				std::cout << "  DIFFERENT" << std::endl;
				return false;
			}
			for (auto *v : vertices) {
				rm307.registerVertex(v, r307);
				rm92.registerVertex(v, r92);
			}
		}

		// ILP solvers
		std::list<std::string> sw = {"Gurobi", "CPLEX", "SCIP", "LPSolve"};

		// generate schedulers
		UniformRationalIISchedulerNew u307(g, rm307, sw);
		UniformRationalIISchedulerNew u92(g, rm92, sw);
		u307.setModulo(30);
		u307.setSamples(7);
		u92.setModulo(9);
		u92.setSamples(2);
		u307.setSolverTimeout(600);
		u92.setSolverTimeout(600);
		u307.setThreads(10);
		u92.setThreads(10);

		// schedule the shit out of them
		std::cout << "Start scheduling for II=30/7" << std::endl;
		u307.schedule();
		std::cout << "Start scheduling for II=9/2" << std::endl;
		u92.schedule();

		// check for solutions
		if (!u307.getScheduleFound()) {
			std::cout << "failed to find schedule for II=30/7" << std::endl;
			return false;
		}
		if (!u92.getScheduleFound()) {
			std::cout << "failed to find schedule for II=9/2" << std::endl;
			return false;
		}

		// do simple bindings
		auto binding307 = Binding::getSimpleRationalIIBinding(u307.getStartTimeVector(), &rm307, 30, 7);
		auto binding92 = Binding::getSimpleRationalIIBinding(u92.getStartTimeVector(), &rm92, 9, 2);

		// save results
		ScheduleAndBindingWriter writer307("fir_SAM_30_7.csv", u307.getStartTimeVector(), binding307, 7, 30);
		ScheduleAndBindingWriter writer92("fir_SAM_9_2.csv", u92.getStartTimeVector(), binding92, 2, 9);
		writer307.write();
		writer92.write();

		return true;
#endif
	}


  bool Tests::firSHIRatIIImplementationsTest() {
#ifndef USE_XERCESC
		cout << "Tests::firSHIRatIIImplementationsTest: XERCESC parsing library is not active! This test is disabled!" << endl;
		return false;
#else
		// read resource model and graph
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;
		string resStr = "benchmarks/origami/fir_SHIRM.xml";
		string graphStr = "benchmarks/origami/fir_SHI.graphml";
		HatScheT::XMLResourceReader readerRes(&rm);
		readerRes.readResourceModel(resStr.c_str());
		HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
		readerGraph.readGraph(graphStr.c_str());

		// create new resource model for minimum FUs with II=30/7=4.29 and II=9/2=4.5 (95.33% of max TP)
		ResourceModel rm75;
		ResourceModel rm32;
		for (auto *r : rm.Resources()) {
			auto vertices = rm.getVerticesOfResource(r);
			auto newLimit75 = (int)ceil(((double)vertices.size()) / (7.0/5.0));
			auto newLimit32 = (int)ceil(((double)vertices.size()) / (3.0/2.0));
			if(newLimit75 == 0) newLimit75 = 1;
			if(newLimit32 == 0) newLimit32 = 1;
			auto *r75 = &rm75.makeResource(r->getName(), newLimit75, r->getLatency(), r->getBlockingTime());
			auto *r32 = &rm32.makeResource(r->getName(), newLimit32, r->getLatency(), r->getBlockingTime());
			std::cout << "Resource '" << r->getName() << "' - 7/5 limit: " << newLimit75 << ", 3/2 limit: " << newLimit32 << std::endl;
			for (auto *v : vertices) {
				rm75.registerVertex(v, r75);
				rm32.registerVertex(v, r32);
			}
		}

		// ILP solvers
		std::list<std::string> sw = {"Gurobi", "CPLEX", "SCIP", "LPSolve"};

		// generate schedulers
		UniformRationalIISchedulerNew u75(g, rm75, sw);
		UniformRationalIISchedulerNew u32(g, rm32, sw);
		u75.setModulo(7);
		u75.setSamples(5);
		u32.setModulo(3);
		u32.setSamples(2);
		u75.setSolverTimeout(600);
		u32.setSolverTimeout(600);
		u75.setThreads(10);
		u32.setThreads(10);

		// schedule the shit out of them
		std::cout << "Start scheduling for II=7/5" << std::endl;
		u75.schedule();
		std::cout << "Start scheduling for II=3/2" << std::endl;
		u32.schedule();

		// check for solutions
		if (!u75.getScheduleFound()) {
			std::cout << "failed to find schedule for II=7/5" << std::endl;
			return false;
		}
		if (!u32.getScheduleFound()) {
			std::cout << "failed to find schedule for II=3/2" << std::endl;
			return false;
		}

		// do simple bindings
		auto binding75 = Binding::getSimpleRationalIIBinding(u75.getStartTimeVector(), &rm75, 7, 5);
		auto binding32 = Binding::getSimpleRationalIIBinding(u32.getStartTimeVector(), &rm32, 3, 2);

		// save results
		ScheduleAndBindingWriter writer75("fir_SHI_7_5.csv", u75.getStartTimeVector(), binding75, 5, 7);
		ScheduleAndBindingWriter writer32("fir_SHI_3_2.csv", u32.getStartTimeVector(), binding32, 2, 3);
		writer75.write();
		writer32.write();

		return true;
#endif
	}


  bool Tests::treeBindTest() {
#ifdef USE_SCALP
		// create scheduling problem
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;

		auto &memR = rm.makeResource("memR", UNLIMITED, 0, 1);
		auto &memW = rm.makeResource("memW", UNLIMITED, 0, 1);
		auto &mult = rm.makeResource("mult", 2, 1, 1);
		auto &cons = rm.makeResource("cons", UNLIMITED, 0, 1);

		auto &x0 = g.createVertex();
		x0.setName("x0");
		auto &x1 = g.createVertex();
		x1.setName("x1");
		auto &constant0_25 = g.createVertex();
		constant0_25.setName("constant0_25");
		auto &mult0 = g.createVertex();
		mult0.setName("mult0");
		auto &mult1 = g.createVertex();
		mult1.setName("mult1");
		auto &mult2 = g.createVertex();
		mult2.setName("mult2");
		auto &y0 = g.createVertex();
		y0.setName("y0");

		rm.registerVertex(&x0, &memR);
		rm.registerVertex(&x1, &memR);
		rm.registerVertex(&constant0_25, &cons);
		rm.registerVertex(&mult0, &mult);
		rm.registerVertex(&mult1, &mult);
		rm.registerVertex(&mult2, &mult);
		rm.registerVertex(&y0, &memW);

		std::map<Edge*,int> portAssignments;
		auto &e0 = g.createEdge(x0,mult0,0);
		portAssignments[&e0] = 0;
		auto &e1 = g.createEdge(mult2,mult0,4);
		portAssignments[&e1] = 1;
		auto &e2 = g.createEdge(x1,mult1,0);
		portAssignments[&e2] = 0;
		auto &e3 = g.createEdge(mult0,mult1,0);
		portAssignments[&e3] = 1;
		auto &e4 = g.createEdge(constant0_25,mult2,0);
		portAssignments[&e4] = 0;
		auto &e5 = g.createEdge(mult1,mult2,0);
		portAssignments[&e5] = 1;
		auto &e6 = g.createEdge(mult2,y0,0);
		portAssignments[&e6] = 0;

		// schedule that badboy
		std::list<std::string> sw = {"Gurobi"};
		int timeout = 5;
		std::map<Vertex*,int> sched;
		std::vector<std::map<Vertex*,int>> ratIISched;
		double intII = 2.0;
		double ratII = 1.5;
		int samples = 2;
		int modulo = 3;

		EichenbergerDavidson97Scheduler scheduler(g,rm,sw);
		scheduler.setQuiet(true);
		scheduler.setSolverTimeout(timeout);
		scheduler.schedule();
		sched = scheduler.getSchedule();
		intII = scheduler.getII();

		std::cout << "Integer-II Schedule with II = " << intII << ":" << std::endl;
		for(auto it : sched) {
			std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
		}
		
		// specify commutative operation types
		std::set<const Resource*> commutativeOps;
		//commutativeOps.insert(&mult);

		// call tree based binding function for integer IIs
		double wMux = 1.0;
		double wReg = 1.0;
		double maxMux = -1.0;
		double maxReg = -1.0;
		TreeBind tb(&g,&rm,sched,intII,portAssignments);
		tb.setMuxLimit(maxMux);
		tb.setRegLimit(maxReg);
		tb.setTimeout(timeout);
		tb.setQuiet(false);
		tb.bind();
		auto treeBind = tb.getBinding();
		auto treeGeneralBind = Utility::convertBindingContainer(&g, &rm, intII, treeBind, sched);
		std::cout << "tree-based binding finished" << std::endl;
		bool treeIIValid = verifyIntIIBinding(&g,&rm,sched,(int)intII,treeBind,commutativeOps);
		bool treeGeneralValid = verifyIntIIBinding(&g,&rm,sched,(int)intII,treeGeneralBind,commutativeOps);
		auto treeNum2x1Muxs = Utility::getNumberOfEquivalent2x1Muxs(treeBind.multiplexerCosts, &g, &rm);
		std::cout << "tree-based binding is " << ((treeIIValid and treeGeneralValid)?"":"not ") << "valid" << std::endl;
		if (not treeIIValid or not treeGeneralValid) return false;
		std::cout << "tree-based implementation multiplexer costs: " << treeBind.multiplexerCosts << std::endl;
		std::cout << "tree-based implementation number of 2x1 multiplexers: " << treeNum2x1Muxs << std::endl;
		std::cout << "tree-based implementation register costs: " << treeBind.registerCosts << std::endl;

		// compare with ILP-based optimal binding
		auto ilpBind = Binding::getILPBasedIntIIBinding(sched,&g,&rm,(int)intII,wMux,wReg,portAssignments,maxMux,maxReg,commutativeOps,{"Gurobi"},timeout,true);
		auto ilpGeneralBind = Utility::convertBindingContainer(&g, &rm, intII, ilpBind, sched);
		std::cout << "ILP-based binding finished" << std::endl;
		bool ilpValid = verifyIntIIBinding(&g,&rm,sched,(int)intII,ilpBind,commutativeOps);
		bool ilpGeneralValid = verifyIntIIBinding(&g,&rm,sched,(int)intII,ilpGeneralBind,commutativeOps);
		auto ilpNum2x1Muxs = Utility::getNumberOfEquivalent2x1Muxs(ilpBind.multiplexerCosts, &g, &rm);
		std::cout << "ILP-based binding is " << ((ilpValid and ilpGeneralValid)?"":"not ") << "valid" << std::endl;
		if (not ilpValid or not ilpGeneralValid) return false;
		std::cout << "ILP-based implementation multiplexer costs: " << ilpBind.multiplexerCosts << std::endl;
		std::cout << "ILP-based implementation number of 2x1 multiplexers: " << ilpNum2x1Muxs << std::endl;
		std::cout << "ILP-based implementation register costs: " << ilpBind.registerCosts << std::endl;

		return (ilpBind.registerCosts == treeBind.registerCosts) and (ilpBind.multiplexerCosts == treeBind.multiplexerCosts) and (treeNum2x1Muxs == ilpNum2x1Muxs);
#else
    return true;
#endif //USE_SCALP
  }

	bool Tests::treeBindCommutativeTest() {
#ifdef USE_SCALP
		// create scheduling problem
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;

		auto &memR = rm.makeResource("memR", 1, 1, 1);
		auto &memW = rm.makeResource("memW", 1, 1, 1);
		auto &mult = rm.makeResource("mult", 1, 1, 1);
		auto &cons = rm.makeResource("cons", UNLIMITED, 0, 1);

		auto &x0 = g.createVertex();
		x0.setName("x0");
		auto &x1 = g.createVertex();
		x1.setName("x1");
		auto &c = g.createVertex();
		c.setName("c");
		auto &mult0 = g.createVertex();
		mult0.setName("mult0");
		auto &mult1 = g.createVertex();
		mult1.setName("mult1");
		auto &y0 = g.createVertex();
		y0.setName("y0");
		auto &y1 = g.createVertex();
		y1.setName("y1");

		rm.registerVertex(&x0, &memR);
		rm.registerVertex(&x1, &memR);
		rm.registerVertex(&c, &cons);
		rm.registerVertex(&mult0, &mult);
		rm.registerVertex(&mult1, &mult);
		rm.registerVertex(&y0, &memW);
		rm.registerVertex(&y1, &memW);

		std::map<Edge*,int> portAssignments;
		auto &e0 = g.createEdge(x0,mult0,0);
		portAssignments[&e0] = 0;
		auto &e1 = g.createEdge(c,mult0,0);
		portAssignments[&e1] = 1;
		auto &e2 = g.createEdge(x1,mult1,0);
		portAssignments[&e2] = 1;
		auto &e3 = g.createEdge(c,mult1,0);
		portAssignments[&e3] = 0;
		auto &e4 = g.createEdge(mult0,y0,0);
		portAssignments[&e4] = 0;
		auto &e5 = g.createEdge(mult1,y1,0);
		portAssignments[&e5] = 0;

		// schedule that badboy
		std::map<Vertex*,int> sched;
		double intII = 2.0;
		sched[&x0] = 0;
		sched[&x1] = 1;
		sched[&c] = 0;
		sched[&mult0] = 1;
		sched[&mult1] = 2;
		sched[&y0] = 2;
		sched[&y1] = 3;

		std::cout << "Integer-II Schedule with II = " << intII << ":" << std::endl;
		for(auto it : sched) {
			std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
		}

		// specify commutative operation types
		std::set<const Resource*> commutativeOps;
		commutativeOps.insert(&mult);

		// call tree based binding function for integer IIs
		double wMux = 1.0;
		double wReg = 1.0;
		double maxMux = -1.0;
		double maxReg = -1.0;
		auto timeout = 10; //seconds
		TreeBind tb(&g,&rm,sched,intII,portAssignments,commutativeOps);
		tb.setMuxLimit(maxMux);
		tb.setRegLimit(maxReg);
		tb.setTimeout(timeout);
		tb.setQuiet(false);
		tb.bind();
		auto treeBind = tb.getBinding();
		auto treeGeneralBind = Utility::convertBindingContainer(&g, &rm, intII, treeBind, sched);
		std::cout << "tree-based binding finished" << std::endl;
		bool treeIIValid = verifyIntIIBinding(&g,&rm,sched,(int)intII,treeBind,commutativeOps);
		bool treeGeneralValid = verifyIntIIBinding(&g,&rm,sched,(int)intII,treeGeneralBind,commutativeOps);
		auto treeNum2x1Muxs = Utility::getNumberOfEquivalent2x1Muxs(treeBind.multiplexerCosts, &g, &rm);
		std::cout << "tree-based binding is " << ((treeIIValid and treeGeneralValid)?"":"not ") << "valid" << std::endl;
		if ((not treeIIValid) or (not treeGeneralValid)) return false;
		std::cout << "tree-based implementation multiplexer costs: " << treeBind.multiplexerCosts << std::endl;
		std::cout << "tree-based implementation number of 2x1 multiplexers: " << treeNum2x1Muxs << std::endl;
		std::cout << "tree-based implementation register costs: " << treeBind.registerCosts << std::endl;

		// compare with ILP-based optimal binding
		auto ilpBind = Binding::getILPBasedIntIIBinding(sched,&g,&rm,(int)intII,wMux,wReg,portAssignments,maxMux,maxReg,commutativeOps,{"Gurobi"},timeout,true);
		auto ilpGeneralBind = Utility::convertBindingContainer(&g, &rm, intII, ilpBind, sched);
		std::cout << "ILP-based binding finished" << std::endl;
		bool ilpValid = verifyIntIIBinding(&g,&rm,sched,(int)intII,ilpBind,commutativeOps);
		bool ilpGeneralValid = verifyIntIIBinding(&g,&rm,sched,(int)intII,ilpGeneralBind,commutativeOps);
		auto ilpNum2x1Muxs = Utility::getNumberOfEquivalent2x1Muxs(ilpBind.multiplexerCosts, &g, &rm);
		std::cout << "ILP-based binding is " << ((ilpValid and ilpGeneralValid)?"":"not ") << "valid" << std::endl;
		if ((not ilpValid) or (not ilpGeneralValid)) return false;
		std::cout << "ILP-based implementation multiplexer costs: " << ilpBind.multiplexerCosts << std::endl;
		std::cout << "ILP-based implementation number of 2x1 multiplexers: " << ilpNum2x1Muxs << std::endl;
		std::cout << "ILP-based implementation register costs: " << ilpBind.registerCosts << std::endl;

		return (ilpBind.registerCosts == treeBind.registerCosts) and (ilpBind.multiplexerCosts == treeBind.multiplexerCosts) and (treeNum2x1Muxs == ilpNum2x1Muxs);
#else
	    return true;
#endif //USE_SCALP
	}

	bool Tests::fccmPaperTest() {
		// create scheduling problem
		HatScheT::ResourceModel rm;
		HatScheT::Graph g;

		auto &memR = rm.makeResource("memR", UNLIMITED, 0, 1);
		auto &memW = rm.makeResource("memW", UNLIMITED, 0, 1);
		auto &mult = rm.makeResource("mult", 2, 1, 1);

		auto &x0 = g.createVertex();
		x0.setName("x0");
		auto &x1 = g.createVertex();
		x1.setName("x1");
		auto &x2 = g.createVertex();
		x2.setName("x2");
		auto &x3 = g.createVertex();
		x3.setName("x3");
		auto &x4 = g.createVertex();
		x4.setName("x4");
		auto &x5 = g.createVertex();
		x5.setName("x5");
		auto &x6 = g.createVertex();
		x6.setName("x6");
		auto &x7 = g.createVertex();
		x7.setName("x7");
		auto &mult0 = g.createVertex();
		mult0.setName("mult0");
		auto &mult1 = g.createVertex();
		mult1.setName("mult1");
		auto &mult2 = g.createVertex();
		mult2.setName("mult2");
		auto &mult3 = g.createVertex();
		mult3.setName("mult3");
		auto &mult4 = g.createVertex();
		mult4.setName("mult4");
		auto &mult5 = g.createVertex();
		mult5.setName("mult5");
		auto &mult6 = g.createVertex();
		mult6.setName("mult6");
		auto &y = g.createVertex();
		y.setName("y");

		rm.registerVertex(&x0, &memR);
		rm.registerVertex(&x1, &memR);
		rm.registerVertex(&x2, &memR);
		rm.registerVertex(&x3, &memR);
		rm.registerVertex(&x4, &memR);
		rm.registerVertex(&x5, &memR);
		rm.registerVertex(&x6, &memR);
		rm.registerVertex(&x7, &memR);
		rm.registerVertex(&mult0, &mult);
		rm.registerVertex(&mult1, &mult);
		rm.registerVertex(&mult2, &mult);
		rm.registerVertex(&mult3, &mult);
		rm.registerVertex(&mult4, &mult);
		rm.registerVertex(&mult5, &mult);
		rm.registerVertex(&mult6, &mult);
		rm.registerVertex(&y, &memW);

		std::map<Edge*,int> portAssignments;
		auto &e0 = g.createEdge(x0,mult0,0);
		portAssignments[&e0] = 0;
		auto &e1 = g.createEdge(x1,mult0,0);
		portAssignments[&e1] = 1;
		auto &e2 = g.createEdge(x2,mult1,0);
		portAssignments[&e2] = 0;
		auto &e3 = g.createEdge(x3,mult1,0);
		portAssignments[&e3] = 1;
		auto &e4 = g.createEdge(x4,mult2,0);
		portAssignments[&e4] = 0;
		auto &e5 = g.createEdge(x5,mult2,0);
		portAssignments[&e5] = 1;
		auto &e6 = g.createEdge(x6,mult3,0);
		portAssignments[&e6] = 0;
		auto &e7 = g.createEdge(x7,mult3,0);
		portAssignments[&e7] = 1;
		auto &e8 = g.createEdge(mult0,mult4,0);
		portAssignments[&e8] = 0;
		auto &e9 = g.createEdge(mult1,mult4,0);
		portAssignments[&e9] = 1;
		auto &e10 = g.createEdge(mult2,mult5,0);
		portAssignments[&e10] = 0;
		auto &e11 = g.createEdge(mult3,mult5,0);
		portAssignments[&e11] = 1;
		auto &e12 = g.createEdge(mult4,mult6,0);
		portAssignments[&e12] = 0;
		auto &e13 = g.createEdge(mult5,mult6,0);
		portAssignments[&e13] = 1;
		auto &e14 = g.createEdge(mult6,y,0);
		portAssignments[&e14] = 0;

		// schedule it
		EichenbergerDavidson97Scheduler scheduler(g,rm, {"Gurobi"});
		scheduler.setQuiet(true);
		scheduler.setSolverTimeout(60);
		scheduler.schedule();
		auto sched = scheduler.getSchedule();
		auto II = (int)scheduler.getII();

		// define schedule by hand (so an optimization by the binding algorithm is actually possible)
		/*
		sched[&x0] = 0;
		sched[&x1] = 0;
		sched[&x2] = 1;
		sched[&x3] = 1;
		sched[&x4] = 0;
		sched[&x5] = 0;
		sched[&x6] = 1;
		sched[&x7] = 1;
		sched[&mult0] = 1;
		sched[&mult1] = 2;
		sched[&mult2] = 1;
		sched[&mult3] = 2;
		sched[&mult4] = 3;
		sched[&mult5] = 3;
		sched[&mult6] = 4;
		sched[&y] = 5;
		 */
		sched[&x0] = 0;
		sched[&x1] = 0;
		sched[&x2] = 0;
		sched[&x3] = 0;
		sched[&x4] = 1;
		sched[&x5] = 1;
		sched[&x6] = 1;
		sched[&x7] = 1;
		sched[&mult0] = 0;
		sched[&mult1] = 0;
		sched[&mult2] = 1;
		sched[&mult3] = 1;
		sched[&mult4] = 2;
		sched[&mult5] = 3;
		sched[&mult6] = 6;
		sched[&y] = 7;

		// verify modulo schedule
		auto schedValid = verifyModuloSchedule(g, rm, sched, II);
		if (!schedValid) {
			std::cout << "Modulo schedule invalid - that should never happen..." << std::endl;
			return false;
		}

		std::cout << "Integer-II Schedule with II = " << II << ":" << std::endl;
		for(auto it : sched) {
			std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
		}

		// compute bindings
		double wMux = 1.0;
		double wReg = 1.0;
		double maxMux = -1.0;
		double maxReg = -1.0;
		auto timeout = 10; //seconds
		std::set<const Resource*> commutativeOps = {&mult};

		// set up algorithm
		TreeBind tb(&g,&rm,sched,II,portAssignments,commutativeOps);
		tb.setMuxCostFactor(wMux);
		tb.setRegCostFactor(wReg);
		tb.setMuxLimit(maxMux);
		tb.setRegLimit(maxReg);
		tb.setTimeout(timeout);
		tb.setQuiet(false);

		// calculate upper bounds for multiplexers and registers
		auto maxPair = Utility::getMaxRegsAndMuxs(&g, &rm, sched, II);

		// minimization
		tb.setObjective(Binding::objective::minimize);
		tb.bind();
		auto minTreeBind = tb.getBinding();
		bool minTreeValid = verifyIntIIBinding(&g,&rm,sched,II,minTreeBind,commutativeOps);
		auto minTreeNum2x1Muxs = Utility::getNumberOfEquivalent2x1Muxs(minTreeBind.multiplexerCosts, &g, &rm);

		// minimization
		tb.setObjective(Binding::objective::maximize);
		tb.bind();
		auto maxTreeBind = tb.getBinding();
		bool maxTreeValid = verifyIntIIBinding(&g,&rm,sched,II,maxTreeBind,commutativeOps);
		auto maxTreeNum2x1Muxs = Utility::getNumberOfEquivalent2x1Muxs(maxTreeBind.multiplexerCosts, &g, &rm);

		// print bindings
		std::cout << "binding with minimal costs:" << std::endl;
		for (auto &it : minTreeBind.resourceBindings) {
			std::cout << "  " << it.first << " - " << it.second << std::endl;
		}
		std::cout << "binding with maximal costs:" << std::endl;
		for (auto &it : maxTreeBind.resourceBindings) {
			std::cout << "  " << it.first << " - " << it.second << std::endl;
		}

		// print results
		std::cout << "Upper bounds for binding problem:" << std::endl;
		std::cout << "  " << maxPair.first << " registers" << std::endl;
		std::cout << "  " << maxPair.second << " multiplexers" << std::endl;
		std::cout << "minimization results:" << std::endl;
		std::cout << "  binding is " << (minTreeValid?"":"not ") << "valid" << std::endl;
		std::cout << "  multiplexer costs: " << minTreeBind.multiplexerCosts << std::endl;
		std::cout << "  number of 2x1 multiplexers: " << minTreeNum2x1Muxs << std::endl;
		std::cout << "  register costs: " << minTreeBind.registerCosts << std::endl;
		std::cout << "maximization results:" << std::endl;
		std::cout << "  binding is " << (maxTreeValid?"":"not ") << "valid" << std::endl;
		std::cout << "  multiplexer costs: " << maxTreeBind.multiplexerCosts << std::endl;
		std::cout << "  number of 2x1 multiplexers: " << maxTreeNum2x1Muxs << std::endl;
		std::cout << "  register costs: " << maxTreeBind.registerCosts << std::endl;

		return minTreeValid and maxTreeValid;
	}

	bool Tests::multiMinRegSchedulerTest() {
		HatScheT::Graph g;
		HatScheT::ResourceModel rm;

		auto &red = rm.makeResource("red", 3, 2, 1);
		auto &green = rm.makeResource("green", 5, 1, 1);

		// critical resource (#vertices=5, limit=3)
		// loop: latency=6, distance=5
		Vertex &r0 = g.createVertex(0);
		Vertex &r1 = g.createVertex(1);
		Vertex &r2 = g.createVertex(2);
		Vertex &r3 = g.createVertex(3);
		Vertex &r4 = g.createVertex(4);
		rm.registerVertex(&r0, &red);
		rm.registerVertex(&r1, &red);
		rm.registerVertex(&r2, &red);
		rm.registerVertex(&r3, &red);
		rm.registerVertex(&r4, &red);
		g.createEdge(r0, r2, 0);
		g.createEdge(r1, r2, 0);
		g.createEdge(r2, r3, 0);
		g.createEdge(r3, r4, 0);
		g.createEdge(r3, r0, 5);

		// non-critical resource (#vertices=22, limit=5)
		// loop: latency=4, distance=3
		Vertex &g5 = g.createVertex(5);
		Vertex &g6 = g.createVertex(6);
		Vertex &g7 = g.createVertex(7);
		Vertex &g8 = g.createVertex(8);
		Vertex &g9 = g.createVertex(9);
		Vertex &g10 = g.createVertex(10);
		Vertex &g11 = g.createVertex(11);
		Vertex &g12 = g.createVertex(12);
		Vertex &g13 = g.createVertex(13);
		Vertex &g14 = g.createVertex(14);
		Vertex &g15 = g.createVertex(15);
		Vertex &g16 = g.createVertex(16);
		Vertex &g17 = g.createVertex(17);
		Vertex &g18 = g.createVertex(18);
		Vertex &g19 = g.createVertex(19);
		Vertex &g20 = g.createVertex(20);
		Vertex &g21 = g.createVertex(21);
		Vertex &g22 = g.createVertex(22);
		Vertex &g23 = g.createVertex(23);
		Vertex &g24 = g.createVertex(24);
		Vertex &g25 = g.createVertex(25);
		Vertex &g26 = g.createVertex(26);
		rm.registerVertex(&g5, &green);
		rm.registerVertex(&g6, &green);
		rm.registerVertex(&g7, &green);
		rm.registerVertex(&g8, &green);
		rm.registerVertex(&g9, &green);
		rm.registerVertex(&g10, &green);
		rm.registerVertex(&g11, &green);
		rm.registerVertex(&g12, &green);
		rm.registerVertex(&g13, &green);
		rm.registerVertex(&g14, &green);
		rm.registerVertex(&g15, &green);
		rm.registerVertex(&g16, &green);
		rm.registerVertex(&g17, &green);
		rm.registerVertex(&g18, &green);
		rm.registerVertex(&g19, &green);
		rm.registerVertex(&g20, &green);
		rm.registerVertex(&g21, &green);
		rm.registerVertex(&g22, &green);
		rm.registerVertex(&g23, &green);
		rm.registerVertex(&g24, &green);
		rm.registerVertex(&g25, &green);
		rm.registerVertex(&g26, &green);
		g.createEdge(g5, g7, 0);
		g.createEdge(g12, g7, 0);
		g.createEdge(g13, g7, 0);
		g.createEdge(g14, g7, 0);
		g.createEdge(g6, g7, 0);
		g.createEdge(g7, g8, 0);
		g.createEdge(g8, g9, 0);
		g.createEdge(g9, g6, 3);
		g.createEdge(g9, g10, 0);
		g.createEdge(g9, g11, 0);

		MinRegMultiScheduler m(g, rm, {"Gurobi", "CPLEX", "SCIP", "LPSolve"});
		m.setQuiet(false);
		m.setSolverQuiet(true);
		m.setThreads(24);
		m.setSolverTimeout(300);
		m.setMaxRuns(1);

		std::cout << "Start reg min scheduling with multiple assignments" << std::endl;
		m.disableMultipleStartTimes();
		m.schedule();
		auto validMulti = m.validateScheduleAndBinding();
		auto numLifetimeRegsMulti = m.getNumLifetimeRegs();

		std::cout << "Start reg min scheduling with single assignments" << std::endl;
		m.enableMultipleStartTimes();
		m.schedule();
		auto validSingle = m.validateScheduleAndBinding();
		auto numLifetimeRegsSingle = m.getNumLifetimeRegs();

		std::cout << "#Lifetime regs (multi)  = " << numLifetimeRegsMulti  << std::endl;
		std::cout << "#Lifetime regs (single) = " << numLifetimeRegsSingle << std::endl;

		return validMulti and validSingle;
	}

	bool Tests::satSchedulerTest() {
#ifdef USE_CADICAL
		HatScheT::Graph g;
		HatScheT::ResourceModel rm;

		auto &red = rm.makeResource("red", 3, 2, 1); // rm.makeResource("red", 3, 2, 1);
		auto &green = rm.makeResource("green", UNLIMITED, 0, 1); // rm.makeResource("green", 5, 1, 1);

		// non-critical resource (#vertices=5, limit=3)
		// loop: latency=6, distance=5
		Vertex &r0 = g.createVertex(0);
		Vertex &r1 = g.createVertex(1);
		Vertex &r2 = g.createVertex(2);
		Vertex &r3 = g.createVertex(3);
		Vertex &r4 = g.createVertex(4);
		rm.registerVertex(&r0, &red);
		rm.registerVertex(&r1, &red);
		rm.registerVertex(&r2, &red);
		rm.registerVertex(&r3, &red);
		rm.registerVertex(&r4, &red);
		g.createEdge(r0, r2, 0);
		g.createEdge(r1, r2, 0);
		g.createEdge(r2, r3, 0);
		g.createEdge(r3, r4, 0);
		g.createEdge(r0, r4, 0);
		g.createEdge(r3, r0, 5);

		Vertex &g5 = g.createVertex(5);
		rm.registerVertex(&g5, &green);
		g.createEdge(g5, r4, 0);

		SATScheduler m(g, rm);
		m.setQuiet(false);
		m.setMaxRuns(1);
		m.setSolverTimeout(300);

		std::cout << "Start SAT scheduling" << std::endl;
		m.schedule();
		auto &schedule = m.getSchedule();
		auto II = m.getII();
		auto SL = m.getScheduleLength();
		std::cout << "II = " << II << " (schedule length = " << SL << ")" << std::endl;
		std::cout << "schedule: " << std::endl;
		for (auto &it : schedule) {
			std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
		}
		if (II < 1) {
			std::cout << "Invalid II" << std::endl;
			return false;
		}
		auto valid = verifyModuloSchedule(g, rm, schedule, II);
		if (!valid) {
			std::cout << "Invalid schedule" << std::endl;
			return false;
		}
		auto expectedII = 2;
		auto expectedSL = 9;
		if (II != expectedII) {
			std::cout << "Expected II = " << expectedII << " but got II = " << II << std::endl;
			return false;
		}
		if (SL != expectedSL) {
			std::cout << "Expected schedule length = " << expectedSL << " but got schedule length = " << SL << std::endl;
			return false;
		}

		// calculate the minimum number of registers for the schedule
		std::map<int, int> numAlive;
		for (auto &vSrc : g.Vertices()) {
			auto tSrc = schedule.at(vSrc);
			auto lSrc = rm.getVertexLatency(vSrc);
			auto latestReadTime = tSrc + lSrc;
			for (auto &e : g.Edges()) {
				if (&e->getVertexSrc() != vSrc) continue;
				auto vDst = &e->getVertexDst();
				auto tDst = schedule.at(vDst);
				auto readTime = tDst + (e->getDistance() * II);
				if (readTime > latestReadTime) latestReadTime = readTime;
			}
			for (int t=tSrc+lSrc+1; t<=latestReadTime; t++) {
				numAlive[t % (int)II]++;
			}
		}
		int numRegs = 0;
		for (auto &it : numAlive) {
			if (it.second > numRegs) numRegs = it.second;
		}

		// now minimize the number of registers for the given II and latency limit
		SATMinRegScheduler s(g, rm);
		s.setQuiet(false);
		s.overrideII(II);
		s.setMaxLatencyConstraint(SL);
		s.setRegMax(numRegs);
		s.setSolverTimeout(600);

		std::cout << "Start SAT min reg scheduling" << std::endl;
		s.schedule();
		if (!s.getScheduleFound()) {
			std::cout << "SATMinRegScheduler failed to find solution - test failed!" << std::endl;
			return false;
		}
		auto &minRegSchedule = s.getSchedule();
		auto minRegII = s.getII();
		auto minRegSL = s.getScheduleLength();
		auto minRegNumRegs = s.getNumRegs();
		std::cout << "min reg II = " << minRegII << ", schedule length = " << minRegSL << " and #Regs = " << minRegNumRegs << std::endl;
		std::cout << "min reg schedule: " << std::endl;
		for (auto &it : minRegSchedule) {
			std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
		}
		if (minRegII < 1) {
			std::cout << "Invalid min reg II" << std::endl;
			return false;
		}
		auto expectedMinReg = 5;
		if (minRegNumRegs != expectedMinReg) {
			std::cout << "Expected min #Regs = " << expectedMinReg << " but got #Regs = " << II << std::endl;
			return false;
		}
		auto minRegValid = verifyModuloSchedule(g, rm, minRegSchedule, minRegII);
		if (!valid) {
			std::cout << "Invalid min reg schedule" << std::endl;
			return false;
		}
		std::cout << "Passed test :)" << std::endl;
		return true;
#else
		std::cout << "Enable CaDiCaL for SAT-based schedulers" << std::endl;
		return true;
#endif
	}

  bool Tests::z3Test() {
	  #ifdef USE_Z3
	      /*!
          * Demonstration of how Z3 can be used to prove validity of
          * De Morgan's Duality Law: {e not(x and y) <-> (not x) or ( not y) }
          */
	      using namespace z3;

          std::cout << "de-Morgan example\n";

          context c;

          expr x = c.bool_const("x");
          expr y = c.bool_const("y");
          expr conjecture = (!(x && y)) == (!x || !y);

          solver s(c);
          // adding the negation of the conjecture as a constraint.
          s.add(!conjecture);
          std::cout << s << "\n";
          std::cout << s.to_smt2();
          std::cout << s.check() << "\n";
          switch (s.check()) {
              case unsat:   std::cout << "\nDe-Morgan is valid\n"; break;
              case sat:     std::cout << "\nDe-Morgan is not valid\n"; return false;
              default:      std::cout << "\nunknown\n"; return false;
          }

          return true;
      #else
          //Z3 not active! Test function disabled!
          return true;
	  #endif
  }

  bool Tests::smtModSchedulerTest() {
      #ifdef USE_Z3

	      HatScheT::Graph g;
	      HatScheT::ResourceModel rm;

          auto &green = rm.makeResource("green", 2, 1, 1);

	      Vertex &o0 = g.createVertex(0);
	      Vertex &o1 = g.createVertex(1);
          Vertex &o2 = g.createVertex(2);
          Vertex &o3 = g.createVertex(3);
          Vertex &o4 = g.createVertex(4);
          Vertex &o5 = g.createVertex(5);
          Vertex &o6 = g.createVertex(6);
          Vertex &o7 = g.createVertex(7);

          rm.registerVertex(&o0, &green);
          rm.registerVertex(&o1, &green);
          rm.registerVertex(&o2, &green);
          rm.registerVertex(&o3, &green);
          rm.registerVertex(&o4, &green);
          rm.registerVertex(&o5, &green);
          rm.registerVertex(&o6, &green);
          rm.registerVertex(&o7, &green);

          g.createEdge(o0, o1, 0);
          g.createEdge(o1, o2, 0);
          g.createEdge(o2, o3, 0);
          g.createEdge(o3, o1, 2);
          g.createEdge(o0, o5, 0);
          g.createEdge(o4, o1, 0);
          g.createEdge(o4, o5, 0);
          g.createEdge(o5, o6, 0);
          g.createEdge(o6, o5, 2);
          g.createEdge(o6, o3, 0);
          g.createEdge(o7, o5, 0);

          SMTModScheduler smt(g, rm);

	      int ii = (int)smt.getII();
	      cout << "II: " << ii << endl;

	      smt.setQuiet(false);
	      smt.schedule();

          auto sched = smt.getSchedule();

          cout << endl;
          for(auto &it : sched) {
              std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
          }

          ii = (int)smt.getII();
          cout << "II: " << ii << endl;

          auto valid = verifyModuloSchedule(g, rm, sched, ii);
          if (!valid) {
              std::cout << "Tests::smtModScheduler: invalid modulo schedule found" << std::endl;
              return false;
          }
          std::cout << "Tests::smtModScheduler: valid modulo schedule found. :-)" << std::endl;
          return true;
      #else
	      cout << "Z3 Solver not found, test disabled." << std::endl;
	      return true;
      #endif
  }

  //TODO Remove Test
  bool Tests::smtVsED97Test() {
#ifdef USE_Z3
      HatScheT::Graph g;
      HatScheT::ResourceModel rm;

      clock_t start, end;


      HatScheT::XMLResourceReader readerRes(&rm);
      string resStr = "benchmarks/origami/fir_SHIRM.xml";
      string graphStr = "benchmarks/origami/fir_SHI.graphml";
      readerRes.readResourceModel(resStr.c_str());
      HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
      readerGraph.readGraph(graphStr.c_str());

      /*for (auto &it : rm.Resources()){
          it->setLimit(UNLIMITED);
      }*/

      //Simple IIR-Filter:
      /*auto &Sum = rm.makeResource("Sum", 1, 1, 1);
      auto &Product = rm.makeResource("Product", 1, 1, 1);
      auto &Constant = rm.makeResource("constant", UNLIMITED, 1, 1);
      auto &LoadStore = rm.makeResource("L_S", 1, 1, 1);

      Vertex &IN = g.createVertex(0);
      Vertex &SUM_0 = g.createVertex(1);
      Vertex &PROD_1 = g.createVertex(2);
      Vertex &PROD_0 = g.createVertex(3);
      Vertex &CONST_A = g.createVertex(4);
      Vertex &CONST_B = g.createVertex(5);
      Vertex &SUM_1 = g.createVertex(6);
      Vertex &OUT = g.createVertex(7);

      IN.setName("IN");
      SUM_0.setName("SUM_0");
      PROD_1.setName("PROD_1");
      PROD_0.setName("PROD_0");
      CONST_A.setName("CONST_A");
      CONST_B.setName("CONST_B");
      SUM_1.setName("SUM_1");
      OUT.setName("OUT");

      rm.registerVertex(&IN, &LoadStore);
      rm.registerVertex(&SUM_0, &Sum);
      rm.registerVertex(&PROD_1, &Product);
      rm.registerVertex(&PROD_0, &Product);
      rm.registerVertex(&CONST_A, &Constant);
      rm.registerVertex(&CONST_B, &Constant);
      rm.registerVertex(&SUM_1, &Sum);
      rm.registerVertex(&OUT, &LoadStore);

      g.createEdge(IN, SUM_0, 0);
      g.createEdge(SUM_0, PROD_0, 1);
      g.createEdge(SUM_0, PROD_1, 1);
      g.createEdge(SUM_0, SUM_1, 0);
      g.createEdge(CONST_B, PROD_1, 0);
      g.createEdge(CONST_A, PROD_0, 0);
      g.createEdge(PROD_0, SUM_0, 0);
      g.createEdge(PROD_1, SUM_1, 0);
      g.createEdge(SUM_1, OUT, 0);*/


      EichenbergerDavidson97Scheduler es(g, rm, {"CPLEX", "Gurobi", "SCIP", "LPSolve"});
      SMTModScheduler smt(g, rm);

      es.setQuiet(true);
      start = clock();
      es.schedule();
      end = clock();
      cout << es.getII() << endl;
      auto sched = es.getSchedule();
      auto ii = es.getII();
      auto valid = verifyModuloSchedule(g, rm, sched, ii);
      for (auto &it : sched){
          cout << it.first->getName() << " : " << it.second << endl;
      }
      if (!valid) {
          std::cout << "Tests::ED97Scheduler: invalid modulo schedule found" << std::endl;
      }else {
          std::cout << "Tests::ED97Scheduler: valid modulo schedule found. :-) II=" << ii  << std::endl << setprecision(5);
      }

      cout << "Time taken by ED97 is : " << fixed
           << double(end - start) / double(CLOCKS_PER_SEC) << setprecision(5);
      cout << " sec " << endl;

      smt.setQuiet(true);
      start = clock();
      smt.schedule();
      end = clock();
      sched = smt.getSchedule();

      for (auto &it : sched){
          cout << it.first->getName() << " : " << it.second << endl;
      }

      ii = smt.getII();

      valid = verifyModuloSchedule(g, rm, sched, ii);
      if (!valid) {
          std::cout << "Tests::smtModScheduler: invalid modulo schedule found" << std::endl;
          return false;
      }
      std::cout << "Tests::smtModScheduler: valid modulo schedule found. :-) II=" << ii << std::endl;
      cout << "Time taken by smt is : " << fixed
           << double(end - start) / double(CLOCKS_PER_SEC) << setprecision(5);
      cout << " sec " << endl;
      return true;
#else
      return true;
#endif
  }
}
