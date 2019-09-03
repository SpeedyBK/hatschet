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

#include "HatScheT/utility/Tests.h"
#include "HatScheT/utility/reader/GraphMLGraphReader.h"
#include "HatScheT/utility/reader/XMLResourceReader.h"
#include "HatScheT/utility/reader/XMLTargetReader.h"
#include "HatScheT/utility/writer/GraphMLGraphWriter.h"
#include "HatScheT/scheduler/ilpbased/MoovacScheduler.h"
#include "HatScheT/scheduler/ilpbased/ModuloSDCScheduler.h"
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

#include <stdio.h>

namespace HatScheT {

bool Tests::moovacTest()
{
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

  if(g.getNumberOfVertices()!=6){
    cout << "graph not read correctly! expected 6 vertices but got " << g.getNumberOfVertices() << endl;
    return false;
  }

  int maxLatencyConstraint = 18;
  HatScheT::MoovacScheduler sched(g, rm, {"CPLEX", "Gurobi"});
  sched.setMaxLatencyConstraint(maxLatencyConstraint);
  sched.setSolverQuiet(false);

  cout << "starting moovac scheduling" << endl;
  sched.schedule();
  cout << "finished moovac scheduling" << endl;

  if(sched.getII() != 5){
    cout << "Wrong II determined: " << sched.getII() << " instead of 5!" << endl;
    return false;
  }

  if(sched.getNoOfImplementedRegisters() != 9){
    cout << "Wrong number of registers determined: " << sched.getNoOfImplementedRegisters() << " instead of 9!" << endl;
    return false;
  }

  return true;
#endif
}

bool Tests::asapHCTest()
{
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

  HatScheT::ASAPScheduler asap(g,rm);
  asap.schedule();

  if(HatScheT::verifyModuloSchedule(g, rm, asap.getSchedule(), asap.getII())) return true;

  cout << "Tests::asapHCTest: Test Failed! Schedule is: " << endl;
  cout << g << endl;
  cout << rm << endl;
  HatScheT::Utility::printSchedule(asap.getSchedule());
  cout << "Tests::asapHCTest: asap HC failed verification!" << endl;

  return false;
#endif
}

bool Tests::alapHCTest()
{
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

  HatScheT::ALAPScheduler alap(g,rm);
  alap.schedule();

  if(HatScheT::verifyModuloSchedule(g, rm, alap.getSchedule(), alap.getII())) return true;

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
  double recMinII = HatScheT::Utility::calcRecMII(&g,&rm);

  double epsilon1 = resMinII - ((double)10)/((double)3);
  double epsilon2 = recMinII - ((double)16)/((double)3);

  if(epsilon1 > 0.001f or epsilon1 < -0.001f) {
    cout << "resMinII is: " << to_string(resMinII) << " instead of 10/3 =" << to_string(((double)10)/((double)3) ) << endl;
    return false;
  }
  if(epsilon2 > 0.001f or epsilon2 < -0.001f) {
    cout << "recMinII is: " << to_string(recMinII) << " instead of 16/3 =" << to_string(((double)16)/((double)3) ) << endl;
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
  HatScheT::MoovacScheduler  ms1(g,rm, {"CPLEX","Gurobi"});
  ms1.schedule();
  int IIOrg = ms1.getII();

  //writer
  string writePath="test.graphml";
  string writeResourcePath="test.xml";
  HatScheT::XMLResourceWriter writerResource(writeResourcePath,&rm);
  writerResource.write();
  HatScheT::GraphMLGraphWriter writerGraph(writePath,&g, &rm);
  writerGraph.write();

  //reader 2
  HatScheT::Graph g2;
  HatScheT::ResourceModel rm2;
  HatScheT::XMLResourceReader readerRes2(&rm2);
  readerRes2.readResourceModel(writeResourcePath.c_str());
  HatScheT::GraphMLGraphReader readerGraph2(&rm2, &g2);
  readerGraph2.readGraph(writePath.c_str());

  //moovac write read graph
  HatScheT::MoovacScheduler  ms2(g2,rm2, {"CPLEX","Gurobi"});
  ms2.schedule();
  int IIWriteRead = ms2.getII();

  //cleanup
  if( remove( "test.graphml" ) != 0 ){
    cout << "Test.readWriteReadScheduleTest: Error deleting File during cleanup process: test.graphml!" << endl;
    return false;
  }
  if( remove( "test.xml" ) != 0 ){
    cout << "Test.readWriteReadScheduleTest: Error deleting File during cleanup process: test.xml!" << endl;
    return false;
  }

  if(IIOrg != IIWriteRead){
    cout << "Test.readWriteReadScheduleTest: Error after write and read a differen II was determined!" << endl;
    cout << "Test.readWriteReadScheduleTest: Org II " << IIOrg << endl;
    cout << "Test.readWriteReadScheduleTest: WriteRead II " << IIWriteRead << endl;
    return false;
  }

  return true;
#endif
}

bool Tests::cpTest(){
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

  int cp = Utility::getCriticalPath(&g,&rm);

  if(cp != 8){
    cout << "Tests.cpTest: critital path calculated: " << cp << endl;
    cout << "Tests.cpTest: the correct value is 8! " << endl;
    return false;
  }
  for(auto it=rm.resourcesBegin(); it!=rm.resourcesEnd(); ++it){
    Resource* r = *it;
    if(r->isUnlimited()==true){
      cout << "Tests.cpTest: unlimited resource detected after critical path calculation: " << r->getName() << endl;
      cout << "Tests.cpTest: This should never happen! Old limits should be restored by the utility function!" << endl;
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

  if(rm.getNumResources() != 3){
    cout << "Incorrect no of resource read: " << rm.getNumResources() << " instead of 3!" << endl;
    return false;
  }

  for(auto it = rm.resourcesBegin(); it!= rm.resourcesEnd(); ++it){
    Resource* r = *it;

    if(r->getName()=="Adder"){
      if(r->getHardwareCost("LUT")!=275){
        cout << "Incorrect LUT costs found for Adder: " << r->getHardwareCost("LUTS") << " instead of 275" << endl;
        return false;
      }
    }

    if(r->getName()=="Multiplier"){
      if(r->getHardwareCost("DSP")!=1){
        cout << "Incorrect DSP costs found for Multiplier: " << r->getHardwareCost("DSPs") << " instead of 1" << endl;
        return false;
      }
    }

    if(r->getName()=="Gain"){
      if(r->getHardwareCost("LUT")!=64){
        cout << "Incorrect LUT costs found for Gain: " << r->getHardwareCost("LUTS") << " instead of 64" << endl;
        return false;
      }
    }
  }

  if(g.getNumberOfVertices() != 11){
    cout << "Incorrect no of vertices read: " << g.getNumberOfVertices() << " instead of 11!" << endl;
    return false;
  }

  if(target.getFamily() != "virtex6"){
    cout << "Incorrec family found: " << target.getFamily() << " instead of virtex6" << endl;
    return false;
  }

  if(target.getName() != "XC6VLX75T"){
    cout << "Incorrect name found: " << target.getName() << " instead of XC6VLX75T" << endl;
    return false;
  }

  if(target.getVendor() != "xilinx"){
    cout << "Incorrect vendor found: " << target.getVendor() << " instead of xilinx" << endl;
    return false;
  }

  if(target.getElement("LUT") != 46560){
    cout << "Incorrect target LUTs found: " << target.getElement("LUT") << " instead of 46560" << endl;
    return false;
  }

  if(target.getElement("DSP") != 288){
    cout << "Incorrect target DSP found: " << target.getElement("DSP") << " instead of 288" << endl;
    return false;
  }

  if(target.getElement("MEM") != 312){
    cout << "Incorrect target MEMs found: " << target.getElement("MEM") << " instead of 312" << endl;
    return false;
  }

  return true;
#endif
}

bool Tests::moduloSDCTest()
{
  try
  {
    HatScheT::ResourceModel rm;
    
    auto &load = rm.makeResource("load", 1, 2, 1);
    auto &add = rm.makeResource("add", -1, 0, 1);

    HatScheT::Graph g;
    
    Vertex& a = g.createVertex(1);
    Vertex& b = g.createVertex(2);
    Vertex& c = g.createVertex(3);
    Vertex& d = g.createVertex(4);

    a.setName("a");
    b.setName("b");
    c.setName("c");
    d.setName("d");

    g.createEdge(a, c ,0);
    g.createEdge(b, c ,0);
    g.createEdge(c, d ,0);
    g.createEdge(d, a ,1);
    
    rm.registerVertex(&a, &load);
    rm.registerVertex(&b, &load);
    rm.registerVertex(&c, &add);
    rm.registerVertex(&d, &load);

    HatScheT::ModuloSDCScheduler m{g,rm,{"CPLEX","Gurobi", "SCIP"}};
    m.setSolverQuiet(true);
    m.setVerbose(true);
    m.schedule();

    auto sch = m.getSchedule();

    bool result = true;
    for(auto&p:sch)
    {
      std::cout << p.first->getName() << " = " << p.second << std::endl;
    }

    if(verifyModuloSchedule(g,rm,sch,m.getII())==false) return false;
    if(m.getII()!=4) return false;
    return result;
  }
  catch(HatScheT::Exception &e)
  {
    std::cout << e.msg << std::endl;
  }
  return false;
}

bool Tests::apiTest()
{
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
      case 3: case 4: case 5: case 6:
        rm.registerVertex(&v, &chained);
        break;
      case 7: case 9: case 11:
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

bool Tests::ulSchedulerTest()
{
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

  HatScheT::ULScheduler uls(g,rm);
  uls.schedule();

  HatScheT::Utility::printSchedule(uls.getSchedule());
  std::map<Vertex *, int> &schedule = uls.getSchedule();
  int foundII = uls.getII();
  bool verified = HatScheT::verifyModuloSchedule(g, rm, schedule, foundII);

  if(verified==false) return false;
  if(uls.getScheduleLength()!=8) return false;

  return true;
#endif
}

bool Tests::moduloSDCTestFiege() {
  try
  {
	HatScheT::ResourceModel rm;

	auto &load = rm.makeResource("load", 3, 2, 1);
	auto &add = rm.makeResource("add", -1, 0, 1);

	HatScheT::Graph g;

	Vertex& a = g.createVertex(1);
	Vertex& b = g.createVertex(2);
    Vertex& c1 = g.createVertex(3);
    Vertex& d1 = g.createVertex(4);
    Vertex& e1 = g.createVertex(5);
	Vertex& f1 = g.createVertex(6);
	Vertex& g1 = g.createVertex(7);
    Vertex& c2 = g.createVertex(8);
    Vertex& d2 = g.createVertex(9);
    Vertex& e2 = g.createVertex(10);
    Vertex& f2 = g.createVertex(11);
	Vertex& g2 = g.createVertex(12);
	Vertex& h2 = g.createVertex(13);
	Vertex& i2 = g.createVertex(14);
	Vertex& j2 = g.createVertex(15);
	Vertex& k2 = g.createVertex(16);
	Vertex& l2 = g.createVertex(17);
	Vertex& m2 = g.createVertex(18);
	Vertex& n2 = g.createVertex(19);
    Vertex& o2 = g.createVertex(20);
    Vertex& p2 = g.createVertex(21);

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

    g.createEdge(a, b ,0);
    g.createEdge(b, c1 ,0);
    g.createEdge(b, c2 ,0);
    g.createEdge(c1, d1 ,0);
    g.createEdge(d1, e1 ,0);
    g.createEdge(e1, f1 ,0);
    g.createEdge(f1, c1 ,1);
	g.createEdge(f1, g1 ,0);
    g.createEdge(c2, d2 ,0);
    g.createEdge(d2, e2 ,0);
    g.createEdge(e2, f2 ,0);
    g.createEdge(f2, c2 ,1);
	g.createEdge(f2, g2 ,0);
	g.createEdge(g2, h2 ,0);
	g.createEdge(h2, i2 ,0);
	g.createEdge(i2, j2 ,0);
	g.createEdge(j2, k2 ,0);
	g.createEdge(k2, h2 ,1);
	g.createEdge(k2, l2 ,0);
	g.createEdge(l2, m2 ,0);
	g.createEdge(m2, h2 ,1);
	g.createEdge(k2, n2 ,0);
    g.createEdge(l2, o2 ,0);
    g.createEdge(m2, p2 ,0);

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

	std::list<std::string> solverList = {"CPLEX","Gurobi","SCIP"};
	HatScheT::ModSDC m(g,rm,solverList);
	m.setPriorityType(PriorityHandler::priorityType::ALASUB);
	m.setSolverQuiet(true);
	m.schedule();

	auto sch = m.getSchedule();

	for(auto&p:sch)
	{
	  std::cout << p.first->getName() << " = " << p.second << std::endl;
	}
	std::cout << "latency = " << m.getScheduleLength() << std::endl;

	if(!verifyModuloSchedule(g,rm,sch,(int)m.getII())){
		return false;
	}

	auto binding = m.getBindings();
	cout << "Binding: " << endl;
	for(auto it : binding) {
		cout << it.first->getName() << ": " << rm.getResource(it.first)->getName() << " " << it.second << endl;
	}
  }
  catch(HatScheT::Exception &e)
  {
	std::cout << e.msg << std::endl;
  }
  return false;
}

  bool Tests::KosarajuTest(){

    HatScheT::Graph KosaGr;
    HatScheT::ResourceModel rm;
    //string graphmlpath = "/home/bkessler/Projects/HatScheT_Debug/KosajaruTest.graphml";
    //string transposedpath = "/home/bkessler/Projects/HatScheT_Debug/TransposedTest.graphml";

    auto &add = rm.makeResource("add", -1, 1, 1);
    auto &mult = rm.makeResource("mult", 1, 1, 1);

    //-----------------------------------------------------------------------------------------------------------------
    //Hardcoding the examplegraph from https://www.geeksforgeeks.org/strongly-connected-components/

    /*Vertex& A = KosaGr.createVertex(0);
    Vertex& B = KosaGr.createVertex(1);
    Vertex& C = KosaGr.createVertex(2);
    Vertex& D = KosaGr.createVertex(3);
    Vertex& E = KosaGr.createVertex(4);
    Vertex& F = KosaGr.createVertex(5);
    Vertex& G = KosaGr.createVertex(6);
    Vertex& H = KosaGr.createVertex(7);
    Vertex& I = KosaGr.createVertex(8);

    KosaGr.createEdge(A, B ,0);
    KosaGr.createEdge(B, C ,0);
    KosaGr.createEdge(C, D ,0);
    KosaGr.createEdge(C, E ,0);
    KosaGr.createEdge(D, A ,1);
    KosaGr.createEdge(E, F ,0);
    KosaGr.createEdge(F, G ,0);
    KosaGr.createEdge(G, E ,1);
    KosaGr.createEdge(H, G ,0);
    KosaGr.createEdge(H, I ,0);

    rm.registerVertex(&A, &add);
    rm.registerVertex(&B, &mult);
    rm.registerVertex(&C, &add);
    rm.registerVertex(&D, &add);
    rm.registerVertex(&E, &add);
    rm.registerVertex(&F, &add);
    rm.registerVertex(&G, &add);
    rm.registerVertex(&H, &add);
    rm.registerVertex(&I, &add);*/

    //-----------------------------------------------------------------------------------------------------------------
    //Graph to test the build supergraph method.

    Vertex& A = KosaGr.createVertex(0);
    Vertex& B = KosaGr.createVertex(1);
    Vertex& C = KosaGr.createVertex(2);
    Vertex& D = KosaGr.createVertex(3);
    Vertex& E = KosaGr.createVertex(4);
    Vertex& F = KosaGr.createVertex(5);
    Vertex& G = KosaGr.createVertex(6);
    Vertex& H = KosaGr.createVertex(7);
    Vertex& I = KosaGr.createVertex(8);
    Vertex& J = KosaGr.createVertex(9);
    Vertex& K = KosaGr.createVertex(10);
    Vertex& L = KosaGr.createVertex(11);
    Vertex& M = KosaGr.createVertex(12);
    Vertex& N = KosaGr.createVertex(13);
    Vertex& O = KosaGr.createVertex(14);

    // SSC 0:
    KosaGr.createEdge(A, B ,1);
    KosaGr.createEdge(B, C ,0);
    KosaGr.createEdge(C, A ,0);

    // SCC 1:
    KosaGr.createEdge(D, E ,1);
    KosaGr.createEdge(E, F ,0);
    KosaGr.createEdge(F, D ,0);

    // SCC 2:
    KosaGr.createEdge(G, H ,1);
    KosaGr.createEdge(H, I ,0);
    KosaGr.createEdge(I, G ,0);

    // SCC 3:
    KosaGr.createEdge(J, K ,1);
    KosaGr.createEdge(K, L ,0);
    KosaGr.createEdge(L, J ,0);

    // SCC 4:
    KosaGr.createEdge(M, N ,1);
    KosaGr.createEdge(N, O ,0);
    KosaGr.createEdge(O, M ,0);

    //0-2
    KosaGr.createEdge(A, G ,0);
    //0-3
    KosaGr.createEdge(A, J ,0);
    //1-3
    KosaGr.createEdge(D, L ,0);
    //1-4
    KosaGr.createEdge(D, M ,0);
    //3-4
    KosaGr.createEdge(L, M ,0);
    //3-2
    KosaGr.createEdge(L, G ,0);

    rm.registerVertex(&A, &add);
    rm.registerVertex(&B, &add);
    rm.registerVertex(&C, &add);
    rm.registerVertex(&D, &add);
    rm.registerVertex(&E, &add);
    rm.registerVertex(&F, &add);
    rm.registerVertex(&G, &add);
    rm.registerVertex(&H, &add);
    rm.registerVertex(&I, &add);
    rm.registerVertex(&J, &add);
    rm.registerVertex(&K, &add);
    rm.registerVertex(&L, &add);
    rm.registerVertex(&M, &add);
    rm.registerVertex(&N, &add);
    rm.registerVertex(&O, &add);

    //------------------------------------------------------------------------------------------------------------------

    //Write the graphml-file for Debugging
    //cout << "Generating graphml file: " << graphmlpath << endl;
    //HatScheT::DotWriter DW(graphmlpath, &KosaGr, &rm);
    //DW.write();
    //Printing the transposed Graph.
    //auto paar = HatScheT::Utility::transposeGraph(&KosaGr);
    //for (auto V:paar.first->Vertices()){
    //  rm.registerVertex(V, &add);
    //}
    //HatScheT::DotWriter DWT(transposedpath, paar.first, &rm);
    //DWT.write();

    //KosarajuSCC KosaSCC(KosaGr);

    //auto Zeugs = KosaSCC.getSCCs();

    //SCC.printSSC();

    DaiZhang19Scheduler GraRed(KosaGr, rm, {"CPLEX"});

    GraRed.schedule();

    SCC scc;

    return false;
  }

}
