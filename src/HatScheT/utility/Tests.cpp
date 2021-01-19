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
#include "HatScheT/scheduler/ilpbased/MoovacMinRegScheduler.h"
#include "HatScheT/scheduler/ilpbased/ModuloSDCScheduler.h"
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

#include <HatScheT/scheduler/ilpbased/RationalIIScheduler.h>
#include <HatScheT/scheduler/dev/ModuloQScheduler.h>
#include <HatScheT/scheduler/dev/SCCQScheduler.h>
#include <stdio.h>
#include <math.h>

#ifdef USE_CADICAL
#include "cadical.hpp"
#endif


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
  HatScheT::MoovacScheduler sched(g, rm, {"CPLEX", "Gurobi", "SCIP", "LPSolve"});
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
  HatScheT::MoovacScheduler  ms1(g,rm, {"CPLEX","Gurobi", "SCIP", "LPSolve"});
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
  HatScheT::MoovacScheduler  ms2(g2,rm2, {"CPLEX","Gurobi", "SCIP", "LPSolve"});
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

    HatScheT::ModuloSDCScheduler m{g,rm,{"CPLEX","Gurobi", "SCIP", "LPSolve"}};
    m.setSolverQuiet(true);
    m.setQuiet(false);
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

  HatScheT::RationalIISchedulerFimmel fimmel{g,rm,{"CPLEX","Gurobi", "SCIP", "LPSolve"}};
  fimmel.schedule();

  cout << "Tests::rationalIISchedulerFimmelTest: expected II is 5.333..." << endl;
  cout << "Tests::rationalIISchedulerFimmelTest: found II is " << fimmel.getII() << endl;

  if(fimmel.getII() < 5.333 or fimmel.getII()>=5.334) return false;
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

  HatScheT::RationalIIScheduler rii{g,rm,{"CPLEX","Gurobi", "SCIP", "LPSolve"}};
  rii.setUniformScheduleFlag(true);
  rii.schedule();

  cout << "Tests::rationalIISchedulerTest: expected II is 5/3" << endl;
  cout << "Tests::rationalIISchedulerTest: found II " << rii.getM_Found() << "/" << rii.getS_Found() << endl;

  if(rii.getM_Found() != 5 or rii.getS_Found() != 3) return false;
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
  HatScheT::ModuloSDCScheduler sdc{g,rm,{"CPLEX","Gurobi", "SCIP", "LPSolve"}};
  sdc.schedule();
  modSDC_II = sdc.getII();
  //------------
  HatScheT::MoovacScheduler moovac{g,rm,{"CPLEX","Gurobi", "SCIP", "LPSolve"}};
  moovac.schedule();
  moovac_II = moovac.getII();
  //------------
  HatScheT::MoovacMinRegScheduler minreg{g,rm,{"CPLEX","Gurobi", "SCIP", "LPSolve"}};
  minreg.schedule();
  moovacminreg_II = minreg.getII();
  //------------
  HatScheT::EichenbergerDavidson97Scheduler ed97{g,rm,{"CPLEX","Gurobi", "SCIP", "LPSolve"}};
  ed97.schedule();
  ED97_II = ed97.getII();

  cout << "Tests::compareModuloSchedulerTest: Expected II is 11" << endl;
  cout << "Tests::compareModuloSchedulerTest: ModuloSDC found II " << modSDC_II << endl;
  cout << "Tests::compareModuloSchedulerTest: MoovacScheduler found II " << moovac_II << endl;
  cout << "Tests::compareModuloSchedulerTest: MoovacMinRegScheduler found II " << moovacminreg_II << endl;
  cout << "Tests::compareModuloSchedulerTest: EichenbergerDavidson97Scheduler found II " << ED97_II << endl;

  if(modSDC_II != 11 or moovac_II != 11 or moovacminreg_II != 11 or ED97_II != 11) return false;
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

    Vertex& A = KosaGr.createVertex(0);
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
    rm.registerVertex(&B, &add);
    rm.registerVertex(&C, &mult);
    rm.registerVertex(&D, &add);
    rm.registerVertex(&E, &add);
    rm.registerVertex(&F, &add);
    rm.registerVertex(&G, &add);
    rm.registerVertex(&H, &add);
    rm.registerVertex(&I, &add);

    KosarajuSCC kscc(KosaGr);

    vector <SCC*> sccs = kscc.getSCCs();

    vector <list<int>> compValues;
    list <int> sccA = {7};
    compValues.push_back(sccA);
    list <int> sccB = {8};
    compValues.push_back(sccB);
    list <int> sccC = {1, 2, 3, 0};
    compValues.push_back(sccC);
    list <int> sccD = {5, 6, 4};
    compValues.push_back(sccD);

    for (auto &it : sccs){
      cout << endl;
      it->printVertexStatus();
      auto v = it->getVerticesOfSCC();
      int i = 0;
      for (auto &itr : v){
        cout << i << ": " << itr->getName() << " ";
        i++;
      }
      cout << endl;
    }

    vector <list<int>> testValues;
    list<int> value;

    for (auto &it : sccs) {
      auto vertexList = it->getVerticesOfSCC();
        value.clear();
        for (auto &itr : vertexList){
          value.push_back(itr->getId());
        }
        testValues.push_back(value);
    }

    for (int j = 0; j < 4; j++){
      if (compValues[j] != testValues[j]){
        return false;
      }
    }

    return true;

  }
    //-----------------------------------------------------------------------------------------------------------------
    //Graph to test the build supergraph method.

  bool Tests::DaiZhangTest(){

    HatScheT::Graph Gr;
    HatScheT::ResourceModel rm;

    auto &red = rm.makeResource("red", 1, 1, 1);
    auto &blue = rm.makeResource("blue", 1, 1, 1);
    auto &green = rm.makeResource("green", -1 , 1, 1);

    Vertex& A = Gr.createVertex(0);
    Vertex& B = Gr.createVertex(1);
    Vertex& C = Gr.createVertex(2);
    Vertex& D = Gr.createVertex(3);
    Vertex& E = Gr.createVertex(4);
    Vertex& F = Gr.createVertex(5);
    Vertex& G = Gr.createVertex(6);
    Vertex& H = Gr.createVertex(7);
    Vertex& I = Gr.createVertex(8);
    Vertex& J = Gr.createVertex(9);
    Vertex& K = Gr.createVertex(10);
    Vertex& L = Gr.createVertex(11);
    Vertex& M = Gr.createVertex(12);
    Vertex& N = Gr.createVertex(13);
    Vertex& O = Gr.createVertex(14);

    // SSC 0:
    Gr.createEdge(A, B ,0);
    Gr.createEdge(B, C ,1);
    Gr.createEdge(C, A ,0);

    // SCC 1:
    Gr.createEdge(D, E ,0);
    Gr.createEdge(E, F ,1);
    Gr.createEdge(F, D ,0);

    // SCC 2:
    Gr.createEdge(G, H ,0);
    Gr.createEdge(H, I ,1);
    Gr.createEdge(I, G ,0);

    // SCC 3:
    Gr.createEdge(J, K ,0);
    Gr.createEdge(K, L ,1);
    Gr.createEdge(L, J ,0);

    // SCC 4:
    Gr.createEdge(M, N ,0);
    Gr.createEdge(N, O ,1);
    Gr.createEdge(O, M ,0);

    //0-2
    Gr.createEdge(A, G ,0);
    //0-3
    Gr.createEdge(A, J ,0);
    //1-3
    Gr.createEdge(D, L ,0);
    //1-4
    Gr.createEdge(D, M ,0);
    //3-4
    Gr.createEdge(L, M ,0);
    //3-2
    Gr.createEdge(L, G ,0);

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
    if(!valid) {
      std::cout << "Tests::moduloQTest: invalid rational II modulo schedule found" << std::endl;
      return false;
    }

    for(unsigned int i=0; i<initIntervals.size(); ++i) {
      auto l = initIntervals[i];
      auto startTimes = startTimesVector[i];
      std::cout << "Tests::moduloQTest: start times for insertion time=" << l << std::endl;
      for(auto it : startTimes) {
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

    HatScheT::RationalIIScheduler rii{g,rm,{"CPLEX","Gurobi", "SCIP", "LPSolve"}};
    rii.setUniformScheduleFlag(true);
    rii.schedule();

    //making all resources unlimited to not trigger the resource MRT check
    for(auto it : rm.Resources()) {
      it->setLimit(-1);
    }

    //generating wrong latemcy sequence
    //<1 1 3 > should not be allowed

    vector<int> ls({1,1,3});


    bool ok = verifyRationalIIModuloSchedule2(g, rm, rii.getStartTimeVector(), ls, rii.getScheduleLength());

    if(ok == true) return false;
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

    HatScheT::RationalIIScheduler rii{g,rm,{"CPLEX","Gurobi", "SCIP", "LPSolve"}};
    rii.setUniformScheduleFlag(true);
    rii.schedule();

    vector<std::map<Vertex*,int> > s = rii.getStartTimeVector();

    for(auto it:g.Vertices()) {
      Vertex* v = it;

      if(v->getName() == "A5") {
        s[0][v]++;
      }
    }

    bool ok = verifyRationalIIModuloSchedule2(g, rm, s, rii.getLatencySequence(), rii.getScheduleLength());

    if(ok == true) return false;
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

		std::cout << "Tests::moduloQTest: finished scheduling - resulting control steps:" << std::endl;
		auto startTimesVector = m.getStartTimeVector();
		auto initIntervals = m.getInitiationIntervals();
		auto latencySequence = m.getLatencySequence();

    auto valid = m.getScheduleValid();
		if(!valid) {
			std::cout << "Tests::moduloQTest: invalid rational II modulo schedule found" << std::endl;
			return false;
		}
		for(unsigned int i=0; i<initIntervals.size(); ++i) {
			auto l = initIntervals[i];
			auto startTimes = startTimesVector[i];
			std::cout << "Tests::moduloQTest: start times for insertion time=" << l << std::endl;
			for(auto it : startTimes) {
				std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
			}
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

    HatScheT::UniformRationalIIScheduler rii(g,rm,{"Gurobi","CPLEX","SCIP","LPSolve"});
    rii.setQuiet(false);
    rii.schedule();
    auto valid = rii.getScheduleValid();
    if(!valid) {
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

		HatScheT::UniformRationalIIScheduler rii(g,rm,{"Gurobi","CPLEX","SCIP","LPSolve"});
		rii.setQuiet(false);
		rii.schedule();
		auto valid = rii.getScheduleValid();
		if(!valid) {
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

    HatScheT::NonUniformRationalIIScheduler rii(g,rm,{"Gurobi","CPLEX","SCIP","LPSolve"});
    rii.setQuiet(false);
    rii.schedule();
    auto valid = rii.getScheduleValid();
    if(!valid) {
      std::cout << "Scheduler found invalid solution" << std::endl;
      return false;
    }

    cout << "Tests::uniformRationalIISchedulerTest: expected II is 5/3" << endl;
    cout << "Tests::uniformRationalIISchedulerTest: found II " << rii.getM_Found() << "/" << rii.getS_Found() << endl;

    return (rii.getM_Found() == 5 and rii.getS_Found() == 3);
#endif
  }

  bool Tests::ratIIOptimalIterationTest() {
    int mMinII = 11;
    int sMinII = 10;
    double minII = double(mMinII)/double(sMinII);
    auto integerII = (int)ceil(double(mMinII)/double(sMinII));
    int sMax = -1;
    auto maxListSize = -1;

    auto solutions = RationalIISchedulerLayer::getRationalIIQueue(sMinII,mMinII,integerII,sMax,maxListSize);

    std::cout << "mMinII = " << mMinII << std::endl;
    std::cout << "sMinII = " << sMinII << std::endl;
    std::cout << "minII = " << minII << std::endl;
    std::cout << "integerII = " << integerII << std::endl;
    std::cout << "Queue size:" << solutions.size() << std::endl;
	 	std::cout << "M/S Queue:" << std::endl;
    for(auto it : solutions) {
      std::cout << "  M = " << it.first << ", S = " << it.second << ", M/S = " << double(it.first)/double(it.second) << std::endl;
    }

    if(solutions.size() != 31) {
      std::cout << "Expected queue size of 31 but got queue size of " << solutions.size() << std::endl;
    }
    return solutions.size()==31;
  }

  bool Tests::ratIIUnrollSchedulerTest() {
#ifndef USE_XERCESC
    cout << "Tests::uniformRationalIISchedulerTest: XERCESC parsing library is not active! This test is disabled!" << endl;
    return false;
#else
    std::list<SchedulerType> intIISchedulers = {ED97,MODULOSDC,MOOVAC,SUCHAHANZALEK};
    for(auto intIIScheduler : intIISchedulers) {
			HatScheT::ResourceModel rm;
			HatScheT::Graph g;
			HatScheT::XMLResourceReader readerRes(&rm);

			string resStr = "benchmarks/Programs/vanDongen/vanDongenRM.xml";
			string graphStr = "benchmarks/Programs/vanDongen/vanDongen.graphml";
			readerRes.readResourceModel(resStr.c_str());

			HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
			readerGraph.readGraph(graphStr.c_str());

			HatScheT::UnrollRationalIIScheduler rii(g,rm,{"Gurobi","CPLEX","SCIP","LPSolve"});
			rii.setIntIIScheduler(SchedulerType::SUCHAHANZALEK);
			rii.setQuiet(false);
			rii.schedule();
			auto valid = rii.getScheduleValid();
			if(!valid) {
				std::cout << "Scheduler found invalid solution" << std::endl;
				return false;
			}

			switch (intIIScheduler) {
				case ED97:
					std::cout << "Tests::uniformRationalIISchedulerTest: intII scheduler: ED97" << std::endl;
					break;
				case MODULOSDC:
					std::cout << "Tests::uniformRationalIISchedulerTest: intII scheduler: MODULOSDC" << std::endl;
					break;
				case MOOVAC:
					std::cout << "Tests::uniformRationalIISchedulerTest: intII scheduler: MOOVAC" << std::endl;
					break;
				case SUCHAHANZALEK:
					std::cout << "Tests::uniformRationalIISchedulerTest: intII scheduler: SUCHAHANZALEK" << std::endl;
					break;
			}

			if(rii.getM_Found() != 16 or rii.getS_Found() != 3) {
				cout << "Tests::uniformRationalIISchedulerTest: expected II is 16/3" << endl;
				cout << "Tests::uniformRationalIISchedulerTest: found II " << rii.getM_Found() << "/" << rii.getS_Found() << endl;
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
    if(!valid) {
      std::cout << "Tests::tcadExampleTest: SCCQ discovered invalid rational-II modulo schedule found" << std::endl;
      return false;
    }
    for(unsigned int i=0; i<initIntervals.size(); ++i) {
      auto l = initIntervals[i];
      auto startTimes = startTimesVector[i];
      std::cout << "Tests::tcadExampleTest: SCCQ - start times for insertion time=" << l << std::endl;
      for(auto it : startTimes) {
        std::cout << "  " << it.first->getName() << " - " << it.second << std::endl;
      }
    }

    UniformRationalIIScheduler u(g, rm, {"Gurobi", "CPLEX", "LPSolve", "SCIP"});
    u.setQuiet(false);
    u.setSolverTimeout(1);
    u.schedule();

    std::cout << "Tests::tcadExampleTest: Uniform rational-II scheduler finished scheduling - resulting control steps:" << std::endl;
    startTimesVector = u.getStartTimeVector();
    latencySequence = u.getLatencySequence();

    valid = u.getScheduleValid();
    if(!valid) {
      std::cout << "Tests::tcadExampleTest: Uniform rational-II scheduler discovered invalid rational-II modulo schedule found" << std::endl;
      return false;
    }
    auto insertionTime = 0;
    for(unsigned int i=0; i<latencySequence.size(); ++i) {
      auto startTimes = startTimesVector[i];
      std::cout << "Tests::tcadExampleTest: Uniform rational-II scheduler - start times for insertion time=" << insertionTime << std::endl;
      for(auto it : startTimes) {
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

		/*
		auto &res = rm.makeResource("res", 2, 2, 1);
		 */
		auto &res = rm.makeResource("res", 2, 1, 1);

		Vertex &v0 = g.createVertex(0);
		Vertex &v1 = g.createVertex(1);
		Vertex &v2 = g.createVertex(2);

		rm.registerVertex(&v0,&res);
		rm.registerVertex(&v1,&res);
		rm.registerVertex(&v2,&res);

		/*
		g.createEdge(v0,v1,0);
		g.createEdge(v1,v2,0);
		g.createEdge(v2,v0,5);
		 */
		g.createEdge(v0,v1,0);
		g.createEdge(v1,v2,0);
		g.createEdge(v2,v0,2);

		ASAPScheduler asap(g,rm);
		EichenbergerDavidson97Scheduler ed97(g,rm,{"Gurobi","CPLEX"});
		NonUniformRationalIIScheduler ratIInon(g,rm,{"Gurobi","CPLEX"});
		UniformRationalIIScheduler ratIIu(g,rm,{"Gurobi","CPLEX"});

		asap.setQuiet(false);
		ed97.setQuiet(false);
		ratIInon.setQuiet(false);
		ratIIu.setQuiet(false);

		asap.schedule();
		ed97.schedule();
		ratIInon.schedule();
		ratIIu.schedule();

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

		rm.registerVertex(&v0,&res);
		rm.registerVertex(&v1,&res);
		rm.registerVertex(&v2,&res);

		g.createEdge(v0,v2,0);
		g.createEdge(v1,v2,0);

		NonUniformRationalIIScheduler ratIInon(g,rm,{"Gurobi","CPLEX"});
		UniformRationalIISchedulerNew ratIIu(g,rm,{"Gurobi","CPLEX"});
		SCCQScheduler ratIIsccq(g,rm,{"Gurobi","CPLEX"});

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

		rm.registerVertex(&v0,&res0);
		rm.registerVertex(&v1,&res0);
		rm.registerVertex(&v2,&res0);
		rm.registerVertex(&v3,&res0);

		rm.registerVertex(&v4,&res1);
		rm.registerVertex(&v5,&res1);
		rm.registerVertex(&v6,&res1);
		rm.registerVertex(&v7,&res1);

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
		g.createEdge(v0,v1,0);
		g.createEdge(v1,v2,0);
		g.createEdge(v2,v3,0);
		g.createEdge(v3,v4,0);
		g.createEdge(v4,v5,0);
		g.createEdge(v5,v6,0);
		g.createEdge(v6,v7,0);
		g.createEdge(v7,v0,8);
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

		SCCQScheduler sccq(g,rm,{"Gurobi"});
		//UniformRationalIISchedulerNew sccq(g,rm,{"Gurobi"});
		//NonUniformRationalIIScheduler sccq(g,rm,{"Gurobi"});

		sccq.setSolverTimeout(48*3600); // 48h timeout

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

		rm.registerVertex(&v0,&res0);
		rm.registerVertex(&v1,&res0);

		// NO SCHEDULE POSSIBLE FOR MIN II
		g.createEdge(v0,v1,0);
		g.createEdge(v1,v0,2);
		EichenbergerDavidson97Scheduler ed97(g,rm,{"Gurobi"});

		ed97.setQuiet(false);

		ed97.schedule();

		return true;
	}

  bool Tests::fibonacciTest() {

    cout << "INSERT AND EXTRACT_MIN TEST:" << endl << endl;

    const int arraysize = 20; //should be an even number
    int numbers[arraysize];
    int sorted_numbers_from_heap[arraysize];

    srand(time(nullptr));
    for (int i = 0; i < arraysize; i++){
      numbers[i] = rand() % 100;
    }
    cout << "Numbers generates..." << endl;
    for (int i : numbers){
      cout << i << " ";
    }

    /////// Insert and Extract_min Test Begin ///////

    //Creating a new Heap and fill it with numbers-Arrays content.
    auto *FIB = new FibonacciHeap <int>;
    for (int i : numbers) {
      FIB->push(i);
    }

    //Extracting the values from the heap again.
    int j = 0;
    while (!FIB->empty()){
      sorted_numbers_from_heap[j] = FIB->get_extract_min();
      j++;
    }

    cout << endl << "Heap done..." << endl;

    //Sort the numbers Array with a simple Bubble Sort
    for (int k = arraysize; k > 1; --k){
      for (int j = 0; j < k-1; ++j){
        if (numbers[j] > numbers[j+1]){
          int temp = numbers[j];
          numbers[j] = numbers[j+1];
          numbers[j+1] = temp;
        }
      }
    }

    cout << "Bubble Sort done..." << endl;

    //Display Results:
    cout <<endl << "Sorted array by F-Heap:" << endl;
    for (int it : sorted_numbers_from_heap){
      cout << it << " ";
    }
    cout << endl << "Sorted array by Bubble Sort:" << endl;
    for (int it : numbers){
      cout << it << " ";
    }

    if (0 != memcmp(sorted_numbers_from_heap, numbers, sizeof(numbers))){
      cout << endl << "Test failed!" << endl;
      return false;
    }

    /////// Insert and Extract_min Test End ///////

    /////// Find Element by Key Test Begin ////////

    for (int i : numbers){
      FIB->push(i);
    }

    int ind = rand() % arraysize;
    auto foundNode = FIB->findNode_by_key(numbers[ind]);
    if (foundNode.first) {
      cout << endl << endl << "Value from Heap: " << foundNode.second->key << " Reference Value: " << numbers[ind] << endl << endl;
      if (foundNode.second->key != numbers[ind]) {
        cout << "Found Element by Key Test failes!" << endl;
        return false;
      }
    }else {
      cout << endl << "Element does not exists in the heap, or can't be found! This should not happen!" << endl;
      return false;
    }

    delete FIB;
    /////// Find Element by Key Test End //////////

    /////// Inserting Elements with Payload ///////
    auto *FIBO = new FibonacciHeap <int>;
    Vertex* vp = nullptr;
    for (int i = 0; i < 10; i++){
      auto *v = new Vertex(9 - i);
      if (i == 5){
        vp = v;
      }
      FIBO->push(i, v);
    }

    ///// Find and Element with known Payload /////
    auto fNode = FIBO->findNode_by_payload(vp);
    if (fNode.first) {
      auto fv = (Vertex *) fNode.second->payload;
      cout << "Key: " << fNode.second->key << " Name: " << fv->getName() << endl << endl;
    }else{
      cout << "Element not found, this should not happen" << endl;
      return false;
    }

    /////// Removing Elements from the Heap ///////
    do {
      auto v = (Vertex*) FIBO->topNode()->payload;
      auto k = FIBO->topNode()->key;
      FIBO->pop();
      cout << "Key: " << k << " Name: "<< v->getName() << endl;
    }while (!FIBO->empty());

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
    list <SDCConstraint> constr;
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
    for (auto &it : constr){
      solver->add_sdc_constraint(it);
    }

    //Printing the system
    solver->print_Constraint_Graph();

    //Compute an initial solution for the given SDC-System.
    solver->compute_inital_solution();
    if (solver->get_solver_status() == 11){
      cout << endl << "System is not feasible." << endl;
    }else if (solver->get_solver_status() == 10){
      cout << endl << "System is feasible." << endl;
      auto spath = solver->get_solution();
      for (auto &it : spath){
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
    if (solver->get_solver_status() == 21){
      cout << endl << "System is not feasible." << endl;
    }else if (solver->get_solver_status() == 20){
      cout << endl << "System is feasible." << endl;
      auto spath = solver->get_solution();
      for (auto &it : spath){
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
}

