#include "HatScheT/utility/Tests.h"
#include "HatScheT/utility/reader/GraphMLGraphReader.h"
#include "HatScheT/utility/reader/GraphMLResourceReader.h"
#include "HatScheT/MoovacScheduler.h"
#include "HatScheT/ModuloSDCScheduler.h"
#include "HatScheT/ResourceModel.h"
#include "HatScheT/utility/writer/DotWriter.h"
#include "HatScheT/scheduler/ASAPScheduler.h"
#include "HatScheT/scheduler/ALAPScheduler.h"
#include "HatScheT/utility/Utility.h"
#include "HatScheT/utility/subgraphs/OccurrenceSetCombination.h"
#include "HatScheT/scheduler/graphBased/SGMScheduler.h"
#include "HatScheT/scheduler/ULScheduler.h"
#include "HatScheT/Verifier.h"

namespace HatScheT {

bool Tests::moovacTest()
{
  HatScheT::ResourceModel rm;
  HatScheT::Graph g;
  HatScheT::GraphMLResourceReader readerRes(&rm);

  string resStr = "graphMLFiles/example/MoovacExampleRM.xml";
  string graphStr = "graphMLFiles/example/MoovacExample.graphml";
  rm = readerRes.readResourceModel(resStr.c_str());

  HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
  g = readerGraph.readGraph(graphStr.c_str());

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
}

bool Tests::asapTest()
{
  HatScheT::ResourceModel rm;
  HatScheT::Graph g;
  HatScheT::GraphMLResourceReader readerRes(&rm);

  string resStr = "graphMLFiles/example/exampleResourceModel.xml";
  string graphStr = "graphMLFiles/example/example.graphml";
  rm = readerRes.readResourceModel(resStr.c_str());

  HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
  g = readerGraph.readGraph(graphStr.c_str());

  HatScheT::ASAPScheduler asap(g,rm);
  asap.schedule();

  int sum = Utility::sumOfStarttimes(asap.getSchedule());

  if(sum==51) return true;
  cout << "Tests::asapTest: Sum of start times expected to be 51, but is " << sum << endl;
  return false;
}

bool Tests::asapHCTest()
{
  HatScheT::ResourceModel rm;
  HatScheT::Graph g;
  HatScheT::GraphMLResourceReader readerRes(&rm);

  string resStr = "graphMLFiles/example/ASAPHCExampleRM.xml";
  string graphStr = "graphMLFiles/example/ASAPHCExample.graphml";
  rm = readerRes.readResourceModel(resStr.c_str());

  HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
  g = readerGraph.readGraph(graphStr.c_str());

  HatScheT::ASAPScheduler asap(g,rm);
  asap.schedule();

  int sum = Utility::sumOfStarttimes(asap.getSchedule());

  if(sum==29) return true;
  cout << "Tests::asapHCTest: Sum of start times expected to be 29, but is " << sum << endl;
  cout << rm << endl;
  cout << g << endl;

  return false;
}

bool Tests::alapHCTest()
{
  HatScheT::ResourceModel rm;
  HatScheT::Graph g;
  HatScheT::GraphMLResourceReader readerRes(&rm);

  string resStr = "graphMLFiles/example/ASAPHCExampleRM.xml";
  string graphStr = "graphMLFiles/example/ASAPHCExample.graphml";
  rm = readerRes.readResourceModel(resStr.c_str());

  HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
  g = readerGraph.readGraph(graphStr.c_str());

  HatScheT::ALAPScheduler alap(g,rm);
  alap.schedule();

  int sum = Utility::sumOfStarttimes(alap.getSchedule());

  if(sum==44) return true;
  cout << "Tests::alapHCTest: Sum of start times expected to be 44, but is " << sum << endl;
  return false;
}

bool Tests::readTest()
{
  HatScheT::ResourceModel rm;
  HatScheT::Graph g;
  HatScheT::GraphMLResourceReader readerRes(&rm);

  string resStr = "graphMLFiles/example/exampleResourceModel.xml";
  string graphStr = "graphMLFiles/example/example.graphml";
  rm = readerRes.readResourceModel(resStr.c_str());

  HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
  g = readerGraph.readGraph(graphStr.c_str());

  if(rm.getNoOfResources() != 3){
    cout << "Incorrect no of resource read: " << rm.getNoOfResources() << " instead of 3!" << endl;
    return false;
  }

  if(rm.getNoOfReservationTables() != 2){
    cout << "Incorrect no of reservation tables read: " << rm.getNoOfReservationTables() << " instead of 2!" << endl;
    return false;
  }

  if(g.getNumberOfVertices() != 6){
    cout << "Incorrect no of vertices read: " << g.getNumberOfVertices() << " instead of 6!" << endl;
    return false;
  }

  return true;
}

bool Tests::moduloSDCTest()
{
  //disabled tests as long as moduloSDC is not working correctly
  //return true;
  try
  {
    HatScheT::ResourceModel rm;
    
    auto &load = rm.makeResource("load", 1, 2);
    auto &add = rm.makeResource("add", -1, 0);

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
  catch(HatScheT::Exception* e)
  {
    std::cout << *e << std::endl;
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

bool Tests::occurrenceSetTest()
{
  HatScheT::Graph g;
  for (int i = 1; i <= 7; ++i)
    g.createVertex(i);

  std::vector<std::pair<int, int>> edges = {
    {1,  5},
    {2,  5},
    {3,  6},
    {4,  6},
    {5,  7},
    {6,  7}
  };
  for (auto fe : edges)
    g.createEdge(g.getVertexById(fe.first), g.getVertexById(fe.second), 0);

  //generate occurrenceSet
  HatScheT::OccurrenceSet occs(&g);

  //generate 3 occurrences
  HatScheT::Occurrence occ1(&g);
  occ1.addEdge(&g.getEdge(&g.getVertexById(3),&g.getVertexById(6)));
  occ1.addEdge(&g.getEdge(&g.getVertexById(4),&g.getVertexById(6)));

  HatScheT::Occurrence occ2(&g);
  occ2.addEdge(&g.getEdge(&g.getVertexById(1),&g.getVertexById(5)));
  occ2.addEdge(&g.getEdge(&g.getVertexById(2),&g.getVertexById(5)));

  HatScheT::Occurrence occ3(&g);
  occ3.addEdge(&g.getEdge(&g.getVertexById(5),&g.getVertexById(7)));
  occ3.addEdge(&g.getEdge(&g.getVertexById(6),&g.getVertexById(7)));

  //try to add first occurrence
  if(occs.addOccurrence(&occ1) == false){
    return false;
  }

  //try to add second occurrence
  if(occs.addOccurrence(&occ2) == false){
    cout << "Tests.occurrenceSetTest: Tried to add a conflict free occurrence but it failed!" << endl;
    return false;
  }

  //try to add third occurrence
  if(occs.addOccurrence(&occ3) == true){
    cout << "Tests.occurrenceSetTest: Tried to add a conflicted occurrence but it shouldnt be possible!" << endl;
    return false;
  }

  return true;
}

bool Tests::occurrenceTest()
{
  HatScheT::Graph g;
  for (int i = 1; i <= 7; ++i)
    g.createVertex(i);

  std::vector<std::pair<int, int>> edges = {
    {1,  5},
    {2,  5},
    {3,  6},
    {4,  6},
    {5,  7},
    {6,  7}
  };
  for (auto fe : edges)
    g.createEdge(g.getVertexById(fe.first), g.getVertexById(fe.second), 0);

  HatScheT::Occurrence occ(&g);

  //add first edge
  if(occ.addEdge(&g.getEdge(&g.getVertexById(3),&g.getVertexById(6))) == false){
    cout << "Tests.occurrenceTest: adding a first edge to occurrence failed!" << endl;
    return false;
  }

  //add second egdge, valid connected subgraph
  if(occ.addEdge(&g.getEdge(&g.getVertexById(4),&g.getVertexById(6))) == false){
    cout << "Tests.occurrenceTest: adding second (valid) edge to occurrence failed!" << endl;
    return false;
  }

  //try to add the edge a second time
  if(occ.addEdge(&g.getEdge(&g.getVertexById(4),&g.getVertexById(6))) == true){
    cout << "Tests.occurrenceTest: adding duplicate edge to occurrence was possible but shouldnt be!" << endl;
    return false;
  }

  //try to add unconnected edge
  if(occ.addEdge(&g.getEdge(&g.getVertexById(5),&g.getVertexById(7))) == true){
    cout << "Tests.occurrenceTest: adding an unconncted edge to occurrence was possible but shouldnt be!" << endl;
    return false;
  }

  //generate another edge that is not in g and try to add it
  HatScheT::Graph g2;
  for (int i = 1; i <= 2; ++i)
    g2.createVertex(i);
  std::vector<std::pair<int, int>> g2edge = {
    {1,  2}
  };
  for (auto fe : g2edge)
    g2.createEdge(g2.getVertexById(fe.first), g2.getVertexById(fe.second), 0);
  if(occ.addEdge(&g2.getEdge(&g2.getVertexById(1),&g2.getVertexById(2))) == true){
    cout << "Tests.occurrenceTest: adding an unconncted edge tfrom another graph was possible but shouldnt be!" << endl;
    return false;
  }

  return true;
}

bool Tests::ulSchedulerTest()
{
  HatScheT::ResourceModel rm;
  HatScheT::Graph g;
  HatScheT::GraphMLResourceReader readerRes(&rm);

  string resStr = "graphMLFiles/example/ASAPHCExampleRM.xml";
  string graphStr = "graphMLFiles/example/ASAPHCExample.graphml";
  rm = readerRes.readResourceModel(resStr.c_str());

  HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
  g = readerGraph.readGraph(graphStr.c_str());

  HatScheT::ULScheduler uls(g,rm);
  uls.schedule();

  uls.printStartTimes();
  std::map<Vertex *, int> &schedule = uls.getSchedule();
  int foundII = uls.getII();
  bool verified = HatScheT::verifyModuloSchedule(g, rm, schedule, foundII);

  if(verified==false) return false;
  if(uls.getScheduleLength()!=8) return false;

  return true;
}

bool Tests::sgmSchedulerTest()
{
  HatScheT::Graph g;
  for (int i = 1; i <= 12; ++i)
    g.createVertex(i);

  std::vector<std::pair<int, int>> edges = {
    {1,  3},
    {2,  3},
    {5,  7},
    {6,  7},
    {4,  8},
    {4,  9},
    {7,  11},
    {3,  8},
    {8,  10},
    {9,  10},
    {10,  11},
    {11,  12}
  };
  for (auto fe : edges)
    g.createEdge(g.getVertexById(fe.first), g.getVertexById(fe.second), 0);

  HatScheT::ResourceModel rm;
  Resource& addRes = rm.makeResource("Adder",3,3,1);
  Resource& multRes = rm.makeResource("Mult",1,3,1);

  for(int i = 1; i <= 12; ++i){
    if(i!=2 && i!=6 && i!=9) rm.registerVertex(&g.getVertexById(i),&addRes);
    else rm.registerVertex(&g.getVertexById(i),&multRes);
  }

  cout << g << endl;
  cout << rm << endl;

  //generate occurrences
  HatScheT::Occurrence occ1(&g);
  occ1.addEdge(&g.getEdge(&g.getVertexById(1),&g.getVertexById(3)));
  occ1.addEdge(&g.getEdge(&g.getVertexById(2),&g.getVertexById(3)));
  HatScheT::Occurrence occ2(&g);
  occ2.addEdge(&g.getEdge(&g.getVertexById(5),&g.getVertexById(7)));
  occ2.addEdge(&g.getEdge(&g.getVertexById(6),&g.getVertexById(7)));
  HatScheT::Occurrence occ3(&g);
  occ3.addEdge(&g.getEdge(&g.getVertexById(8),&g.getVertexById(10)));
  occ3.addEdge(&g.getEdge(&g.getVertexById(9),&g.getVertexById(10)));

  //generate OccurrenceSet cand combination
  OccurrenceSet occs(&g);
  occs.addOccurrence(&occ1);
  occs.addOccurrence(&occ2);
  occs.addOccurrence(&occ3);
  OccurrenceSetCombination occsC(&g);
  occsC.addOccurrenceSet(&occs);

  HatScheT::SGMScheduler sgms(g,rm,{"CPLEX", "Gurobi"},&occsC);
  sgms.setSolverQuiet(true);
  sgms.schedule();

  if(sgms.getII()!=3){
    cout << "Tests.sgmSchedulerTest: wrong II determined: " << sgms.getII() << "(II=3 expected) " << endl;
    return false;
  }

  return true;
}

bool Tests::occurrenceSetCombinationTest()
{
  HatScheT::Graph g;
  for (int i = 1; i <= 7; ++i)
    g.createVertex(i);

  std::vector<std::pair<int, int>> edges = {
    {1,  5},
    {2,  5},
    {3,  6},
    {4,  6},
    {5,  7},
    {6,  7}
  };
  for (auto fe : edges)
    g.createEdge(g.getVertexById(fe.first), g.getVertexById(fe.second), 0);

  //generate 4 occurrences
  HatScheT::Occurrence occ1(&g);
  occ1.addEdge(&g.getEdge(&g.getVertexById(3),&g.getVertexById(6)));
  HatScheT::Occurrence occ2(&g);
  occ2.addEdge(&g.getEdge(&g.getVertexById(1),&g.getVertexById(5)));
  HatScheT::Occurrence occ3(&g);
  occ3.addEdge(&g.getEdge(&g.getVertexById(2),&g.getVertexById(5)));
  HatScheT::Occurrence occ4(&g);
  occ4.addEdge(&g.getEdge(&g.getVertexById(4),&g.getVertexById(6)));

  //generate 2 occurrencesets
  OccurrenceSet occs1(&g);
  occs1.addOccurrence(&occ1);
  occs1.addOccurrence(&occ2);
  OccurrenceSet occs2(&g);
  occs2.addOccurrence(&occ3);
  occs2.addOccurrence(&occ4);

  //generate occurrenceSetCombination
  OccurrenceSetCombination occsC(&g);

  //add first occurrenceSet
  if(occsC.addOccurrenceSet(&occs1)==false){
    cout << "Tests.occurrenceSetCombinationTest: adding a first occurrenceset failed!" << endl;
    return false;
  }

  //try toadd second occurrenceSet with conflicts
  if(occsC.addOccurrenceSet(&occs2)==true){
    cout << "Tests.occurrenceSetCombinationTest: added a second occurrenceset that has conflicts, but this shouldn be possible!" << endl;
    return false;
  }

  return true;
}

}
