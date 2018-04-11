#include "HatScheT/utility/Tests.h"
#include "HatScheT/utility/reader/GraphMLGraphReader.h"
#include "HatScheT/utility/reader/GraphMLResourceReader.h"
#include "HatScheT/MoovacScheduler.h"
#include "HatScheT/ResourceModel.h"
#include "HatScheT/utility/writer/DotWriter.h"
#include "HatScheT/scheduler/ASAPScheduler.h"
#include "HatScheT/scheduler/ALAPScheduler.h"
#include "HatScheT/utility/Utility.h"

namespace HatScheT {

bool Tests::moovacTest()
{
  HatScheT::GraphMLResourceReader readerRes;
  HatScheT::ResourceModel rm;
  HatScheT::Graph g;

  string resStr = "graphMLFiles/example/MoovacExampleRM.xml";
  string graphStr = "graphMLFiles/example/MoovacExample.graphml";
  rm = readerRes.readResourceModel(resStr.c_str());

  HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
  g = readerGraph.readGraph(graphStr.c_str());

  int maxLatencyConstraint = 18;
  HatScheT::MoovacScheduler sched(g, rm, {"CPLEX", "Gurobi"}, maxLatencyConstraint-1);
  sched.setMaxLatencyConstraint(maxLatencyConstraint);
  sched.setSolverQuiet(false);

  sched.schedule();

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
  HatScheT::GraphMLResourceReader readerRes;
  HatScheT::ResourceModel rm;
  HatScheT::Graph g;

  string resStr = "graphMLFiles/example/exampleResourceModel.xml";
  string graphStr = "graphMLFiles/example/example.graphml";
  rm = readerRes.readResourceModel(resStr.c_str());

  HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
  g = readerGraph.readGraph(graphStr.c_str());

  HatScheT::ASAPScheduler asap(g,rm);
  asap.schedule();

  int sum = Utility::sumOfStarttimes(asap.getStartTimes());

  if(sum==51) return true;
  cout << "Tests::asapTest: Sum of start times expected to be 51, but is " << sum << endl;
  return false;
}

bool Tests::asapHCTest()
{
  HatScheT::GraphMLResourceReader readerRes;
  HatScheT::ResourceModel rm;
  HatScheT::Graph g;

  string resStr = "graphMLFiles/example/ASAPHCExampleRM.xml";
  string graphStr = "graphMLFiles/example/ASAPHCExample.graphml";
  rm = readerRes.readResourceModel(resStr.c_str());

  HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
  g = readerGraph.readGraph(graphStr.c_str());

  HatScheT::ASAPScheduler asap(g,rm);
  asap.schedule();

  int sum = Utility::sumOfStarttimes(asap.getStartTimes());

  if(sum==29) return true;
  cout << "Tests::asapHCTest: Sum of start times expected to be 29, but is " << sum << endl;
  return false;
}

bool Tests::alapHCTest()
{
  HatScheT::GraphMLResourceReader readerRes;
  HatScheT::ResourceModel rm;
  HatScheT::Graph g;

  string resStr = "graphMLFiles/example/ASAPHCExampleRM.xml";
  string graphStr = "graphMLFiles/example/ASAPHCExample.graphml";
  rm = readerRes.readResourceModel(resStr.c_str());

  HatScheT::GraphMLGraphReader readerGraph(&rm, &g);
  g = readerGraph.readGraph(graphStr.c_str());

  HatScheT::ALAPScheduler alap(g,rm);
  alap.schedule();

  int sum = Utility::sumOfStarttimes(alap.getStartTimes());

  if(sum==44) return true;
  cout << "Tests::alapHCTest: Sum of start times expected to be 44, but is " << sum << endl;
  return false;
}

bool Tests::readTest()
{
  HatScheT::GraphMLResourceReader readerRes;
  HatScheT::ResourceModel rm;
  HatScheT::Graph g;

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

}
