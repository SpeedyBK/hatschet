#include "HatScheT/utility/Tests.h"
#include "HatScheT/utility/reader/GraphMLGraphReader.h"
#include "HatScheT/utility/reader/GraphMLResourceReader.h"
#include "HatScheT/MoovacScheduler.h"

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
  HatScheT::MoovacScheduler sched(g, rm, {"CPLEX", "Gurobi"}, 2, maxLatencyConstraint-1);
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

}
