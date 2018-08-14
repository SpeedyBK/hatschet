#include <iostream>
#include <list>

#include "HatScheT/Graph.h"
#include "HatScheT/Vertex.h"
#include "HatScheT/Edge.h"
#include "HatScheT/ResourceModel.h"
#include "HatScheT/utility/writer/DotWriter.h"
#include "HatScheT/utility/Verifier.h"
#include "HatScheT/scheduler/ilpbased/MoovacScheduler.h"

using namespace HatScheT;

int main() {

  /*
   * EXAMPLE: MODULO SCHEDULING WITH MOOVAC
   *
   * Suppose we want to iteratively fill an array with the Fibonacci sequence up to the N'th element, i.e.
   *
   *   fib[0] = 0; fib[1] = 1;
   *   for (n = 2; n <= N; ++n)
   *     fib[n] = fib[n-1] + fib[n-2];
   *
   * and pipeline the execution of the loop.
   *
   * To this end, we want to modulo schedule the body of this loop, which can be represented by the following data-flow
   * graph:
   *
   *   dist=1                dist=2
   *   /----\                /----\
   *   |    |                |    |
   *   |    v                v    |
   *   |  [load n-1]   [load n-2] |
   *   |          \     /         |
   *   |           \   /          |
   *   |            v v           |
   *   |           [add]          |
   *   |             |            |
   *   |             v            |
   *   \---------[store n]--------/
   *
   * The edges outgoing from the store operation are inter-iteration dependences. The distance denotes how many
   * iterations later such a dependence has to hold. For example, the edge from the store to the right load operation
   * carries a distance of 2, because the array element written by the store ('n' in iteration 'n') is accessed by the
   * load two iterations later ('n+2-2' in iteration 'n+2').
   * The other edges are intra-iteration dependences, and express the obvious data-flow from the loads, to the addition,
   * to the store operation.
   *
   * In order to express this graph in HatScheT, we instantiate a Graph and the required vertices to represent the
   * operations.
   */
  Graph G;
  auto& load_n_1 = G.createVertex();
  auto& load_n_2 = G.createVertex();
  auto& add      = G.createVertex();
  auto& store_n  = G.createVertex();

  // Then, the edges are constructed.
  G.createEdge(load_n_1,    // source vertex
               add,         // target vertex
               0,           // distance (=0 means intra-iteration, >0 means inter-iteration dependence)
               Edge::Data); // (optional) dependence type to distinguish edge with data flow from pure
                            //            ordering constraints
  G.createEdge(load_n_2, add);
  G.createEdge(add, store_n);
  G.createEdge(store_n, load_n_1, /* distance= */ 1, Edge::Precedence);
  G.createEdge(store_n, load_n_2, /* distance= */ 2, Edge::Precedence);


  /*
   * Note that we haven't specified any vertex latencies yet. In HatScheT, these are not part of the dependence graph.
   * Instead, we will now construct a resource model that represents the target architecture (e.g. operator modules
   * available to a high-level synthesis framework), and associate each vertex with a resource.
   *
   * Modulo scheduling is usually subject to resource constraints, meaning that only a limited number of operations
   * can use such a resource concurrently. In this example, we assume we have only one memory port. However, not all
   * resources have to be limited, as, for example, simple arithmetic operations may be instantiated as needed by an HLS
   * tool.
   */
  ResourceModel RM;
  auto& mem  = RM.makeResource("access to memory port",
                                1,  // limit = number of available resource instances
                                2,  // latency = number of time steps before results is available
                                1); // blocking time = number of time steps that an instance is used by an operation.
                                    // Currently, only fully-pipelined (blocking time=1) resources are supported.
  auto& adder = RM.makeResource("adder", UNLIMITED, 1);

  /*
   * Finally, we associate the vertices with the resources.
   */
  RM.registerVertex(&load_n_1, &mem);
  RM.registerVertex(&load_n_2, &mem);
  RM.registerVertex(&add, &adder);
  RM.registerVertex(&store_n, &mem);

  /*
   * Export the dependence graph as a DOT graph (for debugging purposes).
   */
  DotWriter dotWriter("example.dot", &G, &RM);
  dotWriter.write();

  /*
   * Setup the Moovac scheduler, preferably with the Gurobi solver.
   */
  std::list<std::string> solverWishList = {"Gurobi","CPLEX","SCIP","LPSolve"};
  MoovacScheduler *mv = new HatScheT::MoovacScheduler(G, RM, solverWishList);
  mv->setSolverTimeout(60);  // limit the amount of time (in seconds) the solver spends on a single candidate II
  mv->setThreads(4);         // enable multi-threading mode
  mv->setSolverQuiet(true);  // suppress the solver's progress output

  /*
   * Perform the actual scheduling. HatScheT automatically computes lower and upper bounds for the candidate IIs to try.
   */
  mv->schedule();

  /*
   * If a schedule was found, print it.
   */
  if (mv->getScheduleFound()) {
    std::cout << "Found a schedule for II=" << mv->getII() << " and with length=" << mv->getScheduleLength() << ":"
              << std::endl;
    // Always a good a idea to verify the schedule before proceeding with it...
    if (verifyModuloSchedule(G, RM, mv->getSchedule(), mv->getII())) {
      for (int t = 0; t < mv->getScheduleLength(); ++t) {
        std::cout << "--- time step " << t << " ---" << std::endl;
        for (auto *v : G.Vertices())
          if (mv->getStartTime(*v) == t)
            std::cout << "(" << v->getId() << ")" << std::endl;
      }
    }
  } else {
    std::cout << "No schedule found!" << std::endl;
  }

  /*
   * Cleanup.
   */
  delete mv;
}
