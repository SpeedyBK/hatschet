//
// Created by bkessler on 23/07/19.
//

#include "GraphReduction.h"

#include <cmath>
#include "HatScheT/utility/subgraphs/KosarajuSCC.h"

namespace HatScheT {

  GraphReduction::GraphReduction(Graph &g, ResourceModel &resourceModel, std::list<std::string>  solverWishlist) : SchedulerBase(g, resourceModel), ILPSchedulerBase(solverWishlist) {
    II = -1;
    this->timeouts = 0;
    startTimes.clear();
    scheduleFound = false;
    optimalResult = true;
    computeMinII(&g, &resourceModel);
    minII = ceil(minII);
    computeMaxII(&g, &resourceModel);
  }


  void GraphReduction::schedule() {
    cout << endl << "GraphReduction::schedule: start" << endl;

    //ToDo implement all the stuff! .. WIP

    // Gets the SCCs of graph g.
    cout << endl << "Starting Kosajaru's SCC Algorithm..." << endl;
    KosarajuSCC kscc(this->g);
    this -> sccs = kscc.getSCCs();
    cout << endl << "Finished Kosajaru's SCC Algorithm." << endl;


    //To check wether we have a complex or a basic SCC, we have to iterrate through the SCC vectors, and lookup
    //the resources for each vertex. If all verticies in a SCC have unlimited ressources, the SCC will be a basic
    //component else the component is a complex component.
    cout << endl << "Sorting strongly connected components..." << endl;
    sortSCCs();
    cout << endl << "Sorting done." << endl;


    //Debug Output:
    cout << endl << "Complex:" << endl;

    int i = 0;
    for (auto strongly:complexSCCs){
      cout << "SCC " << i << ": ";
      for (auto V: strongly){
        cout << V->getName() << " ";
      }
      cout << endl;
      i++;
    }

    cout << endl << "Basic:" << endl;

    i = 0;
    for (auto strongly:basicSCCs){
      cout << "SSC " << i << ": ";
      for (auto V: strongly){
        cout << V->getName() << " ";
      }
      cout << endl;
      i++;
    }

    cout << endl << "Trivial:" << endl;

    i = 0;
    for (auto strongly:trivialSCCs){
      cout << "SSC " << i << ": ";
      for (auto V: strongly){
        cout << V->getName() << " ";
      }
      cout << endl;
      i++;
    }

    mapVertexToComponent(basicSCCs);
    mapVertexToComponent(complexSCCs);
    mapVertexToComponent(trivialSCCs);

    getConnectedComponents(complexSCCs);

    for (auto components : basicSCCs) {
      generateGraph(components);
    }

    //ToDo: Check if basic SCCs are connected. If they are not, then put them in a basic-supergraph.

    //ToDo: Do the same for complex SCCs.

    //ToDo: Get a sheduling for each supergraph.

    //ToDo: Get a general scheduling for the complete graph.

    cout << endl << "GraphReduction::schedule: finished" << endl;

  }

  void GraphReduction::sortSCCs() {

    //Iterates through the SCCs, and sorts them into basic, complex, or trivial, depending on the resource model.

    bool complex = false;

    for (auto stronglyConnectedComponent:sccs) {
      if (stronglyConnectedComponent.size() > 1) {
        for (auto V: stronglyConnectedComponent) {
          cout << ".";
          if (resourceModel.getResource(V)->getLimit() != -1) {
            complex = true;
            break;
          } else {
            complex = false;
          }
        }
        if (complex) {
          complexSCCs.push_back(stronglyConnectedComponent);
        } else {
          basicSCCs.push_back(stronglyConnectedComponent);
        }
      } else {
        trivialSCCs.push_back(stronglyConnectedComponent);
      }
    }
  }


  Graph *GraphReduction::generateGraph(vector<Vertex *> SCC) {

    //ToDo: Not done yet...
    cout << endl << "Generating Graph from SCC" << endl;
    auto *h = new Graph();

    for (auto V:SCC) {
      &h->createVertex(V->getId());
    }

    return h;
  }


  void GraphReduction::getConnectedComponents(vector<vector<Vertex *>> SCCs) {
      //ToDo: Has to be done.

    for (auto component:SCCs){
      for (auto V:component){
        for (auto e:g.Edges()){
          if (&e->getVertexSrc() == V){
            cout << e->getVertexSrcName() << " - " << e->getVertexDstName() << endl;
          }
        }
      }
    }
  }

  void GraphReduction::mapVertexToComponent(vector <vector<Vertex*>> SCCs) {

    for (auto component: SCCs){
      for (auto v : component){
        vertexComponentMap[v] = component;
      }
    }

    cout << endl;

    for (auto it : vertexComponentMap){
      cout << it.first->getName() << ": " << &it.second[0]->getName() << endl;
    }
  }
}

