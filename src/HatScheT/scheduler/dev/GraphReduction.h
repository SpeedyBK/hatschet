//
// Created by bkessler on 23/07/19.
//

//UNDER DEVELOPEMENT

#ifndef HATSCHET_GRAPHREDUCTION_H
#define HATSCHET_GRAPHREDUCTION_H

#pragma once

#include <HatScheT/base/SchedulerBase.h>
#include <HatScheT/base/ILPSchedulerBase.h>
#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/base/IterativeSchedulerBase.h>
#include <vector>

namespace HatScheT {

  class GraphReduction : public SchedulerBase, public ILPSchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase {
  public:
    GraphReduction(Graph &g, ResourceModel &resourceModel, std::list<std::string> solverWishlist);

    /*!
     * main method to do the scheduling
     */
    virtual void schedule();


    /*!
     * not needed
     */
    virtual void setObjective() {/* unused */}
    virtual void resetContainer(){/* unused */}
    virtual void constructProblem() {/* unused */}

    /*!
     * \brief sortSCC() iterates through the vector of SCCS, an checks if an SCC is basic or complex. If a SCC is complex,
     * it will be pushed on the complexSCC vector, else it will be pushed on the basicSCC vector.
     */
    void sortSCCs();

    Graph* generateGraph(vector<Vertex*> SCC);

    void getConnectedComponents(vector <vector<Vertex*>> SCCs);

    void mapVertexToComponent (vector <vector<Vertex*>> SCCs);

  private:

    vector <vector<Vertex*>> sccs;

    vector <vector<Vertex*>> basicSCCs;
    vector <vector<Vertex*>> complexSCCs;
    vector <vector<Vertex*>> trivialSCCs;

    map <Vertex*, vector<Vertex*>> vertexComponentMap;

    vector <Graph*> complexSubGraph;
    vector <Graph*> basicSubGraph;


  };

}
#endif //HATSCHET_GRAPHREDUCTION_H
