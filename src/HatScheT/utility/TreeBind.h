//
// Created by nfiege on 03/12/19.
//

#ifndef HATSCHET_TREEBIND_H
#define HATSCHET_TREEBIND_H

#include <HatScheT/Graph.h>
#include <HatScheT/ResourceModel.h>
#include <HatScheT/utility/Binding.h>
#include <vector>
#include <map>
#include <list>
#include <ScaLP/Solver.h>

namespace HatScheT {
  class Binding; // forward declaration to prevent linker errors
  
  class TreeBind {
  public:
    /*!
     * default constructor
     * @param g input graph
     * @param rm input resource model
     * @param sched container with schedule time for each vertex in g
     * @param II initiation interval
     * @param portAssignments container that specifies input ports for
     * dst vertices of each edge in g
     */
    TreeBind(Graph* g, ResourceModel* rm, std::map<Vertex*, int> sched, int II, std::map<Edge*,int> portAssignments);
    /*!
     * specify time budget in seconds 
     * @param timeout 
     */
    void setTimeout(unsigned int timeout);
    /*!
     * suppress debug outputs based on q
     * @param q true: suppress; false: print to stdout
     */
    void setQuiet(bool q);
    /*!
     * main binding algorithm
     */
    void bind();
    /*!
     * @return binding
     */
    Binding::BindingContainer getBinding();
    /*!
     * set weighting factor for multiplexer costs
     */
    void setMuxCostFactor(double wMux);
    /*!
     * set weighting factor for multiplexer costs
     */
    void setRegCostFactor(double wReg);
    /*!
     * @return solution status
     */
    std::string getSolutionStatus();
    
  private:
  	/*!
  	 * track number of iterations in iterativeTreeSearch
  	 */
  	int iterationCounter;
  	/*!
  	 * suppress debug outputs if this is true
  	 */
  	bool quiet;
    /*!
     * this one is a bit ugly. 
     * this is a map from one list element of the binding container 
     * to a list of edges that need this list element
     * this map is needed to manage the current binding container
     */
    std::map<std::pair<std::pair<std::pair<std::string,int>,std::pair<std::string,int>>,std::pair<int,int>>, std::list<Edge*>> bindingEdgeMap;
    /*!
     * removes vertex from current binding and updates costs
     * @param v vertex that gets removed
     */
    void removeBindingInfo(Vertex* v);
    /*!
     * this is called by removeBindingInfo(Vertex*)
     * it removes binding info for a specific edge
     * @param e edge to remove
     */
    void removeBindingInfo(Edge* e);
    /*!
     * adds vertex to current binding and updates costs
     * @param v vertex that gets added
     */
    void addBindingInfo(Vertex* v);
    /*!
     * this is called by addBindingInfo(Vertex*)
     * it adds binding info for a specific edge
     * @param e edge to add
     */
    void addBindingInfo(Edge* v);
    /*!
     * @param e edge of interest
     * @return lifetime of edge
     */
    int getLifetime(const Edge* e);
    /*!
     * store lifetimes for fast lookup
     */
    std::map<const Edge*, int> lifetimes;
    /*!
     * store number of lifetime registers for each FU for fast lookup
     * <resource name, fu number> -> (descendingly sorted) list <<number of lifetime registers, edge that was responsible for that>>
     */
    std::map<std::pair<std::string, int>, std::list<std::pair<int, Edge*>>> lifetimeRegisters;
    /*!
     * container with outgoing edges for fast lookup
     */
    std::map<const Vertex*, std::list<const Edge*>> outgoingEdges;
    /*!
     * container with incoming edges for fast lookup
     */
    std::map<const Vertex*, std::list<const Edge*>> incomingEdges;
    /*!
     * weighting factor for multiplexer costs
     */
    double wMux;
    /*!
     * weighting factor for register costs
     */
    double wReg;
    /*!
     * set the next possible binding based on the resource queues
     * and the previously tried bindings
     * this also updates triedBindings container
     * @param v the vertex that this should compute the next binding for
     * @return if it succeeded
     */
    bool setBinding(Vertex* v);
    /*!
     * main function to compute the binding by iteratively searching
     * a tree
     */
    void iterativeTreeSearch();
    /*!
     * input graph
     */
    Graph* g;
    /*!
     * input resource model
     */
    ResourceModel* rm;
    /*!
     * schedule times for all operations
     */
    std::map<Vertex*, int> sched;
    /*!
     * initiation interval
     */
    int II;
    /*!
     * port assignment for each edge in g
     */
    std::map<Edge*,int> portAssignments;
    /*!
     * time budget in seconds
     */
    double timeBudget;
    /*!
     * best solution found so far
     */
    Binding::BindingContainer bestBinding;
    /*!
     * keep track of current solution
     */
    Binding::BindingContainer currentBinding;
    /*!
     * sequence for tree traversal containing all limited operations
     */
    std::list<Vertex*> queue;
    /*!
     * sequence for resource bindings all limited operations
     */
    std::map<Vertex*, std::list<int>> resourceQueues;
    /*!
     * keep track of the last binding that was tried for each vertex
     */
    std::map<Vertex*, int> lastTriedBinding;
    /*!
     * this container tracks how many resources are occupied 
     * in each modulo slot
     * map from <resource, modulo slot> to list with indices of occupied resources
     */
    std::map<std::pair<const Resource*, int>, std::list<int>> occupiedResources;
    /*!
     * track solution status
     */
    std::string status;
  };
}

#endif //HATSCHET_TREEBIND_H
