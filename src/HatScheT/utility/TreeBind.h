//
// Created by nfiege on 03/12/19.
//

#ifndef HATSCHET_TREEBIND_H
#define HATSCHET_TREEBIND_H

#ifdef USE_SCALP
#include <HatScheT/Graph.h>
#include <HatScheT/ResourceModel.h>
#include <HatScheT/utility/Binding.h>
#include <vector>
#include <map>
#include <list>
#include <ScaLP/Solver.h>
#include <chrono>

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
    TreeBind(Graph* g, ResourceModel* rm, std::map<Vertex*, int> sched, int II, std::map<Edge*,int> portAssignments, std::set<const Resource*> commutativeOps = {});
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
    Binding::RegChainBindingContainer getBinding();
    /*!
     * set weighting factor for multiplexer costs;
     * setting it to a negative value means that registers are optimized as a first objective and multiplexers second
     */
    void setMuxCostFactor(double wMux);
    /*!
     * set weighting factor for register costs
     * setting it to a negative value means that multiplexers are optimized as a first objective and registers second
     */
    void setRegCostFactor(double wReg);
    /*!
     * @return solution status
     */
    std::string getSolutionStatus();
    /*!
     * sets upper limit for mux costs
     * @param l new limit
     */
		void setMuxLimit(double l);
		/*!
		 * sets upper limit for register costs
		 * @param l new limit
		 */
		void setRegLimit(double l);
		/*!
		 * setter for this->pruningEnabled
		 * @param p new value
		 */
		void setPruning(bool p);
		/*!
		 * setter for this->skipEquivalentBindings
		 * @param s new value
		 */
		void setSkipEquivalent(bool s);
		/*!
		 * getter for this->numFeasibleBindings
		 * @return this->numFeasibleBindings
		 */
		double getNumFeasibleBindings();
		/*!
		 * set new optimization objective
		 * @param o new optimization objective
		 */
    void setObjective(Binding::objective o);
  private:
  	/*!
  	 * resources with exactly one vertex associated to them have a trivial binding
  	 * even if they are limited
  	 */
  	std::set<const Resource*> trivialBindings;
  	/*!
  	 * this stores for each commutative operation if we assume in the current binding that its inputs are swapped
  	 */
  	std::map<const Vertex*, bool> commutativeInputsSwapped;
  	/*!
  	 * contains all commutative operation types
  	 * ATTENTION: at the moment, each commutative operation must have EXACTLY 2 inputs
  	 */
		std::set<const Resource*> commutativeOps;
		/*!
		 * compare two bindings and return if A is better than B based on this->objective
		 * @param bindingA A
		 * @param bindingB B
		 * @return if A is better than B
		 */
		bool bindingABetterThanB(Binding::RegChainBindingContainer* bindingA, Binding::RegChainBindingContainer* bindingB);
		/*!
		 * compare two bindings and return if A is better than B or equally good based on this->objective
		 * @param bindingA A
		 * @param bindingB B
		 * @return if A is better than B or if both are equally good
		 */
		bool bindingABetterThanBOrEqual(Binding::RegChainBindingContainer* bindingA, Binding::RegChainBindingContainer* bindingB);
  	/*!
  	 * optimization goal
  	 */
  	Binding::objective obj;
  	/*!
  	 * this pushes a new vertex to the stack
  	 * @param stack the stack
  	 * @param v new vertex
  	 */
  	void pushToStack(std::list<std::pair<std::list<Vertex*>::iterator, int>> &stack, const std::list<Vertex*>::iterator &v);
  	/*!
  	 * cut off subtree from the search space if it only leads to bindings that are equivalent to already explored ones
  	 */
  	bool skipEquivalentBindings;
  	/*!
  	 * cut off subtree from the search space if an incomplete binding already has higher costs than the current best one
  	 */
  	bool pruningEnabled;
  	/*!
  	 * container for time tracking (in seconds)
  	 * map key: algorithm part; map value: time (in seconds) spent in that part
  	 */
  	std::map<std::string, double> timeTracker;
  	/*!
  	 * container to track time points
  	 * map key: algorithm part; map value: last time point
  	 */
  	std::map<std::string, std::chrono::steady_clock::time_point> timePoints;
		/*!
		 * calculate maximum number of feasible bindings
		 * need double precision because this number can be HUGE
		 * maybe even double precision is not enough but then we are doomed anyway
		 */
		double numFeasibleBindings;
		/*!
		 * track number of iterations in iterativeTreeSearch
		 */
		int iterationCounter;
  	/*!
  	 * track number of explored leaf nodes (i.e., number of feasible bindings)
  	 */
  	int leafNodeCounter;
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
     * this one might be even uglier
     * even though this container "only" stores the binding variables for each edge
     */
    std::map<Edge*,std::pair<std::pair<std::pair<std::string,int>,std::pair<std::string,int>>,std::pair<int,int>>*> edgeBindingMap;
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
     * @param nextFU its FU
     */
    void addBindingInfo(Vertex* v, int nextFU);
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
     * upper limit for mux costs
     */
    double maxMux;
    /*!
     * upper limit for register costs
     */
		double maxReg;
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
		 * modulo slots for all operations
		 */
		std::map<Vertex*, int> mod;
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
    Binding::RegChainBindingContainer bestBinding;
    /*!
     * keep track of current solution
     */
    Binding::RegChainBindingContainer currentBinding;
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
     * this container tracks how many and which resources are occupied
     * in each modulo slot
     * map from <resource, modulo slot> to list with indices of occupied resources
     */
    std::map<std::pair<const Resource*, int>, std::list<int>> occupiedResources;
    /*!
     * this container tracks how many operations are bound to which FU for each resource type
     * map from <resource, fu index> to the number of bound operations
     */
    std::map<std::pair<const Resource*, int>, int> numOperationsOnFU;
  };
}
#endif //USE_SCALP

#endif //HATSCHET_TREEBIND_H
