//
// Created by nfiege on 7/1/22.
//

#ifndef HATSCHET_OPTIMALINTEGERIISATBINDING_H
#define HATSCHET_OPTIMALINTEGERIISATBINDING_H

#ifdef USE_CADICAL

#include <HatScheT/Graph.h>
#include <HatScheT/ResourceModel.h>
#include <HatScheT/utility/Binding.h>
#include <HatScheT/utility/BindingBase.h>
#include <HatScheT/scheduler/satbased/CaDiCaLTerminator.h>
#include <vector>
#include <map>
#include <list>
#include <chrono>
#include <memory>
#include <cadical.hpp>

namespace HatScheT {
	class OptimalIntegerIISATBinding : public BindingBase {
	public:
		/*!
		 * define what to minimize first; the other one is minimized afterwards
		 */
		enum firstObjectiveType {
			firstObjectiveMuxMin_t,
			firstObjectiveRegMin_t
		};
		/*!
		 * default constructor
		 * @param g input graph
		 * @param rm input resource model
		 * @param sched container with schedule time for each vertex in g
		 * @param II initiation interval
		 * @param portAssignments container that specifies input ports for
		 * dst vertices of each edge in g
		 * @param commutativeOps container with all commutative operation types
		 */
		OptimalIntegerIISATBinding(Graph* g, ResourceModel* rm, const std::map<Vertex*, int> &sched, const int &II, const std::map<Edge*,int> &portAssignments, const std::set<const Resource*> &commutativeOps = {});
		/*!
		 * define first optimization goal
		 * @param t
		 */
		void setFirstObjective(firstObjectiveType t);
		/*!
		 * override the function that actually does the binding
		 */
		void bind() override;
		/*!
		 * override base class's method
		 * @param b binding container to set
		 */
		void getBinding(Binding::RegChainBindingContainer* b) override;
		/*!
		 * getter for the edge lifetime
		 * @param e the edge in question
		 * @param edgeLifetimes the container where all edge lifetimes should be stored
		 * @param sched the schedule
		 * @param II the initiation interval
		 * @param rm the resource model
		 * @return its lifetime
		 */
		static int getEdgeLifetime(Edge *e, std::map<Edge*, int>* edgeLifetimes, const std::map<Vertex*, int> &sched, const int &II, ResourceModel* rm);
		/*!
		 * computes a complete binding container from
		 * @param fuBindings FU binding for each vertex
		 * @param portAssignments for each edge
		 * @param g associated graph
		 * @param rm associated resource model
		 * @return a binding container
		 */
		static Binding::RegChainBindingContainer getBindingContainerFromBinding(const std::map<const Vertex*, int> &fuBindings, const std::map<Edge*, int> &portAssignments, Graph* g, ResourceModel* rm, const std::map<Vertex*, int> &sched, const int &II);
		/*!
		 * same as above but with another fu bindings container type (vertex names instead of pointers)
		 * @param fuBindings
		 * @param portAssignments
		 * @param g
		 * @param rm
		 * @return
		 */
		static Binding::RegChainBindingContainer getBindingContainerFromBinding(const std::map<std::string, int> &fuBindings, const std::map<Edge*, int> &portAssignments, Graph* g, ResourceModel* rm, const std::map<Vertex*, int> &sched, const int &II);

	private:
		/*!
		 * calculate and define lower bounds according to graph and resource model
		 */
		void defineLowerBounds();
		/*!
		 * first: lower bound on interconnect costs
		 * second: upper bound on interconnect costs
		 */
		std::pair<int, int> interconnectBounds;
		/*!
		 * first: lower bound on interconnect costs with consideration of commutativity
		 * second: upper bound on interconnect costs with consideration of commutativity
		 */
		std::pair<int, int> interconnectCommutativeBounds;
		/*!
		 * first: lower bound on lifetime register costs
		 * second: upper bound on lifetime register costs
		 */
		std::pair<int, int> registerBounds;
		/*!
		 * what to minimize first - MUXs or Regs?
		 */
		firstObjectiveType firstObjective;
		/*!
		 * objective of the SAT formulation
		 */
		enum SATFormulationType {
			regMin_t,
			muxMin_t,
			muxMinCommutative_t,
			regAndMuxMin_t,
			regAndMuxMinCommutative_t
		};
		/*!
		 * elapsed time in the bind method
		 */
		double elapsedTime;
		/*!
		 * unique binding for all unlimited operations
		 * todo: check if they are really needed?!
		 */
		std::map<Vertex*,int> unlimitedOpFUs;
		/*!
		 * for each vertex store all FUs where this vertex can be bound to
		 */
		std::map<const Vertex*, std::set<int>> possibleResourceBindings;
		/*!
		 * maximal number of lifetime registers after a specific FU
		 */
		std::map<std::pair<const Resource*, int>, int> maxResourceLifetimes;
		/*!
		 * calculate content of this->maxResourceLifetimes
		 */
		void calcResourceLifetimes();
		/*!
		 * compute a more or less random binding as a starting point for the optimization procedure
		 */
		void computeInitialBinding();
		/*!
		 * number of input ports of each resource instance
		 */
		std::map<const Resource*, int> numResourcePorts;
		/*!
		 * lifetime of an edge
		 */
		std::map<Edge*,int> edgeLifetimes;
		/*!
		 * connection type of an edge
		 * tuple:
		 *     - src resource type
		 *     - dst resource type
		 *     - src output port
		 *     - dst input port
		 *     - edge lifetime
		 */
		std::map<Edge*, std::tuple<const Resource*, const Resource*, int, int, int>> edgeTypes;
		/*!
		 * same as above but with consideration of commutativity
		 *   -> dst input port is -1 if the dst is commutative
		 */
		std::map<Edge*, std::tuple<const Resource*, const Resource*, int, int, int>> edgeTypesCommutative;
		/*!
		 * manages the edge types container and returns the edge type of a given edge
		 * @param e the edge of interest
		 * @param considerCommutativity whether commutativity of the sink vertex should be considered
		 * @return its type
		 */
		std::tuple<const Resource*, const Resource*, int, int, int>& getEdgeType(Edge* e, const bool &considerCommutativity);
		/*!
		 * lifetime of a variable produced by a vertex
		 */
		std::map<Vertex*, int> variableLifetimes;
		/*!
		 * possible port connections in the final binding
		 */
		std::map<std::pair<int,int>,std::set<int>> possiblePortConnections;
		/*!
		 * possible lifetimes due to the schedule
		 */
		std::map<std::pair<int,int>,std::set<int>> possibleLifetimes;
		/*!
		 * a mapping between resource instances and their unique indices
		 */
		std::map<std::pair<const Resource*,int>,int> fuIndexMap;
		/*!
		 * same as above but reversed
		 */
		std::map<int,std::pair<const Resource*,int>> indexFuMap;
		/*!
		 * resource limits imposed by the schedule
		 */
		std::map<const Resource*,int> resourceLimits;
		/*!
		 * < resource, FU idx, t mod II > -> < all operations that could be executed by this FU in that modulo slot >
		 */
		std::map<std::tuple<const Resource*, int, int>, std::set<Vertex*>> operationsInModSlots;
		/*!
		 * port assignment for each edge in g
		 */
		std::map<Edge*,int> portAssignments;
		/*!
		 * best solution found so far
		 */
		Binding::RegChainBindingContainer binding;
		/*!
		 * solver to check feasibility of the context
		 */
		std::unique_ptr<CaDiCaL::Solver> solver;
		/*!
		 * object to track timeouts
		 */
		CaDiCaLTerminator terminator;
		/*!
		 * count the number of variables needed to model the binding problem
		 */
		int variableCounter;
		/*!
		 * operations -> fu
		 */
		int operationBindingVarCounter;
		/*!
		 * edges -> mux ports
		 */
		int connectionBindingVarCounter;
		/*!
		 * fu registers -> lifetime registers
		 */
		int lifetimeRegBindingVarCounter;
		/*!
		 * edges -> dst input ports
		 */
		int edgePortBindingVarCounter;
		/*!
		 * count the number of clauses needed to model the binding problem
		 */
		int clauseCounter;
		/*!
		 * at least one operation binding
		 */
		int operationBindingClauseCounter;
		/*!
		 * no operation binding conflicts
		 */
		int noBindingConflictsClauseCounter;
		/*!
		 * set register bindings
		 */
		int registerBindingClauseCounter;
		/*!
		 * no register binding conflicts
		 */
		int noRegisterConflictsClauseCounter;
		/*!
		 * at least one mux input binding
		 */
		int interconnectBindingClauseCounter;
		/*!
		 * no mux input binding conflicts
		 */
		int noInterconnectConflictsClauseCounter;
		/*!
		 * at least one port binding
		 */
		int portBindingClauseCounter;
		/*!
		 * at least one edge binding
		 */
		int edgeBindingClauseCounter;
		/*!
		 * no port binding conflicts
		 */
		int noPortBindingConflictsClauseCounter;
		/*!
		 * populate following containers:
		 *     - operationBindingVars
		 *     - connectionBindingVars
		 *     - lifetimeRegBindingVars
		 *     - edgePortBindingVars
		 * @param maxNumRegs maximal number of available registers
		 * @param maxNumConnections maximal number of available connections
		 * @param considerCommutativity whether commutativity should be considered
		 * 		-> e.g. edgePortBinding variables are only created if considerCommutativity == true
		 */
		void createVariables(const int &maxNumRegs, const int &maxNumConnections, const bool &considerCommutativity);
		/*!
		 * like create variables, but for clauses
		 * @param regLimit
		 * @param muxLimit
		 * @param considerCommutativity
		 */
		void createClauses(const int &maxNumRegs, const int &maxNumConnections, const bool &considerCommutativity);
	 	/*!
	 	 * < vertex, FU index where that vertex can be bound to >
	 	 */
		std::map<std::pair<Vertex*, int>, int> operationBindingVars;
		/*!
		 * < edge, MUX port index where that edge can be bound to >
		 */
		std::map<std::pair<Edge*, int>, int> connectionBindingVars;
		/*!
		 * < < FU type, FU index, lifetime register, register index where that specific lifetime register can be bound to >
		 */
		std::map<std::tuple<const Resource*, int, int, int>, int> lifetimeRegBindingVars;
		/*!
		 * < Edge, port number of the commutative sink vertex >
		 */
		std::map<std::pair<Edge*, int>, int> edgePortBindingVars;
	};
}

#endif //USE_CADICAL
#endif //HATSCHET_OPTIMALINTEGERIISATBINDING_H
