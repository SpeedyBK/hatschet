//
// Created by nfiege on 03/12/19.
//

#ifndef HATSCHET_BINDING_H
#define HATSCHET_BINDING_H

#include <HatScheT/Vertex.h>
#include <HatScheT/ResourceModel.h>
#include <vector>

namespace HatScheT {

	class Binding {
	public:
		enum objective {
			minimize,
			maximize
		};
		struct RegChainBindingContainer {
			// a string that holds information about the solution quality (e.g., "TIMEOUT_FEASIBLE" or "OPTIMAL")
			std::string solutionStatus="NOT_SOLVED";
			// register costs for easy tracking
			int registerCosts=-1;
			// multiplexer costs for easy tracking
			int multiplexerCosts=-1;
			// each vertex in a graph is bound to a specific FU of its resource type
			std::map<std::string,int> resourceBindings;
			// list of connections between FUs
			// one list element: pair<pair<pair<src resource type, src FU number>, pair<dst resource type, dst FU number>>, pair<number of lifetime registers, dst input port number>>
			std::list<std::pair<std::pair<std::pair<std::string,int>,std::pair<std::string,int>>,std::pair<int,int>>> fuConnections;
			// container to keep track of port assignments in case there are commutative operations inside the DFG
			std::map<Edge*, int> portAssignments;
		};
		struct RatIIRegChainBindingContainer {
			// a string that holds information about the solution quality (e.g., "TIMEOUT_FEASIBLE" or "OPTIMAL")
			std::string solutionStatus="NOT_SOLVED";
			// register costs for easy tracking
			int registerCosts=-1;
			// multiplexer costs for easy tracking
			int multiplexerCosts=-1;
			// The same as above but every container is actually a vector with one sub-container per sample
			std::vector<std::map<std::string,int>> resourceBindings;
			// This one is actually the same as above... the list might be longer though...
			std::list<std::pair<std::pair<std::pair<std::string,int>,std::pair<std::string,int>>,std::pair<int,int>>> fuConnections;
			// container to keep track of port assignments in case there are commutative operations inside the DFG
			// -> one edge per sample
			std::map<Edge*, std::vector<int>> portAssignments;
		};
		struct BindingContainer {
			// a string that holds information about the solution quality (e.g., "TIMEOUT_FEASIBLE" or "OPTIMAL")
			std::string solutionStatus="NOT_SOLVED";
			// register costs for easy tracking
			int registerCosts=-1;
			// multiplexer costs for easy tracking
			int multiplexerCosts=-1;
			// each vertex in a graph is bound to a set of specific FUs of its resource type
			// this structure supports multiple bindings for each vertex
			std::map<std::string,std::set<int>> resourceBindings;
			// container to keep track of any possible connections
			//   [0]: src resource name
			//   [1]: src index
			//   [2]: src output port
			//   [3]: dst resource name
			//   [4]: dst index
			//   [5]: dst input port
			//   [6]: a set of time steps in which this connection is active (usually controlled by a MUX on the dst input)
			// resource name "register" is reserved for registers only
			// possibilities:
			//   FU -> FU
			//   FU -> register
			//   register -> FU
			//   register -> register
			std::list<std::tuple<std::string, int, int, std::string, int, int, std::set<int>>> connections;
			// container to keep track in which times the registers accept their input data
			std::map<int, std::set<int>> registerEnableTimes;
			// container to keep track of port assignments in case there are commutative operations inside the DFG
			// the pair consists of output port of src vertex and input port of dst vertex
			std::map<Edge*, std::pair<int,int>> portAssignments;
		};
		/*!
		 * overload << operator, useful for, e.g., std::cout << bindingContainer;
		 * @param os ostream object
		 * @param b binding to stream into os
		 * @return updated os
		 */
		friend ostream& operator<<(ostream& os, const BindingContainer& b);
		/*!
		 * assume that resource bindings are set
		 * calculate and set all necessary FU connections and resulting MUX and register costs
		 * @param b binding container with correctly defined and valid resource bindings
		 * @param g graph for which the binding was calculated
		 * @param rm corresponding resource model
		 * @param sched a schedule for g that satisfies the II
		 * @param II initiation interval
		 * @param portAssignments a container with input port connections for each edge destination vertex
		 */
		static void calcFUConnectionsAndCosts(RegChainBindingContainer* b, Graph* g, ResourceModel* rm, std::map<Vertex*, int>* sched, int II, std::map<Edge*,int> *portAssignments);
		/*!
		 * @brief count the total number of needed lifetime registers for that graph, resource model schedule and binding
		 * usable for rational IIs
		 * @param g graph
		 * @param rm resource model
		 * @param schedule schedule container
		 * @param binding binding container
		 * @param M cycle length
		 * @param S number of samples
		 * @param quiet suppress debug messages if true
		 * @return
		 */
		static int countTotalLifetimeRegisters(Graph* g, ResourceModel* rm, std::vector<std::map<Vertex*, int>> schedule, std::vector<std::map<Vertex*, int>> binding, int M, int S, bool quiet=true);
		/*!
		 * @brief same as above but for integer IIs
		 * @param g
		 * @param rm
		 * @param schedule
		 * @param binding
		 * @param II
		 * @return
		 */
		static int countTotalLifetimeRegisters(Graph* g, ResourceModel* rm, std::map<Vertex*, int> schedule, std::map<Vertex*, int> binding, int II, bool quiet=true);
		/*!
		 * @brief create simple binding that just assignes resources as vertices appear in the schedule
		 * @param sched
		 * @param rm
		 * @param II
		 * @return
		 */
		static std::map<const Vertex*,int> getSimpleBinding(const std::map<Vertex*, int> &sched, ResourceModel* rm, int II);
		/*!
		 * @brief create simple rational II binding that just assignes resources as vertices appear in the schedule
		 * @param sched
		 * @param rm
		 * @param M
		 * @param S
		 * @return
		 */
		static std::vector<std::map<const Vertex*,int>> getSimpleRationalIIBinding(std::vector<std::map<Vertex*, int>> sched, ResourceModel* rm, int M, int S);
#ifdef USE_SCALP
		/*! ATTENTION: DO NOT USE THIS AS IT IS NOT FINISHED YET
		 * @brief create an ILP-based binding for an integer-II schedule
		 * goal: minimize mux utilization
		 * based on 'Simultaneous FU and Register Binding Based on Network Flow Method' by Jason Cong and Junjuan Xu
		 * ASSUMPTIONS:
		 *   (1) NO OPERATION IS COMMUTATIVE (i.e., does not find optimal results for e.g. adders)
		 *   (2) REGISTERS ARE IMPLEMENTED WITH ENABLE-INPUTS AND NOT AS REGISTER-CHAINS
		 *   (3) ALL LIFETIMES ARE SHORTER THAN THE II
		 * unroll the graph S times and implement it with cycle length S*II/S if (3) is violated
		 * and set S such that (3) is not violated anymore
		 * @param sched schedule
		 * @param g graph
		 * @param rm resource model
		 * @param II initiation interval
		 * @param portAssignments specify for each edge the input port number of the destination vertex
		 * @param sw solver wishlist
		 * @param timeout max solving time in seconds
		 * @param quiet suppress debug outputs
		 * @return a map of vertex to FU-number
		 */
		static BindingContainer getILPBasedIntIIBindingCong(map<Vertex*, int> sched, Graph* g, ResourceModel* rm,
																																int II, std::map<Edge*,int> portAssignments,
																																std::list<std::string> sw = {}, int timeout= 300,
																																bool quiet = true);
		/*!
		 * @brief getILPBasedIntIIBinding create a binding with minimal number of mux inputs and minimal number of registers
		 * (assuming register sharing unlike the one by Cong and Xu!!!)
		 * @param sched schedule times
		 * @param g graph
		 * @param rm resource model
		 * @param II
		 * @param wMux weighting factor for multiplexer costs
		 * @param wReg weighting factor for register costs
		 * @param portAssignments map that specifies which edge has to be connected to which input port of the destination vertex
		 * @param maxMux upper limit for multiplexer costs; set to -1 for no limit
		 * @param maxReg upper limit for register costs; set to -1 for no limit
		 * @param commutativeOps a set of commutative operation types (e.g. add or mult)
		 * @param sw solver wishlist
		 * @param timeout timeout for ilp solver
		 * @return binding
		 */
		static RegChainBindingContainer getILPBasedIntIIBinding(map<Vertex*, int> sched, Graph *g, ResourceModel* rm, int II,
																														int wMux, int wReg, std::map<Edge*,int> portAssignments, double maxMux=-1.0, double maxReg=-1.0,
																														std::set<const Resource*> commutativeOps = {}, std::list<std::string> sw = {}, int timeout=300, bool quiet=true);
		/*!
		 * @brief getILPBasedRatIIBinding create a binding with minimal number of mux inputs and minimal number of registers
		 * (assuming register sharing unlike the one by Cong and Xu!!!)
		 * @param sched schedule times
		 * @param g graph
		 * @param rm resource model
		 * @param samples II = modulo / samples
		 * @param modulo II = modulo / samples
		 * @param wMux weighting factor for multiplexer costs
		 * @param wReg weighting factor for register costs
		 * @param portAssignments map that specifies which edge has to be connected to which input port of the destination vertex
		 * @param maxMux upper limit for multiplexer costs; set to -1 for no limit
		 * @param maxReg upper limit for register costs; set to -1 for no limit
		 * @param commutativeOps a set of commutative operation types (e.g. add or mult)
		 * @param sw solver wishlist
		 * @param timeout timeout for ilp solver
		 * @return binding
		 */
		static RatIIRegChainBindingContainer getILPBasedRatIIBinding(std::vector<map<Vertex*, int>> sched, Graph *g, ResourceModel* rm, int samples,
																																 int modulo, int wMux, int wReg, std::map<Edge*,int> portAssignments, double maxMux=-1.0, double maxReg=-1.0,
																																 std::set<const Resource*> commutativeOps = {}, std::list<std::string> sw = {}, int timeout=300, bool quiet=true);
	#endif
	};
}

#endif //HATSCHET_BINDING_H
