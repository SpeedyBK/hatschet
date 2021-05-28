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
		struct BindingContainer {
			// each vertex in a graph is bound to a specific FU of its resource type
			std::map<const Vertex*,int> resourceBindings;
			// each variable is bound to a given register (if registers are implemented with enable inputs)
			// ignore this container if registers are implemented as register chains
			std::map<const Vertex*,int> registerBindings;
		};
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
		static std::map<const Vertex*,int> getSimpleBinding(std::map<Vertex*, int> sched, ResourceModel* rm, int II);
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
		/*!
		 * @brief create an ILP-based binding for an integer-II schedule
		 * goal: minimize mux utilization
		 * based on 'Simultaneous FU and Register Binding Based on Network Flow Method' by Jason Cong and Junjuan Xu
		 * THIS ASSUMES THAT EVERY OPERATION IS COMMUTATIVE (e.g. not applicable for subtractors)
		 * THIS ALSO ASSUMES THAT REGISTERS ARE IMPLEMENTED WITH ENABLE-INPUTS AND NOT AS REGISTER-CHAINS
		 * @param sched schedule
		 * @param g graph
		 * @param rm resource model
		 * @param II initiation interval
		 * @param sw solver wishlist
		 * @param timeout max solving time in seconds
		 * @return a map of vertex to FU-number
		 */
		static BindingContainer getILPBasedIntIIBinding(map<Vertex*, int> sched, Graph* g, ResourceModel* rm,
																																int II, std::list<std::string> sw = {}, int timeout=300);
		/*!
		* @brief create an ilp-based binding for a rational II schedule
		* the goal is to minimize MUX and register allocation
		* if you try to understand this: good luck, may the force be with you :-) for questions ask sittel@uni-kassel.de
		* rational II enhancement of 'Simultaneous FU and Register Binding Based on Network Flow Method'
		* Jason Cong and Junjuan Xu
		* DATE 2008
		* @param sched
		* @param rm
		* @param modulo
		* @param initIntervalls
		* @return
		*/
		static vector<std::map<const Vertex*,int> > getILPBasedRatIIBinding(map<Vertex*, int> sched, Graph* g, ResourceModel* rm,
																																				int modulo, vector<int> initIntervalls, std::list<std::string> sw = {}, int timeout=300);
		/*!
		 * @brief getILPMinRegBinding create a binding with minimal number of lifetime registers (assuming register sharing!)
		 * @param sched schedule times
		 * @param g graph
		 * @param rm resource model
		 * @param II
		 * @param sw solver wishlist
		 * @param timeout timeout for ilp solver
		 * @return binding
		 */
		static std::map<const Vertex*,int> getILPMinRegBinding(map<Vertex*, int> sched, Graph *g, ResourceModel* rm, int II, std::list<std::string> sw = {}, int timeout=300);
		/*!
		 * @brief getILPMinRegBinding create a binding with minimal number of mux inputs
		 * (assuming register sharing unlike the one by Cong and Xu!!!)
		 * @param sched schedule times
		 * @param g graph
		 * @param rm resource model
		 * @param II
		 * @param portAssignments map that specifies which edge has to be connected to which input port of the destination vertex
		 * @param commutativeOps a set of commutative operation types (e.g. add or mult)
		 * @param sw solver wishlist
		 * @param timeout timeout for ilp solver
		 * @return binding
		 */
		static std::map<const Vertex*,int> getILPMinMuxBinding(map<Vertex*, int> sched, Graph *g, ResourceModel* rm, int II,
			std::map<Edge*,int> portAssignments, std::set<const Resource*> commutativeOps = {},
			std::list<std::string> sw = {}, int timeout=300);
#endif
	};
}

#endif //HATSCHET_BINDING_H
