//
// Created by nfiege on 11/11/21.
//

#ifndef HATSCHET_OPTIMALINTEGERIIBINDING_H
#define HATSCHET_OPTIMALINTEGERIIBINDING_H

#ifdef USE_SCALP

#include <HatScheT/Graph.h>
#include <HatScheT/ResourceModel.h>
#include <HatScheT/utility/Binding.h>
#include <vector>
#include <map>
#include <list>
#include <ScaLP/Solver.h>

namespace HatScheT {
	class Binding; // forward declaration to prevent linker errors

	class OptimalIntegerIIBinding {
	public:
		/*!
		 * default constructor
		 * @param g input graph
		 * @param rm input resource model
		 * @param sched container with schedule time for each vertex in g
		 * @param II initiation interval
		 * @param portAssignments container that specifies input ports for
		 * dst vertices of each edge in g
		 * @param commutativeOps container with all commutative operation types
		 * @param sw ILP solver wishlist
		 */
		OptimalIntegerIIBinding(Graph* g, ResourceModel* rm, std::map<Vertex*, int> sched, int II, std::map<Edge*,int> portAssignments, std::set<const Resource*> commutativeOps = {}, std::list<std::string> sw = {});
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
		 * set ILP optimization goal
		 * @param o new optimization goal
		 */
		void setObjective(Binding::objective o);

	private:
		/*!
		 * container with all commutative resource types
		 */
		std::set<const Resource*> commutativeOps;
		/*!
		 * ILP solver wishlist
		 */
		std::list<std::string> sw;
		/*!
		 * solution status
		 */
		ScaLP::status status;
		/*!
		 * optimization goal
		 */
		Binding::objective obj;
		/*!
		 * suppress debug outputs if this is true
		 */
		bool quiet;
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
		Binding::RegChainBindingContainer binding;
	};
}

#endif //USE_SCALP

#endif //HATSCHET_OPTIMALINTEGERIIBINDING_H
