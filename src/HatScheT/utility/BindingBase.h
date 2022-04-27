//
// Created by nfiege on 3/4/22.
//

#ifndef HATSCHET_BINDINGBASE_H
#define HATSCHET_BINDINGBASE_H

#include "HatScheT/Graph.h"
#include "HatScheT/ResourceModel.h"
#include "HatScheT/utility/Binding.h"
#include <map>

namespace HatScheT {
	class Binding; // forward declaration to prevent linker errors
	/*!
	 * base class for all (a little more complicated) binding algorithms
	 */
	class BindingBase {
	public:
		/*!
		 * constructor for binding base
		 * @param g graph
		 * @param rm resource model
		 * @param sched schedule
		 * @param II initiation interval
		 */
		BindingBase(Graph* g, ResourceModel* rm, std::map<Vertex*, int> sched, int II,
							std::set<const Resource*> commutativeOps);
		/*!
		 * main binding algorithm
		 */
		virtual void bind();
		/*!
		 * define the binding container with the solution of the binding algorithm
		 * @param b binding container for register chains
		 */
		virtual void getBinding(Binding::RegChainBindingContainer* b);
		/*!
		 * define the binding container with the solution of the binding algorithm
		 * @param b binding container for register chains
		 */
		virtual void getBinding(Binding::BindingContainer* b);
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

	protected:
		/*!
		 * keep track of the state of the (hopefully found) solution
		 */
		std::string solutionStatus;
		/*!
		 * container with all commutative resource types
		 */
		std::set<const Resource*> commutativeOps;
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
		 * time budget in seconds
		 */
		double timeBudget;
	};
}

#endif //HATSCHET_BINDINGBASE_H
