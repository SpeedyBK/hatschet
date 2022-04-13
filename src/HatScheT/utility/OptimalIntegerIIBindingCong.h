//
// Created by nfiege on 3/28/22.
//

#ifndef HATSCHET_OPTIMALINTEGERIIBINDINGCONG_H
#define HATSCHET_OPTIMALINTEGERIIBINDINGCONG_H

#include <HatScheT/utility/BindingBase.h>

namespace HatScheT {
	class OptimalIntegerIIBindingCong : public BindingBase {
	public:
		/*!
		 * constructor for optimal generalized binding
		 * @param g graph
		 * @param rm resource model
		 * @param sched schedule
		 * @param II initiation interval
		 * @param portAssignments info about output of src and input of dst vertices for each edge in g
		 * @param sw ilp solver wishlist (Gurobi, CPLEX, LPSolve or SCIP)
		 */
		OptimalIntegerIIBindingCong(Graph* g, ResourceModel* rm, std::map<Vertex*, int> sched, int II, std::map<Edge*,int> portAssignments, std::list<std::string> sw = {});

		/*!
		 * override binding function from base class
		 * set up ilp solver, add variables and constraints, solve the formulation
		 */
		void bind() override;
		/*!
		 * set b to computed solution; must be called after bind method
		 * @param b binding container to override with computed solution
		 */
		void getBinding(Binding::BindingContainer* b) override;

	private:
		/*!
		 * container with solver wishlist
		 */
		std::list<std::string> sw;
		/*!
		 * container with port assignments
		 * <edge -> <output port of src vertex, input port of dst vertex>>
		 */
		std::map<Edge*, int> portAssignments;
		/*!
		 * container to store computed solution
		 */
		Binding::BindingContainer bin;
	};
}


#endif //HATSCHET_OPTIMALINTEGERIIBINDINGCONG_H
