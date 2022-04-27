//
// Created by nfiege on 3/4/22.
//

#ifndef HATSCHET_OPTIMALINTEGERIIGENERALIZEDBINDING_H
#define HATSCHET_OPTIMALINTEGERIIGENERALIZEDBINDING_H

#include <HatScheT/utility/BindingBase.h>

namespace HatScheT {
	class OptimalIntegerIIGeneralizedBinding : public BindingBase {
	public:
		/*!
		 * constructor for optimal generalized binding
		 * @param g graph
		 * @param rm resource model
		 * @param sched schedule
		 * @param II initiation interval
		 * @param portAssignments info about output of src and input of dst vertices for each edge in g
		 * @param commutativeOps set with commutative operations
		 * @param sw ilp solver wishlist (Gurobi, CPLEX, LPSolve or SCIP)
		 */
		OptimalIntegerIIGeneralizedBinding(Graph* g, ResourceModel* rm, std::map<Vertex*, int> sched, int II, std::map<Edge*,std::pair<int,int>> portAssignments, std::set<const Resource*> commutativeOps = {}, std::list<std::string> sw = {});

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
		/*!
		 * set this->allocMinRegs to a
		 * @param a new value for this->allocMinRegs
		 */
		void setAllocMinRegs(bool a);
		/*!
		 * set this->allowMultipleBindings to m
		 * @param m a new value for this->allowMultipleBindings
		 */
		void setAllowMultipleBindings(bool m);

	private:
		/*!
		 * if this is set, operations/variables are bound to AT LEAST one operator/register instead of EXACTLY one
		 */
		bool allowMultipleBindings;
		/*!
		 * container with solver wishlist
		 */
		std::list<std::string> sw;
		/*!
		 * container with port assignments
		 * <edge -> <output port of src vertex, input port of dst vertex>>
		 */
		std::map<Edge*, std::pair<int, int>> portAssignments;
		/*!
		 * container to store computed solution
		 */
		Binding::BindingContainer bin;
		/*!
		 * if this is set, the binding algorithm has to use the minimal amount of registers possible
		 */
		bool allocMinRegs;
		/*!
		 * total number of registers in hw implementation
		 */
		int numRegs;
		/*!
		 * total number of FUs in hw implementation
		 */
		int numFUs;
		/*!
		 * container to compute combined FU/Reg index from resource type and resource index
		 */
		std::map<std::pair<Resource*, int>, int> iFU;
		/*!
		 * container to compute combined FU/Reg index from register index
		 */
		std::map<int, int> iReg;
		/*!
		 * container to store binding variables
		 * tuple consists of:
		 *   (1) vertex index
		 *   (2) output port of vertex
		 *   (3) FU/Reg index
		 *   (4) time step
	   * if one of these variables is 1, then
	   *   the variable produced by vertex (1)
	   *   on port (2)
	   *   is present at the output of FU/Reg (3)
	   *   in time step (4)
		 */
		std::map<std::tuple<int, int, int, int>, ScaLP::Variable> bVars;
		/*!
		 * container to store connection variables
		 * tuple consists of
		 *   (1) src FU/Reg index
		 *   (2) src output port
		 *   (3) dst FU/Reg index
		 *   (4) dst input port
	   * if one of these variables is 1, then there is a connection
	   *   from FU/Reg (1)
	   *   output port (2)
	   *   to FU/Reg (3)
	   *   input port (4)
		 */
		std::map<std::tuple<int, int, int, int>, ScaLP::Variable> cVars;
		/*!
		 * container to store register variables
		 * key: reg index
		 * if one of these variables is 1 then the register is needed for hw implementation
		 * because at least one variable is stored there in at least 1 time step
		 * this is needed if a non-minimal number of registers is allocated
		 * then, the binding algorithm might allocate less than the given number of registers
		 */
		std::map<int, ScaLP::Variable> rVars;
	};
}

#endif //HATSCHET_OPTIMALINTEGERIIGENERALIZEDBINDING_H
