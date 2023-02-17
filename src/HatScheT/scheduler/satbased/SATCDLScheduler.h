//
// Created by nfiege on 1/26/23.
//

#ifndef HATSCHET_SATCDLSCHEDULER_H
#define HATSCHET_SATCDLSCHEDULER_H

#include <HatScheT/base/SATSchedulerBase.h>

#ifdef USE_CADICAL
namespace HatScheT {
	class SATCDLScheduler : public SATSchedulerBase {
	public:
		SATCDLScheduler(Graph& g, ResourceModel& resourceModel, int II=-1);
		void scheduleIteration() override;

	private:
		void initSATSolver();
		std::map<const Vertex*, int> computeModuloSlots();
		std::set<std::pair<const Vertex*, int>> solveSDC(std::map<const Vertex*, int> &moduloSlots);
		void addConflicts(std::set<std::pair<const Vertex*, int>>& conflicts);
		void createExactly1Constraint(std::vector<int>& vars);
		void createAtMostConstraint(std::vector<int>& vars, int rhs);
		std::map<std::pair<const Vertex*, int>, int> mrtVariables;

		/*!
		 * create a bitheap in SAT
		 * @param x vector of input bitvectors
		 * @return the result
		 */
		std::vector<int> create_bitheap(const std::vector<std::vector<int>> &x);
		/*!
		 * clauses for a half adder (i.e., 2:2 compressor, a full adder with c_i=0)
		 * @param a < variable, whether the bit should be negated at the adder input >
		 * @param b < variable, whether the bit should be negated at the adder input >
		 * @param sum < variable, whether the bit should be negated at the adder output >
		 * @param c_o < variable, whether the bit should be negated at the adder output >
		 */
		void create_half_adder(std::pair<int, bool> a, std::pair<int, bool> b, std::pair<int, bool> sum, std::pair<int, bool> c_o={-1, false});
		/*!
		 * clauses for a full adder (i.e., 3:2 compressor)
		 * @param a < variable, whether the bit should be negated at the adder input >
		 * @param b < variable, whether the bit should be negated at the adder input >
		 * @param c_i < variable, whether the bit should be negated at the adder input >
		 * @param sum < variable, whether the bit should be negated at the adder output >
		 * @param c_o < variable, whether the bit should be negated at the adder output >
		 */
		void create_full_adder(std::pair<int, bool> a, std::pair<int, bool> b, std::pair<int, bool> c_i, std::pair<int, bool> sum, std::pair<int, bool> c_o={-1, false});
		/*!
		 * helper function to create an arbitrary clause:
		 * @param a < variable idx, negate >
		 *   -> negate the variable if negate == true
		 */
		void create_arbitrary_clause(const std::vector<std::pair<int, bool>> &a);
		/*!
		 * time tracker for SAT solver
		 */
		double SATSolverTime;
		/*!
		 * time tracker for SDC solver
		 */
		double SDCSolverTime;
	};
}
#endif

#endif //HATSCHET_SATCDLSCHEDULER_H
