//
// Created by nfiege on 1/25/23.
//

#ifndef HATSCHET_SDCSOLVERBELLMANFORD_H
#define HATSCHET_SDCSOLVERBELLMANFORD_H

#include <set>
#include <map>
#include <vector>
#include <list>
#include <utility>

namespace HatScheT {
	class SDCSolverBellmanFord {
	public:
		SDCSolverBellmanFord() = default;
		/*!
		 * set solver quiet
		 * @param q
		 */
		void setQuiet(bool q) { this->quiet = q; }
		/*!
		 * add base constraint t_u - t_v <= c
		 * @param u
		 * @param v
		 * @param c
		 */
		void addBaseConstraint(int u, int v, int c);
		/*!
		 * add additional constraint t_u - t_v <= c
		 * @param u
		 * @param v
		 * @param c
		 */
		void addAdditionalConstraint(int u, int v, int c);
		/*!
		 * clear all additional constraints
		 */
		void clearAdditionalConstraints();
		/*!
		 * solve sdc
		 */
		void solve();
		/*!
		 * @return whether the SDC was solved
		 */
		bool getSolutionFound() const { return not this->solution.empty(); }
		/*!
		 * @return the solution container
		 */
		std::map<int, int> getSolution() const { return this->solution; }
		/*!
		 * @return the solution but normalized such that the smallest value is zero
		 */
		std::map<int, int> getNormalizedSolution() const;
		/*!
		 * @return all additional conflict constraints
		 */
		std::list<std::tuple<int, int, int>> getAdditionalConflicts() const { return this->additionalConflicts; }
		/*!
		 * @return all base conflict constraints
		 */
		std::list<std::tuple<int, int, int>> getBaseConflicts() const { return this->baseConflicts; }
		/*!
		 * @return all base and additional constraints
		 */
		std::list<std::tuple<int, int, int>> getCurrentConstraints() const;
	private:
		/*!
		 * suppress status messages if set
		 */
		bool quiet = true;
		/*!
		 * whether the start node was created -> this happens in the very first call to this->solve()
		 */
		bool startNodeCreated = false;
		/*!
		 * set of variable indices
		 */
		std::set<int> variables;
		/*!
		 * base SDC constraints
		 */
		std::map<std::pair<int, int>, int> baseConstraints;
		/*!
		 * additional SDC constraints
		 */
		std::map<std::pair<int, int>, int> additionalConstraints;
		/*!
		 * solution container
		 */
		std::map<int, int> solution;
		/*!
		 * conflict base constraint container due to a loop with negative distance in the SDC graph
		 * it consists of all base conflicts that are part of this loop
		 */
		std::list<std::tuple<int, int, int>> baseConflicts;
		/*!
		 * conflict additional constraint container due to a loop with negative distance in the SDC graph
		 * it consists of all additional conflicts that are part of this loop
		 */
		std::list<std::tuple<int, int, int>> additionalConflicts;
	};
}

#endif //HATSCHET_SDCSOLVERBELLMANFORD_H
