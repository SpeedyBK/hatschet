//
// Created by nfiege on 1/10/23.
//

#ifndef HATSCHET_SDCSOLVERINCREMENTAL_H
#define HATSCHET_SDCSOLVERINCREMENTAL_H

#include <unordered_map>
#include <bits/stdc++.h>
#include <set>
#include <unordered_set>
#include <utility>
#include <queue>
#include <HatScheT/utility/FibonacciHeap.h>
//#include <boost/heap/fibonacci_heap.hpp>

namespace HatScheT {
#define BOOST_QUEUE 0

	class PriorityQueueElement {
	public:
		bool operator < (PriorityQueueElement const & obj) const {
			return this->key < obj.key;
		}
		bool operator > (PriorityQueueElement const & obj) const {
			return this->key > obj.key;
		}
		PriorityQueueElement(int key, int data) : key(key), data(data) {}
		int key;
		int data;
	};

	typedef struct PriorityQueueElementCompare {
		bool operator()(const PriorityQueueElement &p1,
										const PriorityQueueElement &p2) const {
			return p1.key > p2.key;
		}
	} PriorityQueueElementCompare;

	/*!
	 * hash function for a pair of hashable types
	 */
	class hashPair {
	public:
		template <class T1, class T2> size_t operator() (const std::pair<T1, T2>& p) const {
			auto hash1 = std::hash<T1>{}(p.first);
			auto hash2 = std::hash<T2>{}(p.second);
			if (hash1 != hash2) {
				return hash1 ^ hash2;
			}
			else {
				return hash1;
			}
		}
	};

	/*!
	 * hash function for a tuple of 3 hashable types
	 */
	class hashTuple3 {
	public:
		template <class T1, class T2, class T3> size_t operator() (const std::tuple<T1, T2, T3>& t) const {
			auto hash1 = std::hash<T1>{}(std::get<0>(t));
			auto hash2 = std::hash<T2>{}(std::get<1>(t));
			auto hash3 = std::hash<T3>{}(std::get<2>(t));
			auto curHash = hash1;
			if (curHash != hash2) {
				curHash = curHash ^ hash2;
			}
			if (curHash != hash3) {
				curHash = curHash ^ hash3;
			}
			return curHash;
		}
	};

	class SDCSolverIncremental {
	public:
		/*!
		 * constructor
		 */
		SDCSolverIncremental() = default;
		/*!
		 * set solver quiet
		 * @param q
		 */
		void setQuiet(bool q) {this->quiet = q;}
		/*!
		 * override current solution with a new one
		 * -> should only be used for debugging purposes!
		 * @param newSolution has to be valid!!!!
		 */
		void overrideSolution(const std::map<int,int> &newSolution);
		/*!
		 * add base constraint t_u - t_v <= c
		 * -> a base constraint cannot be removed anymore after it has been added!
		 * @param u
		 * @param v
		 * @param c
		 */
		void addBaseConstraint(const int &u, const int &v, const int &c);
		/*!
		 * add additional constraint t_u - t_v <= c
		 * -> an additional constraint can be removed after it has been added!
		 * @param u
		 * @param v
		 * @param c
		 * @return whether there is a solution after adding the constraint
		 */
		bool addAdditionalConstraint(const int &u, const int &v, const int &c);
		/*!
		 * remove additional constraint t_u - t_v <= c
		 * -> an additional constraint only be removed after it has been added!
		 * @param u
		 * @param v
		 * @param c
		 * @return whether there is a solution after removing the constraint
		 */
		bool removeAdditionalConstraint(const int &u, const int &v, const int &c);
		/*!
		 * compute the initial solution for the SDC based on only the base constraints
		 * @return whether there is a solution
		 */
		bool computeInitialSolution();
		/*!
		 * @return whether the SDC is solved
		 */
		bool getSolutionFound() const { return this->solutionStatus == 1; }
		/*!
		 * @return the solution to the SDC
		 */
		std::map<int, int> getSolution() const;
		/*!
		 * @return the solver's current predecessor state
		 */
		std::unordered_map<int, int> getCurrentPredecessorState() const { return this->currentPredecessors; }
		//std::map<int, int> getCurrentPredecessorState() const { return this->currentPredecessors; }
		/*!
		 * @return the solver's current conflicts
		 */
		std::list<std::tuple<int, int, int>> getConflicts() const { return this->conflictConstraints; }
		/*!
		 * @return the solver's current constraints in the form t_u - t_v <= c
		 * tuple<0> u
		 * tuple<1> v
		 * tuple<2> c
		 */
		std::list<std::tuple<int, int, int>> getCurrentConstraints() const;

	private:
		//std::map<std::tuple<int, int, int>, int> constraintInContainers;
		std::unordered_map<std::tuple<int, int, int>, int, hashTuple3> constraintInContainers;
		void addConstraintToContainers(const int& u, const int& v, const int& c);
		void removeConstraintFromContainers(const int& u, const int& v, const int& c);
		void clearQueue();
		/*!
		 * verifier for debugging
		 * @return whether the current solution is valid (if it should be)
		 */
		bool currentSolutionValid();
		/*!
		 * suppress debug outputs if quiet = true
		 */
		bool quiet = true;
		/*!
		 * a set of constraints that leads to a conflict in the SDC
		 */
		std::list<std::tuple<int, int, int>> conflictConstraints;
		/*!
		 * map < SDC node v -> v's successors >
		 * i.e., all u for which there is a constraint u - v <= c
		 */
		std::unordered_map<int, std::unordered_set<int>> succ;
		//std::map<int, std::set<int>> succ;
		/*!
		 * map < SDC node v -> its predecessor in the current solver state >
		 */
		std::unordered_map<int, int> currentPredecessors;
		//std::map<int, int> currentPredecessors;
		/*!
		 * accesses this->solution to get a value for u
		 * @param u
		 * @return u's value if it exists or 0 if it doesn't
		 */
		int getCurrentSDCSolutionValue(const std::unordered_map<int, int> &solutionContainer, const int &u);
		//int getCurrentSDCSolutionValue(const std::map<int, int> &solutionContainer, const int &u);
		/*!
		 * map < SDC node -> whether it is in the current heap >
		 */
		std::unordered_map<int, bool> isInHeap;
		//std::map<int, bool> isInHeap;
		/*!
		 * implements the AdjustHeap function from the paper
		 * @param u
		 * @param key
		 */
		void adjustHeap(const int &u, int key);
		/*!
		 * implements the FindAndDeleteMin function from the paper
		 * @return the next element in the priority queue and delete it from the queue
		 */
		std::pair<int, int> findAndDeleteMin();
		/*!
		 * implements the InsertHeap function from the paper
		 * @param u SDC node
		 * @param key its key
		 */
		void insertHeap(const int &u, int key);
		/*!
		 * get key of node u
		 * @param u
		 * @return oo if node u is not in the heap
		 */
		int getKeyOf(int u);
		/*!
		 * store the priority queue key of each node in the SDC
		 * map < node idx -> key queue key >
		 */
		std::unordered_map<int, int> keyOf;
		//std::map<int, int> keyOf;
		/*!
		 * removes constraint t_u - t_v <= c from unprocessed container
		 * @param u
		 * @param v
		 * @param c
		 */
		void removeFromUnprocessed(const int &u, const int &v, const int &c);
		/*!
		 * container UnProcessed from paper (is managed in addConstraint and deleteConstraint)
		 * the tuple represents the constraint t_u - t_v <= c
		 *   tuple<0>: u
		 *   tuple<1>: v
		 *   tuple<2>: c
		 */
		std::unordered_set<std::tuple<int, int, int>, hashTuple3> unprocessed;
		//std::set<std::tuple<int, int, int>> unprocessed;
		/*!
		 * function AddToFeasible from paper (Fig. 1)
		 * @param u
		 * @param v
		 * @param c
		 * @return whether the SDC is feasible after adding constraint t_u - t_v <= c
		 */
		bool addToFeasible(const int &u, const int &v, const int &c);
		/*!
		 * function AddConstraint from paper (Fig. 3)
		 * @param u
		 * @param v
		 * @param c
		 * @return whether the SDC is feasible after adding constraint t_u - t_v <= c
		 */
		bool addConstraint(const int &u, const int &v, const int &c);
		/*!
		 * function DeleteConstraint from paper (Fig. 4)
		 * @param u
		 * @param v
		 * @param c
		 * @return whether the SDC is feasible after removing constraint t_u - t_v <= c
		 */
		bool deleteConstraint(const int &u, const int &v, const int &c);
		/*!
		 * a fibonacci heap to implement the PriorityQueue from the paper
		 * the heap's key is a simple integer
		 */
#if BOOST_QUEUE
		typedef boost::heap::fibonacci_heap<PriorityQueueElement, boost::heap::compare<PriorityQueueElementCompare>> queue_t;
		typedef typename queue_t::handle_type handle_t;
		queue_t priorityQueue;
		/*!
		 * map < SDC node -> its Fibonacci node in the heap >
		 */
		//std::unordered_map<int, FibonacciHeap<int>::FibNode*> fibNode;
		//std::map<int, handle_t> fibNodeHandle;
		std::unordered_map<int, handle_t> fibNodeHandle;
		//std::map<int, PriorityQueueElement> fibNode;
#else
		std::priority_queue<PriorityQueueElement, std::vector<PriorityQueueElement>, std::greater<PriorityQueueElement>> priorityQueue;
#endif
		//FibonacciHeap<int> priorityQueue;
		//std::priority_queue<std::pair<int, int>, std::vector<std::pair<int,int>>, SDCSolverIncrementalPriorityQueueComp> priorityQueue;
		/*!
		 * container to store all variables
		 */
		std::unordered_set<int> variables;
		//std::set<int> variables;
		/*!
		 * current solution status
		 *   0 : unsolved
		 *   1 : solved and satisfiable
		 *  -1 : solved and unsatisfiable
		 */
		int solutionStatus = 0;
		/*!
		 * this container holds current constraints in the form "t_u - t_v <= c"
		 * resulting from base constraints and additional constraints
		 * map< <u, v> -> c>
		 */
		std::unordered_map<std::pair<int, int>, int, hashPair> constraints;
		//std::map<std::pair<int, int>, int> constraints;
		/*!
		 * this container holds current constraints in the form "t_u - t_v <= c"
		 * resulting from base constraints and additional constraints
		 * map< <u, v> -> all values for c in ascending order >
		 */
		std::unordered_map<std::pair<int, int>, std::multiset<int>, hashPair> rightHandSides;
		//std::map<std::pair<int, int>, std::multiset<int>> rightHandSides;
		/*!
		 * this container holds only base constraints in the form "t_u - t_v <= c"
		 * map< <u, v> -> c>
		 */
		std::unordered_map<std::pair<int, int>, int, hashPair> baseConstraints;
		//std::map<std::pair<int, int>, int> baseConstraints;
		/*!
		 * this container holds only additional constraints in the form "t_u - t_v <= c"
		 * map< <u, v> -> c>
		 */
		std::unordered_map<std::pair<int, int>, int, hashPair> additionalConstraints;
		//std::map<std::pair<int, int>, int> additionalConstraints;
		/*!
		 * integer for all variables that satisfy the constraints
		 */
		std::unordered_map<int, int> solution;
		//std::map<int, int> solution;
	};
}

#endif //HATSCHET_SDCSOLVERINCREMENTAL_H
