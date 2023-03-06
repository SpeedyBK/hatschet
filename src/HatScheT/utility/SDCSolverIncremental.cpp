//
// Created by nfiege on 1/10/23.
//

#include "SDCSolverIncremental.h"
#include <HatScheT/utility/Exception.h>

namespace HatScheT {

	void SDCSolverIncremental::addBaseConstraint(const int &u, const int &v, const int &c) {
		if (u <= 0 or v <= 0) {
			throw HatScheT::Exception("SDCSolverIncremental: Only positive variable indices are allowed (you provided constraint t_"+std::to_string(u)+" - t_"+std::to_string(v)+" <= "+std::to_string(c)+")");
		}
		try {
			this->baseConstraints.at({u, v}) = std::min(this->baseConstraints.at({u, v}), c);
		}
		catch (std::out_of_range&) {
			this->baseConstraints[{u, v}] = c;
		}
		this->variables.insert(u);
		this->variables.insert(v);
	}

	bool SDCSolverIncremental::addAdditionalConstraint(const int &u, const int &v, const int &c) {
		return this->addConstraint(u, v, c);
	}

	bool SDCSolverIncremental::removeAdditionalConstraint(const int &u, const int &v, const int &c) {
		return this->deleteConstraint(u, v, c);
	}

	bool SDCSolverIncremental::computeInitialSolution() {
		if (!this->quiet) std::cout << "SDCSolverIncremental::computeInitialSolution: start" << std::endl;
		bool systemFeasible = true;
		this->solutionStatus = 1;
		// add src constraints
		if (!this->quiet) std::cout << "SDCSolverIncremental::computeInitialSolution: adding src constraints" << std::endl;
		auto src = -1; // all other IDs are expected to be non-negative
		this->solution[src] = 0;
		for (auto &u : this->variables) {
			this->solution[u] = 0;
			systemFeasible &= this->addConstraint(u, src, 0);
			if (!this->quiet) std::cout << "SDCSolverIncremental::computeInitialSolution: SDC is " << (systemFeasible?"feasible":"infeasible") << std::endl;
			if (this->getSolutionFound() and !this->quiet) {
				std::cout << "SDCSolverIncremental::computeInitialSolution: current solution: " << std::endl;
				for (auto &it : this->solution) {
					if (!this->quiet) std::cout << "    t_" << it.first << " = " << it.second << std::endl;
				}
			}
		}
		// add base constraints
		if (!this->quiet) std::cout << "SDCSolverIncremental::computeInitialSolution: adding base constraints" << std::endl;
		for (auto &it : this->baseConstraints) {
			systemFeasible &= this->addConstraint(it.first.first, it.first.second, it.second);
			if (!this->quiet) std::cout << "SDCSolverIncremental::computeInitialSolution: SDC is " << (systemFeasible?"feasible":"infeasible") << std::endl;
			if (this->getSolutionFound() and !this->quiet) {
				std::cout << "SDCSolverIncremental::computeInitialSolution: current solution: " << std::endl;
				for (auto &solIt : this->solution) {
					std::cout << "    t_" << solIt.first << " = " << solIt.second << std::endl;
				}
			}
		}
		if (!this->quiet) std::cout << "SDCSolverIncremental::computeInitialSolution: end" << std::endl;
		return systemFeasible;
	}

	void SDCSolverIncremental::removeFromUnprocessed(const int &u, const int &v, const int &c) {
		auto it = this->unprocessed.find({u, v, c});
		this->unprocessed.erase(it);
	}

	bool SDCSolverIncremental::addToFeasible(const int &u, const int &v, const int &c) {
		if (!this->quiet) std::cout << "SDCSolverIncremental::addToFeasible t_" << u << " - t_" << v << " <= " << c << std::endl;
		//this->constraints[{u, v}] = c;
		// manage constraint containers
		this->addConstraintToContainers(u, v, c);
		//auto &length = c;
		auto &length = this->constraints.at({u, v});
		if (!this->quiet) std::cout << "SDCSolverIncremental::addToFeasible: active constraint for u=" << u << " and v=" << v << " is 't_" << u << " - t_" << v << " <= " << this->constraints.at({u,v}) << "'" << std::endl;
		auto solutionDash = this->solution;
		//this->priorityQueue.clear();
		this->clearQueue();
		this->insertHeap(u, 0);
		//std::unordered_map<int, int> potentialNewPredecessor;
		std::map<int, int> potentialNewPredecessor;
		std::map<int, int> idxDone;
		potentialNewPredecessor[u] = v;
		while (!this->priorityQueue.empty()) {
			auto xPair = this->findAndDeleteMin();
			auto x = xPair.first;
			auto distX = xPair.second;
			idxDone[x]++;
#if BOOST_QUEUE
#else
			if (x == INT_MIN) {
				// key queue is actually empty and only contained duplicate elements with invalid keys
				break;
			}
#endif
			//auto newVal = this->getCurrentSDCSolutionValue(this->solution, v) + this->constraints.at({u, v}) + (this->getCurrentSDCSolutionValue(this->solution, x) + distX - this->getCurrentSDCSolutionValue(this->solution, u));
			auto newVal = this->getCurrentSDCSolutionValue(this->solution, v) + length + (this->getCurrentSDCSolutionValue(this->solution, x) + distX - this->getCurrentSDCSolutionValue(this->solution, u));
			if (!this->quiet) std::cout << "SDCSolverIncremental::addToFeasible got from heap x=" << x << " with distX=" << distX << " and length=" << length << " and newVal=" << newVal << " and curVal=" << this->getCurrentSDCSolutionValue(this->solution, x) << std::endl;
			if (newVal < this->getCurrentSDCSolutionValue(this->solution, x)) {
				this->currentPredecessors[x] = potentialNewPredecessor[x];
				if (x == v) {
					// infeasible system -> remove constraint again
					//this->constraints.erase({u, v});
					this->removeConstraintFromContainers(u, v, c);
					return false;
				}
				else {
					solutionDash[x] = newVal;
					for (auto &y : this->succ[x]) {
						auto scaledPathLength = distX + (this->getCurrentSDCSolutionValue(this->solution, x) + this->constraints.at({y, x}) - this->getCurrentSDCSolutionValue(this->solution, y));
						if (!this->quiet) std::cout << "SDCSolverIncremental::addToFeasible: y=" << y << " is successor of x=" << x << " with key " << this->getKeyOf(y) << " and constraint=" << this->constraints.at({y, x}) << std::endl;
						if (idxDone[y] >= 1) {
							if (!this->quiet) std::cout << "SDCSolverIncremental::addToFeasible: already adjusted y=" << y << std::endl;
							continue; // already adjusted this idx
						}
						if (scaledPathLength < this->getKeyOf(y)) {
							if (!this->quiet) std::cout << "SDCSolverIncremental::addToFeasible: adjust heap y=" << y << " with scaledPathLength=" << scaledPathLength << std::endl;
							this->adjustHeap(y, scaledPathLength);
							potentialNewPredecessor[y] = x;
						}
					}
				}
			}
		}
		// feasible system -> update solution
		this->solution = solutionDash;
		return true;
	}

	void SDCSolverIncremental::addConstraintToContainers(const int &u, const int &v, const int &c) {
		int currentConstraintValue;
		try {
			currentConstraintValue = this->constraints.at({u, v});
		}
		catch (std::out_of_range&) {
			currentConstraintValue = INT32_MAX;
		}
		if (c < currentConstraintValue) {
			currentConstraintValue = c;
		}
		this->constraints[{u, v}] = currentConstraintValue;
		this->rightHandSides[{u, v}].insert(c);
		this->succ[v].insert(u);
		this->constraintInContainers[{u, v, c}]++;
	}

	bool SDCSolverIncremental::addConstraint(const int &u, const int &v, const int &c) {
		if (!this->quiet) std::cout << "SDCSolverIncremental::addConstraint: adding constraint t_" << u << " - t_" << v << " <= " << c << std::endl;
		// manage constraint containers
		//this->addConstraintToContainers(u, v, c); // -> already happens in addToFeasible
		// actually perform the constraint insertion algorithm
		if (this->unprocessed.empty()) {
			// system is feasible before addition of new constraint
			if (!this->addToFeasible(u, v, c)) {
				// adding constraint causes infeasibility
				this->unprocessed.insert({u, v, c});
				this->solutionStatus = -1;
				this->conflictConstraints = {{u, v, c}};
				auto pred = v;
				auto predNew = this->currentPredecessors.at(v);
				//std::cout << "#q# infeasibility due to constraint t_u - t_v <= c: t_" << u << " - t_" << v << " <= " << c << std::endl;
				//std::cout << "#q# pred(u) = pred(" << u << ") = " << this->currentPredecessors.at(u) << std::endl;
				//std::cout << "#q# pred(v) = pred(" << v << ") = " << this->currentPredecessors.at(v) << std::endl;
				while (pred != u) {
					//std::cout << "#q# pred = " << pred << " and predNew = " << predNew << std::endl;
					auto rhs = this->constraints.at({pred, predNew});
					this->conflictConstraints.emplace_back(pred, predNew, rhs);
					pred = predNew;
					predNew = this->currentPredecessors.at(predNew);
				}
				return false;
			}
			else {
				// adding constraint does not cause infeasibility
				// verify current solution
				if (!this->currentSolutionValid()) {
					throw Exception("SDCSolverIncremental::addConstraint: computed invalid solution after adding constraint -> that should never happen!");
				}
				return true;
			}
		}
		else {
			// system is already infeasible before addition of new constraint
			// just add constraint to unprocessed constraint list
			this->unprocessed.insert({u, v, c});
			return false;
		}
	}

	void SDCSolverIncremental::removeConstraintFromContainers(const int &u, const int &v, const int &c) {
		if (this->constraintInContainers[{u, v, c}] == 0) return; // constraint was already removed
		this->constraintInContainers[{u, v, c}]--;
		auto *rhsSet = &this->rightHandSides.at({u, v});
		auto it = rhsSet->find(c);
		rhsSet->erase(it);
		auto newRhsIt = rhsSet->begin();
		if (newRhsIt == rhsSet->end()) {
			// there is no constraint between u and v left
			auto constrIt = this->constraints.find({u, v});
			if (constrIt != this->constraints.end()) {
				this->constraints.erase(constrIt);
			}
			// hence, u is not v's successor anymore
			auto *succSet = &this->succ[v];
			succSet->erase(succSet->find(u));
		}
		else {
			// update constraint to new minimum value
			this->constraints[{u, v}] = *newRhsIt;
		}
	}

	bool SDCSolverIncremental::deleteConstraint(const int &u, const int &v, const int &c) {
		if (!this->quiet) {
			std::cout << "SDCSolverIncremental::deleteConstraint: start removing constraint 't_" << u << " - t_" << v << " <= " << c << "'" << std::endl;
		}
		// manage constraint containers
		this->removeConstraintFromContainers(u, v, c);
		// actually perform the algorithm
		if (this->unprocessed.find({u, v, c}) != this->unprocessed.end()) {
			this->removeFromUnprocessed(u, v, c);
		}
		else {
			//this->constraints.erase(this->constraints.find({u, v})); // was already erased before
			while (!this->unprocessed.empty()) {
				auto &constraint = *this->unprocessed.begin();
				auto uConstr = std::get<0>(constraint);
				auto vConstr = std::get<1>(constraint);
				auto cConstr = std::get<2>(constraint);
				if (this->addToFeasible(uConstr, vConstr, cConstr)) {
					this->removeFromUnprocessed(uConstr, vConstr, cConstr);
				}
				else {
					break;
				}
			}
		}
		if (this->unprocessed.empty()) {
			this->conflictConstraints.clear();
			this->solutionStatus = 1;
			if (!this->quiet) {
				std::cout << "SDCSolverIncremental::deleteConstraint: finished removing constraint 't_" << u << " - t_" << v << " <= " << c << "'; solution is valid now" << std::endl;
			}
			return true;
		}
		else {
			this->solutionStatus = -1;
			if (!this->quiet) {
				std::cout << "SDCSolverIncremental::deleteConstraint: finished removing constraint 't_" << u << " - t_" << v << " <= " << c << "'; solution remains invalid" << std::endl;
			}
			return false;
		}
	}

	void SDCSolverIncremental::adjustHeap(const int &u, int key) {
#if BOOST_QUEUE
		if (!this->isInHeap[u]) {
			this->insertHeap(u, key);
		}
		else {
			this->priorityQueue.decrease(this->fibNodeHandle.at(u), PriorityQueueElement(key, u));
			this->keyOf[u] = key;
		}
#else
		this->insertHeap(u, key); // just put it in again and handle double elements when popping
#endif
	}

	std::pair<int, int> SDCSolverIncremental::findAndDeleteMin() {
		/*
		std::cout << "#q# 1" << std::endl;
		auto curFibNode = this->priorityQueue.extract_min();
		std::cout << "#q# 2" << std::endl;
		auto key = curFibNode->key;
		std::cout << "#q# 3" << std::endl;
		auto x = *((int*)curFibNode->payload);
		std::cout << "#q# 4" << std::endl;
		this->keyOf.erase(this->keyOf.find(x));
		std::cout << "#q# 5" << std::endl;
		this->isInHeap[x] = false;
		std::cout << "#q# 6" << std::endl;
		return std::make_pair(x, key);
		 */
#if BOOST_QUEUE
#else
		do {
#endif
			auto minElement = this->priorityQueue.top();
			this->priorityQueue.pop();
#if BOOST_QUEUE
			this->isInHeap[minElement.data] = false;
#else
			if (!this->isInHeap[minElement.data]) {
				continue;
			}
			this->isInHeap[minElement.data] = false;
#endif
			this->keyOf.erase(this->keyOf.find(minElement.data));
			return {minElement.data, minElement.key};
#if BOOST_QUEUE
#else
		}
		while (!this->priorityQueue.empty());
		return {INT_MIN, INT_MIN};
#endif
	}

	void SDCSolverIncremental::insertHeap(const int &u, int key) {
#if BOOST_QUEUE
		this->fibNodeHandle[u] = this->priorityQueue.push(PriorityQueueElement(key, u));
#else
		//this->fibNode[u] = this->priorityQueue.push(key, (void*)(&u));
		this->priorityQueue.push(PriorityQueueElement(key, u));
#endif
		this->keyOf[u] = key;
		this->isInHeap[u] = true;
	}

	int SDCSolverIncremental::getKeyOf(int u) {
		try {
			return this->keyOf.at(u);
		}
		catch (std::out_of_range&) {
			return INT_MAX;
		}
	}

	int SDCSolverIncremental::getCurrentSDCSolutionValue(const std::unordered_map<int, int> &solutionContainer, const int &u) {
	//int SDCSolverIncremental::getCurrentSDCSolutionValue(const std::map<int, int> &solutionContainer, const int &u) {
		try {
			return solutionContainer.at(u);
		}
		catch (std::out_of_range&) {
			return 0;
		}
	}

	std::map<int, int> SDCSolverIncremental::getSolution() const {
		std::map<int, int> returnMe;
		for (auto &it : this->solution) {
			if (it.first >= 0) returnMe[it.first] = it.second;
		}
		return returnMe;
	}

	std::list<std::tuple<int, int, int>> SDCSolverIncremental::getCurrentConstraints() const {
		std::list<std::tuple<int, int, int>> returnMe;
		/*
		for (auto &it : this->baseConstraints) {
			returnMe.emplace_back(it.first.first, it.first.second, it.second);
		}
		for (auto &it : this->additionalConstraints) {
			returnMe.emplace_back(it.first.first, it.first.second, it.second);
		}
		 */
		for (auto &it : this->constraints) {
			returnMe.emplace_back(it.first.first, it.first.second, it.second);
		}
		return returnMe;
	}

	void SDCSolverIncremental::overrideSolution(const std::map<int, int> &newSolution) {
		int maxVal = INT32_MIN;
		for (auto &it : newSolution) {
			maxVal = std::max(maxVal, it.second);
			this->solution[it.first] = it.second;
		}
		this->solution[-1] = maxVal;
	}

	bool SDCSolverIncremental::currentSolutionValid() {
		if (!this->unprocessed.empty()) return true; // cannot verify invalid solution
		bool allGood = true;
		for (auto &it : this->constraints) {
			auto u = it.first.first;
			auto v = it.first.second;
			auto c = it.second;
			auto t_u = this->solution.at(u);
			auto t_v = this->solution.at(v);
			if (t_u - t_v > c) {
				if (!this->quiet) {
					std::cout << "SDCSolverIncremental::currentSolutionValid: constraint 't_" << u << " - t_" << v << " <= " << c << "' violated!" << std::endl;
					std::cout << "  -> t_" << u << " = " << t_u << std::endl;
					std::cout << "  -> t_" << v << " = " << t_v << std::endl;
					std::cout << "  -> c = " << c << std::endl;
				}
				allGood = false;
			}
		}
		return allGood;
	}

	void SDCSolverIncremental::clearQueue() {
#if BOOST_QUEUE
		this->priorityQueue = queue_t();
		this->fibNodeHandle.clear();
#else
		this->priorityQueue = std::priority_queue<PriorityQueueElement, std::vector<PriorityQueueElement>, std::greater<PriorityQueueElement>>();
#endif
		this->isInHeap.clear();
		this->keyOf.clear();

	}
}