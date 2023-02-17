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
		this->constraints[{u, v}] = c;
		auto solutionDash = this->solution;
		//this->priorityQueue.clear();
		this->priorityQueue = std::priority_queue<PriorityQueueElement, std::vector<PriorityQueueElement>, std::greater<PriorityQueueElement>>();
		this->insertHeap(u, 0);
		//std::unordered_map<int, int> potentialNewPredecessor;
		std::map<int, int> potentialNewPredecessor;
		potentialNewPredecessor[u] = v;
		while (!this->priorityQueue.empty()) {
			auto xPair = this->findAndDeleteMin();
			auto x = xPair.first;
			auto distX = xPair.second;
			if (x == INT_MIN) {
				// key queue is actually empty and only contained duplicate elements with invalid keys
				break;
			}
			auto newVal = this->getCurrentSDCSolutionValue(this->solution, v) + this->constraints.at({u, v}) + (this->getCurrentSDCSolutionValue(this->solution, x) + distX - this->getCurrentSDCSolutionValue(this->solution, u));
			if (!this->quiet) std::cout << "SDCSolverIncremental::addToFeasible got from heap x=" << x << " with distX=" << distX << " and newVal=" << newVal << " and curVal=" << this->getCurrentSDCSolutionValue(this->solution, x) << std::endl;
			if (newVal < this->getCurrentSDCSolutionValue(this->solution, x)) {
				this->currentPredecessors[x] = potentialNewPredecessor[x];
				if (x == v) {
					// infeasible system -> remove constraint again
					this->constraints.erase({u, v});
					return false;
				}
				else {
					solutionDash[x] = newVal;
					for (auto &y : this->succ[x]) {
						auto scaledPathLength = distX + (this->getCurrentSDCSolutionValue(this->solution, x) + this->constraints.at({y, x}) - this->getCurrentSDCSolutionValue(this->solution, y));
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

	bool SDCSolverIncremental::addConstraint(const int &u, const int &v, const int &c) {
		if (!this->quiet) std::cout << "SDCSolverIncremental::addConstraint: adding constraint t_" << u << " - t_" << v << " <= " << c << std::endl;
		// manage constraint containers
		int currentConstraintValue;
		try {
			currentConstraintValue = this->constraints.at({u, v});
		}
		catch (std::out_of_range&) {
			currentConstraintValue = INT32_MAX;
		}
		if (c < currentConstraintValue) {
			this->constraints[{u, v}] = currentConstraintValue;
		}
		this->rightHandSides[{u, v}].insert(c);
		this->succ[v].insert(u);
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

	bool SDCSolverIncremental::deleteConstraint(const int &u, const int &v, const int &c) {
		// manage constraint containers
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
			return true;
		}
		else {
			this->solutionStatus = -1;
			return false;
		}
	}

	void SDCSolverIncremental::adjustHeap(const int &u, int key) {
		this->insertHeap(u, key); // just put it in again and handle double elements when popping
		/*
		if (!this->isInHeap[u]) {
			this->insertHeap(u, key);
		}
		else {
			this->priorityQueue.decrease_key(this->fibNode.at(u), key);
		}
		 */
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
		do {
			auto minElement = this->priorityQueue.top();
			this->priorityQueue.pop();
			if (!this->isInHeap[minElement.data]) {
				continue;
			}
			this->isInHeap[minElement.data] = false;
			this->keyOf.erase(this->keyOf.find(minElement.data));
			return {minElement.data, minElement.key};
		}
		while (!this->priorityQueue.empty());
		return {INT_MIN, INT_MIN};
	}

	void SDCSolverIncremental::insertHeap(const int &u, int key) {
		//this->fibNode[u] = this->priorityQueue.push(key, (void*)(&u));
		this->priorityQueue.push(PriorityQueueElement(key, u));
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

	//int SDCSolverIncremental::getCurrentSDCSolutionValue(const std::unordered_map<int, int> &solutionContainer, const int &u) {
	int SDCSolverIncremental::getCurrentSDCSolutionValue(const std::map<int, int> &solutionContainer, const int &u) {
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
		for (auto &it : this->baseConstraints) {
			returnMe.emplace_back(it.first.first, it.first.second, it.second);
		}
		for (auto &it : this->additionalConstraints) {
			returnMe.emplace_back(it.first.first, it.first.second, it.second);
		}
		return returnMe;
	}
}