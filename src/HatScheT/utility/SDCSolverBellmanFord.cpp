//
// Created by nfiege on 1/25/23.
//

#include "SDCSolverBellmanFord.h"
#include <HatScheT/utility/Exception.h>
#include <limits>
#include <vector>
#include <iostream>
#include <algorithm>

namespace HatScheT {
	void SDCSolverBellmanFord::addBaseConstraint(int u, int v, int c) {
		if (u < 0 or v < 0) {
			throw HatScheT::Exception("SDCSolverBellmanFord: Only non-negative variable indices supported");
		}
		this->variables.insert(u);
		this->variables.insert(v);
		try {
			this->baseConstraints.at({u, v}) = std::min(c, this->baseConstraints.at({u, v}));
		}
		catch (std::out_of_range&) {
			this->baseConstraints[{u, v}] = c;
		}
	}

	void SDCSolverBellmanFord::addAdditionalConstraint(int u, int v, int c) {
		if (u < 0 or v < 0) {
			throw HatScheT::Exception("SDCSolverBellmanFord: Only non-negative variable indices supported");
		}
		this->variables.insert(u);
		this->variables.insert(v);
		try {
			this->additionalConstraints.at({u, v}) = std::min(c, this->additionalConstraints.at({u, v}));
		}
		catch (std::out_of_range&) {
			this->additionalConstraints[{u, v}] = c;
		}
	}

	void SDCSolverBellmanFord::clearAdditionalConstraints() {
		this->additionalConstraints.clear();
	}

	void SDCSolverBellmanFord::solve() {
		// clear some containers
		this->additionalConflicts.clear();
		this->baseConflicts.clear();
		this->solution.clear();
		if (!this->startNodeCreated) {
			for (auto &i : this->variables) {
				this->baseConstraints[{i, -1}] = 0;
			}
			this->startNodeCreated = true;
			//this->variables.insert(-1);
		}
		// init distance container
		std::map<int, double> distance;
		for (auto &i : this->variables) {
			distance[i] = std::numeric_limits<double>::infinity();
		}
		distance[-1] = 0;
		// init predecessor container
		std::map<int, int> predecessors;
		// main loop
		std::vector<std::map<std::pair<int, int>, int>*> constraints = {&this->baseConstraints, &this->additionalConstraints};
		for (int n=0; n<this->variables.size(); n++) {
			bool terminate = true;
			// process constraints
			for (auto &constraintContainerPtr : constraints) {
				for (auto &it : *constraintContainerPtr) {
					auto &u = it.first.first;
					auto &v = it.first.second;
					auto &c = it.second;
					auto &distU = distance.at(u);
					auto &distV = distance.at(v);
					auto sum = distV + c;
					if (sum < distU) {
						distU = sum;
						predecessors[u] = v;
						terminate = false;
					}
				}
			}
			if (terminate) {
				if (!this->quiet) {
					if (n == this->variables.size()-1) {
						std::cout << "Terminated normally after " << this->variables.size()-1 << " iterations" << std::endl;
					}
					else {
						std::cout << "Terminated early after " << n+1 << "/" << this->variables.size() << " iterations" << std::endl;
					}
				}
				break;
			}
		}
		// check conflict
		bool conflict = false;
		for (auto &constraintContainerPtr : constraints) {
			for (auto &it : *constraintContainerPtr) {
				auto &u = it.first.first;
				auto &v = it.first.second;
				auto &c = it.second;
				auto &distU = distance.at(u);
				auto &distV = distance.at(v);
				if (distV + c >= distU) continue;
				conflict = true;
				if (!this->quiet) std::cout << "Found conflict constraint t_" << u << " - t_" << v << " <= " << c << "! :(" << std::endl;
				auto conflictNode = u;
				auto predecessor = v;
				std::vector<int> visited = {u};
				while (true) { // break manually
					// update conflict node and predecessor
					conflictNode = predecessor;
					predecessor = predecessors.at(conflictNode);
					auto visitedIt = std::find(visited.begin(), visited.end(), conflictNode);
					if (visitedIt != visited.end()) {
						while (visitedIt != visited.end()) {
							auto n = *visitedIt;
							auto p = predecessors.at(n);
							auto baseValue = std::numeric_limits<double>::infinity();
							auto additionalValue = std::numeric_limits<double>::infinity();
							try {
								baseValue = this->baseConstraints.at({n, p});
							}
							catch (std::out_of_range&) {
								// the constraint was not a base constraint...
							}
							try {
								additionalValue = this->additionalConstraints.at({n, p});
							}
							catch (std::out_of_range&) {
								// the constraint was not an additional constraint...
							}
							if (baseValue == std::numeric_limits<double>::infinity() and additionalValue == std::numeric_limits<double>::infinity()) {
								// oh no
								throw HatScheT::Exception("Oh no, could not find conflict constraint in either base or additional constraints");
							}
							else if (additionalValue < baseValue) {
								this->additionalConflicts.emplace_back(n, p, (int)additionalValue);
							}
							else {
								this->baseConflicts.emplace_back(n, p, (int)baseValue);
							}
							std::advance(visitedIt, 1);
						}
						break; // stop if we found a loop
					}
					visited.emplace_back(conflictNode);
				}
				break;
			}
			if (conflict) break;
		}
		if (conflict) {
			if (!this->quiet) std::cout << "Failed to find solution due to conflict(s)" << std::endl;
			return;
		}
		// save result if we found a solution
		for (auto &i : this->variables) {
			this->solution[i] = distance.at(i);
		}
	}

	std::list<std::tuple<int, int, int>> SDCSolverBellmanFord::getCurrentConstraints() const {
		std::list<std::tuple<int, int, int>> returnMe;
		for (auto &it : this->baseConstraints) {
			returnMe.emplace_back(it.first.first, it.first.second, it.second);
		}
		for (auto &it : this->additionalConstraints) {
			returnMe.emplace_back(it.first.first, it.first.second, it.second);
		}
		return returnMe;
	}

	std::map<int, int> SDCSolverBellmanFord::getNormalizedSolution() const {
		std::map<int, int> returnMe;
		int minVal = 0;
		for (auto &it : this->solution) {
			minVal = std::min(minVal, it.second);
		}
		for (auto &it : this->solution) {
			returnMe[it.first] = it.second-minVal;
		}
		return returnMe;
	}
}