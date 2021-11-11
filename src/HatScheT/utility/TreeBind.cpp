#include "TreeBind.h"
#include <HatScheT/utility/Exception.h>
#include <stdexcept>
#include <algorithm>
#include <sstream>
#include <limits>

namespace HatScheT {
  TreeBind::TreeBind(Graph* g, ResourceModel* rm, std::map<Vertex*, int> sched, int II, std::map<Edge*,int> portAssignments) 
  : g(g), rm(rm), sched(sched), II(II), portAssignments(portAssignments), currentBinding(), bestBinding(),
  timeBudget(300.0), wMux(1.0), wReg(1.0), quiet(true), pruningEnabled(true),
  skipEquivalentBindings(true),
  maxMux(std::numeric_limits<double>::infinity()), maxReg(std::numeric_limits<double>::infinity()) {
    // check if port assignments are complete
    for(auto e : g->Edges()) {
      try {
        // chaining edges do not need a port assignment
        if (!e->isDataEdge()) continue; 
        // check if port assignment is set
        portAssignments.at(e);
      }
      catch (std::out_of_range&) {
        // throw error for missing port assignment
        auto vSrc = e->getVertexSrcName();
        auto vDst = e->getVertexDstName();
        throw HatScheT::Exception("Missing port assignment for edge '"+vSrc+"' -> '"+vDst+"'");
      }
    }

    // compute total number of leaf nodes
    // i.e. total number of feasible bindings that must be explored
    this->numFeasibleBindings = 1.0;
    for (auto r : rm->Resources()) {
    	if (r->isUnlimited()) continue;
    	auto vs = rm->getVerticesOfResource(r);
    	auto limit = r->getLimit();
    	std::map<int,int> numVertices;
			for (auto v : vs) {
				numVertices[this->sched[const_cast<Vertex *>(v)] % this->II]++;
			}
			for (int i=0; i<this->II; i++) {
				double fraction = 1.0;
				for (int j=limit-numVertices[i]+1; j<=limit; j++) {
					fraction *= (double) j;
				}
				this->numFeasibleBindings *= fraction;
			}
    }
    std::cout << "TreeBind::Constructor: number of vertices: " << this->g->getNumberOfVertices() << std::endl;
    std::cout << "TreeBind::Constructor: number of feasible bindings for this problem: " << this->numFeasibleBindings << std::endl;

    // compute queue for tree traversal
    // at the moment we just insert all limited operations
    // in order of appearance in memory
    for (auto r : rm->Resources()) {
      if (r->isUnlimited()) continue;
      auto vs = rm->getVerticesOfResource(r);
      for (auto v : vs) {
        // omit const keyword because it is annoying
        this->queue.push_back(const_cast<Vertex*>(v));
      }
    }
    
    // compute FU queues for each limited operation
    // at the moment just try 0, ... , #FUs-1 in ascending order
    for (auto r : rm->Resources()) {
      if (r->isUnlimited()) continue;
      auto numFUs = r->getLimit();
      auto vs = rm->getVerticesOfResource(r);
      for (auto vc : vs) {
        // omit const keyword because it is annoying
        auto v = const_cast<Vertex*>(vc);
        for (int limit=0; limit<numFUs; limit++) {
          this->resourceQueues[v].push_back(limit);
        }
        this->lastTriedBinding[v] = this->resourceQueues[v].back();
      }
    }
  }

  void TreeBind::setQuiet(bool q) {
  	this->quiet = q;
  }
  
  void TreeBind::setRegCostFactor(double wReg) {
    this->wReg = wReg;
  }
  
  void TreeBind::setMuxCostFactor(double wMux) {
    this->wMux = wMux;
  }
  
  void TreeBind::setTimeout(unsigned int timeout) {
    this->timeBudget = (double) timeout;
  }
  
  void TreeBind::bind() {
  	// time tracking
  	this->timePoints["bind_start"] = std::chrono::steady_clock::now();
    // store info about edges for efficient lookup
    for (auto &e : this->g->Edges()) {
      if (!e->isDataEdge()) continue;
      auto *vSrc = &e->getVertexSrc();
      auto *vDst = &e->getVertexDst();
      this->outgoingEdges[vSrc].push_back(e);
      this->incomingEdges[vDst].push_back(e);
    }
    
    // init status
    this->currentBinding.solutionStatus = "TIMEOUT_INFEASIBLE";

    // fix binding for operations with unlimited resources
    for (auto r : this->rm->Resources()) {
      if (!r->isUnlimited()) continue;
      int resourceCounter = 0;
      auto vs = rm->getVerticesOfResource(r);
      for (auto v : vs) {
        this->currentBinding.resourceBindings[v->getName()] = resourceCounter++;
        if (!this->quiet) {
        	std::cout << "TreeBind::bind: fixed binding '" << v->getName() << "' -> '" << this->currentBinding.resourceBindings[v->getName()] << "'" << std::endl;
        }
      }
    }

    // fix connections between unlimited operations
    // and init binding costs
    // even though they will never produce multiplexers
    // but they might produce registers
		this->currentBinding.multiplexerCosts = 0;
		this->currentBinding.registerCosts = 0;
    for (auto &rSrc : this->rm->Resources()) {
      if (!rSrc->isUnlimited()) continue;
      auto vs = rm->getVerticesOfResource(rSrc);
      for (auto &vSrc : vs) {
      	int maxLifetime = 0;
        for (auto &e : this->outgoingEdges[vSrc]) {
          auto *vDst = &e->getVertexDst();
          auto *rDst = this->rm->getResource(vDst);
          if (!rDst->isUnlimited()) continue;
					if (!this->quiet) {
						std::cout << "TreeBind::bind: found edge between unlimited resources: '" << vSrc->getName() << "' -> '" << vDst->getName() << "'" << std::endl;
					}
					this->addBindingInfo(const_cast<Edge*>(e));
        }
      }
    }
		if (!this->quiet) {
			std::cout << "TreeBind::bind: initialized binding costs for unlimited resources" << std::endl;
		}
    
    // do a full tree search for operations with limited resources ...
    this->iterativeTreeSearch();

		// time tracking
		this->timePoints["bind_end"] = std::chrono::steady_clock::now();
		this->timeTracker["total_time"] += ((double)std::chrono::duration_cast<std::chrono::microseconds>(this->timePoints["bind_end"] - this->timePoints["bind_start"]).count()) / 1000000.0;

		// print solution
		//if (!this->quiet) {
			std::cout << "TreeBind::bind: found solution after " << this->iterationCounter << " iterations" << std::endl;
			std::cout << "  status: " << this->bestBinding.solutionStatus << std::endl;
			std::cout << "  explored " << this->leafNodeCounter << " of " << this->numFeasibleBindings << " leaf nodes" << std::endl;
			std::cout << "  times: " << std::endl;
			for (auto &it : this->timeTracker) {
				std::cout << "    " << it.first << ": " << it.second << " sec" << std::endl;
			}
			std::cout << "  final resource bindings:" << std::endl;
			for (auto &it : this->bestBinding.resourceBindings) {
				std::cout << "    " << it.first << " -> " << it.second << std::endl;
			}
			if (this->bestBinding.resourceBindings.empty()) {
				std::cout << "    empty" << std::endl;
			}
			std::cout << "  final fu connections:" << std::endl;
			for (auto &it : this->bestBinding.fuConnections) {
				std::cout << "    " << it.first.first.first << " (" << it.first.first.second << ") -> " << it.first.second.first << " (" << it.first.second.second << ") port '" << it.second.second << "' over '" << it.second.first << "' lifetime registers" << std::endl;
			}
			if (this->bestBinding.fuConnections.empty()) {
				std::cout << "    empty" << std::endl;
			}
			std::cout << "  multiplexer costs: " << this->bestBinding.multiplexerCosts << std::endl;
			std::cout << "  register costs: " << this->bestBinding.registerCosts << std::endl;
			std::cout << "  objective: " << this->wMux << " * " << this->bestBinding.multiplexerCosts << " + " << this->wReg << " * " << this->bestBinding.registerCosts << std::endl;
		//}
  }
  
  Binding::BindingContainer TreeBind::getBinding() {
    return this->bestBinding;
  }
  
  void TreeBind::iterativeTreeSearch() {
		// time tracking
  	if (!this->quiet) {
			std::cout << "TreeBind::iterativeTreeSearch: start" << std::endl;
  	}
  	// init iteration counter
		this->iterationCounter = 0;
		this->leafNodeCounter = 0;
    // init occupiedResources
    for (auto r : this->rm->Resources()) {
      for (int i=0; i<this->II; i++) {
        this->occupiedResources[std::make_pair(r, i)] = std::list<int>();
      }
    }

    // init stack
		// and push first vertex on stack based on the number of its possible resource bindings
    std::list<std::list<Vertex*>::iterator> stack;
		if (!this->queue.empty()) {
			this->pushToStack(stack, this->queue.begin());
			/*
			for (int i=0; i<this->rm->getResource(this->queue.front())->getLimit(); i++) {
				stack.push_front(this->queue.begin());
			}
			 */
		}
		else {
			// there are no limited resources
			// in this case there is no need to do any binding because everything is implemented in parallel
			// just update final binding
			this->bestBinding = this->currentBinding;
		}
		if (!this->quiet) {
			std::cout << "TreeBind::iterativeTreeSearch: initialized stack" << std::endl;
		}

		// init time tracker
		this->timePoints["iterativeTreeSearch_start"] = std::chrono::steady_clock::now();
		auto elapsedTime = 0.0;
		this->timeTracker["startup"] = ((double)std::chrono::duration_cast<std::chrono::milliseconds>(this->timePoints["iterativeTreeSearch_start"] - this->timePoints["bind_start"]).count()) / 1000.0;
    
    while (!stack.empty() and this->timeBudget > elapsedTime) {
			this->timePoints["iteration_start"] = std::chrono::steady_clock::now();
    	// debug info
			this->iterationCounter++;
			if (!this->quiet) {
				std::cout << std::endl << "TreeBind::iterativeTreeSearch: start new iteration (" << this->iterationCounter << ")" << std::endl;
				std::cout << "  elapsed time = " << elapsedTime << std::endl;
			}

      // pop front vertex from stack
      auto currentVertexIterator = stack.front();
      auto currentVertex = *currentVertexIterator;
      //auto currentResource = this->rm->getResource(currentVertex); // O(log n)
      //auto currentScheduleTime = this->sched[currentVertex]; // O(log n)
      stack.pop_front();
			if (!this->quiet) {
				std::cout << "  next vertex: " << currentVertex->getName() << std::endl;
				std::cout << "  current resource bindings:" << std::endl;
				for (auto &it : this->currentBinding.resourceBindings) {
					std::cout << "    " << it.first << " -> " << it.second << std::endl;
				}
				if (this->currentBinding.resourceBindings.empty()) {
					std::cout << "    empty" << std::endl;
				}
				std::cout << "  current fu connections:" << std::endl;
				for (auto &it : this->currentBinding.fuConnections) {
					std::cout << "    " << it.first.first.first << " (" << it.first.first.second << ") -> " << it.first.second.first << " (" << it.first.second.second << ") port '" << it.second.second << "' over '" << it.second.first << "' lifetime registers" << std::endl;
				}
				if (this->currentBinding.fuConnections.empty()) {
					std::cout << "    empty" << std::endl;
				}
				std::cout << "  multiplexer costs: " << this->currentBinding.multiplexerCosts << std::endl;
				std::cout << "  register costs: " << this->currentBinding.registerCosts << std::endl;
			}

			this->timePoints["iteration_stack_handling"] = std::chrono::steady_clock::now();
			this->timeTracker["iteration_stack_handling"] += ((double)std::chrono::duration_cast<std::chrono::microseconds>(this->timePoints["iteration_stack_handling"] - this->timePoints["iteration_start"]).count()) / 1000000.0;

      // remove binding info from binding for all vertices until the current one
      // obviously only if binding info already exists...
      // simultaneously update current costs
      // we must through the queue starting from the current vertex until the end to remove all
      // nodes from the tree until the end
      bool startRemoving = false;
      for (auto v : this->queue) {
      	if (v == currentVertex) {
      		startRemoving = true;
      	}
      	if (startRemoving) {
					this->removeBindingInfo(v);
					if (!this->quiet) {
						std::cout << "  removed binding info for '" << v->getName() << "'" << std::endl;
					}
      	}
      }

			this->timePoints["iteration_remove_binding_info"] = std::chrono::steady_clock::now();
			this->timeTracker["iteration_remove_binding_info"] += ((double)std::chrono::duration_cast<std::chrono::microseconds>(this->timePoints["iteration_remove_binding_info"] - this->timePoints["iteration_stack_handling"]).count()) / 1000000.0;

      // calculate new binding info for current node 
      // and insert it into binding container
      // simultaneously update current costs
      this->addBindingInfo(currentVertex);
			if (!this->quiet) {
				std::cout << "  added binding info for '" << currentVertex->getName() << "' to FU " << this->currentBinding.resourceBindings[currentVertex->getName()] << std::endl;
			}

			this->timePoints["iteration_add_binding_info"] = std::chrono::steady_clock::now();
			this->timeTracker["iteration_add_binding_info"] += ((double)std::chrono::duration_cast<std::chrono::microseconds>(this->timePoints["iteration_add_binding_info"] - this->timePoints["iteration_remove_binding_info"]).count()) / 1000000.0;

			// sanity checks - ToDo: remove me when all bugs are fixed
			// check if costs are correctly tracked
			int actualMuxCosts = this->currentBinding.fuConnections.size();
			int actualRegCosts = 0;
			for (auto r : this->rm->Resources()) {
				int limit = r->getLimit();
				if (r->isUnlimited()) limit = this->rm->getVerticesOfResource(r).size();
				for (int i=0; i<limit; i++) {
					int maxRegs = 0;
					for (auto &it : this->currentBinding.fuConnections) {
						if (it.first.first.first != r->getName()) continue;
						if (it.first.first.second != i) continue;
						if (maxRegs < it.second.first) maxRegs = it.second.first;
					}
					actualRegCosts += maxRegs;
				}
			}
			if (actualMuxCosts != this->currentBinding.multiplexerCosts) {
				std::cout << "TreeBind::iterativeTreeSearch: expected mux costs '" << actualMuxCosts << "' but got '" << this->currentBinding.multiplexerCosts << "'" << std::endl;
				throw HatScheT::Exception("actual mux costs are not equal to calculated mux costs - this should never happen");
			}
			if (actualRegCosts != this->currentBinding.registerCosts) {
				std::cout << "TreeBind::iterativeTreeSearch: expected reg costs '" << actualRegCosts << "' but got '" << this->currentBinding.registerCosts << "'" << std::endl;
				throw HatScheT::Exception("actual reg costs are not equal to calculated reg costs - this should never happen");
			}
			// check if no unnecessary connections are present
			for (auto &it : this->currentBinding.fuConnections) {
				// check if this connection is needed for any edge
				bool needed = false;
				for (auto &e : this->g->Edges()) {
					auto vSrc = &e->getVertexSrc();
					auto vDst = &e->getVertexDst();
					// check if binding exists
					auto fuSrcPtr = this->currentBinding.resourceBindings.find(vSrc->getName());
					auto fuDstPtr = this->currentBinding.resourceBindings.find(vDst->getName());
					if (fuSrcPtr == this->currentBinding.resourceBindings.end()) continue;
					if (fuDstPtr == this->currentBinding.resourceBindings.end()) continue;
					// check if resources match
					auto rSrc = this->rm->getResource(vSrc);
					auto rDst = this->rm->getResource(vDst);
					if (rSrc->getName() != it.first.first.first) continue;
					if (rDst->getName() != it.first.second.first) continue;
					// check if fus match
					auto fuSrc = fuSrcPtr->second;
					auto fuDst = fuDstPtr->second;
					if (fuSrc != it.first.first.second) continue;
					if (fuDst != it.first.second.second) continue;
					// check if lifetime matches
					auto lifetime = this->getLifetime(e);
					if (lifetime != it.second.first) continue;
					// check if port matches
					auto port = this->portAssignments[e];
					if (port != it.second.second) continue;
					needed = true;
					break;
				}
				if (!needed) {
					std::cout << "TreeBind::iterativeTreeSearch: fu connection '" << it.first.first.first << "' (" << it.first.first.second << ") -> '" << it.first.second.first << "' (" << it.first.second.second << "), port '" << it.second.second << "' over '" << it.second.first << "' lifetime registers is unnecessary" << std::endl;
					throw HatScheT::Exception("detected unnecessary fu connection - this should never happen");
				}
			}

      // check if we hit a leaf node
      // if so: check if we need to update best binding
      // push children on stack (push front for depth first search)
      if (currentVertex == this->queue.back()) {
				if (!this->quiet) {
					std::cout << "  hit leaf node" << std::endl;
				}
				this->leafNodeCounter++;
        if (
        	// current binding is feasible
					this->currentBinding.multiplexerCosts <= this->maxMux
					and
					this->currentBinding.registerCosts <= this->maxReg
					and
					(
          // current binding is better than best binding
          ((this->wMux * this->currentBinding.multiplexerCosts + this->wReg * this->currentBinding.registerCosts) < (this->wMux * this->bestBinding.multiplexerCosts + this->wReg * this->bestBinding.registerCosts))
          or
          // best binding is not feasible
          (this->bestBinding.multiplexerCosts < 0 or this->bestBinding.registerCosts < 0)
          )
					){
          this->bestBinding = this->currentBinding;
          this->bestBinding.solutionStatus = "TIMEOUT_FEASIBLE";
					if (!this->quiet) {
						std::cout << "  updated best solution" << std::endl;
					}
        }
      }
      else {
				if (!this->quiet) {
					std::cout << "  did not hit leaf node" << std::endl;
				}
				// check if we can prune the search tree starting from here
				if (this->pruningEnabled) {
					if (this->currentBinding.multiplexerCosts > 0 and this->currentBinding.registerCosts > 0 and this->bestBinding.multiplexerCosts > 0 and this->bestBinding.registerCosts > 0) {
						if ((this->wMux * this->currentBinding.multiplexerCosts + this->wReg * this->currentBinding.registerCosts) > (this->wMux * this->bestBinding.multiplexerCosts + this->wReg * this->bestBinding.registerCosts)) {
							if (!this->quiet) {
								std::cout << "  pruned search space because" << std::endl;
								std::cout << "    current register costs: " << this->currentBinding.registerCosts << std::endl;
								std::cout << "    current multiplexer costs: " << this->currentBinding.multiplexerCosts << std::endl;
								std::cout << "    best register costs: " << this->bestBinding.registerCosts << std::endl;
								std::cout << "    best multiplexer costs: " << this->bestBinding.multiplexerCosts << std::endl;
							}
							this->timePoints["iteration_end"] = std::chrono::steady_clock::now();
							this->timeTracker["iteration_pruning"] += ((double)std::chrono::duration_cast<std::chrono::microseconds>(this->timePoints["iteration_end"] - this->timePoints["iteration_add_binding_info"]).count()) / 1000000.0;
							elapsedTime = ((double)std::chrono::duration_cast<std::chrono::milliseconds>(this->timePoints["iteration_end"] - this->timePoints["iterativeTreeSearch_start"]).count()) / 1000.0;
							continue;
						}
						else if (!this->quiet) {
							std::cout << "  could not prune search space" << std::endl;
						}
					}
				}

				// no pruning possible => push children on stack
        auto nextVertexIterator = std::next(currentVertexIterator);
				/*
        auto nextVertex = *nextVertexIterator;
        auto nextResource = this->rm->getResource(nextVertex); // O(log n)
        auto nextScheduleTime = this->sched[nextVertex]; // O(log n)
        for (int i=0; i<nextResource->getLimit() - this->occupiedResources[std::make_pair(nextResource, nextScheduleTime % this->II)].size(); i++) { // O(m*n)
          stack.push_front(nextVertexIterator);
        }
        */
				this->pushToStack(stack, nextVertexIterator);
      }
      
      // update elapsed time
			this->timePoints["iteration_end"] = std::chrono::steady_clock::now();
			elapsedTime = ((double)std::chrono::duration_cast<std::chrono::milliseconds>(this->timePoints["iteration_end"] - this->timePoints["iterativeTreeSearch_start"]).count()) / 1000.0;
			if (!this->quiet) {
				std::cout << "  updated elapsed time (" << elapsedTime << "); starting new iteration now..." << std::endl;
			}
    }
    // we have found the optimum solution iff the stack is empty
    // i.e. we didn't terminate because of a timeout
    if (stack.empty()) {
    	if (this->bestBinding.multiplexerCosts >= 0 and this->bestBinding.registerCosts >= 0) {
				if (!this->quiet) {
					std::cout << "TreeBind::iterativeTreeSearch: found optimum solution" << std::endl;
				}
				this->bestBinding.solutionStatus = "OPTIMAL";
    	}
    	else {
				if (!this->quiet) {
					std::cout << "TreeBind::iterativeTreeSearch: problem is infeasible" << std::endl;
				}
				this->bestBinding.solutionStatus = "INFEASIBLE";
			}
    }
    else if (!this->quiet) {
			std::cout << "TreeBind::iterativeTreeSearch: did not find optimum solution" << std::endl;
		}
  }
  
  void TreeBind::removeBindingInfo(Edge* e) {
		if (!this->quiet) {
			std::cout << "    removing binding info for edge '" << e->getVertexSrcName() << "' -> '" << e->getVertexDstName() << "' with distance '" << e->getDistance() << "'" << std::endl;
		}
		// get lifetime
		auto lifetime = this->getLifetime(e);
		// get correct binding variable
		auto itPtr = this->edgeBindingMap.find(e); // O(log n)
		auto it = *(itPtr->second);
		this->edgeBindingMap.erase(itPtr);
		// pair<pair<pair<src resource type, src FU number>, pair<dst resource type, dst FU number>>, pair<number of lifetime registers, dst input port number>>
		auto bindingEdgeMapIterator = this->bindingEdgeMap.find(it); // O(log n)
		if (bindingEdgeMapIterator == this->bindingEdgeMap.end()) {
			std::stringstream con;
			con << "'" << it.first.first.first << "' (" << it.first.first.second << ") -> '" << it.first.second.first << "' (" << it.first.second.second << ") port '" << it.second.second << "' over '" << it.second.first << "' lifetime registers";
			throw HatScheT::Exception("could not find responsible edges for FU connection ("+con.str()+") - this should never happen...");
		}
		auto &edges = (*bindingEdgeMapIterator).second;
		auto edgeIterator = std::find(edges.begin(), edges.end(), e); // O(n) this should never return edges.end()
		edges.erase(edgeIterator); // O(1)?
		auto &regList = this->lifetimeRegisters[std::make_pair(it.first.first.first, it.first.first.second)]; // O(log n)
		regList.erase(std::find(regList.begin(), regList.end(), std::make_pair(lifetime, e)));
		// delete connection if necessary
		if (edges.empty()) {
			if (!this->quiet) {
				std::cout << "    deleting connection '" << it.first.first.first << "' (" << it.first.first.second << ") -> '" << it.first.second.first << "' (" << it.first.second.second << ") port '" << it.second.second << "' over '" << it.second.first << "' lifetime registers" << std::endl;
			}
			// update mux costs
			this->currentBinding.multiplexerCosts--;
			// handle containers
			this->bindingEdgeMap.erase(bindingEdgeMapIterator); // O(1)
			auto itPtr2 = std::find(this->currentBinding.fuConnections.begin(), this->currentBinding.fuConnections.end(), it); // O(n)
			this->currentBinding.fuConnections.erase(itPtr2); // O(1)
		}
		else if (!this->quiet) {
			std::cout << "    can not delete connection '" << it.first.first.first << "' (" << it.first.first.second << ") -> '" << it.first.second.first << "' (" << it.first.second.second << ") port '" << it.second.second << "' over '" << it.second.first << "' lifetime registers" << std::endl;
			std::cout << "    because of following edges:" << std::endl;
			for (auto &edgeIt : edges) {
				std::cout << "      " << edgeIt->getVertexSrcName() << " -> " << edgeIt->getVertexDstName() << " with distance " << edgeIt->getDistance() << std::endl;
			}
		}
		// update reg costs
		if (regList.empty()) {
			this->currentBinding.registerCosts -= lifetime;
			if (!this->quiet) {
				std::cout << "    updated register costs to " << this->currentBinding.registerCosts << std::endl;
			}
		}
		else {
			auto newRegs = regList.front().first;
			// only update register costs if they decrease due to the elemination of this connection
			if (newRegs < lifetime) {
				this->currentBinding.registerCosts -= (lifetime - newRegs);
				if (!this->quiet) {
					std::cout << "    updated register costs to " << this->currentBinding.registerCosts << std::endl;
				}
			}
		}

		/*
		//if (edges.empty()) {
			// this was the only resource that was responsible
			// for this specific connection
			// now we must erase the connection
			// i.e., handle mux costs;
			this->currentBinding.multiplexerCosts--;
			// handle register costs;
			if (regList.front().second == e) {
				this->currentBinding.registerCosts -= regList.front().first;
				regList.pop_front();
				if (!regList.empty()) {
					this->currentBinding.registerCosts += regList.front().first;
				}
				if (!this->quiet) {
					std::cout << "    updated register costs to " << this->currentBinding.registerCosts << std::endl;
				}
			}
			// handle containers
			bindingEdgeMap.erase(bindingEdgeMapIterator); // O(1)
			auto itPtr = std::find(this->currentBinding.fuConnections.begin(), this->currentBinding.fuConnections.end(), it); // O(n)
			this->currentBinding.fuConnections.erase(itPtr); // O(1)
		//}
		 */
  }
  
  void TreeBind::addBindingInfo(Edge* e) {
		if (!this->quiet) {
			std::cout << "    adding binding info for edge '" << e->getVertexSrcName() << "' -> '" << e->getVertexDstName() << "' with distance '" << e->getDistance() << "'" << std::endl;
		}
    // check if there already exists a connection 
    // between the correct FUs with the correct number of lifetime 
    // registers to the correct port
    auto srcV = &e->getVertexSrc();
    auto dstV = &e->getVertexDst();
    // only continue if both vertices already have a binding
    auto srcFuIt = this->currentBinding.resourceBindings.find(srcV->getName());
    auto dstFuIt = this->currentBinding.resourceBindings.find(dstV->getName());
    if (srcFuIt == this->currentBinding.resourceBindings.end()) {
      return;
    }
    if (dstFuIt == this->currentBinding.resourceBindings.end()) {
      return;
    }
    auto srcFu = (*srcFuIt).second;
    auto dstFu = (*dstFuIt).second;
    auto srcRes = this->rm->getResource(srcV);
    auto dstRes = this->rm->getResource(dstV);
    auto lifetime = this->getLifetime(e);
    auto port = this->portAssignments[e];
    auto connectionMadness = std::make_pair(std::make_pair(std::make_pair(srcRes->getName(), srcFu), std::make_pair(dstRes->getName(), dstFu)), std::make_pair(lifetime, port));
		// add port connection
		this->bindingEdgeMap[connectionMadness].push_back(e);
		auto duplicateIt = std::find(this->currentBinding.fuConnections.begin(), this->currentBinding.fuConnections.end(), connectionMadness);
    if (duplicateIt != this->currentBinding.fuConnections.end()) {
      // connection already exists and can be re-used for this edge
      if (!this->quiet) {
      	std::cout << "      connection already exists and will be reused for this edge" << std::endl;
      }
      // need to update containers, though...
			this->edgeBindingMap[e] = &(*duplicateIt);
			auto &regList = this->lifetimeRegisters[std::make_pair(srcRes->getName(), srcFu)];
			bool inserted = false;
			for (auto it = regList.begin(); it != regList.end(); it++) {
				if (it->first < lifetime) {
					regList.insert(it, std::make_pair(lifetime, e));
					inserted = true;
					break;
				}
			}
			if (!inserted) {
				regList.emplace_back(std::make_pair(lifetime, e));
			}
      return;
    }
    // increment mux costs because we need one more connection
    this->currentBinding.multiplexerCosts++;
    
    // check if enough lifetime registers are available
    // and insert edge into lifetime register list
    auto &regList = this->lifetimeRegisters[std::make_pair(srcRes->getName(), srcFu)];
    if (regList.empty()) {
      // this is the first operation bound to this fu
      regList.push_front(std::make_pair(lifetime, e));
      this->currentBinding.registerCosts += lifetime;
			if (!this->quiet) {
				std::cout << "    this is the first operation bound to this fu" << std::endl;
				std::cout << "    incremented register costs by " << lifetime << std::endl;
				std::cout << "    updated register costs to " << this->currentBinding.registerCosts << std::endl;
			}
    }
    else {
      // there is at least one other operation bound to this fu
      auto availableRegs = regList.front().first;
      if (availableRegs < lifetime) {
        this->currentBinding.registerCosts += (lifetime - availableRegs);
				if (!this->quiet) {
					std::cout << "    this is not the first operation bound to this fu" << std::endl;
					std::cout << "    incremented register costs by " << lifetime-availableRegs << std::endl;
					std::cout << "    updated register costs to " << this->currentBinding.registerCosts << std::endl;
				}
      }
      bool inserted = false;
      for (auto it = regList.begin(); it != regList.end(); it++) {
        if (it->first < lifetime) {
          regList.insert(it, std::make_pair(lifetime, e));
          inserted = true;
          break;
        }
      }
      if (!inserted) {
        regList.emplace_back(std::make_pair(lifetime, e));
      }
    }
    // add binding info
		this->currentBinding.fuConnections.emplace_back(connectionMadness);
		this->edgeBindingMap[e] = &this->currentBinding.fuConnections.back();
  }
  
  void TreeBind::removeBindingInfo(Vertex* v) {
  	// maybe a binding for this vertex does not exist yet...
  	// skip it then
		auto mapIt = this->currentBinding.resourceBindings.find(v->getName());
		if (mapIt == this->currentBinding.resourceBindings.end()) return;
		if (!this->quiet) {
			std::cout << "    start removing binding info for vertex '" << mapIt->first << "' to FU " << mapIt->second << std::endl;
		}

    // handle incoming edges from other FUs
    for (auto &eConst : this->incomingEdges[v]) {
      auto* e = const_cast<Edge*>(eConst);
      if (!this->quiet) {
				std::cout << "    incoming edge '" << e->getVertexSrcName() << "' -> '" << e->getVertexDstName() << "' with distance '" << e->getDistance() << "'" << std::endl;
      }
			if (this->currentBinding.resourceBindings.find(e->getVertexSrcName()) == this->currentBinding.resourceBindings.end()) {
				if (!this->quiet) {
					std::cout << "    ... skipped" << std::endl;
				}
				continue;
			}
      this->removeBindingInfo(e);
    }
    
    // handle outgoing edges to other FUs
    for (auto &eConst : this->outgoingEdges[v]) {
      auto* e = const_cast<Edge*>(eConst);
			if (!this->quiet) {
				std::cout << "    outgoing edge '" << e->getVertexSrcName() << "' -> '" << e->getVertexDstName() << "' with distance '" << e->getDistance() << "'" << std::endl;
			}
			if (this->currentBinding.resourceBindings.find(e->getVertexDstName()) == this->currentBinding.resourceBindings.end()) {
				if (!this->quiet) {
					std::cout << "    ... skipped" << std::endl;
				}
				continue;
			}
      this->removeBindingInfo(e);
    }
    
    // remove binding variable for that vertex and update occupied resources
    auto fu = (*mapIt).second;
    this->lastTriedBinding[v] = fu;
    auto moduloSlot = this->sched[v] % this->II;
    auto res = this->rm->getResource(v);
    this->occupiedResources[std::make_pair(res, moduloSlot)].remove(fu);
    this->currentBinding.resourceBindings.erase(mapIt);
  }
  
  void TreeBind::addBindingInfo(Vertex* v) {
    // add binding variable for that vertex and update occupied resources
    auto moduloSlot = this->sched[v] % this->II;
    auto res = this->rm->getResource(v);
    auto resSlotPair = std::make_pair(res, moduloSlot);
    auto vertexQueue = this->resourceQueues[v];
    auto nextBindingIt = std::find(vertexQueue.begin(), vertexQueue.end(), this->lastTriedBinding[v]);
    for (int i=0; i<res->getLimit(); i++) {
      if (*nextBindingIt == vertexQueue.back()) {
				nextBindingIt = vertexQueue.begin();
      }
      else {
				nextBindingIt = std::next(nextBindingIt);
      }
      if (std::find(this->occupiedResources[resSlotPair].begin(), this->occupiedResources[resSlotPair].end(), *nextBindingIt) == this->occupiedResources[resSlotPair].end()) {
        this->occupiedResources[resSlotPair].push_back(*nextBindingIt);
        break;
      }
    }
    auto nextBinding = *nextBindingIt;
    this->currentBinding.resourceBindings[v->getName()] = nextBinding;
    if (!this->quiet) {
    	std::cout << "  fixed binding '" << v->getName() << "' -> '" << nextBinding << "'" << std::endl;
    }
    
    // handle incoming edges from other FUs
    for (auto &eConst : this->incomingEdges[v]) {
      auto* e = const_cast<Edge*>(eConst);
			if (!this->quiet) {
				std::cout << "    incoming edge '" << e->getVertexSrcName() << "' -> '" << e->getVertexDstName() << "' with distance '" << e->getDistance() << "'" << std::endl;
			}
			if (this->currentBinding.resourceBindings.find(e->getVertexSrcName()) == this->currentBinding.resourceBindings.end()) {
				if (!this->quiet) {
					std::cout << "    ... skipped" << std::endl;
				}
				continue;
			}
      this->addBindingInfo(e);
    }
    
    // handle outgoing edges to other FUs
    for (auto &eConst : this->outgoingEdges[v]) {
      auto* e = const_cast<Edge*>(eConst);
			if (!this->quiet) {
				std::cout << "    outgoing edge '" << e->getVertexSrcName() << "' -> '" << e->getVertexDstName() << "' with distance '" << e->getDistance() << "'" << std::endl;
			}
			if (this->currentBinding.resourceBindings.find(e->getVertexDstName()) == this->currentBinding.resourceBindings.end()) {
				if (!this->quiet) {
					std::cout << "    ... skipped" << std::endl;
				}
				continue;
			}
      this->addBindingInfo(e);
    }
  }
  
  std::string TreeBind::getSolutionStatus() {
    return this->bestBinding.solutionStatus;
  }
  
  int TreeBind::getLifetime(const Edge* e) {
    try {
      return this->lifetimes.at(e);
    }
    catch (std::out_of_range&) {
    	auto src = &e->getVertexSrc();
    	auto dst = &e->getVertexDst();
      auto lifetime = this->sched[dst] - this->sched[src] - this->rm->getVertexLatency(src) + (this->II * e->getDistance());
      if (!this->quiet) {
      	std::cout << "    lifetime for edge '" << src->getName() << "' -> '" << dst->getName() << "' with distance '" << e->getDistance() << "' is '" << lifetime << "'" << std::endl;
      }
      this->lifetimes[e] = lifetime;
      return lifetime;
    }
  }

	void TreeBind::setMuxLimit(double l) {
  	if (l >= 0.0) this->maxMux = l;
  	else this->maxMux = std::numeric_limits<double>::infinity();
  }

	void TreeBind::setRegLimit(double l) {
		if (l >= 0.0) this->maxReg = l;
		else this->maxReg = std::numeric_limits<double>::infinity();
	}

	void TreeBind::setPruning(bool p) {
		this->pruningEnabled = p;
	}

	void TreeBind::setSkipEquivalent(bool s) {
		this->skipEquivalentBindings = s;
	}

	void TreeBind::pushToStack(std::list<std::list<Vertex*>::iterator> &stack, const list<Vertex *>::iterator &v) {
  	bool firstFreeResource = true;
		auto vertex = *v;
  	auto res = this->rm->getResource(vertex);
  	auto t = this->sched[vertex] % this->II;
  	auto occ = this->occupiedResources[std::make_pair(res, t)];
  	auto numOcc = occ.size();
  	auto numAddedChildren = 0;
  	auto maxAddedChildren = res->getLimit() - numOcc;
  	std::map<int,int> numOperationsOnFUs;
  	for (int i=0; i<this->II; i++) {
  		for (auto it : this->occupiedResources[std::make_pair(res, i)]) {
  			numOperationsOnFUs[it]++;
  		}
  	}
  	bool addedFreeResource = false;
		for (int i=0; i<res->getLimit(); i++) {
			// check if we can skip this resource because it is already occupied
			if (std::find(occ.begin(), occ.end(), i) != occ.end()) {
				continue;
			}
			// check if we can skip this resource because of search space pruning
			else if (numOperationsOnFUs[i] == 0) {
				if (this->skipEquivalentBindings and addedFreeResource) {
					continue;
				}
				else {
					addedFreeResource = true;
				}
			}
			// we found the first free resource => push it
			stack.push_front(v);
			numAddedChildren++;
			// check if enough children are added
			if (numAddedChildren >= maxAddedChildren) break;
		}
	}

	double TreeBind::getNumFeasibleBindings() {
		return this->numFeasibleBindings;
	}
}
